import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, QoSHistoryPolicy
import yaml
import os

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.start_time = self.get_clock().now().to_msg().sec
        self.max_exploration_time = 100
        self.goal_timeout = 15
        self.previous_unknown_cells = float('inf')
        self.visited_frontiers = []
        self.map_path = "/home/jaeheyoung/Rokey/7.PracticalProject/Week6_Drive2/rbvacuum_ws/src/automap/map"
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.is_exploring = True
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,  # 메시지 저장 방침 설정
            depth=10,                            # 저장할 메시지의 개수 설정
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.current_position = (0.0, 0.0)
        self.current_goal = None  # 현재 목표를 추적하는 변수
        self.goal_reached = False  # 목표 도달 여부 추적 변수
        
        self.last_goal_time = None
        self.last_position = None  # 로봇이 이동 중인지 체크하기 위한 변수
        
        self.timer = self.create_timer(10.0, self.print_elapsed_time)

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        if self.current_goal:
            # 목표 도달 여부 판단
            distance = np.sqrt((self.current_position[0] - self.current_goal[0])**2 + (self.current_position[1] - self.current_goal[1])**2)
            if distance < 0.3:
                # 목표 도달 시 새로운 프론티어 계산 후 이동
                self.goal_reached = True
                self.current_goal = None
                self.get_logger().info("Goal reached!")
                self.update_frontiers_after_goal()
            elif self.last_position and np.allclose(self.current_position, self.last_position, atol=0.1):
                # 로봇이 이동하지 않는 동안 goal_timeout 경과 시
                if self.last_goal_time and (self.get_clock().now().to_msg().sec - self.last_goal_time > self.goal_timeout) and not self.is_moving():
                    self.get_logger().info("Goal timeout reached. Trying new goal.")
                    self.goal_reached = False
                    self.current_goal = None
                    self.update_frontiers_after_goal()
            self.last_position = self.current_position

        if self.goal_timeout_reached() and not self.is_moving():
            self.get_logger().info("Goal timeout reached. Trying new goal.")
            self.goal_reached = False
            self.current_goal = None
            self.last_goal_time = None
            self.update_frontiers_after_goal()
        
    def map_callback(self, map_msg):
        """맵 데이터 수신 및 탐색 로직 실행"""
        self.map_msg = map_msg
        if not self.is_exploring:
            return

        # 종료 조건 확인
        if self.check_termination_conditions(map_msg):
            self.get_logger().info("Exploration complete.")
            self.is_exploring = False
            return

        # 프론티어 탐지 및 최적 목표 선정
        frontiers = self.detect_frontiers(map_msg)
        if frontiers:
            target = self.select_best_frontier(frontiers)
            if target and not self.is_goal_in_progress():
                self.publish_goal(target)
                self.current_goal = target  # 새 목표 설정
                self.goal_reached = False  # 목표 도달 여부 초기화
                self.last_goal_time = self.get_clock().now().to_msg().sec
        elif not frontiers:  # 프론티어가 없을 경우
            if self.current_goal:  # 목표가 설정되어 있을 경우
                self.get_logger().info("No more frontiers to explore. All frontiers have been explored.")
                self.is_exploring = False  # 탐사를 종료
            else:  # 목표가 없으면 새로운 목표를 찾기 위해 갱신
                self.get_logger().info("No goal set, trying to find a new goal.")
                self.update_frontiers_after_goal()  # 목표가 없으면 프론티어를 갱신하고 새로운 목표 설정
            
    def check_termination_conditions(self, map_msg):
        """탐색 종료 조건 확인"""
        frontiers = self.detect_frontiers(map_msg)
        # 1. 탐색 시간 초과
        current_time = self.get_clock().now().to_msg().sec
        if current_time - self.start_time > self.max_exploration_time:
            self.get_logger().info("Maximum exploration time reached.")
            self.save_map_to_file(map_msg, self.map_path)  # 맵 저장
            self.is_exploring = False
            return True

        # 2. 미탐색 영역 수렴
        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        unknown_cells = np.count_nonzero(map_data == -1)
        if unknown_cells >= self.previous_unknown_cells and not frontiers:
            self.get_logger().info("No more frontiers to explore.")
            self.save_map_to_file(map_msg, self.map_path)  # 맵 저장
            self.is_exploring = False
            return True
        self.previous_unknown_cells = unknown_cells

        return False

    def detect_frontiers(self, map_msg):
        """미탐색 경계를 픽셀 단위로 감지한 뒤, 일정 크기 이상의 클러스터 평균 위치를 프론티어로 반환"""
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin
        map_data = np.array(map_msg.data).reshape((height, width))

        # 1. 픽셀 단위 프론티어 탐지
        frontier_pixels = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                cell_value = map_data[y, x]
                if 0 <= cell_value < 100:
                    neighbors = map_data[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors:
                        # 프론티어 픽셀
                        frontier_pixels.append((x, y))

        # 2. 픽셀 단위 프론티어를 클러스터링
        # 간단한 BFS/DFS를 이용한 클러스터링
        visited = set()
        frontier_clusters = []
        min_cluster_size = 5  # 최소 픽셀 수 (임의 값, 상황에 맞게 조정)
        
        # 8방향 탐색을 위한 상대 좌표
        directions = [(-1, -1), (-1, 0), (-1, 1),
                    (0, -1),           (0, 1),
                    (1, -1),  (1, 0),  (1, 1)]

        # frontier_pixels를 빠르게 접근하기 위해 set 변환
        frontier_set = set(frontier_pixels)

        for px, py in frontier_pixels:
            if (px, py) not in visited:
                # 새로운 클러스터 시작
                cluster = []
                stack = [(px, py)]
                visited.add((px, py))

                while stack:
                    cx, cy = stack.pop()
                    cluster.append((cx, cy))
                    # 8방향 이웃 확인
                    for dx, dy in directions:
                        nx, ny = cx + dx, cy + dy
                        if (nx, ny) in frontier_set and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            stack.append((nx, ny))

                # 하나의 클러스터 완성
                frontier_clusters.append(cluster)

        # 3. 일정 픽셀 수 이상의 클러스터에 대해서 평균 위치 계산
        final_frontiers = []
        for cluster in frontier_clusters:
            if len(cluster) >= min_cluster_size:
                # 평균 위치 계산
                xs = [c[0] for c in cluster]
                ys = [c[1] for c in cluster]
                avg_x = sum(xs) / len(xs)
                avg_y = sum(ys) / len(ys)

                # 맵 좌표로 변환
                world_x = origin.position.x + (avg_x * resolution)
                world_y = origin.position.y + (avg_y * resolution)
                final_frontiers.append((world_x, world_y))

        return final_frontiers

    # def detect_frontiers(self, map_msg):
    #     """미탐색 경계 탐지"""
    #     width = map_msg.info.width
    #     height = map_msg.info.height
    #     resolution = map_msg.info.resolution
    #     origin = map_msg.info.origin
    #     map_data = np.array(map_msg.data).reshape((height, width))

    #     frontiers = []
    #     for y in range(1, height - 1):
    #         for x in range(1, width - 1):
    #             cell_value = map_data[y, x]
    #             # 인플레이션된 영역도 free로 간주하기 위해
    #             # 0 <= cell_value < 100 인 경우(실제 점유가 아닌 경우) 탐사 가능 셀로 판단
    #             if 0 <= cell_value < 100:
    #                 neighbors = map_data[y-1:y+2, x-1:x+2].flatten()
    #                 # 인접 영역 중 -1(unknown)이 있으면 프론티어로 판단
    #                 if -1 in neighbors:
    #                     world_x = origin.position.x + (x * resolution)
    #                     world_y = origin.position.y + (y * resolution)
    #                     frontiers.append((world_x, world_y))
    #     return frontiers
    
    def update_frontiers_after_goal(self):
        """목표 도달 또는 시간 초과 후 새로운 프론티어 탐지 및 목표 설정 (방문 기록 사용 안 함)"""
        if not self.is_exploring:
            return

        # 매번 새로운 프론티어 계산
        frontiers = self.detect_frontiers(self.map_msg)
        if frontiers:
            target = self.select_best_frontier(frontiers)  # 새로 계산한 프론티어 중 가장 가까운 목표 선택
            if target:
                self.publish_goal(target)
                self.current_goal = target  # 새로운 목표 설정
                self.goal_reached = False  # 목표 도달 여부 초기화
                self.last_goal_time = self.get_clock().now().to_msg().sec
            else:
                self.get_logger().info("No frontiers available to set as goal.")
        else:
            self.get_logger().info("No frontiers detected after goal timeout. Trying new goal.")
            # 프론티어가 없는 경우 기본 좌표(0.0, 0.0)등으로 이동 시도하거나, 탐사 종료 로직 추가 가능
            self.publish_goal((0.0, 0.0))  
            self.current_goal = (0.0, 0.0)  
            self.goal_reached = False
    
    # def select_best_frontier(self, frontiers):
    #     """프론티어 중 로봇과 가장 먼 프론티어를 선택"""
    #     if not frontiers:
    #         return None

    #     robot_position = self.current_position
    #     max_distance = float('-inf')
    #     best_frontier = None

    #     for frontier in frontiers:
    #         distance = np.sqrt((robot_position[0] - frontier[0])**2 + (robot_position[1] - frontier[1])**2)
    #         if distance > max_distance:
    #             max_distance = distance
    #             best_frontier = frontier

    #     return best_frontier
    
    def select_best_frontier(self, frontiers):
        """프론티어 중 로봇과 가장 가까운 프론티어를 선택하되,
        이전에 선택했던 목표(프론티어)는 무시한다."""
        if not frontiers:
            return None

        robot_position = self.current_position
        # 프론티어까지의 거리 계산
        distances = []
        for frontier in frontiers:
            distance = np.sqrt((robot_position[0] - frontier[0])**2 + (robot_position[1] - frontier[1])**2)
            distances.append((distance, frontier))

        # 거리 순으로 정렬(가장 가까운 순)
        distances.sort(key=lambda x: x[0])

        # 가장 가까운 것부터 순회하며 visited_frontiers에 없는 프론티어 반환
        for dist, frontier in distances:
            if frontier not in self.visited_frontiers:
                return frontier

        # 모두 방문했던 프론티어라면 None 반환
        return None
    
    def is_goal_in_progress(self):
        """현재 목표가 진행 중인지 확인 (이동 중이면 목표 변경을 방지)"""
        return self.current_goal is not None and not self.goal_reached
    
    def is_moving(self):
        """로봇이 이동 중인지 확인 (위치 변화가 있을 때 이동 중)"""
        if self.last_position:
            distance = np.sqrt((self.current_position[0] - self.last_position[0])**2 + (self.current_position[1] - self.last_position[1])**2)
            return distance > 0.1  # 일정 거리 이상 이동하면 이동 중
        return False
    
    def publish_goal(self, target):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Publishing goal: {target}")
        self.goal_pub.publish(goal)
        
    def goal_timeout_reached(self):
        """목표 시간 초과 여부 확인"""
        if self.last_goal_time and (self.get_clock().now().to_msg().sec - self.last_goal_time > self.goal_timeout):
            return True
        return False
        
    def print_elapsed_time(self):
        """경과 시간을 10초마다 출력"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        self.get_logger().info(f"Elapsed time: {elapsed_time:.2f} seconds")
        
    def save_map_to_file(self, map_msg, file_path):
        """맵 데이터를 지정된 경로에 YAML 파일 형식으로 저장"""
        if not os.path.exists(file_path):
            os.makedirs(file_path)  # 폴더가 없다면 생성
        
        # 맵 데이터 추출
        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        
        # 맵의 메타데이터
        map_metadata = {
            'image': 'map.pgm',  # 실제 이미지 파일 이름 (따로 저장할 경우 지정)
            'resolution': map_msg.info.resolution,
            'origin': {
                'position': {
                    'x': map_msg.info.origin.position.x,
                    'y': map_msg.info.origin.position.y,
                    'z': map_msg.info.origin.position.z,
                },
                'orientation': {
                    'x': map_msg.info.origin.orientation.x,
                    'y': map_msg.info.origin.orientation.y,
                    'z': map_msg.info.origin.orientation.z,
                    'w': map_msg.info.origin.orientation.w
                }
            },
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        # YAML 파일로 저장
        yaml_file = os.path.join(file_path, 'map.yaml')
        with open(yaml_file, 'w') as f:
            yaml.dump(map_metadata, f)
        
        # 맵 이미지를 PGM 파일로 저장 (선택 사항)
        pgm_file = os.path.join(file_path, 'map.pgm')
        with open(pgm_file, 'wb') as f:
            f.write(self.convert_map_to_pgm(map_data, map_msg.info.width, map_msg.info.height))
        self.get_logger().info(f"Map saved to {yaml_file} and {pgm_file}")

    def convert_map_to_pgm(self, map_data, width, height):
        """맵 데이터를 PGM 파일 형식으로 변환"""
        # PGM 파일 헤더
        header = f"P5\n{width} {height}\n255\n".encode()

        # 맵 데이터를 [0, 255] 범위로 변환하여 PGM 포맷에 맞게 저장
        image_data = np.array(map_data, dtype=np.uint8)
        image_data[image_data == -1] = 128  # 미탐색 영역은 128로 설정 (중간 회색)
        image_data[image_data == 0] = 255   # 비어있는 공간은 흰색
        image_data[image_data == 100] = 0   # 점유된 공간은 검은색
        return header + image_data.tobytes()

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()