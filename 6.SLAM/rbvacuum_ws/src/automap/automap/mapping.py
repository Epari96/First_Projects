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
        self.max_exploration_time = 180
        self.goal_timeout = 10
        self.previous_unknown_cells = float('inf')
        self.visited_frontiers = []  # 방문했던 프론티어 기록
        self.map_path = "/home/jaeheyoung/Rokey/7.PracticalProject/Week6_Drive2/rbvacuum_ws/src/automap/map"
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.is_exploring = True
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.current_position = (0.0, 0.0)
        self.current_goal = None  # 현재 목표 프론티어
        self.goal_reached = False
        
        self.last_goal_time = None
        self.last_position = None
        
        self.timer = self.create_timer(10.0, self.print_elapsed_time)

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        if self.current_goal:
            distance = np.sqrt((self.current_position[0] - self.current_goal[0])**2 + (self.current_position[1] - self.current_goal[1])**2)
            if distance < 0.3:
                # 목표 도달 시
                old_goal = self.current_goal
                self.goal_reached = True
                self.current_goal = None
                self.get_logger().info("Goal reached!")
                
                # 목표 도달했으므로 방문 기록 추가
                if old_goal not in self.visited_frontiers:
                    self.visited_frontiers.append(old_goal)

                self.update_frontiers_after_goal()
            elif self.last_position and np.allclose(self.current_position, self.last_position, atol=0.1):
                # 로봇 이동 없음 + goal_timeout 발생 시
                if self.last_goal_time and (self.get_clock().now().to_msg().sec - self.last_goal_time > self.goal_timeout) and not self.is_moving():
                    self.get_logger().info("Goal timeout reached. Trying new goal.")
                    old_goal = self.current_goal
                    self.goal_reached = False
                    self.current_goal = None

                    # 목표 타임아웃 시 이전 목표 방문 기록 추가
                    if old_goal and old_goal not in self.visited_frontiers:
                        self.visited_frontiers.append(old_goal)

                    self.update_frontiers_after_goal()
            self.last_position = self.current_position

        # goal_timeout_reached() 별도 체크
        if self.goal_timeout_reached() and not self.is_moving():
            self.get_logger().info("Goal timeout reached. Trying new goal.")
            old_goal = self.current_goal
            self.goal_reached = False
            self.current_goal = None
            self.last_goal_time = None

            # 목표 타임아웃 시 이전 목표 방문 기록 추가
            if old_goal and old_goal not in self.visited_frontiers:
                self.visited_frontiers.append(old_goal)

            self.update_frontiers_after_goal()
        
    def map_callback(self, map_msg):
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
                self.current_goal = target
                self.goal_reached = False
                self.last_goal_time = self.get_clock().now().to_msg().sec
        else:  # 프론티어가 없을 경우
            if self.current_goal:
                self.get_logger().info("No more frontiers to explore. All frontiers have been explored.")
                self.is_exploring = False
            else:
                self.get_logger().info("No goal set, trying to find a new goal.")
                self.update_frontiers_after_goal()
            
    def check_termination_conditions(self, map_msg):
        frontiers = self.detect_frontiers(map_msg)
        current_time = self.get_clock().now().to_msg().sec
        if current_time - self.start_time > self.max_exploration_time:
            self.get_logger().info("Maximum exploration time reached.")
            self.save_map_to_file(map_msg, self.map_path)  # 맵 저장
            self.is_exploring = False
            return True

        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        unknown_cells = np.count_nonzero(map_data == -1)
        if unknown_cells >= self.previous_unknown_cells and not frontiers:
            self.get_logger().info("No more frontiers to explore.")
            self.save_map_to_file(map_msg, self.map_path)
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

        frontier_pixels = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                cell_value = map_data[y, x]
                if 0 <= cell_value < 80:
                    neighbors = map_data[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors:
                        frontier_pixels.append((x, y))

        visited = set()
        frontier_clusters = []
        min_cluster_size = 20
        directions = [(-1, -1), (-1, 0), (-1, 1),
                      (0, -1),           (0, 1),
                      (1, -1),  (1, 0),  (1, 1)]
        frontier_set = set(frontier_pixels)

        for px, py in frontier_pixels:
            if (px, py) not in visited:
                cluster = []
                stack = [(px, py)]
                visited.add((px, py))

                while stack:
                    cx, cy = stack.pop()
                    cluster.append((cx, cy))
                    for dx, dy in directions:
                        nx, ny = cx + dx, cy + dy
                        if (nx, ny) in frontier_set and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            stack.append((nx, ny))

                frontier_clusters.append(cluster)

        final_frontiers = []
        for cluster in frontier_clusters:
            if len(cluster) >= min_cluster_size:
                xs = [c[0] for c in cluster]
                ys = [c[1] for c in cluster]
                avg_x = sum(xs) / len(xs)
                avg_y = sum(ys) / len(ys)
                world_x = origin.position.x + (avg_x * resolution)
                world_y = origin.position.y + (avg_y * resolution)
                final_frontiers.append((world_x, world_y))

        return final_frontiers

    def update_frontiers_after_goal(self):
        if not self.is_exploring:
            return

        frontiers = self.detect_frontiers(self.map_msg)
        if frontiers:
            target = self.select_best_frontier(frontiers)
            if target:
                self.publish_goal(target)
                self.current_goal = target
                self.goal_reached = False
                self.last_goal_time = self.get_clock().now().to_msg().sec
            else:
                self.get_logger().info("No frontiers available to set as goal.")
                self.get_logger().info(f"Frontier List: {frontiers}")
        else:
            self.get_logger().info("No frontiers detected after goal timeout. Trying new goal.")
            # self.publish_goal((0.0, 0.0))
            # self.current_goal = (0.0, 0.0)
            self.goal_reached = False
    
    def select_best_frontier(self, frontiers):
        """프론티어 중 로봇과 가장 가까운 프론티어를 선택하되,
        이전에 선택했던 목표(프론티어)는 무시한다."""
        if not frontiers:
            return None

        robot_position = self.current_position
        distances = []
        for frontier in frontiers:
            distance = np.sqrt((robot_position[0] - frontier[0])**2 + (robot_position[1] - frontier[1])**2)
            distances.append((distance, frontier))

        distances.sort(key=lambda x: x[0])

        for dist, frontier in distances:
            if frontier not in self.visited_frontiers:
                return frontier
        return None
    
    def is_goal_in_progress(self):
        return self.current_goal is not None and not self.goal_reached
    
    def is_moving(self):
        if self.last_position:
            distance = np.sqrt((self.current_position[0] - self.last_position[0])**2 + (self.current_position[1] - self.last_position[1])**2)
            return distance > 0.2
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
        if self.last_goal_time and (self.get_clock().now().to_msg().sec - self.last_goal_time > self.goal_timeout):
            return True
        return False
        
    def print_elapsed_time(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        self.get_logger().info(f"Elapsed time: {elapsed_time:.2f} seconds")
        
    def save_map_to_file(self, map_msg, file_path):
        if not os.path.exists(file_path):
            os.makedirs(file_path)
        
        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        
        map_metadata = {
            'image': 'map.pgm',
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

        yaml_file = os.path.join(file_path, 'map.yaml')
        with open(yaml_file, 'w') as f:
            yaml.dump(map_metadata, f)
        
        pgm_file = os.path.join(file_path, 'map.pgm')
        with open(pgm_file, 'wb') as f:
            f.write(self.convert_map_to_pgm(map_data, map_msg.info.width, map_msg.info.height))
        self.get_logger().info(f"Map saved to {yaml_file} and {pgm_file}")

    def convert_map_to_pgm(self, map_data, width, height):
        header = f"P5\n{width} {height}\n255\n".encode()
        image_data = np.array(map_data, dtype=np.uint8)
        image_data[image_data == -1] = 128
        image_data[image_data == 0] = 255
        image_data[image_data == 100] = 0
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
