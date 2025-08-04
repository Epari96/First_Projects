import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger
import time


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # 상태 변수
        self.tracking = False  # Tracker의 추적 상태
        self.timeout = 10  # Tracker 타이머의 타임아웃 시간(초)
        self.start_tracking_time = None  # Tracker 타이머 시작 시간
        self.initial_position = None  # 프로그램 시작 시 로봇의 초기 위치
        self.first_position = None  # Observer로부터 받은 초기 좌표
        self.tracker_position = None  # Tracker로부터 받은 타겟 좌표
        self.last_tracker_update = time.time()  # Tracker 업데이트 시간
        self.current_goal_handle = None  # 현재 Goal Handle
        self.tracking_coord = []  # 추적 좌표를 저장하는 리스트

        # Observer와 Tracker로부터 좌표를 구독하는 Subscriber
        self.observer_subscriber = self.create_subscription(
            String,
            'observer/target_coordinates',
            self.observer_callback,
            10
        )
        self.tracker_subscriber = self.create_subscription(
            String,
            'tracker/target_coordinates',
            self.tracker_callback,
            10
        )

        # cmd_vel 토픽에 Twist 메시지를 퍼블리시하는 Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # nav2로 목표 좌표를 보내기 위한 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Observer와 Tracker를 종료하기 위한 Service Clients
        self.observer_client = self.create_client(Trigger, 'observer/terminate')
        self.tracker_client = self.create_client(Trigger, 'tracker/terminate')
        
        # activate 서비스 클라이언트
        self.observer_activate_client = self.create_client(Trigger, 'observer/activate')

        # Service가 준비될 때까지 대기
        while not self.observer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Observer service not available, waiting...')
        while not self.tracker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Tracker service not available, waiting...')

        self.get_logger().info('Controller Node has been started.')

    def observer_callback(self, msg):
        """
        Observer로부터 초기 좌표를 수신했을 때 호출되는 콜백 함수
        """
        self.get_logger().info(f"Received initial position from observer: {msg.data}")
        self.first_position = self.parse_coordinates(msg.data)  # 좌표를 파싱
        if self.initial_position is None:
            self.initial_position = self.get_current_position()  # 프로그램 시작 시 초기 위치 저장
        self.send_goal(self.first_position)  # nav2로 Goal 전송

    def tracker_callback(self, msg):
        """
        Tracker로부터 좌표를 수신했을 때 호출되는 콜백 함수
        """
        if self.first_position is None:
            self.get_logger().info("Ignoring tracker update as observer has not sent a goal yet.")
            return  # Observer의 명령이 없으면 무시

        self.last_tracker_update = time.time()  # Tracker 업데이트 시간 갱신
        self.get_logger().info(f"Received position from tracker: {msg.data}")
        self.tracker_position = self.parse_coordinates(msg.data)  # 좌표를 파싱

        if len(self.tracking_coord) == 0:
            self.start_tracking()  # Tracker 타이머 시작
            
        self.tracking_coord.append(self.tracker_position)

        # # 기존 목표를 취소하고 새로운 목표를 보내는 순서 수정
        # if self.current_goal_handle:
        #     self.get_logger().info("Cancelling current goal...")
        #     cancel_future = self.current_goal_handle.cancel_goal_async()
        #     rclpy.spin_until_future_complete(self, cancel_future)
        #     if cancel_future.result().accepted:
        #         self.get_logger().info("Current goal cancelled successfully.")
        #     else:
        #         self.get_logger().warning("Failed to cancel the current goal.")
        
        # 새로운 목표 전송
        self.send_goal(self.tracker_position)
        
    def send_activate_request(self):
        if not self.observer_activate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "observer/activate" not available.')
            return
        
        request = Trigger.Request()
        future = self.observer_activate_client.call_async(request)
        future.add_done_callback(self.activate_response_callback)
        
    def activate_response_callback(self, future):
        response = future.result()
        self.get_logger().info(f"Observer activated: {response.success}, {response.message}")

    def check_tracker_timeout(self):
        """
        Tracker가 5초 이상 업데이트되지 않았을 때 회전
        """
        if time.time() - self.last_tracker_update > 5 and not self.tracking:
            self.rotate_turtlebot()

    def parse_coordinates(self, data):
        """
        좌표 데이터를 파싱하는 함수 (예: "x:1.0,y:2.0" 형식)
        """
        coords = data.split(',')
        x = float(coords[0].split(':')[1])
        y = float(coords[1].split(':')[1])
        return (x, y)

    def get_current_position(self):
        """
        프로그램 시작 시 로봇의 초기 위치를 기록
        """
        # Placeholder function for getting the robot's current position
        # In a real system, this would fetch the position from localization or odometry
        return (0.0, 0.0)  # 기본값으로 초기화

    def send_goal(self, position):
        """
        nav2로 목표 좌표를 발신하는 함수
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # 좌표계 설정
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = position[0]
        goal_msg.pose.pose.position.y = position[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info(f"Sending goal: {position}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        """
        현재 목표를 취소하는 함수
        """
        if self.current_goal_handle:
            self.get_logger().info("Cancelling current goal...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            if cancel_future.result().accepted:
                self.get_logger().info("Current goal cancelled.")
            else:
                self.get_logger().warning("Failed to cancel the current goal.")

    def goal_response_callback(self, future):
        """
        Goal 전송 결과를 처리하는 콜백 함수
        """
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Goal 수행 결과를 처리하는 콜백 함수
        """
        result = future.result().result
        self.get_logger().info('Goal result received.')
        if not self.tracking:
            self.rotate_turtlebot()  # TurtleBot3를 회전시킴
        else:
            self.handle_timeout_or_complete()  # 타임아웃 여부를 확인 후 처리

    def rotate_turtlebot(self):
        """
        TurtleBot3를 제자리에서 한 바퀴 회전시키는 함수
        """
        self.get_logger().info('Rotating TurtleBot...')
        twist = Twist()
        twist.angular.z = 0.5  # 회전 속도
        start_time = time.time()

        while time.time() - start_time < 5:  # 10초 동안 회전
            if self.tracking:
                self.get_logger().info("New goal received from tracker. Stopping rotation.")
                twist.angular.z = 0
                self.cmd_vel_publisher.publish(twist)
                break
            else:
                self.cmd_vel_publisher.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.1)  # 다른 콜백 실행 허용
        twist.angular.z = 0
        self.cmd_vel_publisher.publish(twist)
        
        if not self.tracking:
            self.get_logger().info('Rotation complete. Returning to initial position...')
            self.send_goal(self.initial_position)  # 초기 위치로 복귀

            # 초기 위치로 복귀 완료 후 사용자 입력 대기
            self.nav_to_pose_client.wait_for_server()
            self.get_logger().info('Waiting for goal result...')
            result_future = self.current_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            self.get_logger().info('Returned to initial position. Waiting for user input...')
            self.wait_for_user_input()  # 새 목표 대기 or 시퀀스 종료 명령 대기

    def start_tracking(self):
        """
        Tracker 타이머를 시작하는 함수
        """
        
        self.start_tracking_time = time.time()
        self.get_logger().info('Started tracking with timeout.')

    def handle_timeout_or_complete(self):
        """
        Tracker 타이머가 타임아웃 되었거나 목표가 완료되었을 때 처리하는 함수
        """
        if time.time() - self.start_tracking_time > self.timeout:
            self.get_logger().info('Tracking timeout, returning to initial position.')
            self.tracking = False
            self.send_goal(self.initial_position)  # 초기 위치로 이동
            self.wait_for_user_input()  # 새 목표 대기 or 시퀀스 종료 명령 대기

    def wait_for_user_input(self):
        """
        사용자 입력 대기
        """
        while True:
            choice = input("Do you want to wait for a new observer goal?\nor terminate system? (wait/terminate): ").strip().lower()
            if choice == 'wait':
                self.get_logger().info("Waiting for a new goal from observer...")
                self.send_activate_request()
                return
            elif choice == 'terminate':
                self.get_logger().info("Shutting down all system...")
                self.terminate_system()
                return
            else:
                self.get_logger().info("Wrong input. Try again.")

    def terminate_system(self):
        """
        Observer와 Tracker를 종료하는 함수
        """
        self.get_logger().info('Terminating observer and tracker...')
        observer_request = Trigger.Request()
        tracker_request = Trigger.Request()

        # Observer 종료 요청
        observer_future = self.observer_client.call_async(observer_request)
        rclpy.spin_until_future_complete(self, observer_future)
        if observer_future.result().success:
            self.get_logger().info('Observer terminated successfully.')
        else:
            self.get_logger().error('Failed to terminate observer.')

        # Tracker 종료 요청
        tracker_future = self.tracker_client.call_async(tracker_request)
        rclpy.spin_until_future_complete(self, tracker_future)
        if tracker_future.result().success:
            self.get_logger().info('Tracker terminated successfully.')
        else:
            self.get_logger().error('Failed to terminate tracker.')

        self.get_logger().info('Shutting down...')
        rclpy.shutdown()
        exit(0)  # 프로세스 강제 종료

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        controller_node.get_logger().info('Controller Node stopped manually.')
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
