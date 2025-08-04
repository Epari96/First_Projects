#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient

class FollowWaypointsNode(Node):
    def __init__(self):
        super().__init__('follow_waypoints_node')  # 노드 이름을 설정

        # 액션 클라이언트 설정: /follow_waypoints 서버와 통신
        self.action_client = ActionClient(self, FollowWaypoints, '/tb1/follow_waypoints')

        # 로봇이 따라갈 웨이포인트 정의
        self.waypoints = [
            self.create_pose(-4.0, 5.5, 0.0, 0.0, 0.0, 0.0, 1.0),  # 첫 번째 좌표, x axis direction
            self.create_pose(6.0, 5.5, 0.0, 0.0, 0.0, -0.707, 0.707),  # 두 번째 좌표, -y axis direction
            self.create_pose(7.0, -6.0, 0.0, 0.0, 0.0, 1.0, 0.0),  # 세 번째 좌표, -x axis direction
            self.create_pose(-4.0, -6.5, 0.0, 0.0, 0.0, 0.707, 0.707),  # 네 번째 좌표, y axis direction
        ]

        self.send_waypoints()  # 웨이포인트 전송을 시작.
        
        # PoseStamped 메시지를 생성하는 헬퍼 함수.
        # param x, y, z: 위치 (position)
        # param qx, qy, qz, qw: 방향 (orientation) - 쿼터니언
        # return: PoseStamped 메시지
    def create_pose(self, x, y, z, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # 모든 좌표는 'map' 프레임을 기준으로 설정
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # /follow_waypoints 액션 서버에 웨이포인트를 전송.
    def send_waypoints(self):
        goal_msg = FollowWaypoints.Goal()  # FollowWaypoints 액션의 목표 메시지 생성
        goal_msg.poses = self.waypoints  # 웨이포인트 목록을 Goal 메시지에 추가

        self.action_client.wait_for_server()  # 액션 서버가 준비될 때까지 대기

        self.get_logger().info('Sending waypoints...')  # 로깅 메시지 출력
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)  # 비동기적으로 목표 전송
        self._send_goal_future.add_done_callback(self.goal_response_callback)  # 응답 콜백 등록


        # 액션 서버에서 목표 수락 여부에 대한 응답을 처리.
        # :param future: 서버로부터의 응답 결과
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')  # 목표가 거부된 경우 메시지 출력
            return

        self.get_logger().info('Goal accepted')  # 목표가 수락된 경우 메시지 출력
        self._get_result_future = goal_handle.get_result_async()  # 결과를 비동기적으로 기다림
        self._get_result_future.add_done_callback(self.result_callback)  # 결과 콜백 등록
        
        # 웨이포인트를 모두 따라간 후 결과를 처리.
        # param future: 서버에서 반환된 결과
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Completed waypoint following. Restarting...')  # 완료 메시지 출력
        self.send_waypoints()  # 웨이포인트 순환을 다시 시작

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = FollowWaypointsNode()  # FollowWaypointsNode 인스턴스 생성
    rclpy.spin(node)  # 노드 실행 및 이벤트 대기
    node.destroy_node()  # 노드 종료 시 자원 해제
    rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
