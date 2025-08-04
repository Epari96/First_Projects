#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import os
import select
import sys
import getkey

import rclpy  # Needed to create a ROS node 
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist  # Message that moves base
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
from sensor_msgs.msg import JointState  # JointState 메시지 추가

'''원래 강사님이 주신 xyz코드에서 변형한것
p를 누르면 현재 조인트 각도 출력
특정포즈(각도지정)를 만들어두고 버튼 누르면 해당 포즈로 이동
포즈1 : z
포즈2 : x
포즈3 : c
'''

# 로봇의 관절 간 거리와 오프셋을 정의
j1_z_offset = 77
r1 = 130
r2 = 124
r3 = 150

th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5 * math.pi - th1_offset

# 특정 포즈 정의 (각도 설정)
poses = {
    #라디안으로 입력
    'initial_pose': [0.0, 0.0, 0.0, 0.0],  # 초기 포즈
    'poseM': [0.0, -1.2, 1.3, 0.0],  # 마커보는자세
    'pose0': [0.0, 0.0 , 0.0 , 0.0], #정자세
    'poseB': [0.0,-0.2 , -0.3, 2.10], # 상자보는 각도
    'pose1': [0.28,0.4 , 0.1, 0.78], #1번칸
    'pose2': [0.0,0.4 , 0.1, 0.78],  #2번칸의 블럭잡을때
    'pose3': [-0.28,0.4 , 0.1, 0.78], # 3번칸
    'pose': [], # 디택트

    'poseC': [-math.pi / 2, 0.2 , 0.1, 1.0],  # 예: 컨베이어에 내려두기
}

# 두 관절 사이의 각도를 계산하는 함수
def solv2(r1, r2, r3):
    d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)

    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)

    return s1, s2

# 로봇 팔의 목표 위치에 대한 관절 각도를 계산하는 함수
def solv_robot_arm2(x, y, z, r1, r2, r3):
    z = z + r3 - j1_z_offset

    Rt = math.sqrt(x**2 + y**2 + z**2)
    Rxy = math.sqrt(x**2 + y**2)
    St = math.asin(z / Rt)
    Sxy = math.atan2(y, x)

    s1, s2 = solv2(r1, r2, Rt)

    sr1 = math.pi / 2 - (s1 + St)
    sr2 = s1 + s2
    sr2_ = sr1 + sr2
    sr3 = math.pi - sr2_

    return Sxy, sr1, sr2, sr3, St, Rt

# 사용법 안내
usage = """
Control Your OpenManipulator!
---------------------------
Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)

w: forward
s: backward
d: turn right
a: turn left

<space>: stop

= 그리퍼 닫기
- 그리파 열기

p: Print current joint angles

INIT : (1)

CTRL-C to quit
"""

joint_angle_delta = 0.1  # radian

# ROS 2 노드 클래스 정의
class Turtlebot3ManipulationTest(Node): 
    qos = QoSProfile(depth=10)
        
    def __init__(self): 
        super().__init__('turtlebot3_manipulation_test')
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

        self.move_cmd = Twist() 
        self.move_cmd.linear.x = 1.3   # Modify this value to change speed 
        self.move_cmd.angular.z = 0.8  # Modify this value to cause rotation rad/s 

        self.trajectory_msg = JointTrajectory()

        Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(215, -53, 0  , r1, r2, r3)
        
        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        #point.positions = [0.0] * 4 
        point.velocities = [0.0] * 4
        point.accelerations = [0.0] * 4
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 500
        
        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)
        # 포즈에 추가
        self.add_pose('initial_pose', point.positions)

    def add_pose(self, pose_name, angles):
        poses[pose_name] = angles

    # 그리퍼 명령 전송
    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0
    
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return
          
        self.gripper_action_client.send_goal_async(goal)

    # 포즈로 이동
    def move_to_pose(self, pose, velocity=0.1 , acceleration=0.005):
        if pose in poses:
            self.trajectory_msg.points[0].positions = poses[pose]
            self.trajectory_msg.points[0].velocities = [velocity] * 4
            self.trajectory_msg.points[0].accelerations = [acceleration] * 4 
            self.joint_pub.publish(self.trajectory_msg)
            print(f'Moving to {pose} with angles: {poses[pose]}')
            # 동작 완료를 기다리기 위해 잠시 대기
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3.0))  # 1초 대기
        else:
            print(f'Pose {pose} not defined.')

    #현재각도 출력
    def print_joint_angles(self):
        current_positions = self.trajectory_msg.points[0].positions
        print(f'Current Joint Angles:')
        print(f'Joint 1: {current_positions[0]:.2f} rad')
        print(f'Joint 2: {current_positions[1]:.2f} rad')
        print(f'Joint 3: {current_positions[2]:.2f} rad')
        print(f'Joint 4: {current_positions[3]:.2f} rad')


node = None

def main(args=None):
    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        node = Turtlebot3ManipulationTest()
    except Exception as e:
        print(e)

    try:
        while rclpy.ok():
            key_value = getkey.getkey()
            
            if key_value == '0':
                node.trajectory_msg.points[0].positions = [0.0] * 4
                node.joint_pub.publish(node.trajectory_msg)
                print('joint1 +')
            elif key_value == 'y':
                node.trajectory_msg.points[0].positions[0] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint1 +')
            elif key_value == 'h':
                node.trajectory_msg.points[0].positions[0] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint1 -')
            elif key_value == 'u':
                node.trajectory_msg.points[0].positions[1] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint2 +')
            elif key_value == 'j':
                node.trajectory_msg.points[0].positions[1] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint2 -')
            elif key_value == 'i':
                node.trajectory_msg.points[0].positions[2] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint3 +')
            elif key_value == 'k':
                node.trajectory_msg.points[0].positions[2] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint3 -')
            elif key_value == 'o':
                node.trajectory_msg.points[0].positions[3] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint4 +')
            elif key_value == 'l':
                node.trajectory_msg.points[0].positions[3] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint4 -')

            elif key_value == 'w':
                node.move_cmd.linear.x = 0.1
                node.move_cmd.angular.z = 0.
                node.cmd_vel.publish(node.move_cmd) 
                print('forward')
            elif key_value == 's':
                node.move_cmd.linear.x = -0.1
                node.move_cmd.angular.z = 0.
                node.cmd_vel.publish(node.move_cmd) 
                print('backward')
            elif key_value == 'd':
                node.move_cmd.linear.x = 0.
                node.move_cmd.angular.z = -0.1
                node.cmd_vel.publish(node.move_cmd) 
                print('right')
            elif key_value == 'a':
                node.move_cmd.linear.x = 0.
                node.move_cmd.angular.z = 0.1
                node.cmd_vel.publish(node.move_cmd) 
                print('left')
            elif key_value == ' ':
                node.move_cmd.linear.x = 0.
                node.move_cmd.angular.z = 0.
                node.cmd_vel.publish(node.move_cmd) 
                print('stop')

            elif key_value == '=':
                node.send_gripper_goal(-0.015)
                print('close')
            elif key_value == '-':
                node.send_gripper_goal(0.01)
                print('open')

            elif key_value == 'z':  # 정자세로 이동
                node.move_to_pose('pose0', velocity=0.01, acceleration=0.005) 
            elif key_value == 'm':  # 마커보는자세로 이동
                node.move_to_pose('poseM', velocity=0.01, acceleration=0.005) 
            elif key_value == '1':  # 포즈 1로 이동
                node.move_to_pose('pose1', velocity=0.1, acceleration=0.02)
            elif key_value == '2':  # 포즈 2로 이동
                node.move_to_pose('pose2', velocity=0.1, acceleration=0.02)
            elif key_value == '3':  # 포즈 3로 이동
                node.move_to_pose('pose3', velocity=0.1, acceleration=0.02) 
            elif key_value == 'b':  # 박스보는 각도
                node.move_to_pose('poseB', velocity=0.1, acceleration=0.02)
            elif key_value == 'x':  # 
                node.move_to_pose('initial_pose', velocity=0.1, acceleration=0.02)  
            elif key_value == 'c': # 컨베이어로 이동
                node.move_to_pose('pose0', velocity=0.01, acceleration=0.005)
                node.move_to_pose('poseC', velocity=0.01, acceleration=0.005)
                node.send_gripper_goal(0.01)
                node.move_to_pose('pose0', velocity=0.01, acceleration=0.005)

            elif key_value == 'p':  # 현재 각도 출력
                node.print_joint_angles()

            elif key_value == 'q':
                break
            
    except Exception as e:
        print(e)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__": 
    main() 

