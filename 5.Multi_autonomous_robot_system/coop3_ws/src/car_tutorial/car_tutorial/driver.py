import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import tkinter as tk

class Driver(Node):
    def __init__(self):
        super().__init__("drive")
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = [
            "robot_arm_joint1", "robot_arm_joint2", "robot_arm_joint4", "robot_arm_joint6", "left_wheel_joint", "right_wheel_joint"
        ]
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)
        self.timer_1 = self.create_timer(0.1, self.publish_jointstate)

    def update_joint_states(self, positions):
        self.joint_states.position = positions
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_states.publish(self.joint_states)

    def publish_jointstate(self):
        self.pub_joint_states.publish(self.joint_states)

    def interpolate_joint_movement(self, target_positions, steps=50, delay=0.05):
        current_positions = self.joint_states.position
        for step in range(steps + 1):
            interpolated_positions = [
                current + (target - current) * (step / steps)
                for current, target in zip(current_positions, target_positions)
            ]
            self.update_joint_states(interpolated_positions)
            time.sleep(delay)

def fold_arm(driver):
    driver.interpolate_joint_movement([0.0, -1.5, 3.0, -3.0, 0.0, 0.0])

def unfold_arm(driver):
    driver.interpolate_joint_movement([0.0, -0.9, 2.0, 0.45, 0.0 ,0.0])

def forward(driver):
    driver.interpolate_joint_movement([0.0, -1.5, 3.0, -3.0, -4.0, -4.0])

def backward(driver):
    driver.interpolate_joint_movement([0.0, -1.5, 3.0, -3.0, 4.0, 4.0])


def create_gui(driver):
    root = tk.Tk()
    root.title("Robot Arm Control")

    fold_button = tk.Button(root, text="접기", command=lambda: fold_arm(driver))
    fold_button.pack(pady=10)

    unfold_button = tk.Button(root, text="펼치기", command=lambda: unfold_arm(driver))
    unfold_button.pack(pady=10)

    forward_button = tk.Button(root, text="전진", command=lambda: forward(driver))
    forward_button.pack(pady=10)

    backward_button = tk.Button(root, text="후진", command=lambda: backward(driver))
    backward_button.pack(pady=10)

    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    
    create_gui(driver)
    
    executor = MultiThreadedExecutor()
    rclpy.spin(driver, executor=executor)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()