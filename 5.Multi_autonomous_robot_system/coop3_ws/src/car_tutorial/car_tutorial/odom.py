import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

class Odom(Node):
    def __init__(self):
        super().__init__("odom")
        # Parameters
        self.timer_frequently = 0.1
        # init variable
        self.linear = 0.0
        self.angular = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)
        # subscriber
        self.sub_cmd_vel = self.create_subscription(Twist, "raw_vel", self.raw_vel_callback, 10)
        # publisher
        self.pub_odometry = self.create_publisher(Odometry, "odom", 10)
        # timer
        self.timer = self.create_timer(self.timer_frequently, self.publish_odometry)

    def raw_vel_callback(self, msg):
        self.get_logger().info(f"recv cmd_vel message {msg}")
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def publish_odometry(self):
        self.delta_x = self.linear * np.cos(self.pos_z) * self.timer_frequently
        self.delta_y = self.linear * np.sin(self.pos_z) * self.timer_frequently
        self.delta_z = self.angular * self.timer_frequently
        self.pos_x += self.delta_x
        self.pos_y += self.delta_y
        self.pos_z += self.delta_z
        q = quaternion_from_euler(0.0, 0.0, self.pos_z)
        
        # Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.vel_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vel_z
        
        # TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        self.pub_odometry.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    odom = Odom()
    rclpy.spin(odom)
    odom.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
