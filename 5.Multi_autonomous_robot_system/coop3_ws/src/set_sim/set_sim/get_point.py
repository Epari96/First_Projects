import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class ClickedPointListener(Node):
    def __init__(self):
        super().__init__('clicked_point_listener')
        self.subscription = self.create_subscription(
            PointStamped,
            '/tb1/clicked_point',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Clicked Point: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}')

def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
