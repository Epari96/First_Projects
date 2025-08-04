import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        # Publisher to send coordinates
        self.publisher = self.create_publisher(String, 'tracker/target_coordinates', 10)

        # Service to listen for termination requests
        self.terminate_service = self.create_service(Trigger, 'tracker/terminate', self.terminate_callback)

        self.get_logger().info('Tracker Node is ready. Waiting for input...')

    def publish_coordinates(self):
        """
        Wait for user input and publish coordinates to the topic.
        """
        while True:
            try:
                x = float(input("Enter x-coordinate for Tracker: "))
                y = float(input("Enter y-coordinate for Tracker: "))
                msg = String()
                msg.data = f"x:{x},y:{y}"
                self.publisher.publish(msg)
                self.get_logger().info(f"Published coordinates: {msg.data}")
            except ValueError:
                self.get_logger().error("Invalid input. Please enter numerical values.")

    def terminate_callback(self, request, response):
        """
        Termination service callback to stop the node.
        """
        self.get_logger().info("Received termination signal from Controller. Shutting down...")
        response.success = True
        response.message = "Tracker terminated successfully."
        rclpy.shutdown()
        return response


def main(args=None):
    rclpy.init(args=args)
    tracker_node = TrackerNode()
    try:
        tracker_node.publish_coordinates()
    except KeyboardInterrupt:
        tracker_node.get_logger().info('Tracker Node stopped manually.')
    finally:
        tracker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
