import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import time


class DummyNav2Node(Node):
    def __init__(self):
        super().__init__('dummy_nav2_node')

        self.process_time = 5
        # Action server to simulate navigation behavior
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Dummy Nav2 Node is ready.')

    def goal_callback(self, goal_request):
        self.get_logger().info("Goal accepted.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Handles cancellation requests for active goals.
        """
        self.get_logger().info("Cancel request received for goal.")
        # Check if there is an active goal to cancel
        if not goal_handle.is_active:
            self.get_logger().info("No active goal to cancel.")
            return CancelResponse.REJECT

        goal_handle.canceled()  # Mark the goal as canceled
        self.get_logger().info("Goal successfully canceled.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal: {goal_handle.request.pose.pose.position.x}, "
                            f"{goal_handle.request.pose.pose.position.y}")
        
        # Simulate navigation with a countdown and feedback
        for i in range(self.process_time, 0, -1):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal cancellation requested during execution.")
                goal_handle.canceled()
                self.get_logger().info("Goal has been canceled successfully.")
                return NavigateToPose.Result()
            
            self.get_logger().info(f"Navigating... {i} seconds remaining.")
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose = goal_handle.request.pose
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        self.get_logger().info("Goal execution succeeded.")
        goal_handle.succeed()  # Mark the goal as succeeded
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    dummy_nav2_node = DummyNav2Node()
    try:
        rclpy.spin(dummy_nav2_node)
    except KeyboardInterrupt:
        dummy_nav2_node.get_logger().info('Dummy Nav2 Node stopped manually.')
    finally:
        dummy_nav2_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
