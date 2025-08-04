       
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FireDetect(Node):
    def __init__(self):
        super().__init__('fire_detect')
        # /camera/image_raw 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # 방지: 리소스 정리가 노드 종료 시 이루어지지 않는 경우
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Receiving image frame')
        try:
            # ROS 2 Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # OpenCV를 통해 이미지 창에 표시
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # GUI 이벤트 처리 시간 확보
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    fire_detect = FireDetect()
    try:
        rclpy.spin(fire_detect)
    except KeyboardInterrupt:
        print("Ctrl+C pressed. Exiting...")
        pass
    finally:
        fire_detect.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
    main()
