import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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
            
            # HSV 색상 공간으로 변환
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 빨간색의 HSV 범위 정의
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])
            
            # 두 범위의 마스크를 생성
            mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # 빨간색 영역의 크기 계산
            red_area = cv2.countNonZero(red_mask)
            if red_area > 5000:  # 임계값 이상의 빨간색 영역
                self.get_logger().info(f"Red area detected! Size: {red_area}")
            
            # OpenCV를 통해 이미지 창에 표시
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # GUI 이벤트 처리 시간 확보
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

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
