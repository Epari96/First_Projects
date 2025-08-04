import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.model = YOLO('/home/rokey/주행3/src/my_robot/my_robot/train5_best.pt')
                          
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'rgb_image/local_image',
            self.listener_callback_rgb,
            10
        )
        self.subscription_rgb  # prevent unused variable warning

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        
        # YOLOv8 객체 탐지
        results = self.model(image_np)[0]  # 모델 실행
        
        # 탐지된 객체의 바운딩 박스와 클래스 정보 출력
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0]  # 바운딩 박스 좌표
            confidence = box.conf[0]  # 신뢰도
            class_id = int(box.cls[0])  # 클래스 ID
            class_name = results.names[class_id]  # 클래스 이름
            
            # 중앙 픽셀 좌표 계산
            Xc = (x1 + x2) / 2
            Yc = (y1 + y2) / 2
            
            # 상대 픽셀 거리 계산
            Xp = 640 - Xc
            Yp = 360 - Yc
            
            # 실제 거리 변환 (좌표계 뒤집기 및 변환 수식 적용)
            Xr = Yp * (-105 / 618)  # 실제 x 거리
            Yr = Xp * (-105 / 618)  # 실제 y 거리
            
            # 베이스까지 거리 변환 (좌표계 뒤집기 및 변환 수식 적용)
            Xb = Xr + 170  #  x 거리
            Yb = Yr        #  y 거리
            # 바운딩 박스와 객체 정보 출력
            self.get_logger().info(f'찾은 물체 : {class_name}, 카메라로부터 상대거리: (Xr: {Xr:.2f} mm, Yr: {Yr:.2f} mm) ')
                                   #with confidence {confidence:.2f} at [{x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f}]')
            self.get_logger().info(f'찾은 물체 : {class_name}, 베이스로부터 상대거리: (Xb: {Xb:.2f} mm, Yb: {Yb:.2f} mm) ')
            #self.get_logger().info(f'Central Pixel Coordinates: (Xc: {Xc:.2f}, Yc: {Yc:.2f})')
            #self.get_logger().info(f'Relative Pixel Distances: (Xp: {Xp:.2f}, Yp: {Yp:.2f})')
            #self.get_logger().info(f'Real Distances: (Xr: {Xr:.2f} mm, Yr: {Yr:.2f} mm)')

        # 이미지 표시 (선택사항)
        # cv2.imshow('RGB Image', image_np)
        cv2.waitKey(1)  # 1ms 대기하여 창 업데이트

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
