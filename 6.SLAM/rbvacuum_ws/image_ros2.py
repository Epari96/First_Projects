import rclpy  # ROS2 Python 라이브러리
from rclpy.node import Node
import cv2  # OpenCV 라이브러리 (이미지 처리용)
import numpy as np  # Numpy 라이브러리 (수학 연산용)

# 실시간 매칭을 위한 ROS2 노드 클래스 정의
class RealTimeMatcher(Node):
    def __init__(self):
        # ROS2 노드 초기화
        super().__init__('real_time_matcher')

        # 카메라를 사용하기 위한 비디오 캡처 객체 초기화
        self.video_capture = cv2.VideoCapture(0)
        if not self.video_capture.isOpened():
            self.get_logger().error("Unable to access the camera")  # 카메라 초기화 실패 메시지
            raise RuntimeError("Camera initialization failed")

        # ORB 특징점 검출기 생성
        self.orb = cv2.ORB_create(
            nfeatures=1500,  # 특징점 개수 제한
            scaleFactor=1.1,  # 이미지 피라미드 크기 조정 비율
            edgeThreshold=10  # 에지 영역에서의 검출 강도
        )

        # 매칭할 이미지 경로와 로드
        self.image1_path = "/home/rokey/man_orig.png"  # 첫 번째 기준 이미지
        self.image2_path = "/home/rokey/ext_orig.png"  # 두 번째 기준 이미지
        self.image1 = cv2.imread(self.image1_path)  # 이미지 로드
        self.image2 = cv2.imread(self.image2_path)

        if self.image1 is None or self.image2 is None:
            self.get_logger().error("Error: Unable to load images")  # 이미지 로드 실패 시 메시지 출력
            raise RuntimeError("Image loading failed")

        # ORB를 사용해 이미지에서 특징점과 디스크립터를 추출
        self.keypoints1, self.descriptors1 = self.orb.detectAndCompute(self.image1, None)
        self.keypoints2, self.descriptors2 = self.orb.detectAndCompute(self.image2, None)

        # 매칭 기준 및 설정값
        self.match_threshold = 15  # 특징점 매칭 최소 개수
        self.real_dimensions = {"man": 23, "ext": 18}  # 기준 이미지의 실제 폭 (cm)
        self.focal_length = 600  # 카메라 초점 거리 (픽셀 단위, 실험을 통해 조정 필요)
        self.frame_count = 0  # 프레임 수 초기화
        self.frame_interval = 5  # 프레임 처리 간격

        # ROS2 타이머 설정: 0.1초 간격으로 프레임 처리 실행
        self.timer = self.create_timer(0.1, self.process_frame)

    def resize_to_same_width(self, img1, img2):
        """두 이미지의 너비를 동일하게 맞춥니다."""
        target_width = min(img1.shape[1], img2.shape[1])  # 두 이미지 중 더 작은 너비로 맞춤
        img1_resized = cv2.resize(img1, (target_width, img1.shape[0]))  # 첫 번째 이미지 리사이즈
        img2_resized = cv2.resize(img2, (target_width, img2.shape[0]))  # 두 번째 이미지 리사이즈
        return img1_resized, img2_resized

    def calculate_distance(self, keypoints, matches, real_width):
        """특징점 매칭을 기반으로 물체의 거리를 계산합니다."""
        if len(matches) < 2:
            return None  # 매칭된 점이 부족하면 거리 계산 불가

        # 매칭된 점들의 x좌표를 기반으로 이미지 상 너비 계산
        x_coords = [keypoints[m.trainIdx].pt[0] for m in matches]
        width_in_pixels = max(x_coords) - min(x_coords)

        if width_in_pixels < 10:  # 너비가 너무 작으면 거리 계산 불가
            return None

        # 거리 계산: 실제 물체의 너비와 이미지 상 너비, 초점 거리 사용
        distance = (real_width * self.focal_length) / width_in_pixels
        return round(distance, 2)  # 소수점 2자리까지 반환

    def calculate_angle(self, keypoints, matches):
        """매칭된 점들의 각도를 계산합니다."""
        if len(matches) < 2:
            return None  # 매칭된 점이 부족하면 각도 계산 불가

        # 두 매칭된 점의 좌표를 가져와서 각도 계산
        p1 = keypoints[matches[0].trainIdx].pt
        p2 = keypoints[matches[1].trainIdx].pt
        delta_x = p2[0] - p1[0]
        delta_y = p2[1] - p1[1]
        angle = np.degrees(np.arctan2(delta_y, delta_x))  # 라디안을 각도로 변환
        return round(angle, 2)  # 소수점 2자리까지 반환

    def process_frame(self):
        """실시간 프레임을 처리하여 매칭 결과를 계산하고 시각화합니다."""
        ret, frame = self.video_capture.read()
        if not ret:
            self.get_logger().error("Unable to read video frame")  # 카메라 프레임 읽기 실패
            return

        if self.frame_count % self.frame_interval == 0:
            # ORB를 사용하여 실시간 프레임에서 특징점과 디스크립터 추출
            keypoints_frame, descriptors_frame = self.orb.detectAndCompute(frame, None)
            if descriptors_frame is None:
                self.frame_count += 1
                return

            # BFMatcher를 사용하여 기준 이미지와 실시간 프레임의 특징점 매칭
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
            display_text = ""

            # 첫 번째 기준 이미지와 매칭
            good_matches1 = [m for m, n in bf.knnMatch(self.descriptors1, descriptors_frame, k=2) if m.distance < 0.75 * n.distance]
            if len(good_matches1) >= self.match_threshold:
                distance1 = self.calculate_distance(keypoints_frame, good_matches1, self.real_dimensions["man"])
                angle1 = self.calculate_angle(keypoints_frame, good_matches1)
                if distance1 and angle1 is not None:
                    display_text += f"man: {distance1:.2f}cm, Angle: {angle1:.2f}°"

            # 두 번째 기준 이미지와 매칭
            good_matches2 = [m for m, n in bf.knnMatch(self.descriptors2, descriptors_frame, k=2) if m.distance < 0.75 * n.distance]
            if len(good_matches2) >= self.match_threshold:
                distance2 = self.calculate_distance(keypoints_frame, good_matches2, self.real_dimensions["ext"])
                angle2 = self.calculate_angle(keypoints_frame, good_matches2)
                if distance2 and angle2 is not None:
                    display_text += f" | ext: {distance2:.2f}cm, Angle: {angle2:.2f}°"

            # 매칭 결과 이미지 생성
            result1 = cv2.drawMatches(self.image1, self.keypoints1, frame, keypoints_frame, good_matches1[:10], None)
            result2 = cv2.drawMatches(self.image2, self.keypoints2, frame, keypoints_frame, good_matches2[:10], None)

            # 두 결과 이미지의 너비를 동일하게 맞춘 후 결합
            result1_resized, result2_resized = self.resize_to_same_width(result1, result2)
            combined_result = np.vstack((result1_resized, result2_resized))

            # 결과 이미지에 계산된 정보 추가
            cv2.putText(combined_result, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            resized_combined_result = cv2.resize(combined_result, (960, 540))  # 결과 이미지 크기 조정
            cv2.imshow("Real-time Matches", resized_combined_result)  # 결과 표시

        self.frame_count += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키 입력 시 종료
            self.destroy_node()
            cv2.destroyAllWindows()


def main(args=None):
    """ROS2 노드를 실행하고 종료 시 자원 해제."""
    rclpy.init(args=args)
    node = RealTimeMatcher()
    try:
        rclpy.spin(node)  # ROS2 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated by user')
    finally:
        node.video_capture.release()  # 카메라 자원 해제
        cv2.destroyAllWindows()  # OpenCV 창 닫기
        rclpy.shutdown()  # ROS2 종료


if __name__ == "__main__":
    main()