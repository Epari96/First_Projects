#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ORBFeatureDetector(Node):
    def __init__(self):
        super().__init__('orb_feature_detector')
        img_dir_path = "/home/jaeheyoung/Rokey/7.PracticalProject/Week6_Drive2/rbvacuum_ws/src/detector/resource/"
        
        # 파라미터: 두 개의 레퍼런스 이미지 경로
        self.declare_parameter('reference_image_names', ['/home/jaeheyoung/Rokey/7.PracticalProject/Week6_Drive2/rbvacuum_ws/src/detector/resource/ext_orig.png', '/home/jaeheyoung/Rokey/7.PracticalProject/Week6_Drive2/rbvacuum_ws/src/detector/resource/man_orig.png'])
        reference_image_paths = self.get_parameter('reference_image_names').get_parameter_value().string_array_value
        if len(reference_image_paths) < 2:
            self.get_logger().error("Need at least two reference images.")
            return

        # ORB 초기화
        self.orb = cv2.ORB_create(500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # 레퍼런스 이미지들 로드 및 특징점 추출
        self.references = []
        for i, ref_path in enumerate(reference_image_paths):
            ref_img = cv2.imread(ref_path, cv2.IMREAD_GRAYSCALE)
            if ref_img is None:
                self.get_logger().error(f"Failed to load reference image: {ref_path}")
                continue
            ref_kp, ref_des = self.orb.detectAndCompute(ref_img, None)
            if ref_des is None or len(ref_des) < 10:
                self.get_logger().error(f"No enough features in reference image: {ref_path}")
                continue
            self.references.append({
                'path': ref_path,
                'image': ref_img,
                'kp': ref_kp,
                'des': ref_des
            })

        if len(self.references) < 2:
            self.get_logger().error("Not enough valid reference images loaded.")
            return

        # 구독자 & 퍼블리셔
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.marker_pub = self.create_publisher(Marker, 'detected_image_marker', 10)

        self.current_odom = None

    def odom_callback(self, msg:Odometry):
        self.current_odom = msg

    def image_callback(self, msg:Image):
        # 카메라 이미지 수신 -> ORB 검출 및 매칭
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kp, des = self.orb.detectAndCompute(gray, None)
        if des is None or len(des) < 10:
            # 특징점이 너무 적으면 처리 불가
            return

        # 각 레퍼런스 이미지에 대해 매칭 시도
        for i, ref_data in enumerate(self.references):
            ref_des = ref_data['des']
            ref_kp = ref_data['kp']
            ref_img = ref_data['image']

            # KNN 매칭
            knn_matches = self.matcher.knnMatch(ref_des, des, k=2)
            
            # Lowe's ratio test
            good = []
            ratio_thresh = 0.75
            for m,n in knn_matches:
                if m.distance < ratio_thresh * n.distance:
                    good.append(m)

            # 충분한 매칭점이 있을 경우 Homography 계산
            if len(good) > 10:
                src_pts = np.float32([ ref_kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                if M is not None:
                    h, w = ref_img.shape
                    # 기준 이미지 모서리 좌표
                    corners = np.float32([[0,0],[w,0],[w,h],[0,h]]).reshape(-1,1,2)
                    projected_corners = cv2.perspectiveTransform(corners, M)

                    # 픽셀 좌표 로깅
                    self.get_logger().info(f"[Ref {i}] Detected image corners (pixels): {projected_corners.reshape(-1,2)}")

                    # 현재 odom 위치와 함께 출력
                    if self.current_odom:
                        x = self.current_odom.pose.pose.position.x
                        y = self.current_odom.pose.pose.position.y
                        self.get_logger().info(f"[Ref {i}] At odom position: x={x}, y={y}")

                    # Marker 퍼블리시 (단순히 odom 프레임에 사각형 모양 마커 예시)
                    marker = Marker()
                    marker.header.frame_id = 'odom'  # odom좌표계에 표시
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.scale.x = 0.05
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0

                    if self.current_odom:
                        ox = self.current_odom.pose.pose.position.x
                        oy = self.current_odom.pose.pose.position.y
                    else:
                        ox, oy = 0.0, 0.0

                    # 사각형을 odom 근방에 표시 (픽셀좌표를 단순히 스케일다운)
                    scale_factor = 0.001
                    points = []
                    for c in projected_corners.reshape(-1,2):
                        p = Point()
                        p.x = ox + c[0]*scale_factor
                        p.y = oy + c[1]*scale_factor
                        p.z = 0.0
                        points.append(p)

                    points.append(points[0])
                    marker.points = points

                    # 마커 식별을 위해 namespace나 id 부여
                    marker.ns = f"ref_image_{i}"
                    marker.id = i

                    self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ORBFeatureDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
