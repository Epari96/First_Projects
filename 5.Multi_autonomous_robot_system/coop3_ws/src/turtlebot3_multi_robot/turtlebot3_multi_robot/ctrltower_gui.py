import os
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor
from PyQt5.QtCore import QThread, Qt
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
from geometry_msgs.msg import Twist

class CameraSubscriber(Node):
    def __init__(self, topic_name, update_widget_callback):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            update_widget_callback,
            10
        )
        self.bridge = CvBridge()


class OdomSubscriber(Node):
    def __init__(self, main_window, robot, topic):
        super().__init__(f'odom_subscriber_{robot}')
        self.main_window = main_window
        self.robot = robot
        self.subscription = self.create_subscription(Odometry, topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.main_window.update_odom_data(self.robot, (x, y))


class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.is_emergency = False
        self.odom_data_tb1 = "PATROL: N/A"
        self.odom_data_tb2 = "FIRE TRUCK: N/A"
        self.odom_points_tb1 = []
        self.odom_points_tb2 = []
        self.emergency_coords = []  # Emergency 전환 시 로봇1의 좌표를 저장할 리스트
        self.waypoint_process = None
        
        
        
        # /cmd_vel 퍼블리셔 생성
        self.node = Node("cmd_vel_publisher")  # Node를 직접 생성
        self.cmd_vel_publisher = self.node.create_publisher(Twist, "/tb1/cmd_vel", 10)
        self.cmd_vel_publisher_tb2 = self.node.create_publisher(Twist, "/tb2/cmd_vel", 10)


        self.initUI()

        # ROS 2 노드 생성
        self.camera_node_tb1 = CameraSubscriber('/tb1/camera/image_raw', self.update_camera_widget)
        self.camera_node_tb2 = CameraSubscriber('/tb2/camera/image_raw', self.update_camera_widget_2)
        self.odom_node_tb1 = OdomSubscriber(self, "tb1", "/tb1/odom")
        self.odom_node_tb2 = OdomSubscriber(self, "tb2", "/tb2/odom")
        
        
        
        
    def create_cmd_vel_publisher(self):
        # /cmd_vel 퍼블리셔 생성
        rclpy.init()  # ROS 2 노드 초기화
        self.node = Node("cmd_vel_publisher")
        return self.node.create_publisher(Twist, "/tb1/cmd_vel", 10)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if self.cmd_vel_publisher:
            print(f"Publishing Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
            self.cmd_vel_publisher.publish(twist)
            print("Robot should stop now.")
        else:
            print("cmd_vel_publisher is None!")


    
    

    def mm_to_px(self, mm):
        dpi = QApplication.primaryScreen().logicalDotsPerInch()
        return int(mm * dpi / 25.4)

    def initUI(self):
        self.setWindowTitle("Control Tower")
        self.setGeometry(100, 100, self.mm_to_px(250), self.mm_to_px(235))
        self.create_safety_button()
        self.create_camera_widget()
        self.create_camera_title()
        self.create_second_camera_widget()
        self.create_second_camera_title()
        self.create_right_panel()
        self.create_right_panel_title()
        self.create_action_buttons()
        self.create_odom_labels()
        self.show()

    def create_safety_button(self):
        self.safety_button = QPushButton("SAFETY", self)
        self.safety_button.setFixedSize(self.mm_to_px(220), self.mm_to_px(15))
        self.safety_button.move((self.mm_to_px(250) - self.mm_to_px(220)) // 2, self.mm_to_px(10))
        self.safety_button.setStyleSheet(
            """
            QPushButton {
                background-color: green;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                font-size: 14px;
            }
            """
        )

    def create_camera_widget(self):
        self.camera_widget = QLabel(self)
        self.camera_widget.setFixedSize(self.mm_to_px(110), self.mm_to_px(95))
        self.camera_widget.setStyleSheet("background-color: black; border: 2px solid black;")
        self.camera_widget.move(self.mm_to_px(5), self.mm_to_px(32))
        self.camera_widget.setScaledContents(True)

    def create_camera_title(self):
        self.camera_title = QLabel('<span style="color: blue;">●</span> PATROL', self)
        self.camera_title.setStyleSheet(
            "background-color: white; color: black; font-weight: bold; font-size: 16px;"
            "border: 2px solid black; padding: 5px; border-radius: 5px;"
        )
        self.camera_title.move(self.mm_to_px(10), self.mm_to_px(28))

    def create_second_camera_widget(self):
        self.camera_widget_2 = QLabel(self)
        self.camera_widget_2.setFixedSize(self.mm_to_px(110), self.mm_to_px(95))
        self.camera_widget_2.setStyleSheet("background-color: black; border: 2px solid black;")
        self.camera_widget_2.move(self.mm_to_px(5), self.mm_to_px(135))
        self.camera_widget_2.setScaledContents(True)

    def create_second_camera_title(self):
        self.camera_title_2 = QLabel('<span style="color: red;">●</span> FIRE TRUCK', self)
        self.camera_title_2.setStyleSheet(
            "background-color: white; color: black; font-weight: bold; font-size: 16px;"
            "border: 2px solid black; padding: 5px; border-radius: 5px;"
        )
        self.camera_title_2.move(self.mm_to_px(10), self.mm_to_px(131))
        self.camera_title_2.adjustSize()

    def create_right_panel(self):
        self.right_panel = QLabel(self)
        self.right_panel.setFixedSize(self.mm_to_px(125), self.mm_to_px(95))
        self.right_panel.setStyleSheet("background-color: white; border: 2px solid black;")
        self.right_panel.move(self.mm_to_px(120), self.mm_to_px(32))

        image_path = os.path.expanduser("~/Rokey/7.PracticalProject/Week5_Coop3/coop3_ws/src/turtlebot3_multi_robot/map/real_world_image.png")
        self.map_pixmap = QPixmap(image_path)
        if not self.map_pixmap.isNull():
            self.map_pixmap = self.map_pixmap.scaled(self.right_panel.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        else:
            self.map_pixmap = QPixmap(self.right_panel.size())
            self.map_pixmap.fill(Qt.white)

        self.right_panel.setPixmap(self.map_pixmap)

    def create_right_panel_title(self):
        self.right_title = QLabel("LOCATION", self)
        self.right_title.setStyleSheet(
            "background-color: white; color: black; font-weight: bold; font-size: 16px;"
            "border: 2px solid black; padding: 5px; border-radius: 5px;"
        )
        self.right_title.move(self.mm_to_px(125), self.mm_to_px(28))
        self.right_title.adjustSize()

    def create_action_buttons(self):
        self.deploy_button = QPushButton("출동", self)
        self.deploy_button.setFixedSize(self.mm_to_px(52), self.mm_to_px(52))
        self.deploy_button.move(self.mm_to_px(130), self.mm_to_px(140))
        self.deploy_button.clicked.connect(self.handle_deploy_action)

        self.return_button = QPushButton("순찰 시작", self)
        self.return_button.setFixedSize(self.mm_to_px(52), self.mm_to_px(20))
        self.return_button.move(self.mm_to_px(130), self.mm_to_px(198))
        self.return_button.clicked.connect(self.handle_return_action)
        
        self.create_fire_start_button(180, 140, 50, 20)
        self.create_fire_end_button(180, 170, 50, 20)
        
        self.return_home_button = QPushButton("복귀", self)
        self.return_home_button.setFixedSize(self.mm_to_px(40), self.mm_to_px(20))
        self.return_home_button.move(self.mm_to_px(190), self.mm_to_px(198))
        self.return_home_button.clicked.connect(self.handle_return_home_action)
        
        
    def handle_return_home_action(self):
        """
        복귀 버튼 클릭 핸들러. 로봇2를 (-1.5, 0.5) 좌표로 이동시킴.
        """
        target_x, target_y = -1.5, 0.5
        print(f"로봇2 복귀 버튼 클릭! 목표 좌표: ({target_x}, {target_y})")
        self.move_robot_to_target(target_x, target_y)
        

    def create_fire_start_button(self, x_mm, y_mm, width_mm, height_mm):
        self.fire_start_button = QPushButton("소화 시작", self)
        self.fire_start_button.setFixedSize(self.mm_to_px(40), self.mm_to_px(20))
        self.fire_start_button.move(self.mm_to_px(190), self.mm_to_px(145))
        

    def create_fire_end_button(self, x_mm, y_mm, width_mm, height_mm):
        self.fire_end_button = QPushButton("소화 종료", self)
        self.fire_end_button.setFixedSize(self.mm_to_px(40), self.mm_to_px(20))
        self.fire_end_button.move(self.mm_to_px(190), self.mm_to_px(170))
        self.fire_end_button.clicked.connect(self.handle_fire_end_action)

        

    def handle_fire_start_action(self):
        # SAFETY -> EMERGENCY 상태로 변경
        self.safety_button.setText("EMERGENCY")
        
    

    def handle_fire_end_action(self):
        self.safety_button.setText("SAFETY")
        print("소화 종료!") 
        self.safety_button.setStyleSheet(
            """
            QPushButton {
                background-color: green;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                font-size: 14px;
            }
            """
        )

    def create_odom_labels(self):
        self.odom_label_tb1 = QLabel(self)
        self.odom_label_tb1.setStyleSheet("background-color: transparent; color: blue; font-weight: bold; font-size: 14px;")
        self.odom_label_tb1.setText(self.odom_data_tb1)
        self.odom_label_tb1.move(470, 136)
        self.odom_label_tb1.adjustSize()

        self.odom_label_tb2 = QLabel(self)
        self.odom_label_tb2.setStyleSheet("background-color: transparent; color: red; font-weight: bold; font-size: 14px;")
        self.odom_label_tb2.setText(self.odom_data_tb2)
        self.odom_label_tb2.move(470, 151)
        self.odom_label_tb2.adjustSize()

    def update_odom_data(self, robot, position):
        if robot == "tb1":
            self.odom_data_tb1 = f"PATROL: ({position[0]:.2f}, {position[1]:.2f})"
            self.odom_points_tb1.append(position)
            self.odom_label_tb1.setText(self.odom_data_tb1)
            self.odom_label_tb1.adjustSize()
        elif robot == "tb2":
            self.odom_data_tb2 = f"FIRE TRUCK: ({position[0]:.2f}, {position[1]:.2f})"
            self.odom_points_tb2.append(position)
            self.odom_label_tb2.setText(self.odom_data_tb2)
            self.odom_label_tb2.adjustSize()
        self.draw_points()

    def draw_points(self):
        pixmap = self.map_pixmap.copy()
        painter = QPainter(pixmap)

        # TB1 points
        painter.setPen(QColor("blue"))
        for point in self.odom_points_tb1:
            px_x = self.mm_to_px(point[0] + 62.5)
            px_y = self.mm_to_px(62.5 - point[1])
            painter.drawEllipse(px_x, px_y, 5, 5)

        # TB2 points
        painter.setPen(QColor("red"))
        for point in self.odom_points_tb2:
            px_x = self.mm_to_px(point[0] + 62.5)
            px_y = self.mm_to_px(62.5 - point[1])
            painter.drawEllipse(px_x, px_y, 5, 5)

        painter.end()
        self.right_panel.setPixmap(pixmap)

    def handle_deploy_action(self):
        print("출동 버튼 클릭!")
        
        # emergency_coords 리스트 확인
        if len(self.emergency_coords) == 0:
            print("Emergency 좌표 리스트가 비어 있습니다. 이동할 좌표가 없습니다.")
            return

        # 로봇 1의 첫 번째 좌표 가져오기
        target_x, target_y = self.emergency_coords[0]
        print(f"로봇2가 이동할 목표 좌표: ({target_x}, {target_y})")

        # 로봇 2를 해당 좌표로 이동
        self.move_robot_to_target(target_x, target_y)




    def handle_return_action(self):
        # 순찰 시작 버튼 클릭
        print("순찰 시작")
        
        
        self.start_waypoint_process()  # waypoint.py 실행

    def start_waypoint_process(self):
        # waypoint.py 실행
        if self.waypoint_process is None:
            waypoint_script = os.path.expanduser("~/Rokey/7.PracticalProject/Week5_Coop3/coop3_ws/src/turtlebot3_multi_robot/turtlebot3_multi_robot/waypoint.py")  # waypoint.py의 올바른 경로
            self.waypoint_process = subprocess.Popen(["python3", waypoint_script])
            print("waypoint.py 실행 중...")

        
    def is_fire(self, cv_image):
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
        if red_area > 2000:  # 임계값 이상의 빨간색 영역
            print(f"Red area detected! Size: {red_area}")
            # SAFETY 버튼을 빨간색으로 변경하고 문구를 EMERGENCY로 변경
            self.safety_button.setStyleSheet(
                """
                QPushButton {
                    background-color: red;
                    color: white;
                    font-weight: bold;
                    border-radius: 5px;
                    font-size: 14px;
                }
                """
            )
            self.safety_button.setText("EMERGENCY")
            # 로봇 1의 현재 좌표값 저장
            if len(self.odom_points_tb1) > 0:  # 로봇1의 마지막 위치가 존재할 경우
                current_coords = self.odom_points_tb1[-1]  # 마지막 위치의 (x, y) 좌표
                self.emergency_coords.append(current_coords)  # 리스트에 추가
                print(f"Emergency 전환 시 로봇1 좌표 저장: {current_coords}")


            
          

    def update_camera_widget(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.camera_widget.setPixmap(QPixmap.fromImage(q_image))
            self.is_fire(cv_image)
        except Exception as e:
            print(f"Error updating camera_widget: {e}")

    def update_camera_widget_2(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.camera_widget_2.setPixmap(QPixmap.fromImage(q_image))
        except Exception as e:
            print(f"Error updating camera_widget_2: {e}")
            
            
    def move_robot_to_target(self, target_x, target_y):
        """
        로봇2를 목표 위치로 이동시킵니다.
        """
        # 로봇2의 현재 위치 가져오기
        if len(self.odom_points_tb2) == 0:
            print("로봇2의 초기 위치를 알 수 없습니다.")
            return

        current_x, current_y = self.odom_points_tb2[-1]

        # 목표 위치와의 차이 계산
        dx = target_x - current_x
        dy = target_y - current_y
        distance = np.sqrt(dx**2 + dy**2)

        # 로봇 이동을 위한 Twist 메시지 생성
        twist = Twist()
        twist.linear.x = 0.2 if distance > 0.1 else 0.0  # 거리 10cm 이하로 가까워지면 멈춤
        twist.angular.z = np.arctan2(dy, dx) if distance > 0.1 else 0.0

        # 메시지 발행
        self.cmd_vel_publisher_tb2.publish(twist)
        print(f"로봇2 이동: 목표 ({target_x:.2f}, {target_y:.2f}), 현재 ({current_x:.2f}, {current_y:.2f})")



def main():
    rclpy.init()  # ROS 2 초기화
    app = QApplication(sys.argv)
    main_app = MainApp()

    executor = MultiThreadedExecutor()
    executor.add_node(main_app.node)  # cmd_vel_publisher 노드 추가
    executor.add_node(main_app.camera_node_tb1)
    executor.add_node(main_app.camera_node_tb2)
    executor.add_node(main_app.odom_node_tb1)
    executor.add_node(main_app.odom_node_tb2)

    ros_thread = QThread()
    ros_thread.run = executor.spin
    ros_thread.start()

    sys.exit(app.exec_())



if __name__ == "__main__":
    main()