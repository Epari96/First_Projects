import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QListWidget, QLabel, QGroupBox, QFormLayout, QScrollArea, QFrame
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import threading

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from order_interface.srv import Order

import sqlite3  # 추가된 부분
import subprocess  # 추가된 부분
import os  # 추가된 부분

class POS(Node):
    def __init__(self):
        super().__init__('pos')
        
        self.callback_group = ReentrantCallbackGroup()

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        # subscriber
        self.Called = self.create_subscription(String, 'call_waiter', self.table_calling, QOS_RKL10V)
        
        self.navigate_to_pose_action_client = ActionClient(self,
            NavigateToPose,
            'navigate_to_pose')
        
        self.arithmetic_service_server = self.create_service(
            Order,
            'order_service',
            self.handle_order,
            callback_group=self.callback_group)
        
        self.cost = {'짜장면': 8000, '짬뽕': 8500, '탕수육': 20000, '제로 콜라': 3000}
    
        # Define positions for Call, Return, and Tables
        self.predefined_positions = {
            "call": [3.0, -1.0],
            "return": [1.0, -1.0],
            "table_1": [3.0, 2.0],
            "table_2": [2.0, 2.0],
            "table_3": [1.0, 2.0],
            "table_4": [3.2, 1.0],
            "table_5": [2.2, 1.0],
            "table_6": [1.2, 1.0]
        }
        self.position = [0.0, 0.0]
        
        # GUI window, widget
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("POS")
        self.window.setGeometry(100, 100, 1000, 600)
        self.main_widget = QWidget(self.window)
        self.window.setCentralWidget(self.main_widget)
        
        # split half for map/others
        main_layout = QHBoxLayout()
        self.main_widget.setLayout(main_layout)
        
        # temp map widget. have to change to map video later
        map_label = QLabel('map')
        map = QPixmap('/home/jaeheyoung/robot_service_ws/src/robot_serving/resource/map_image.jpg')  # 이미지 파일 경로
        map_label.setPixmap(map.scaled(400, 500, Qt.KeepAspectRatio))  # 이미지 크기 조정
        map_label.setAlignment(Qt.AlignCenter)
        
        # add temp map widget to left side of main layout
        main_layout.addWidget(map_label)
        
        consol_layout = QVBoxLayout()
        main_layout.addLayout(consol_layout)
    
        robot_control_layout = QHBoxLayout()
        consol_layout.addLayout(robot_control_layout)
        
        # create robot select button
        self.current_robot_button = None
        self.current_robot = 0
        robot_button_layout = QVBoxLayout()
        robot_control_layout.addLayout(robot_button_layout)
        self.robot1_button = QPushButton("로봇1")
        self.robot1_button.setCheckable(True)
        self.robot1_button.setFixedWidth(80) 
        self.robot1_button.setFixedHeight(100)
        self.robot1_button.clicked.connect(lambda: self.toggle_robot_button(self.robot1_button, 1))
        self.robot2_button = QPushButton("로봇2")
        self.robot2_button.setCheckable(True)
        self.robot2_button.setFixedWidth(80)
        self.robot2_button.setFixedHeight(100)
        self.robot2_button.clicked.connect(lambda: self.toggle_robot_button(self.robot2_button, 2))
        robot_button_layout.addWidget(self.robot1_button)
        robot_button_layout.addWidget(self.robot2_button)
        
        # robot order layoutQGridLayout
        robot_order_layout = QVBoxLayout()
        robot_control_layout.addLayout(robot_order_layout)
        
        # call return button layout
        CB_button_layout = QHBoxLayout()
        robot_order_layout.addLayout(CB_button_layout)
        self.call_button = QPushButton("Call Robot")
        self.call_button.setFixedHeight(80)
        # self.call_button.clicked.connect(lambda: self.robot_call())
        self.call_button.clicked.connect(lambda:
            (self.navigate_to("call"),
            self.robot_call))
        self.return_button = QPushButton("Return Robot")
        self.return_button.setFixedHeight(80)
        # self.return_button.clicked.connect(lambda: self.robot_return())
        self.return_button.clicked.connect(lambda:
            (self.navigate_to("return"),
            self.robot_return))
        CB_button_layout.addWidget(self.call_button)
        CB_button_layout.addWidget(self.return_button)
        
        # robot serve table select layout
        RS_table_layout = QGridLayout()
        robot_order_layout.addLayout(RS_table_layout)
        self.RS_table_buttons = []
        # robot serving table buttons
        for i in range(6):
            RST_button = QPushButton(f"Table {i+1}")
            RST_button.setFixedHeight(55)
            # RST_button.clicked.connect(lambda _, idx=i+1: self.serve_to_table(idx))
            RST_button.clicked.connect(lambda _, idx=i: self.navigate_to(f"table_{idx+1}"))
            self.RS_table_buttons.append(RST_button)
                # 버튼을 2행 3열로 배치
        for i, RST_button in enumerate(self.RS_table_buttons):
            row = i // 3  # 0번째~2번째 버튼은 첫 행(0), 3번째~5번째 버튼은 두 번째 행(1)
            col = i % 3   # 열은 0, 1, 2 순서로 반복
            RS_table_layout.addWidget(RST_button, row, col)
            
        # table status layout
        table_status_layout = QVBoxLayout()
        consol_layout.addLayout(table_status_layout)
        
        # order list table layout
        OL_table_layout = QHBoxLayout()
        table_status_layout.addLayout(OL_table_layout)
        
        # table order list select buttons
        self.current_table_button = None
        self.TOL_buttons = []
        for i in range(6):
            TOL_button = QPushButton(f"Table {i+1}")
            TOL_button.setCheckable(True)  # 버튼을 토글 가능하도록 설정
            TOL_button.clicked.connect(lambda _, button=TOL_button, idx=i+1, : self.toggle_table_button(button, idx))
            TOL_button.setFixedHeight(50)
            self.TOL_buttons.append(TOL_button)
        for i, button in enumerate(self.TOL_buttons):
            OL_table_layout.addWidget(button)
            
        self.original_button_color = self.TOL_buttons[0].palette().button().color().name()
            
        # table purchase layout
        table_purchase_layout = QHBoxLayout()
        table_status_layout.addLayout(table_purchase_layout)
        
        #sample dict    
        self.empty_order = {}
        self.order_dict1 = {
            "짜장면": 1,
            "짬뽕": 2,
            "탕수육": 1,
        }
        self.order_dict2 = {
            "짜장면": 3,
            "짬뽕": 2,
            "탕수육": 2,
            "콜라": 4
        }
        
        # order list layout
        self.order_list_layout = QVBoxLayout()
        for key, value in self.order_dict1.items():
            label = QLabel(f"{key} - {value}")
            self.order_list_layout.addWidget(label)
        table_purchase_layout.addLayout(self.order_list_layout)
        
        # purchase/summary button layout
        purchase_button_layout = QVBoxLayout()
        table_purchase_layout.addLayout(purchase_button_layout)
        self.summary_button = QPushButton("Sales Table")
        self.summary_button.setFixedWidth(80) 
        self.summary_button.setFixedHeight(100)
        self.summary_button.clicked.connect(lambda: self.sales_table())
        self.purchase_button = QPushButton("Purchase")
        self.purchase_button.setFixedWidth(80)
        self.purchase_button.setFixedHeight(100)
        self.purchase_button.clicked.connect(lambda: self.table_purchase())
        purchase_button_layout.addWidget(self.summary_button)
        purchase_button_layout.addWidget(self.purchase_button)
        
        
    def navigate_to(self, target):
        if target in self.predefined_positions:
            self.position = self.predefined_positions[target]
            self.send_goal()

    def send_goal(self):
        if not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=1.0):
            print("[WARN] Navigate action server is not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.position[0]
        goal_msg.pose.pose.position.y = self.position[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation
        
        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("[WARN] Action goal rejected.")
            return

        print("로봇이 움직입니다!")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status
        if result == 2:
            print("배송지 도착!")
            
        # else:
        #     print(f"[WARN] Goal failed with status: {result}")
            
    def table_purchase(self):  # 추가된 함수
        # 주문 내용을 list.db에 저장
        db_path = os.path.expanduser("~/robot_service_ws/src/robot_serving/resource/list.db")
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()    
        # 테이블 생성
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS orders (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                item_name TEXT,
                quantity INTEGER
            )
        ''')
        # 주문 내역 추가
        for item, quantity in self.order_dict1.items():
            cursor.execute("INSERT INTO orders (item_name, quantity) VALUES (?, ?)", (item, quantity))
        
        conn.commit()
        conn.close()
        
        print("Purchased and saved to database!")
        self.update_order_list(self.empty_order)

        
    def sales_table(self):  # 추가된 함수
        # Sales_table.py 파일 실행
        script_path = os.path.expanduser("~/robot_service_ws/src/robot_serving/robot_serving/Sales_table.py")
        subprocess.Popen(["python3", script_path])
        print("Open sales table")
        

        
    def toggle_robot_button(self, button, robot_num):
        # 버튼이 눌려 있는 경우
        if button.isChecked():
            # 현재 버튼과 다른 버튼이 눌려 있을 때 기존 버튼 해제
            if self.current_robot_button and self.toggle_robot_button != button:
                self.current_robot_button.setChecked(False)
            # 현재 버튼을 새로운 눌린 버튼으로 설정
            self.current_robot_button = button
            self.current_robot = robot_num
        else:
            # 버튼이 해제된 경우 current_button 리셋
            self.current_robot_button = None
            self.current_robot = 0
            
    def robot_call(self):
        print(f"Robot {self.current_robot} Called")
    
    def robot_return(self):
        print(f"Robot {self.current_robot} is return to stage")

    def serve_to_table(self, table_num):
        print(f"Robot {self.current_robot} is seving to table {table_num}")
        
    def toggle_table_button(self, button, table_num):
        if button.isChecked():
            if self.current_table_button and self.current_table_button != button:
                self.current_table_button.setChecked(False)
            self.current_table_button = button
        else:
            self.current_table_button = None

        if table_num == 1:
            button.setStyleSheet(f"background-color: {self.original_button_color}")

        # Show the selected table's orders
        if table_num == 1:
            self.update_order_list(self.order_dict1)
        elif table_num == 2:
            self.update_order_list(self.order_dict2)
            
    def update_order_list(self, order_dict):
        # Clear previous orders
        for i in reversed(range(self.order_list_layout.count())): 
            self.order_list_layout.itemAt(i).widget().deleteLater()
        
        # Display the selected order list
        for key, value in order_dict.items():
            label = QLabel(f"{key} - {value}")
            self.order_list_layout.addWidget(label)
        

    def handle_order(self, request, response):
        item_name = request.item_name
        quantity = request.quantity
        is_Receipt = request.is_receipt

        if is_Receipt:  # 수령 완료 요청 처리
            self.get_logger().info(f"수령 완료 신호 수신")
            response.success = True
            response.message = "수령 완료 처리되었습니다."
            self.TOL_buttons[0].setStyleSheet("background-color: green")
            return response
        
        if item_name in self.cost and quantity > 0:
            response.success = True
            if item_name in self.cost:
                response.message = f"주문이 성공적으로 처리되었습니다: {item_name} X {quantity}"
                self.order_dict1 = {}
                self.order_dict1[item_name] = quantity
                self.update_order_list(self.order_dict1)
                self.TOL_buttons[0].setStyleSheet("background-color: blue")
            else:
                response.message = f"로봇이 돌아갑니다!"
        else:
            response.success = False
            response.message = "주문 처리 실패: 잘못된 메뉴 또는 수량입니다."
        
        return response
        
        
    def table_calling(self, msg):
        print('table1 is calling waiter')
        self.TOL_buttons[0].setStyleSheet("background-color: red")
        
    def run(self):
        # ROS2 스핀을 위한 별도 스레드 생성
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        # PyQt 메인 루프 시작
        self.window.show()
        sys.exit(self.app.exec_())

    def _spin_ros(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = POS()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()