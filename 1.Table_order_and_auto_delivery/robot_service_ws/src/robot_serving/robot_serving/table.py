import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QLabel, QGroupBox, QFormLayout, QScrollArea
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import threading

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from order_interface.srv import Order

class TableOrder(Node):
    def __init__(self):
        super().__init__('table_order')
        
        QOS_RKL10V = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        durability=QoSDurabilityPolicy.VOLATILE)
        
        # publisher
        self.CallWaiter = self.create_publisher(String, 'call_waiter', QOS_RKL10V)
        
        self.client = self.create_client(Order, 'order_service')  # 서비스 클라이언트 생성

        self.order_data = {item: 0 for item in ["짜장면", "짬뽕", "탕수육", "제로 콜라"]}
        self.temp_order_data = {item: 0 for item in ["짜장면", "짬뽕", "탕수육", "제로 콜라"]}
        self.cost = {'짜장면': 8000, '짬뽕': 8500, '탕수육': 20000, '제로 콜라': 3000}
        
        self.quantity_labels = {}
        self.sum_cost_labels = {}
        
        # GUI window, widget
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("Table Order")
        self.window.setGeometry(100, 100, 800, 400)
        self.main_widget = QWidget(self.window)
        self.window.setCentralWidget(self.main_widget)
        
        # split half for menu/order window
        main_layout = QHBoxLayout()
        self.main_widget.setLayout(main_layout)
        
        
        # left layout
        menu_layout = QVBoxLayout()
        
        items = ["짜장면", "짬뽕", "탕수육", "제로 콜라"]
        for item in items:
            groupbox = QGroupBox("")
            group_layout = QHBoxLayout()
            groupbox.setLayout(group_layout)
            # 이미지 (왼쪽)
            image_label = QLabel()
            pixmap = QPixmap(f'/home/jaeheyoung/robot_service_ws/src/robot_serving/resource/{item}.jpg')  # 이미지 파일 경로
            image_label.setPixmap(pixmap.scaled(100, 100, Qt.KeepAspectRatio))  # 이미지 크기 조정
            image_label.setAlignment(Qt.AlignCenter)  # 중앙 정렬
            # 항목 (왼쪽)
            item_label = QLabel(item)
            item_label.setAlignment(Qt.AlignCenter)  # 중앙 정렬
            # 버튼 (오른쪽)
            self.quantity_labels[item] = QLabel(f"Quantity: {self.temp_order_data[item]}")
            self.sum_cost_labels[item] = QLabel(f"Sum Cost: {self.cost[item] * self.temp_order_data[item]}")
            # - 버튼 (수량 감소)
            minus_button = QPushButton("-")
            minus_button.clicked.connect(lambda _, item=item: self.adjust_temp_quantity(item, -1))
            # + 버튼 (수량 증가)
            plus_button = QPushButton("+")
            plus_button.clicked.connect(lambda _, item=item: self.adjust_temp_quantity(item, 1))
            # 담기 버튼 추가
            add_button = QPushButton("담기")
            add_button.clicked.connect(lambda _, item=item: self.add_to_right_list(item))
            # 금액, 소계금액 라벨
            cost_label = QLabel('Cost:{}'.format(self.cost[item]))
            self.sum_cost_label = QLabel('Sum Cost:{}'.format(self.cost[item]*self.temp_order_data[item]))
            # 그룹박스 내 왼쪽과 오른쪽 레이아웃 구성
            group_left_layout = QVBoxLayout()
            group_left_layout.addWidget(image_label)
            group_left_layout.addWidget(item_label)

            group_right_cost = QHBoxLayout()
            group_right_cost.addWidget(self.quantity_labels[item])
            group_right_cost.addWidget(minus_button)
            group_right_cost.addWidget(plus_button)

            add_button_layout = QHBoxLayout()
            add_button_layout.addWidget(self.sum_cost_labels[item])
            add_button_layout.addWidget(add_button)

            group_right_layout = QVBoxLayout()
            group_right_layout.addWidget(cost_label)
            group_right_layout.addLayout(group_right_cost)
            group_right_layout.addLayout(add_button_layout)

            # 세로로 레이아웃 분할
            group_layout.addLayout(group_left_layout)
            group_layout.addLayout(group_right_layout)

            menu_layout.addWidget(groupbox)
        
        main_layout.addLayout(menu_layout)
        
        
        # right layout
        self.order_layout = QVBoxLayout()  # 오른쪽 레이아웃 생성
        
        # 스크롤 가능한 위쪽 레이아웃
        scroll_area = QScrollArea()
        scroll_area.setFixedHeight(400)  # 고정 크기 설정
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(scroll_content)
        scroll_area.setWidget(scroll_content)
        
        # total cost layout
        self.total_cost = sum(self.order_data[item] * self.cost[item] for item in self.order_data if item in self.cost)
        self.total_cost_label = QLabel(f"Total Cost: {self.total_cost}")
        self.total_cost_label.setAlignment(Qt.AlignRight)  # 중앙 정렬
        
        # 아래쪽 버튼 레이아웃
        button_layout = QHBoxLayout()
        call_button = QPushButton("수령완료")
        call_button.setFixedWidth(100)
        call_button.setFixedHeight(60)
        call_button.clicked.connect(self.confirm_receipt)
        button_layout = QHBoxLayout()
        waiter_button = QPushButton("직원\n호출")
        waiter_button.setFixedWidth(50)
        waiter_button.setFixedHeight(60)
        waiter_button.clicked.connect(self.call_waiter)
        order_button = QPushButton("주문")
        order_button.setFixedHeight(60)
        order_button.clicked.connect(self.send_order)
        
        button_layout.addWidget(waiter_button)
        button_layout.addWidget(call_button)
        button_layout.addWidget(order_button)
        

        # 오른쪽 레이아웃에 스크롤 영역과 버튼 레이아웃 추가
        self.order_layout.addWidget(scroll_area)
        self.order_layout.addWidget(self.total_cost_label)
        
        self.order_layout.addLayout(button_layout)
        main_layout.addLayout(self.order_layout)
    
    
    def adjust_temp_quantity(self, item, delta):
        # 수량 조정
        self.temp_order_data[item] += delta
        if self.temp_order_data[item] < 0:
            self.temp_order_data[item] = 0  # 수량이 0보다 작아지지 않도록 제한

        # 개별 수량 라벨 및 소계금액 라벨 업데이트
        self.quantity_labels[item].setText(f"Quantity: {self.temp_order_data[item]}")
        sum_cost = self.temp_order_data[item] * self.cost[item]
        self.sum_cost_labels[item].setText(f"Sum Cost: {sum_cost}")


    def add_to_right_list(self, item):
        # 주문 데이터 업데이트
        self.order_data[item] += self.temp_order_data[item]
        self.temp_order_data[item] = 0  # temp_order_data 초기화

        # 왼쪽 수량 라벨 업데이트
        self.quantity_labels[item].setText(f"Quantity: {self.temp_order_data[item]}")
        self.sum_cost_labels[item].setText(f"Sum Cost: 0")

        # 기존 항목 제거 후 새로운 항목 추가
        for i in reversed(range(self.scroll_layout.count())):
            widget = self.scroll_layout.itemAt(i).widget()
            if widget and widget.property("item_name") == item:
                self.scroll_layout.takeAt(i).widget().deleteLater()

        # 오른쪽 주문 목록에 새 항목 추가 (그룹박스 형식)
        groupbox = QGroupBox()
        groupbox.setFixedHeight(100)
        groupbox.setProperty("item_name", item)
        layout = QHBoxLayout()

        # 항목명 라벨
        item_label = QLabel(item)
        layout.addWidget(item_label)

        # 수량 라벨
        quantity_label = QLabel(f"Quantity: {self.order_data[item]}")
        layout.addWidget(quantity_label)

        # + 버튼 (수량 증가)
        plus_button = QPushButton("+")
        plus_button.clicked.connect(lambda _, item=item: self.adjust_right_quantity(item, 1, quantity_label))
        layout.addWidget(plus_button)

        # - 버튼 (수량 감소)
        minus_button = QPushButton("-")
        minus_button.clicked.connect(lambda _, item=item: self.adjust_right_quantity(item, -1, quantity_label))
        layout.addWidget(minus_button)

        # 삭제 버튼
        delete_button = QPushButton("삭제")
        delete_button.clicked.connect(lambda _, item=item, box=groupbox: self.remove_item(box, item))
        layout.addWidget(delete_button)

        # 그룹박스에 레이아웃 추가 후 스크롤 영역에 그룹박스 추가
        groupbox.setLayout(layout)
        self.scroll_layout.addWidget(groupbox)

        # 전체 총 금액 업데이트
        self.update_total_cost()

    def remove_item(self, groupbox, item):
        # 주문 목록에서 해당 그룹박스 삭제 및 order_data 초기화
        self.scroll_layout.removeWidget(groupbox)
        groupbox.deleteLater()
        self.order_data[item] = 0  # 해당 항목의 수량을 0으로 초기화
        self.update_total_cost()

    def adjust_right_quantity(self, item, delta, quantity_label):
        # 오른쪽 주문 수량 조절 및 업데이트
        self.order_data[item] += delta
        if self.order_data[item] < 0:
            self.order_data[item] = 0  # 수량이 음수가 되지 않도록 제한
        # 수량 및 총 금액 라벨 업데이트
        quantity_label.setText(f"Quantity: {self.order_data[item]}")
        # 전체 총 금액 업데이트
        self.update_total_cost()

    def update_total_cost(self):
        # 전체 총 금액 계산 후 라벨 업데이트
        self.total_cost = sum(self.order_data[i] * self.cost[i] for i in self.order_data)
        self.total_cost_label.setText(f"Total Cost: {self.total_cost}")

        # 수령 완료 신호를 주방에 보내는 메서드
    def confirm_receipt(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스가 사용 가능해질 때까지 대기 중...')
        
        request = Order.Request()  # 요청 객체 생성
        request.item_name = "수령완료"  # 수령 완료 메시지
        request.quantity = 1  # 수량은 1로 설정 (필요에 따라 수정 가능)
        request.is_receipt = True  # 수령 완료 메시지

        future = self.client.call_async(request)  # 비동기 호출
        rclpy.spin_until_future_complete(self, future)  # 응답 대기

        if future.result() is not None:
            self.get_logger().info(f"수령 완료: {future.result().message}")
        else:
            self.get_logger().error('서비스 호출 실패')

        
    def send_order(self):
        for item, quantity in self.order_data.items():
            if quantity > 0:
                client = self.create_client(Order, 'order_service')
                while not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('서비스가 사용 가능해질 때까지 대기 중...')
                
                request = Order.Request()
                request.item_name = item
                request.quantity = quantity

                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                
                if future.result() is not None:
                    self.get_logger().info(future.result().message)
                else:   
                    self.get_logger().error('서비스 호출 실패')
        self.order_data = {item: 0 for item in self.order_data}
        self.update_total_cost()

    def call_waiter(self):
        msg = String()
        msg.data = "Calling"
        self.CallWaiter.publish(msg)
        self.get_logger().info(msg.data)

    
    def run(self):
        # ROS2 스핀을 위한 별도 스레드 생성
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        # PyQt 메인 루프 시작
        self.window.show()
        sys.exit(self.app.exec_())

    def _spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = TableOrder()
    node.run()

if __name__ == '__main__':
    main()