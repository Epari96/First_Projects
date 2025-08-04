import sys
import sqlite3
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QLabel, QTableWidget, QTableWidgetItem)
from PyQt5.QtGui import QColor
from PyQt5.QtCore import QTimer

# 데이터베이스 경로 설정
db_path = os.path.expanduser("~/robot_service_ws/src/robot_serving/resource/list.db")

# 메뉴별 가격 정보
menu_prices = {
    "짜장면": 8000,
    "짬뽕": 8500,
    "탕수육": 20000,
    "제로콜라": 2000
}

class DataEntryApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kiosk Sales Summary")
        self.setGeometry(100, 100, 600, 800)
        
        # 메인 위젯 설정
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()
        main_widget.setLayout(layout)
        
        # 일일 매출 표
        self.daily_sales_table = QTableWidget()
        self.daily_sales_table.setColumnCount(2)
        self.daily_sales_table.setHorizontalHeaderLabels(["Date", "Daily Sales"])
        layout.addWidget(QLabel("Daily Sales:"))
        layout.addWidget(self.daily_sales_table)
        
        # 월 매출 표
        self.monthly_sales_table = QTableWidget()
        self.monthly_sales_table.setColumnCount(2)
        self.monthly_sales_table.setHorizontalHeaderLabels(["Month", "Monthly Sales"])
        layout.addWidget(QLabel("Monthly Sales:"))
        layout.addWidget(self.monthly_sales_table)

        # 메뉴별 매출 표 (EA 열 포함)
        self.menu_sales_table = QTableWidget()
        self.menu_sales_table.setColumnCount(3)
        self.menu_sales_table.setHorizontalHeaderLabels(["Menu", "EA", "Total Sales"])
        layout.addWidget(QLabel("Menu Sales:"))
        layout.addWidget(self.menu_sales_table)
        
        # 데이터베이스에서 데이터를 불러와 표시
        self.load_sales_data()

        # 주기적으로 데이터를 업데이트하기 위한 타이머 설정 (1초마다 갱신)
        self.timer = QTimer()
        self.timer.timeout.connect(self.load_sales_data)
        self.timer.start(1000)  # 1000 밀리초(1초)

    def load_sales_data(self):
        """데이터베이스에서 일일 매출, 월 매출, 메뉴별 매출 정보를 불러와 각 표에 표시합니다."""
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # 일일 매출 데이터 불러오기
        cursor.execute('''
            SELECT DATE('now') AS order_date, item_name, SUM(quantity) AS total_quantity
            FROM orders
            GROUP BY order_date, item_name;
        ''')
        daily_sales_results = cursor.fetchall()
        
        # 일일 매출 합산
        daily_total = sum(menu_prices[item_name] * quantity for _, item_name, quantity in daily_sales_results)
        self.populate_table(self.daily_sales_table, [(daily_sales_results[0][0], daily_total)])

        # 월 매출 데이터 불러오기
        cursor.execute('''
            SELECT strftime('%Y-%m', DATE('now')) AS month, item_name, SUM(quantity) AS total_quantity
            FROM orders
            GROUP BY month, item_name;
        ''')
        monthly_sales_results = cursor.fetchall()

        # 월 매출 합산
        monthly_total = sum(menu_prices[item_name] * quantity for _, item_name, quantity in monthly_sales_results)
        self.populate_table(self.monthly_sales_table, [(monthly_sales_results[0][0], monthly_total)])

        # 메뉴별 매출 데이터 불러오기
        cursor.execute('''
            SELECT item_name, SUM(quantity) AS ea, SUM(quantity) AS total_quantity
            FROM orders
            GROUP BY item_name
            ORDER BY ea DESC;
        ''')
        menu_sales_results = cursor.fetchall()

        # 메뉴별 매출 계산
        menu_sales_data = [(item_name, ea, menu_prices[item_name] * ea) for item_name, ea, _ in menu_sales_results]
        self.populate_table(self.menu_sales_table, menu_sales_data)

        conn.close()

    def populate_table(self, table, data):
        """QTableWidget에 데이터를 채우고 스타일을 설정합니다."""
        table.setRowCount(len(data))
        for row_idx, row_data in enumerate(data):
            for col_idx, value in enumerate(row_data):
                # 숫자 데이터에 세 자리마다 콤마 추가
                formatted_value = f"{value:,.0f}" if isinstance(value, (int, float)) else str(value)
                item = QTableWidgetItem(formatted_value)
                table.setItem(row_idx, col_idx, item)

                # 총 매출 및 판매 수량 열에 색상을 설정 (예: 연한 빨간색)
                if (table == self.daily_sales_table and col_idx == 1) or \
                   (table == self.monthly_sales_table and col_idx == 1) or \
                   (table == self.menu_sales_table and col_idx == 2):
                    item.setBackground(QColor(255, 204, 204))

                elif col_idx == 1 and table == self.menu_sales_table:
                    item.setBackground(QColor(240, 240, 240))

def main():
    app = QApplication(sys.argv)
    window = DataEntryApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
