from PyQt5.QtWidgets import QApplication, QPushButton, QMainWindow
from PyQt5.QtGui import QColor, QPalette
import sys

class ToggleButton(QPushButton):
    def __init__(self, text):
        super().__init__(text)
        self.is_triggered = False  # 버튼 색이 변경된 상태인지 여부
        self.clicked.connect(self.reset_color)  # 버튼 클릭 시 원래 색으로 리셋

    def trigger_color_change(self):
        """콜백 함수에 의해 호출되어 버튼 색을 변경하는 메서드"""
        if not self.is_triggered:  # 이미 색이 변경된 상태가 아닐 때만 실행
            self.is_triggered = True
            palette = self.palette()
            palette.setColor(QPalette.Button, QColor("red"))  # 버튼 색을 빨간색으로 변경
            self.setPalette(palette)
            self.setAutoFillBackground(True)

    def reset_color(self):
        """버튼 클릭 시 호출되어 버튼 색을 원래대로 돌리는 메서드"""
        if self.is_triggered:
            self.is_triggered = False
            palette = self.palette()
            palette.setColor(QPalette.Button, QColor("lightgray"))  # 원래 색으로 복구
            self.setPalette(palette)
            self.setAutoFillBackground(True)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Color Toggle Button Example")
        self.setGeometry(100, 100, 300, 200)

        # 토글 버튼 생성
        self.toggle_button = ToggleButton("Toggle Button")
        self.toggle_button.setFixedSize(150, 50)
        self.setCentralWidget(self.toggle_button)

        # 콜백 함수에서 버튼 색 변경 트리거
        self.some_callback()

    def some_callback(self):
        """임의의 콜백 함수에서 버튼 색 변경 트리거"""
        self.toggle_button.trigger_color_change()

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
