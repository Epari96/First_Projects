import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        self.subscription = self.create_subscription(
            String,
            'conveyor_belt_status',
            self.listener_callback,
            10
        )
        self.serial = serial.Serial('/dev/ttyACM0', 115200)  # 아두이노 포트 설정

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # 아두이노로 명령 전송
        if command.startswith("F") :
            distance = int(command[1:])  # 거리 추출
            step_count = int(distance / 0.0096)  # mm를 스텝으로 변환
            command_to_send = f"F{step_count}"  # 스텝 수로 명령 생성
            self.serial.write((command_to_send + '\n').encode())  # 아두이노에 명령 전송
        
        elif command.startswith("STOP"):
            self.serial.write("STOP\n".encode())  # 아두이노에 STOP 명령 전송

def main(args=None):
    rclpy.init(args=args)
    conveyor_node = ConveyorNode()
    rclpy.spin(conveyor_node)
    conveyor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
