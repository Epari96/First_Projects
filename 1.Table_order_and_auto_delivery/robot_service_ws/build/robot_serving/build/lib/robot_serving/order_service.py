from rclpy.node import Node
from rclpy import spin
from rclpy import init, shutdown
from order_interface.srv import Order

class OrderService(Node):
    def __init__(self):
        super().__init__('order_service')
        self.srv = self.create_service(Order, 'order_service', self.handle_order)
        self.cost = {'짜장면': 8000, '짬뽕': 8500, '탕수육': 20000, '제로 콜라': 3000} 
    
    def handle_order(self, request, response):
        item_name = request.item_name
        quantity = request.quantity
        is_receipt = request.is_receipt

        if is_receipt:  # 수령 완료 요청 처리
            self.get_logger().info(f"수령 완료 신호 수신")
            response.success = True
            response.message = "수령 완료 처리되었습니다."
            return response
        
        if item_name in self.cost and quantity > 0:
            response.success = True
            response.message = f"주문이 성공적으로 처리되었습니다: {quantity} x {item_name}"
        else:
            response.success = False
            response.message = "주문 처리 실패: 잘못된 메뉴 또는 수량입니다."
        
        return response

def main(args=None):
    init(args=args)
    order_service = OrderService()
    spin(order_service)
    shutdown()

if __name__ == '__main__':
    main()
