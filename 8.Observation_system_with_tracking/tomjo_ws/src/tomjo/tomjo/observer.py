import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node
from flask import Flask, Response
from std_msgs.msg import String
import cv2  # OpenCV
import numpy as np
from ultralytics import YOLO
import threading
import time  # 좌표가 안정화된 시간을 체크하기 위함
from std_srvs.srv import Trigger
import sqlite3  # SQLite 사용

##############################
# 전역 변수 및 설정
##############################
app = Flask(__name__)
frame_to_display = None  # Flask 서버에서 스트리밍할 프레임 (MJPEG)

# 마우스 클릭 관련
coordinates = []          # 최대 4개의 점 [p1, p2, p3, p4] (픽셀 좌표)
calibrated = False        # 4점 모두 찍혀 호모그래피를 계산했는지 여부
homography_matrix = None  # 투시 변환 행렬 (cv2.getPerspectiveTransform 결과)

# 4개의 실제 좌표 (사용자 지정)
real_pts = np.float32([
    [40.0,  0.0],  # P1
    [ 0.0,  0.0],  # P2
    [ 0.0, 30.0],  # P3
    [40.0, 30.0],  # P4
])

# 4개의 map 좌표
real_pts = np.float32([
    [-27.5, -25.0],  # P1
    [-27.5, 15.2],  # P2
    [-110.0, 15.2],  # P3
    [-110.0, -39.7],  # P4
])

# 오프셋 설정. 필요하면 사용
OFFSET_X = 4.0
OFFSET_Y = 0.0

##############################
# Flask 라우트
##############################
@app.route('/')
def video_feed():
    global frame_to_display

    def generate():
        while True:
            if frame_to_display is None:
                continue
            _, buffer = cv2.imencode('.jpg', frame_to_display)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


##############################
# Flask 서버 쓰레드 함수
##############################
def start_flask_server():
    app.run(host='0.0.0.0', port=8000, debug=False)


##############################
# 마우스 콜백 (OpenCV)
##############################
def mouse_callback(event, x, y, flags, param):
    """
    4점을 클릭하면, 호모그래피를 계산.
    p1->(40,0), p2->(0,0), p3->(0,30), p4->(40,30) 순서로 매핑.
    """
    global coordinates, calibrated, homography_matrix

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(coordinates) >= 4:
            return  # 이미 4점을 모두 찍었으면 무시

        coordinates.append((x, y))
        print(f"[Mouse] Clicked point {len(coordinates)} (pixel): ({x}, {y})")

        if len(coordinates) == 4:
            print(">>> 4 points collected. Calculating homography...")
            src_pts = np.array(coordinates, dtype=np.float32)
            H = cv2.getPerspectiveTransform(src_pts, real_pts)
            homography_matrix = H
            calibrated = True

            print("Homography matrix:\n", H)
            print("Calibration done!")
            # 더 이상 클릭 못하게
            cv2.setMouseCallback('LocalCamera', lambda *args: None)


##############################
# 투시 변환으로 픽셀->실제좌표 변환 함수
##############################
def transform_pixel_to_real(px, py, H):
    pts_src = np.array([[[px, py]]], dtype=np.float32)
    pts_dst = cv2.perspectiveTransform(pts_src, H)
    real_x, real_y = pts_dst[0, 0]
    return real_x, real_y


##############################
# ROS2 노드 정의
##############################
class CamDetectPublisher(Node):
    def __init__(self):
        super().__init__('cam_detect_publisher')

        # ----------------------------
        # 초기 상태: 활성화(active)
        # ----------------------------
        self.state = "active" #"standby"

        # YOLO 모델 로드
        try:
            self.model = YOLO('/home/jaeheyoung/Rokey/7.PracticalProject/Week8_Intel2/tomjo_ws/src/tomjo/resource/global_best3.pt')
            self.get_logger().info("YOLO 모델 로드 성공")
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            return

        # 카메라 열기
        self.cap = cv2.VideoCapture(7, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Camera could not be opened!")
            return

        # 디버그 로그
        self.get_logger().info(
            f"Camera opened: {self.cap.isOpened()}, "
            f"width: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, "
            f"height: {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}, "
            f"FPS: {self.cap.get(cv2.CAP_PROP_FPS)}"
        )

        # ----------------------------
        # SQLite DB 초기화
        # ----------------------------
        self.db_conn = sqlite3.connect('example.db', check_same_thread=False)
        self.db_cursor = self.db_conn.cursor()

        # 테이블 생성 (car/dummy 인식 로그 저장)
        self.db_cursor.execute("""
        CREATE TABLE IF NOT EXISTS detection_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            class_name TEXT,
            x INTEGER,
            y INTEGER,
            detection_time TEXT
        )
        """)

        # 테이블 생성 (AMR 이벤트 로그 저장)
        self.db_cursor.execute("""
        CREATE TABLE IF NOT EXISTS amr_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            event TEXT,
            event_time TEXT
        )
        """)

        self.db_conn.commit()

        # AMR 이벤트 토픽 구독 (예: "amr_event")
        self.amr_sub = self.create_subscription(
            String,
            'amr_event',  # AMR 출발/복귀 이벤트를 받는 토픽
            self.amr_callback,
            10
        )

        # 좌표 퍼블리셔 (car, dummy 등)
        self.coord_pub = self.create_publisher(String, 'observer/target_coordinates', 10)

        # 종료 서비스
        self.terminate_service = self.create_service(Trigger, 'observer/terminate', self.terminate_callback)

        # 활성화 서비스
        self.activate_service = self.create_service(Trigger, 'observer/activate', self.activate_callback)

        # 좌표 안정화 체크 변수
        self.last_seen_int_x = None
        self.last_seen_int_y = None
        self.last_seen_time = None
        self.last_published_int_x = None
        self.last_published_int_y = None
        self.publish_interval = 1.0  # 1초

        # 주기적 콜백 (5Hz = 0.2s)
        self.timer = self.create_timer(0.2, self.timer_callback)

        # 마지막으로 DB에 car/dummy 좌표를 저장한 시각 추적용
        self.last_car_save_time = 0.0
        self.last_dummy_save_time = 0.0
        self.save_interval = 1.0  # 1초 간격으로 DB 저장

    def amr_callback(self, msg):
        """
        AMR 출발/복귀 등 이벤트를 SQLite DB에 저장
        """
        event_str = msg.data  # 예: "start" 또는 "return" 등
        now_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        self.db_cursor.execute(
            "INSERT INTO amr_log (event, event_time) VALUES (?, ?)",
            (event_str, now_str)
        )
        self.db_conn.commit()

        self.get_logger().info(f"AMR Event '{event_str}' stored at {now_str}")

    def activate_callback(self, request, response):
        """
        다시 활성화 상태로 전환
        """
        self.get_logger().info("Received activation request. Switching to 'active' state.")
        self.state = "active"
        response.success = True
        response.message = "Node is now active."
        return response

    def terminate_callback(self, request, response):
        """
        종료 서비스 콜백
        """
        self.get_logger().info("Received termination signal. Shutting down...")
        response.success = True
        response.message = "Observer terminated successfully."
        rclpy.shutdown()
        self.db_conn.close()  # DB 연결 종료
        exit(0)
        return response

    def timer_callback(self):
        """
        0.2초마다 실행:
         - YOLO 추론(항상)
         - car, dummy 클래스 인식 시 → 1초 간격으로 DB 저장
         - 활성 상태면 car 좌표 퍼블리시 후 standby 전환
        """
        global frame_to_display
        global coordinates, calibrated, homography_matrix
        global OFFSET_X, OFFSET_Y

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        # YOLO 추론 (항상)
        try:
            results = self.model(frame)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 추론 중 오류: {e}")
            return

        car_published_this_frame = False

        # 바운딩박스 처리
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = box.conf[0]
            class_id = int(box.cls[0])
            class_name = (
                results.names[class_id]
                if (results.names and class_id in results.names)
                else str(class_id)
            )

            # 중심점
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # 바운딩 박스 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{class_name} {confidence:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # car 또는 dummy만 처리
            if class_name in ["car", "dummy"] and calibrated and homography_matrix is not None:
                # 픽셀 -> 실제좌표
                real_x, real_y = transform_pixel_to_real(center_x, center_y, homography_matrix)
                final_x = (real_x + OFFSET_X)/100
                final_y = (real_y + OFFSET_Y)/100

                # 1) DB 저장용: 정수 좌표
                int_x = int(round(final_x))
                int_y = int(round(final_y))

                # 2) 화면 표시용: 소수점 첫째 자리 좌표
                display_x = f"{final_x:.1f}"
                display_y = f"{final_y:.1f}"

                # 좌표 표시(프레임)
                text_xy = f"({display_x},{display_y})"
                cv2.putText(
                    frame,
                    text_xy,
                    (center_x + 10, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2
                )

                # ----------------------------
                # 1) DB 저장 (car / dummy)
                # ----------------------------
                now_time = time.time()
                now_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

                if class_name == "car":
                    # 일정 간격(save_interval=1초) 지나면 저장
                    if (now_time - self.last_car_save_time) >= self.save_interval:
                        self.db_cursor.execute("""
                            INSERT INTO detection_log(class_name, x, y, detection_time)
                            VALUES (?, ?, ?, ?)
                        """, (class_name, int_x, int_y, now_str))
                        self.db_conn.commit()

                        self.last_car_save_time = now_time
                        self.get_logger().info(f"[DB] Stored CAR at {now_str} => ({int_x}, {int_y})")

                elif class_name == "dummy":
                    # 일정 간격(save_interval=1초) 지나면 저장
                    if (now_time - self.last_dummy_save_time) >= self.save_interval:
                        self.db_cursor.execute("""
                            INSERT INTO detection_log(class_name, x, y, detection_time)
                            VALUES (?, ?, ?, ?)
                        """, (class_name, int_x, int_y, now_str))
                        self.db_conn.commit()

                        self.last_dummy_save_time = now_time
                        self.get_logger().info(f"[DB] Stored DUMMY at {now_str} => ({int_x}, {int_y})")

                # ----------------------------
                # 2) 활성 상태 => 좌표 퍼블리시 후 standby
                # ----------------------------
                if class_name == "car" and self.state == "active":
                    # (기존 로직) 안정화 체크
                    t_now = time.time()

                    if self.last_seen_int_x is None:
                        self.last_seen_int_x = int_x
                        self.last_seen_int_y = int_y
                        self.last_seen_time = t_now
                    else:
                        if (int_x != self.last_seen_int_x) or (int_y != self.last_seen_int_y):
                            self.last_seen_int_x = int_x
                            self.last_seen_int_y = int_y
                            self.last_seen_time = t_now
                        else:
                            if (t_now - self.last_seen_time) >= self.publish_interval:
                                if (self.last_published_int_x != int_x or
                                    self.last_published_int_y != int_y):
                                    # 퍼블리시
                                    msg = String()
                                    msg.data = f"x:{int_x},y:{int_y}"
                                    self.coord_pub.publish(msg)
                                    self.get_logger().info(f"[Stable] Published coordinates: {msg.data}")

                                    self.last_published_int_x = int_x
                                    self.last_published_int_y = int_y
                                    car_published_this_frame = True

        # 마우스 클릭 4점 표시
        for i, (cx, cy) in enumerate(coordinates):
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        if len(coordinates) == 4:
            cv2.line(frame, coordinates[0], coordinates[1], (0, 255, 0), 2)
            cv2.line(frame, coordinates[1], coordinates[2], (0, 255, 0), 2)
            cv2.line(frame, coordinates[2], coordinates[3], (0, 255, 0), 2)
            cv2.line(frame, coordinates[3], coordinates[0], (0, 255, 0), 2)

        # Flask 스트리밍용 프레임
        global frame_to_display
        frame_to_display = frame.copy()

        # 로컬 화면 표시
        cv2.imshow("LocalCamera", frame)
        cv2.waitKey(1)

        # car 퍼블리시 후 standby 전환
        if car_published_this_frame:
            self.state = "standby"
            self.get_logger().info("[State] Car published => Switching to 'standby'.")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        # DB 커넥션 종료
        self.db_conn.close()
        super().destroy_node()


##############################
# 메인 함수
##############################
def main(args=None):
    rclpy.init(args=args)
    node = CamDetectPublisher()

    # OpenCV 창 & 마우스 콜백
    cv2.namedWindow('LocalCamera')
    cv2.setMouseCallback('LocalCamera', mouse_callback)

    # Flask 서버 별도 쓰레드
    flask_thread = threading.Thread(target=start_flask_server, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("ROS2 노드와 Flask 서버 종료")


if __name__ == '__main__':
    main()
