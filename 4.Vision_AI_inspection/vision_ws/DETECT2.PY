import requests
import cv2
import matplotlib.pyplot as plt
import random

# 이미지 파일 경로 및 모델 요청 설정
IMAGE_FILE_PATH = "/home/jaeheyoung/Rokey/7.PracticalProject/Week4_Intel1/vision_ws/vision-ai-inference-practice/5. conveyor-system/data_learn2/241129_103501.jpg"
image = open(IMAGE_FILE_PATH, "rb").read()
URL = "http://192.168.10.13:8885/inference/run"
params = {"min_confidence": "0.5", "base_model": "YOLOv6-N"}
headers = {"accept": "application/json"}
files = {"file": image}

# 모델 API 호출
response = requests.post(url=URL, headers=headers, params=params, files=files)
response_data = response.json()
print(response_data)

# class_number와 클래스 이름 및 색상 매핑
class_labels = {
    1: ("BOOTSEL", (255, 0, 0)),       # 빨강
    2: ("CHIPSET", (0, 255, 0)),   # 초록
    3: ("HOLE", (0, 0, 255)),      # 파랑
    4: ("OSCILLATOR", (255, 255, 0)), # 노랑
    5: ("PICO", (255, 0, 255)),    # 보라
    6: ("USB", (0, 255, 255)) # 청록
}

# 이미지 로드 및 디텍션 정보 처리
img = cv2.imread(IMAGE_FILE_PATH)
type = response_data['objects']

anotation = {}

for item in type:
    start_point = (int(item['bbox'][0]), int(item['bbox'][1]))
    end_point = (int(item['bbox'][2]), int(item['bbox'][3]))
    
    # 클래스 이름과 색상 가져오기
    class_number = item['class_number']
    class_name, color = class_labels.get(class_number, (f"Unknown({class_number})", (128, 128, 128)))  # 알 수 없는 클래스 대비
    thickness = 2

    # 바운딩 박스 그리기
    result = cv2.rectangle(img, start_point, end_point, color, thickness)
    
    # 클래스 이름으로 텍스트 그리기
    position = (int(item['bbox'][0]), int(item['bbox'][1]) - 10)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    result = cv2.putText(img, class_name, position, font, font_scale, color, thickness, cv2.LINE_AA)

    # 어노테이션 갱신
    anotation[class_name] = anotation.get(class_name, 0) + 1

# 어노테이션 텍스트 추가
print(anotation)
text_anotation = str(anotation)
cv2.putText(img, text_anotation, (20, 20), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

# 이미지 표시 및 종료
cv2.imshow("result", img)
cv2.waitKey(0)
cv2.destroyAllWindows()  # 모든 창 닫기

