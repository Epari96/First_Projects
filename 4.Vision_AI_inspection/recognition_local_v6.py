import time
import serial
import requests
import numpy
from io import BytesIO
from pprint import pprint
from datetime import datetime
import pandas as pd
import cv2

# API endpoint URL
url = "http://192.168.10.13:8885/inference/run"

# save file path
dir_path = "/home/rokey/conv_system/test1/"

model = "YOLOv6-N" # YOLOv6-N, YOLOv6-M, YOLOv6-L, YOLOv6-L6
confidence = 0.39

# Setting condition of detecting (key: filtering class, value: counts)
conditions = {'BOOTSEL': 1,
              'PICO': 1,
              'CHIPSET': 1,
              'OSCILLATOR': 1,
              'USB': 1,
              'HOLE': 4
              }

class_labels = {
    1: ("BOOTSEL", (255, 0, 0)),
    2: ("CHIPSET", (0, 255, 0)),
    3: ("HOLE", (0, 0, 255)),
    4: ("OSCILLATOR", (255, 255, 0)),
    5: ("PICO", (255, 0, 255)),
    6: ("USB", (0, 255, 255)) 
}

ser = serial.Serial("/dev/ttyACM0", 9600)

def get_img():
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        print("Camera Error")
        exit(-1)
    ret, img = cam.read()
    cam.release()
    return img

def crop_img(img, size_dict):
    x = size_dict["x"]
    y = size_dict["y"]
    w = size_dict["width"]
    h = size_dict["height"]
    img = img[y : y + h, x : x + w]
    return img
        
def test_start_server():
    # Parameters
    params = {
        "model_id": "4b6a9ce6-c324-43b9-a378-d95681f8fce2"  # Replace with actual model ID
    }
    # Send POST request
    response = requests.post(url, params=params)
    # Check if request was successful
    if response.status_code == 200:
        print("Server started successfully!")
        print("Response:", response.json())
    else:
        print(f"Error: {response.status_code}")
        print(response.text)
        
def test_image_detection(image_path, url, confidence, model):
    # Parameters
    params = {
        "min_confidence": confidence,
        "base_model": model
    }
    # Prepare the file
    with open(image_path, 'rb') as image_file:
        files = {'file': image_file}
    # Send POST request
        response = requests.post(url, params=params, files=files)
    # return response
    if response.status_code == 200:
        return response.json()
    else:
        print(f"Error: {response.status_code}")
        print(response.text)

# exel save
detection_records = []
def save_detection_summary():
    if detection_records:
        df = pd.DataFrame(detection_records)
        filename = datetime.now().strftime("%y%m%d_%H%M%S")
        excel_path = f"detection_summary_{filename}.xlsx"
        df.to_excel(excel_path, index=False)
        print(f"Detection summary saved to {excel_path}")

if __name__ == "__main__":
    print("Testing server start...")
    # test_start_server()
    print("Start screening (Press 'q' to quit)...")
    try:
        while True:
            # check key input
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' key for quit
                print("Exiting loop...")
                break

            data = ser.read()
            if data == b"0":
                print("Witing for product...")
                t1 = time.time()
                img = get_img()
                crop_info = {"x": 0, "y": 30, "width": 800, "height": 380}
                if crop_info is not None:
                    img = crop_img(img, crop_info)
                
                # Save original image
                filename = datetime.now().strftime("%y%m%d_%H%M%S")
                cv2.imwrite(f"{dir_path}original/{filename}.jpg", img)
                print("Original image saved!")
                
                # Send image to server for object detection
                image_path = f"{dir_path}original/{filename}.jpg"
                result = test_image_detection(image_path, url, confidence, model)
                
                # Get detected objects            
                chupopchus = result['objects']
                anotation = {"FILENAME": filename}

                # Draw each detected object at image
                for item in chupopchus:
                    start_point = (int(item['bbox'][0]), int(item['bbox'][1]))
                    end_point = (int(item['bbox'][2]), int(item['bbox'][3]))
                    
                    # Get class name
                    class_number = item['class_number']
                    class_name, color = class_labels.get(class_number, (f"Unknown({class_number})", (128, 128, 128)))
                    thickness = 2

                    # Add bbox
                    result = cv2.rectangle(img, start_point, end_point, color, thickness)
                    
                    # Add class text
                    position = (int(item['bbox'][0]), int(item['bbox'][1]) - 10)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.6
                    result = cv2.putText(img, class_name, position, font, font_scale, color, thickness, cv2.LINE_AA)

                    # Refresh annotation
                    anotation[class_name] = anotation.get(class_name, 0) + 1
                
                # Add text anotation
                y_offset = 30 
                x_position = 20 
                line_height = 20 
                for key, value in anotation.items():
                    text_line = f"{key}: {value}"
                    result = cv2.putText(
                        img, text_line, (x_position, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1, cv2.LINE_AA)
                    y_offset += line_height
                
                # Save object detected image
                cv2.imwrite(f"{dir_path}catch/{filename}.jpg", img)
                
                # Save image at "defects" diretory and show image
                # if can't catch all condition
                defect_image = False
                for key, threshold in conditions.items():
                    if key not in anotation or anotation[key] != threshold:
                        defect_image = True
                        break  
                if defect_image:
                    cv2.imwrite(f"{dir_path}defects/{filename}.jpg", img)
                    print(filename, "has defects! Saved at defects directory.")
                    cv2.imshow(f"{filename}", img)
                
                # Add detection record for Excel
                detection_records.append(anotation)
                
                ser.write(b"1")
                t2 = time.time()
                print(f"Processing time: {(t2 - t1):.2f} seconds")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # exel saved
        save_detection_summary()
        print("Detection records saved. Exiting program.")
