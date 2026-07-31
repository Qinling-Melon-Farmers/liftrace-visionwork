import cv2

def detect_cameras():
    index = 0
    available_cameras = []
    while index < 10:  # 尝试前 10 个索引
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            available_cameras.append(index)
            cap.release()
        index += 1
    return available_cameras

cameras = detect_cameras()
print("可用的摄像头索引:", cameras)