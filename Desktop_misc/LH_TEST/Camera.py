import cv2

# 如果 1 打不开，请尝试改为 0
cap = cv2.VideoCapture(0
)

if not cap.isOpened():
    print("无法打开摄像头，请检查索引号或连接")
    exit()

while True:
    # 读取一帧图像
    ret, frame = cap.read()

    # 如果读取失败（例如摄像头被拔掉）
    if not ret:
        print("无法接收帧，正在退出...")
        break

    # 显示画面
    cv2.imshow("Camera Feed", frame)

    # 关键点：等待 1 毫秒，并检查是否按下了 'q' 键
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()