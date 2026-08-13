echo ===yolo_test===
head -60 ~/Desktop/LH_TEST/yolo_test.py
echo ===camera_test_py===
head -30 ~/Desktop/LH_TEST/Camera.py
echo ===camera_grab_mjpg===
timeout 25 python3 -c '
import cv2
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
ok, f = cap.read()
print("ok=", ok, "shape=", f.shape if ok else "none")
cap.release()
' 2>&1 | tail -5
