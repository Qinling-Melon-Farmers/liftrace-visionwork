echo ===camera_processes===
ps aux | grep -iE "camera|ros|v4l" | grep -v grep | head -8
echo ===video1_grab===
timeout 20 python3 -c '
import cv2
cap = cv2.VideoCapture("/dev/video1")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
ok, f = cap.read()
print("video1 ok=", ok, "shape=", f.shape if ok else "none")
cap.release()
' 2>&1 | tail -3
echo ===cv2_backend===
timeout 20 python3 -c '
import cv2
print("backends:", cv2.videoio_registry.get_backends() if hasattr(cv2,"videoio_registry") else "n/a")
for i in [0,1]:
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    print("idx", i, "opened=", cap.isOpened())
    if cap.isOpened():
        ok, f = cap.read()
        print("  read ok=", ok, "shape=", f.shape if ok else "none")
        cap.release()
' 2>&1 | tail -6
echo ===v4l2_test===
timeout 10 v4l2-ctl --stream-mmap --stream-count=3 -d /dev/video0 2>&1 | tail -3
