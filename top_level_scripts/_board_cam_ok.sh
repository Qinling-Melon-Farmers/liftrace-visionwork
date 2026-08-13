echo ===formats===
v4l2-ctl --list-formats-ext -d /dev/video0 2>&1 | grep -E "\[|Size|Interval" | head -20
echo ===cv2_grab===
timeout 25 python3 -c '
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
for i in range(5):
    ok, f = cap.read()
    if ok:
        print("frame", i, "ok shape=", f.shape)
        cv2.imwrite("/tmp/ks2a543_test.jpg", f)
        print("saved")
        break
cap.release()
' 2>&1 | tail -4
echo ===saved===
ls -la /tmp/ks2a543_test.jpg 2>/dev/null
echo ===fps_test===
timeout 10 python3 -c '
import cv2, time
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
t0 = time.time()
n = 0
while time.time() - t0 < 5:
    ok, f = cap.read()
    if ok: n += 1
cap.release()
print("fps =", n / 5.0)
' 2>&1 | tail -2
