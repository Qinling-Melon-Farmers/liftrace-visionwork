echo ===formats_spca===
v4l2-ctl --list-formats-ext -d /dev/video0 2>&1 | head -15
echo ===cv2_grab_spca===
timeout 25 python3 -c '
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
print("opened=", cap.isOpened())
if cap.isOpened():
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    ok, f = cap.read()
    print("ok=", ok, "shape=", f.shape if ok else "none")
    if ok:
        cv2.imwrite("/tmp/cam_spca_test.jpg", f)
        print("saved /tmp/cam_spca_test.jpg")
    cap.release()
' 2>&1 | tail -4
echo ===file_check===
ls -la /tmp/cam_spca_test.jpg 2>/dev/null
