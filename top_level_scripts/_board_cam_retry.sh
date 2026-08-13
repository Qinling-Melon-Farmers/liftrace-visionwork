echo ===current_uvc_params===
cat /sys/module/uvcvideo/parameters/timeout 2>/dev/null || echo "no timeout param"
echo ===reload_uvc_with_timeout===
sudo -n modprobe -r uvcvideo 2>&1 | head -2
sudo -n modprobe uvcvideo timeout=5000 2>&1 | head -2
sleep 2
cat /sys/module/uvcvideo/parameters/timeout 2>/dev/null
echo ===grab_retry===
timeout 25 python3 -c '
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
ok, f = cap.read()
print("ok=", ok, "shape=", f.shape if ok else "none")
if ok:
    cv2.imwrite("/tmp/board_cam_test.jpg", f)
    print("saved /tmp/board_cam_test.jpg")
cap.release()
' 2>&1 | tail -4
echo ===dmesg_new===
sudo -n dmesg 2>/dev/null | grep -iE "uvc|video" | tail -4
