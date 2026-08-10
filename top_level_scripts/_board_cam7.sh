echo ===full_formats===
v4l2-ctl --list-formats-ext -d /dev/video0 2>&1 | grep -E "\[|Size|Interval" | head -20
echo ===grab_default_yuyv===
timeout 20 python3 -c '
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
# 不设 FOURCC，用默认格式
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ok, f = cap.read()
print("default ok=", ok, "shape=", f.shape if ok else "none")
cap.release()
' 2>&1 | tail -3
echo ===grab_320x240===
timeout 20 python3 -c '
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
ok, f = cap.read()
print("320x240 ok=", ok, "shape=", f.shape if ok else "none")
if ok:
    cv2.imwrite("/tmp/cam_low.jpg", f)
    print("saved")
cap.release()
' 2>&1 | tail -3
echo ===usb_tree===
lsusb -t 2>&1 | head -12
echo ===dmesg_usb===
sudo -n dmesg 2>/dev/null | grep -iE "usb.*(error|fail|timeout)|uvc.*(error|fail|timeout)" | tail -8
