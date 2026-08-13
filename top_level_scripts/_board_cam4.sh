echo ===video_nodes_now===
ls -la /dev/video* 2>&1
echo ===usb_now===
lsusb | grep -iE "camera|2650|1bcf" | head -3
echo ===try_each_video===
for dev in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do
  if [ -e "$dev" ]; then
    echo "--- $dev ---"
    timeout 15 python3 -c "
import cv2, sys
cap = cv2.VideoCapture('$dev', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
ok, f = cap.read()
print('ok=', ok, 'shape=', f.shape if ok else 'none')
if ok:
    cv2.imwrite('/tmp/cam_${dev##*/}.jpg', f)
    print('saved /tmp/cam_${dev##*/}.jpg')
cap.release()
" 2>&1 | tail -2
  fi
done
echo ===dmesg_tail===
sudo -n dmesg 2>/dev/null | tail -5
