echo ===lsusb_full===
lsusb
echo ===usb_tree===
lsusb -t 2>&1
echo ===video_owners===
v4l2-ctl --list-devices 2>&1
echo ===try_grab===
for dev in /dev/video0 /dev/video1; do
  if [ -e "$dev" ]; then
    echo "--- $dev ---"
    timeout 18 v4l2-ctl --stream-mmap --stream-count=3 --stream-to=/tmp/grab_${dev##*/}.raw -d "$dev" 2>&1 | tail -2
    ls -la /tmp/grab_${dev##*/}.raw 2>/dev/null
  fi
done
