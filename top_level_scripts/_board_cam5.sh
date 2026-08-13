sleep 5
echo ===video_nodes===
ls -la /dev/video* 2>&1
echo ===v4l2_devices===
v4l2-ctl --list-devices 2>&1
echo ===try_all===
for dev in /dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4; do
  if [ -e "$dev" ]; then
    echo "--- $dev ---"
    timeout 12 v4l2-ctl --stream-mmap --stream-count=2 -d "$dev" 2>&1 | tail -2
  fi
done
