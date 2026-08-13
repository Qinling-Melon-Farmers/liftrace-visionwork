echo ===all_usb===
lsusb
echo ===video_owners===
v4l2-ctl --list-devices 2>&1
echo ===spca_full_formats===
v4l2-ctl --list-formats-ext -d /dev/video0 2>&1 | head -30
echo ===spca_grab_fps===
timeout 15 v4l2-ctl --set-fmt-video=width=160,height=120,pixelformat=YUYV --stream-mmap --stream-count=3 --stream-to=/tmp/spca.raw -d /dev/video0 2>&1 | tail -3
ls -la /tmp/spca.raw 2>/dev/null
