echo ===dev_perms===
ls -la /dev/video* /dev/media0 2>&1
echo ===groups===
id
echo ===usb===
lsusb | grep -iE "camera|KS2A|2a43|174f" | head -3
lsusb | head -8
echo ===sudo_grab===
timeout 20 sudo -n v4l2-ctl --stream-mmap --stream-count=3 -d /dev/video0 2>&1 | tail -4
echo ===dmesg_cam===
sudo -n dmesg 2>/dev/null | grep -iE "uvc|video|camera" | tail -6
