#!/bin/bash
# 摄像头恢复 watchdog v2:
# 1) 等 /dev/video0 出现 (30 分钟窗口, 每 10s 检查)
# 2) 等 20s 让流稳定, 再用 v4l2-ctl 快检 3 帧, 过滤"枚举成功但流坏"的设备
# 3) 通过后自动跑完整 GUI 测试
for i in $(seq 1 180); do
  if [ -e /dev/video0 ]; then
    echo "[watchdog] video0 appeared at attempt $i" >> /tmp/cam_watchdog.log
    sleep 20
    timeout 15 v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=3 >/dev/null 2>&1
    if [ $? -ne 0 ]; then
      echo "[watchdog] stream check FAILED at $i, keep waiting" >> /tmp/cam_watchdog.log
      sleep 10
      continue
    fi
    echo "[watchdog] stream check OK at $i, start test" >> /tmp/cam_watchdog.log
    export DISPLAY=:0
    rm -f /tmp/cam_report.json /tmp/cam_gui_test.log
    python3 /tmp/cam_gui_test.py >> /tmp/cam_gui_test.log 2>&1
    echo "[watchdog] test done rc=$?" >> /tmp/cam_watchdog.log
    exit 0
  fi
  sleep 10
done
echo "[watchdog] timeout no usable video0 for 30min" >> /tmp/cam_watchdog.log
exit 1
