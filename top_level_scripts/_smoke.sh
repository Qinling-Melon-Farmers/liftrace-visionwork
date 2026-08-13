#!/bin/bash
set -e
echo "=== md5 check ==="
md5sum ~/board_eval/merged_standard_fp32.rknn
echo "---local---"
echo "cbb3d8dc9f620c0f0ffaeb5306b46ab2  (expect compare later)"
echo "=== make tiny test video from legacy sample images ==="
python3 - <<'PY'
import cv2, os, glob
imgs = sorted(glob.glob(os.path.expanduser("~/Visual/src/yolov5_detect/scripts/*.jpg")))
print("sample imgs:", imgs)
cap = cv2.VideoWriter(os.path.expanduser("~/board_eval/smoke.mp4"),
                      cv2.VideoWriter_fourcc(*"mp4v"), 10, (640, 640))
for i in range(30):
    for p in imgs:
        img = cv2.imread(p)
        img = cv2.resize(img, (640, 640))
        cap.write(img)
cap.release()
print("smoke.mp4 written")
PY
ls -la ~/board_eval/
echo "=== smoke new chain rknn (10 frames) ==="
cd ~/board_eval && timeout 300 python3 board_realtime_rknn_viewer.py --video smoke.mp4 --json /tmp/smoke_new.json --output-video /tmp/smoke_new.mp4 --output-width 640 --stride 1 --warmup 1 --no-window merged_standard_fp32.rknn 2>&1 | grep -E "MODEL_READY|ERROR|FATAL|frames_measured|wall_fps|throughput" | head -20
echo "=== smoke legacy rknn dual (10 frames) ==="
cd ~/board_eval && timeout 300 python3 board_legacy_rknn_dual_viewer.py --video smoke.mp4 --json /tmp/smoke_legacy.json --output-video /tmp/smoke_legacy.mp4 --output-width 640 --stride 1 --warmup 1 --no-window 2>&1 | grep -E "MODEL_READY|ERROR|FATAL|frames_measured|wall_fps|throughput" | head -20
echo "=== smoke legacy pytorch (6 frames) ==="
cd ~/board_eval && timeout 600 python3 board_legacy_pt_video.py --video smoke.mp4 --json /tmp/smoke_pt.json --output-video /tmp/smoke_pt.mp4 --output-width 640 --max-frames 6 --warmup 1 2>&1 | grep -E "MODEL_READY|ERROR|FATAL|frames_measured|wall_fps|throughput" | head -20
echo "=== smoke done ==="
