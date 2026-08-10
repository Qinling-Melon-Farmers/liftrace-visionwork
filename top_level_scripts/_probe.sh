#!/bin/bash
set -e
echo "=== board basic ==="
uname -m; nproc; lsb_release -ds 2>/dev/null || cat /etc/os-release | head -2
echo "=== python env ==="
python3 -c "import cv2,numpy,ultralytics;print('cv2',cv2.__version__,'np',numpy.__version__,'ultralytics',ultralytics.__version__)"
python3 -c "import rknnlite;print('rknnlite ok')"
python3 -c "import torch;print('torch',torch.__version__,'cuda',torch.cuda.is_available())" 2>&1 | tail -1
echo "=== legacy models ==="
ls -la ~/Visual/src/yolov5_detect/ 2>&1 | head -30
ls -la ~/Visual/src/yolov5_detect/best_rknn_model/ ~/Visual/src/yolov5_detect/tank_rknn_model/ 2>&1
echo "=== board_eval dir ==="
ls -la ~/board_eval/ 2>&1
echo "=== disk space ==="
df -h ~ | tail -1
echo "=== cpu freq ==="
grep "cpu MHz" /proc/cpuinfo | head -8
