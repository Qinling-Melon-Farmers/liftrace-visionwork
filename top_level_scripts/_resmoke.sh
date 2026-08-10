#!/bin/bash
cd /home/orangepi/board_eval
echo "=== smoke legacy rknn dual with --u8-input (10 frames) ==="
timeout 300 python3 board_legacy_rknn_dual_viewer.py --video smoke.mp4 --json /tmp/smoke_legacy2.json --output-video /tmp/smoke_legacy2.mp4 --output-width 640 --stride 1 --warmup 1 --no-window --u8-input 2>&1 | grep -E "MODEL_READY|ERROR|FATAL" | head -6
python3 - <<'PY'
import json
d = json.load(open("/tmp/smoke_legacy2.json"))
print("LEGACY_RKNN_u8 class_hist:", d["derived"]["class_hist_total"])
print("first_det:", d["derived"]["first_detections"][:3])
print("p50_total_ms:", d["metrics_ms"]["total_ms"]["p50"])
PY
echo "=== video md5 check ==="
md5sum real_target.mp4
