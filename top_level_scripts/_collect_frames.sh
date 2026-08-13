#!/bin/bash
# 板端: 从三路输出视频各抽一帧带检测的 JPG 到 ~/board_eval/frames/
mkdir -p /home/orangepi/board_eval/frames
python3 - <<'PY'
import cv2, os
jobs = [("out_legacy_rknn.mp4", "frame_legacy_rknn.jpg"),
        ("out_new_rknn.mp4", "frame_new_rknn.jpg"),
        ("out_legacy_pt.mp4", "frame_legacy_pt.jpg")]
out = os.path.expanduser("~/board_eval/frames")
os.makedirs(out, exist_ok=True)
for video, name in jobs:
    path = os.path.join(os.path.expanduser("~/board_eval"), video)
    if not os.path.exists(path):
        print("SKIP missing", video)
        continue
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    target = min(2000, max(100, total // 2))  # 中段, 目标大概率在画面
    picked = None
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx == target:
            picked = frame
            break
        idx += 1
    cap.release()
    if picked is None:  # 退化为第 100 帧
        cap = cv2.VideoCapture(path)
        for _ in range(100):
            ok, picked = cap.read()
            if not ok:
                break
        cap.release()
    dest = os.path.join(out, name)
    cv2.imwrite(dest, picked)
    print("WROTE", dest, "total", total, "picked_idx", idx)
PY
ls -la /home/orangepi/board_eval/frames/
