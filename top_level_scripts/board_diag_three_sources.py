#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三输入源对照诊断: 同一 fp32 模型+同一 letterbox 预处理, 对比
图片目录 / 视频抽帧 / 相机实时帧(raw 与去畸变) 的逐类分数分布,
用于定位实时链路检出弱的主因。只读+相机采集, 无执行机构。"""
import glob
import json
import time

import cv2
import numpy as np
from rknnlite.api import RKNNLite

BASE = "/home/orangepi/liftrace_board_eval_20260716"
MODEL = BASE + "/models/merged_standard_fp32.rknn"
NAMES = ["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"]
IMGSZ = 640

CAL_W, CAL_H = 1920, 1080
K = np.array([[581.2568, 0.0, 1043.5], [0.0, 580.9240, 513.0979], [0.0, 0.0, 1.0]])
D = np.array([0.0349, -0.0426, 0.0, 0.0, 0.0076])


def letterbox(frame):
    h, w = frame.shape[:2]
    r = min(IMGSZ / w, IMGSZ / h)
    nw, nh = round(w * r), round(h * r)
    canvas = np.full((IMGSZ, IMGSZ, 3), 114, dtype=np.uint8)
    top, left = (IMGSZ - nh) // 2, (IMGSZ - nw) // 2
    canvas[top:top + nh, left:left + nw] = cv2.resize(frame, (nw, nh))
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return (rgb.astype(np.float32) / 255.0)[None, ...]


def top_scores(rt, frame):
    out = rt.inference(inputs=[letterbox(frame)])
    arr = np.squeeze(np.asarray(out[0]))
    if arr.ndim == 2 and arr.shape[0] < arr.shape[1]:
        arr = arr.T
    scores = arr[:, 4:4 + len(NAMES)]
    per_cls = scores.max(axis=0)
    return {NAMES[i]: round(float(per_cls[i]), 3) for i in range(len(NAMES))}, \
        int((scores.max(axis=1) >= 0.5).sum())


def brightness_stats(frame):
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    lap = cv2.Laplacian(g, cv2.CV_64F).var()  # 清晰度(模糊越低)
    return {"mean": round(float(g.mean()), 1), "std": round(float(g.std()), 1),
            "sharpness_lapvar": round(float(lap), 1)}


def main():
    rt = RKNNLite()
    assert rt.load_rknn(MODEL) == 0 and rt.init_runtime() == 0
    report = {}

    # 1) 图片目录
    imgs = sorted(glob.glob(BASE + "/images/*.jpg"))
    rows = []
    for p in imgs:
        f = cv2.imread(p)
        s, n = top_scores(rt, f)
        rows.append({"file": p.rsplit("/", 1)[-1], "hw": list(f.shape[:2]),
                     "img_stats": brightness_stats(f), "above05": n, "cls_max": s})
    report["dir_images"] = rows

    # 2) 视频抽帧(等间隔 6 帧)
    cap = cv2.VideoCapture(BASE + "/real_target.mp4")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    rows = []
    for idx in np.linspace(0, total - 1, 6).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ok, f = cap.read()
        if not ok:
            continue
        s, n = top_scores(rt, f)
        rows.append({"frame": int(idx), "hw": list(f.shape[:2]),
                     "img_stats": brightness_stats(f), "above05": n, "cls_max": s})
    cap.release()
    report["video_frames"] = rows

    # 3) 相机实时帧: raw 与去畸变各测同一帧
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAL_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAL_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    for _ in range(10):  # 丢弃自动曝光收敛前的帧
        cap.read()
        time.sleep(0.05)
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (CAL_W, CAL_H), 0)
    m1, m2 = cv2.initUndistortRectifyMap(K, D, None, newK, (CAL_W, CAL_H), cv2.CV_16SC2)
    rows = []
    for i in range(4):
        ok, f = cap.read()
        if not ok:
            continue
        cv2.imwrite(BASE + "/diag_cam_%d.jpg" % i, f)
        s_raw, n_raw = top_scores(rt, f)
        rect = cv2.remap(f, m1, m2, cv2.INTER_LINEAR)
        s_rec, n_rec = top_scores(rt, rect)
        rows.append({"idx": i, "img_stats": brightness_stats(f),
                     "raw": {"above05": n_raw, "cls_max": s_raw},
                     "rectified": {"above05": n_rec, "cls_max": s_rec}})
        time.sleep(0.2)
    cap.release()
    report["camera_frames"] = rows
    rt.release()

    out = BASE + "/diag_three_sources.json"
    with open(out, "w") as fh:
        json.dump(report, fh, indent=2)
    print(json.dumps(report, indent=2))
    print("SAVED", out)


if __name__ == "__main__":
    main()
