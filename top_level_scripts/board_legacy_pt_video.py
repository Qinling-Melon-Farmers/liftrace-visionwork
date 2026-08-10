#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""旧链 PyTorch 路视频回放评测（ultralytics best.pt + tank.pt, CPU）。

复刻旧机载 yolo_detect.py 的链语义：
* standard 模型 4 类（bridge/panzer/pillbox/tent）
* tank 模型单独输出 tank（旧链由 /detect/tank_control 门控，离线回放恒跑）
* standard 三帧多数投票输出语义标签
* 合并 tank 为六分类全局语义的第 4 类（与 RKNN 双模型评测一致）

仅推理/回放，不启动 ROS、不做任何执行机构操作。
"""
import argparse
import json
import os
import sys
import time
from collections import Counter, deque

import cv2
import numpy as np
from ultralytics import YOLO

GLOBAL_NAMES = ["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"]
STANDARD_NAMES = ["bridge", "panzer", "pillbox", "tent"]
TANK_NAMES = ["tank"]
COLORS = [(255, 128, 0), (0, 200, 255), (0, 255, 0), (255, 0, 255), (0, 128, 255), (0, 0, 255)]
IMGSZ = 640


def percentile(values, p):
    return float(np.percentile(np.asarray(values, dtype=np.float64), p)) if values else None


def standard_vote(history):
    if len(history) < 3:
        return ""
    counts = Counter(cid for frame in history for cid in frame)
    if not counts:
        return "Nothing"
    return STANDARD_NAMES[max(counts, key=lambda cid: (counts[cid], -cid))]


def infer_model(model, frame, conf, imgsz):
    started = time.perf_counter()
    results = model.predict(frame, imgsz=imgsz, conf=conf, device="cpu", verbose=False)
    infer_ms = (time.perf_counter() - started) * 1000.0
    post_started = time.perf_counter()
    dets = []
    for result in results:
        if result.boxes is None or len(result.boxes) == 0:
            continue
        xyxy = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        cls_ids = result.boxes.cls.cpu().numpy().astype(int)
        for box, c, cid in zip(xyxy, confs, cls_ids):
            x1, y1, x2, y2 = map(int, box)
            dets.append((x1, y1, x2, y2, int(cid), float(c)))
    post_ms = (time.perf_counter() - post_started) * 1000.0
    return dets, infer_ms, post_ms


def annotate(frame, detections, selected_label, timing, fps):
    for x1, y1, x2, y2, cid, score in detections:
        color = COLORS[cid % len(COLORS)]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, "%s %.3f" % (GLOBAL_NAMES[cid], score), (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    cv2.putText(frame, "legacy pytorch  fps %.1f  total %.0f ms  std %.0f  tank %.0f  dets %d" %
                (fps, timing["total_ms"], timing["standard_infer_ms"],
                 timing["tank_infer_ms"], len(detections)),
                (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    cv2.putText(frame, "standard_vote(3 frames): %s" % (selected_label or "pending"),
                (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, "raw video; torch cpu; tank merged as class 4",
                (12, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True)
    parser.add_argument("--standard-model", default="/home/orangepi/Visual/src/yolov5_detect/best.pt")
    parser.add_argument("--tank-model", default="/home/orangepi/Visual/src/yolov5_detect/tank.pt")
    parser.add_argument("--json")
    parser.add_argument("--output-video")
    parser.add_argument("--output-width", type=int, default=1280)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=IMGSZ)
    parser.add_argument("--max-frames", type=int, default=0, help="最多处理帧数, 0=全部")
    parser.add_argument("--warmup", type=int, default=5)
    args = parser.parse_args(argv)

    load_started = time.perf_counter()
    standard_model = YOLO(args.standard_model, task="detect")
    tank_model = YOLO(args.tank_model, task="detect")
    load_ms = (time.perf_counter() - load_started) * 1000.0
    print("MODEL_READY std=%s tank=%s load_ms=%.0f" %
          (args.standard_model, args.tank_model, load_ms), flush=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print("FATAL: video open failed", args.video, flush=True)
        return 1
    orientation_meta = float(cap.get(getattr(cv2, "CAP_PROP_ORIENTATION_META", 48)))
    cap.set(getattr(cv2, "CAP_PROP_ORIENTATION_AUTO", 49), 0)
    source_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)

    writer = None
    output_size = None
    width = height = 0
    frame_idx = -1
    sampled = measured = output_frames = 0
    started = time.perf_counter()
    last = started
    fps_ema = None
    history = deque(maxlen=3)
    timings = {key: [] for key in ("standard_infer_ms", "tank_infer_ms",
                                   "standard_post_ms", "tank_post_ms", "total_ms")}
    class_hist = [0] * len(GLOBAL_NAMES)
    standard_selected_hist = Counter()
    det_counts = []
    max_confs = []
    first_detections = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if width == 0:
                height, width = frame.shape[:2]
                if args.output_video:
                    out_dir = os.path.dirname(args.output_video)
                    if out_dir:
                        os.makedirs(out_dir, exist_ok=True)
                    out_fps = source_fps if source_fps > 0 else 5.0
                    out_width = min(width, max(1, args.output_width))
                    out_height = int(round(height * out_width / width))
                    output_size = (out_width, out_height)
                    writer = cv2.VideoWriter(args.output_video,
                                             cv2.VideoWriter_fourcc(*"mp4v"),
                                             out_fps, output_size)
                    if not writer.isOpened():
                        raise RuntimeError("video writer open failed: " + args.output_video)
                    print("OUTPUT_VIDEO_READY %s %.3f FPS %dx%d" %
                          (args.output_video, out_fps, out_width, out_height), flush=True)
            frame_idx += 1
            if 0 < args.max_frames <= frame_idx:
                break
            sampled += 1
            total_started = time.perf_counter()
            std_dets, std_infer, std_post = infer_model(standard_model, frame, args.conf, args.imgsz)
            tank_dets, tank_infer, tank_post = infer_model(tank_model, frame, args.conf, args.imgsz)
            detections = list(std_dets) + [(x1, y1, x2, y2, cid + len(STANDARD_NAMES), score)
                                           for x1, y1, x2, y2, cid, score in tank_dets]
            total_ms = (time.perf_counter() - total_started) * 1000.0
            history.append([d[4] for d in std_dets])
            selected = standard_vote(history)
            if sampled <= args.warmup:
                continue
            measured += 1
            timing = {"standard_infer_ms": std_infer, "tank_infer_ms": tank_infer,
                      "standard_post_ms": std_post, "tank_post_ms": tank_post,
                      "total_ms": total_ms}
            for key, value in timing.items():
                timings[key].append(value)
            det_counts.append(len(detections))
            max_confs.append(max((d[5] for d in detections), default=0.0))
            for d in detections:
                class_hist[d[4]] += 1
            if selected:
                standard_selected_hist[selected] += 1
            if first_detections is None:
                first_detections = [{"xyxy": list(d[:4]), "class_id": d[4],
                                     "class_name": GLOBAL_NAMES[d[4]], "confidence": d[5]}
                                    for d in detections]
            now = time.perf_counter()
            instant_fps = 1.0 / max(now - last, 1.0e-6)
            last = now
            fps_ema = instant_fps if fps_ema is None else fps_ema * 0.9 + instant_fps * 0.1
            if writer is not None:
                rendered = frame.copy()
                annotate(rendered, detections, selected, timing, fps_ema)
                ow, oh = output_size
                if rendered.shape[1] != ow or rendered.shape[0] != oh:
                    rendered = cv2.resize(rendered, (ow, oh), interpolation=cv2.INTER_AREA)
                writer.write(rendered)
                output_frames += 1
            if measured % 50 == 0:
                print("STATS measured=%d total_p50_so_far=%.1f std=%.1f tank=%.1f dets=%d" %
                      (measured, percentile(timings["total_ms"], 50),
                       percentile(timings["standard_infer_ms"], 50),
                       percentile(timings["tank_infer_ms"], 50), len(detections)), flush=True)
    finally:
        cap.release()
        if writer is not None:
            writer.release()

    elapsed = time.perf_counter() - started
    report = {
        "model_family": "legacy_standard_plus_tank_pytorch_cpu",
        "source": args.video,
        "models": {"standard": args.standard_model, "tank": args.tank_model},
        "engine": {"torch_cpu": True, "imgsz": args.imgsz, "conf": args.conf},
        "legacy_semantics": {
            "standard_classes": STANDARD_NAMES,
            "tank_class_offset": len(STANDARD_NAMES),
            "standard_vote_window": 3,
            "standard_vote_unit": "processed frames",
            "tank_control": "offline replay always runs tank; original ROS chain gates it with /detect/tank_control",
        },
        "model_load_ms": load_ms,
        "input": {"width": width, "height": height, "fps": source_fps},
        "video_orientation": {"metadata_degrees": orientation_meta},
        "sampling": {"stride": 1, "warmup": args.warmup, "frames_sampled": sampled,
                     "frames_measured": measured, "output_video": args.output_video,
                     "output_frames": output_frames, "max_frames": args.max_frames},
        "metrics_ms": {key: {"p50": percentile(values, 50), "p95": percentile(values, 95),
                             "mean": float(np.mean(values)) if values else None}
                       for key, values in timings.items()},
        "derived": {
            "throughput_from_total_p50_fps": 1000.0 / percentile(timings["total_ms"], 50)
            if timings["total_ms"] else None,
            "wall_fps": measured / elapsed if elapsed > 0 else None,
            "median_detections": float(np.median(det_counts)) if det_counts else None,
            "median_max_confidence": float(np.median(max_confs)) if max_confs else None,
            "class_hist_total": class_hist,
            "standard_selected_hist": dict(standard_selected_hist),
            "first_detections": first_detections or [],
        },
    }
    if args.json:
        out_dir = os.path.dirname(args.json)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
