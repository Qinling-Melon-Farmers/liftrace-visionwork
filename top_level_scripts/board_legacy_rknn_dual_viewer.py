#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""旧 standard+tank 双 RKNN 视频回放器。

旧机载视觉工程的可见源码是两个 Ultralytics 模型：

* standard: bridge/panzer/pillbox/tent
* tank: tank

仓库没有旧 RKNN 推理源码，只有板端转换后的二进制。因此本工具只复用
旧链确定的模型拆分、类别顺序、三帧 standard 语义投票和 tank 中心候选，
不把它描述成原始隐藏 RKNN runner 的逐行复刻。

视频文件按原始像素回放，不做相机去畸变；这与离线视频评测口径一致。
默认 ``--conf 0.001`` 是为了保留旧 RKNN 输出中极低分的诊断候选。正式
P/R/mAP 仍按 ``conf=0.25`` 计算，低分候选不能视为有效识别。

板端实测（2026-08-10, toolkit 2.3.2 转出的旧链模型）：best-rk3588.rknn /
tank-rk3588.rknn 的输入要求 uint8 0-255（RGB），若按新链的 float32 0-1
喂入，输出全为 ~0.001 噪声、零检测。因此回放评测需加 ``--u8-input``。
"""
import argparse
import json
import os
import sys
import time
from collections import Counter, deque

import cv2
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
import board_realtime_rknn_viewer as viewer  # noqa: E402

try:
    from rknnlite.api import RKNNLite
except ImportError:  # 本机契约测试不需要板端运行时
    RKNNLite = None


DEFAULT_STANDARD_MODEL = (
    "/home/orangepi/Visual/src/yolov5_detect/"
    "best_rknn_model/best-rk3588.rknn"
)
DEFAULT_TANK_MODEL = (
    "/home/orangepi/Visual/src/yolov5_detect/"
    "tank_rknn_model/best-rk3588.rknn"
)
STANDARD_NAMES = ["bridge", "panzer", "pillbox", "tent"]
TANK_NAMES = ["tank"]
GLOBAL_NAMES = viewer.NAMES
COLORS = viewer.COLORS


def combine_detections(standard_dets, tank_dets):
    """把旧双模型输出合并成当前六分类坐标语义。"""
    result = list(standard_dets)
    result.extend((x1, y1, x2, y2, cid + len(STANDARD_NAMES), score)
                  for x1, y1, x2, y2, cid, score in tank_dets)
    return result


def standard_vote(history):
    """复用旧 yolo_detect.py 的三帧多数语义；不足三帧不产生结果。"""
    if len(history) < 3:
        return ""
    counts = Counter(cid for frame in history for cid in frame)
    if not counts:
        return "Nothing"
    return STANDARD_NAMES[max(counts, key=lambda cid: (counts[cid], -cid))]


def load_runtime(model_path):
    if RKNNLite is None:
        raise RuntimeError("rknnlite is not installed")
    runtime = RKNNLite()
    if runtime.load_rknn(model_path) != 0:
        raise RuntimeError("load RKNN failed: " + model_path)
    if runtime.init_runtime() != 0:
        raise RuntimeError("init RKNN runtime failed: " + model_path)
    print("MODEL_READY", model_path, flush=True)
    return runtime


def percentile(values, p):
    return float(np.percentile(np.asarray(values, dtype=np.float64), p)) if values else None


def letterbox_u8(frame):
    """旧链 RKNN 模型输入：letterbox 640 + RGB uint8 0-255（不加 /255）。"""
    h, w = frame.shape[:2]
    r = min(viewer.IMGSZ / w, viewer.IMGSZ / h)
    nw, nh = round(w * r), round(h * r)
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((viewer.IMGSZ, viewer.IMGSZ, 3), 114, dtype=np.uint8)
    top, left = (viewer.IMGSZ - nh) // 2, (viewer.IMGSZ - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return rgb[None, ...], r, left, top


def infer_model(runtime, frame, names, conf_threshold, u8_input=False):
    if u8_input:
        tensor, ratio, left, top = letterbox_u8(frame)
    else:
        tensor, ratio, left, top = viewer.letterbox(frame)
    started = time.perf_counter()
    outputs = runtime.inference(inputs=[tensor])
    infer_ms = (time.perf_counter() - started) * 1000.0
    post_started = time.perf_counter()
    detections = viewer.decode(
        outputs, ratio, left, top, class_names=names,
        conf_thres=conf_threshold, iou_thres=viewer.IOU_THRES,
    )
    post_ms = (time.perf_counter() - post_started) * 1000.0
    return detections, infer_ms, post_ms


def annotate(frame, detections, selected_label, timing, fps):
    for x1, y1, x2, y2, cid, score in detections:
        color = COLORS[cid % len(COLORS)]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame, "%s %.4f" % (GLOBAL_NAMES[cid], score),
            (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2,
        )
    cv2.putText(
        frame,
        "legacy dual  fps %.1f  total %.0f ms  std %.0f  tank %.0f  dets %d"
        % (fps, timing["total_ms"], timing["standard_infer_ms"],
           timing["tank_infer_ms"], len(detections)),
        (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2,
    )
    cv2.putText(
        frame, "standard_vote(3 samples): %s" % (selected_label or "pending"),
        (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
    )
    cv2.putText(
        frame, "raw video; no undistortion; tank model merged as class 4",
        (12, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1,
    )


def run_video(args):
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print("FATAL: video open failed", args.video, flush=True)
        return 1
    # OpenCV/GStreamer on the board may apply MP4 rotation metadata. Raw pixel
    # coordinates are required for a reproducible detector comparison.
    orientation_meta = float(cap.get(getattr(cv2, "CAP_PROP_ORIENTATION_META", 48)))
    orientation_before = float(cap.get(getattr(cv2, "CAP_PROP_ORIENTATION_AUTO", 49)))
    cap.set(getattr(cv2, "CAP_PROP_ORIENTATION_AUTO", 49), 0)
    orientation_after = float(cap.get(getattr(cv2, "CAP_PROP_ORIENTATION_AUTO", 49)))
    source_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)

    standard_rt = load_runtime(args.standard_model)
    tank_rt = load_runtime(args.tank_model)
    writer = None
    width = height = 0
    output_size = None
    frame_idx = -1
    sampled = measured = output_frames = 0
    started = time.perf_counter()
    last = started
    fps_ema = None
    history = deque(maxlen=3)
    timings = {key: [] for key in (
        "standard_infer_ms", "tank_infer_ms", "standard_post_ms",
        "tank_post_ms", "total_ms",
    )}
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
                    output_dir = os.path.dirname(args.output_video)
                    if output_dir:
                        os.makedirs(output_dir, exist_ok=True)
                    output_fps = source_fps / max(1, args.stride) if source_fps else 5.0
                    output_width = min(width, max(1, args.output_width))
                    output_height = int(round(height * output_width / width))
                    output_size = (output_width, output_height)
                    writer = cv2.VideoWriter(
                        args.output_video, cv2.VideoWriter_fourcc(*"mp4v"),
                        output_fps, output_size,
                    )
                    if not writer.isOpened():
                        raise RuntimeError("video writer open failed: " + args.output_video)
                    print("OUTPUT_VIDEO_READY %s %.3f FPS %dx%d" %
                          (args.output_video, output_fps, output_width, output_height),
                          flush=True)
            frame_idx += 1
            if frame_idx % max(1, args.stride) != 0:
                continue
            sampled += 1
            total_started = time.perf_counter()
            standard_dets, std_infer, std_post = infer_model(
                standard_rt, frame, STANDARD_NAMES, args.conf, u8_input=args.u8_input,
            )
            tank_dets, tank_infer, tank_post = infer_model(
                tank_rt, frame, TANK_NAMES, args.conf, u8_input=args.u8_input,
            )
            detections = combine_detections(standard_dets, tank_dets)
            total_ms = (time.perf_counter() - total_started) * 1000.0
            history.append([d[4] for d in standard_dets])
            selected = standard_vote(history)
            if sampled <= args.warmup:
                continue
            measured += 1
            timing = {
                "standard_infer_ms": std_infer,
                "tank_infer_ms": tank_infer,
                "standard_post_ms": std_post,
                "tank_post_ms": tank_post,
                "total_ms": total_ms,
            }
            for key, value in timing.items():
                timings[key].append(value)
            det_counts.append(len(detections))
            max_confs.append(max((d[5] for d in detections), default=0.0))
            for detection in detections:
                class_hist[detection[4]] += 1
            if selected:
                standard_selected_hist[selected] += 1
            if first_detections is None:
                first_detections = [
                    {"xyxy": list(d[:4]), "class_id": d[4],
                     "class_name": GLOBAL_NAMES[d[4]], "confidence": d[5]}
                    for d in detections
                ]
            now = time.perf_counter()
            instant_fps = 1.0 / max(now - last, 1.0e-6)
            last = now
            fps_ema = instant_fps if fps_ema is None else fps_ema * 0.9 + instant_fps * 0.1
            if writer is not None or args.show:
                rendered = frame.copy()
                annotate(rendered, detections, selected, timing, fps_ema)
                if writer is not None:
                    ow, oh = output_size
                    if rendered.shape[1] != ow or rendered.shape[0] != oh:
                        rendered = cv2.resize(rendered, (ow, oh), interpolation=cv2.INTER_AREA)
                    writer.write(rendered)
                    output_frames += 1
                if args.show:
                    cv2.imshow("legacy dual standard+tank RKNN", rendered)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
            if measured % 100 == 0:
                print("STATS measured=%d total_p50_so_far=%.1f std=%.1f tank=%.1f dets=%d" %
                      (measured, percentile(timings["total_ms"], 50),
                       percentile(timings["standard_infer_ms"], 50),
                       percentile(timings["tank_infer_ms"], 50), len(detections)),
                      flush=True)
    finally:
        cap.release()
        standard_rt.release()
        tank_rt.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = time.perf_counter() - started
    report = {
        "model_family": "legacy_standard_plus_tank_rknn",
        "source": args.video,
        "models": {
            "standard": args.standard_model,
            "tank": args.tank_model,
        },
        "legacy_semantics": {
            "standard_classes": STANDARD_NAMES,
            "tank_class_offset": len(STANDARD_NAMES),
            "standard_vote_window": 3,
            "standard_vote_unit": "sampled frames; stride changes it from original source-frame window",
            "tank_control": "offline replay always runs tank; original ROS chain gates it with /detect/tank_control",
            "tank_output": "merged detection box; original ROS chain sends its box center to /visual/service",
            "rknn_source_status": "legacy RKNN runner source is absent; binary behavior is evaluated with the old split and mapping",
        },
        "preprocess": {
            "raw_video_undistortion": False,
            "letterbox": 640,
            "rgb": True,
            "u8_0_255_input": args.u8_input,
        },
        "threshold": {"video_display_conf": args.conf, "official_metric_conf": 0.25, "iou": 0.5},
        "input": {"width": width, "height": height, "fps": source_fps},
        "video_orientation": {"metadata_degrees": orientation_meta,
                               "auto_before": orientation_before,
                               "auto_after": orientation_after},
        "sampling": {"stride": args.stride, "warmup": args.warmup,
                     "frames_sampled": sampled, "frames_measured": measured,
                     "output_video": args.output_video, "output_frames": output_frames},
        "metrics_ms": {
            key: {"p50": percentile(values, 50), "p95": percentile(values, 95),
                  "mean": float(np.mean(values)) if values else None}
            for key, values in timings.items()
        },
        "derived": {
            "throughput_from_total_p50_fps":
                1000.0 / percentile(timings["total_ms"], 50) if timings["total_ms"] else None,
            "wall_fps": measured / elapsed if elapsed > 0 else None,
            "median_detections": float(np.median(det_counts)) if det_counts else None,
            "median_max_confidence": float(np.median(max_confs)) if max_confs else None,
            "class_hist_total": class_hist,
            "standard_selected_hist": dict(standard_selected_hist),
            "first_detections": first_detections or [],
        },
    }
    if args.json:
        output_dir = os.path.dirname(args.json)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True)
    parser.add_argument("--standard-model", default=DEFAULT_STANDARD_MODEL)
    parser.add_argument("--tank-model", default=DEFAULT_TANK_MODEL)
    parser.add_argument("--json")
    parser.add_argument("--output-video")
    parser.add_argument("--output-width", type=int, default=1280)
    parser.add_argument("--stride", type=int, default=12)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--conf", type=float, default=0.001)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--u8-input", action="store_true",
                        help="旧链 RKNN 输入为 uint8 0-255（板端实测要求，否则零检测）")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    args.show = args.show and not args.no_window
    return run_video(args)


if __name__ == "__main__":
    sys.exit(main())
