#!/usr/bin/env python3
"""本地开发机（RTX）PyTorch 推理基准，与板端 RKNN 结果对比。

用法: python3 local_pt_bench.py --video <mp4> --model <pt> [--tank <pt>] [--max-frames N]
"""
import argparse
import json
import statistics
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def bench_model(video, model_path, tank_path=None, max_frames=300, warmup=5):
    model = YOLO(model_path, task="detect")
    tank = YOLO(tank_path, task="detect") if tank_path else None

    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {video}")
    fps_src = cap.get(cv2.CAP_PROP_FPS)

    infer_times = []
    total_times = []
    det_counts = []
    frame_idx = 0
    measured = 0

    while measured < max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1

        t0 = time.perf_counter()
        r = model.predict(frame, verbose=False, conf=0.25)
        t_infer = time.perf_counter() - t0

        det = 0
        if r and r[0].boxes is not None:
            det += len(r[0].boxes)
        if tank is not None:
            t0t = time.perf_counter()
            rt = tank.predict(frame, verbose=False, conf=0.25)
            t_infer += time.perf_counter() - t0t
            if rt and rt[0].boxes is not None:
                det += len(rt[0].boxes)

        t_total = time.perf_counter() - t0
        if frame_idx > warmup:
            infer_times.append(t_infer * 1000)
            total_times.append(t_total * 1000)
            det_counts.append(det)
            measured += 1
        if measured % 50 == 0:
            print(f"  progress {measured}/{max_frames}")

    cap.release()

    def stats(xs):
        xs = sorted(xs)
        n = len(xs)
        return {
            "p50": xs[n // 2], "p95": xs[int(n * 0.95) - 1],
            "mean": sum(xs) / n,
        }

    return {
        "model": model_path,
        "tank_model": tank_path,
        "frames_measured": measured,
        "source_fps": fps_src,
        "infer_ms": stats(infer_times),
        "total_ms": stats(total_times),
        "median_detections": statistics.median(det_counts) if det_counts else 0,
        "throughput_fps": 1000.0 / stats(total_times)["p50"] if total_times else 0,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--video", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--tank", default=None)
    p.add_argument("--max-frames", type=int, default=300)
    p.add_argument("--warmup", type=int, default=5)
    p.add_argument("--json", default=None)
    args = p.parse_args()

    print(f"bench {args.model}" + (f" + {args.tank}" if args.tank else ""))
    result = bench_model(args.video, args.model, args.tank,
                         args.max_frames, args.warmup)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.json:
        Path(args.json).write_text(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
