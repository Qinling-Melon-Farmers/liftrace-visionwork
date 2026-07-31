#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OrangePi-side RKNN evaluation on the complete v5 YOLO image corpus.

The evaluator deliberately uses raw image files without camera undistortion.
It supports one six-class RKNN or the legacy standard(4-class)+tank(1-class)
pair by applying per-model class offsets before computing metrics.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import board_realtime_rknn_viewer as viewer  # noqa: E402


CLASS_NAMES = viewer.NAMES
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def image_paths(dataset):
    image_root = Path(dataset) / "images"
    return sorted(
        p for p in image_root.rglob("*")
        if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
    )


def label_path(image_path, dataset):
    relative = image_path.relative_to(Path(dataset) / "images")
    return Path(dataset) / "labels" / relative.with_suffix(".txt")


def read_labels(path):
    result = []
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) != 5:
            continue
        cid, cx, cy, width, height = map(float, fields)
        result.append((int(cid), [cx, cy, width, height]))
    return result


def xywhn_to_xyxy(box, width, height):
    cx, cy, bw, bh = box
    return [
        (cx - bw / 2.0) * width,
        (cy - bh / 2.0) * height,
        (cx + bw / 2.0) * width,
        (cy + bh / 2.0) * height,
    ]


def box_iou(left, right):
    lx1, ly1, lx2, ly2 = left
    rx1, ry1, rx2, ry2 = right
    overlap = max(0.0, min(lx2, rx2) - max(lx1, rx1)) * max(0.0, min(ly2, ry2) - max(ly1, ry1))
    left_area = max(0.0, lx2 - lx1) * max(0.0, ly2 - ly1)
    right_area = max(0.0, rx2 - rx1) * max(0.0, ry2 - ry1)
    return overlap / max(left_area + right_area - overlap, 1.0e-12)


def compute_threshold_metrics(gt, predictions, conf_threshold, iou_threshold, class_count):
    tp = fp = fn = 0
    for key, gt_rows in gt.items():
        gt_by_class = {cid: [] for cid in range(class_count)}
        for cid, box in gt_rows:
            if 0 <= cid < class_count:
                gt_by_class[cid].append(box)
        pred_by_class = {cid: [] for cid in range(class_count)}
        for cid, confidence, box in predictions.get(key, []):
            if confidence >= conf_threshold and 0 <= cid < class_count:
                pred_by_class[cid].append((confidence, box))
        for cid in range(class_count):
            matched = set()
            for confidence, pred_box in sorted(pred_by_class[cid], reverse=True):
                best_iou, best_index = 0.0, None
                for index, gt_box in enumerate(gt_by_class[cid]):
                    if index in matched:
                        continue
                    overlap = box_iou(pred_box, gt_box)
                    if overlap > best_iou:
                        best_iou, best_index = overlap, index
                if best_index is not None and best_iou >= iou_threshold:
                    matched.add(best_index)
                    tp += 1
                else:
                    fp += 1
            fn += len(gt_by_class[cid]) - len(matched)
    return {"tp": tp, "fp": fp, "fn": fn}


def average_precision(gt, predictions, class_id, iou_threshold, class_count):
    total_gt = sum(1 for rows in gt.values() for cid, _ in rows if cid == class_id)
    if total_gt == 0:
        return None
    records = []
    for key, rows in predictions.items():
        for cid, confidence, box in rows:
            if cid == class_id:
                records.append((confidence, key, box))
    records.sort(key=lambda row: row[0], reverse=True)
    matched = {}
    tp, fp = [], []
    for confidence, key, pred_box in records:
        gt_boxes = [box for cid, box in gt.get(key, []) if cid == class_id]
        used = matched.setdefault(key, set())
        best_iou, best_index = 0.0, None
        for index, gt_box in enumerate(gt_boxes):
            if index in used:
                continue
            overlap = box_iou(pred_box, gt_box)
            if overlap > best_iou:
                best_iou, best_index = overlap, index
        if best_index is not None and best_iou >= iou_threshold:
            used.add(best_index)
            tp.append(1)
            fp.append(0)
        else:
            tp.append(0)
            fp.append(1)
    if not tp:
        return 0.0
    tp_cum = np.cumsum(tp)
    fp_cum = np.cumsum(fp)
    recall = tp_cum / max(float(total_gt), 1.0)
    precision = tp_cum / np.maximum(tp_cum + fp_cum, 1.0)
    recall_points = np.linspace(0.0, 1.0, 101)
    sampled = []
    for point in recall_points:
        sampled.append(float(np.max(precision[recall >= point])) if np.any(recall >= point) else 0.0)
    return float(np.mean(sampled))


def metrics_report(gt, predictions, conf_threshold, class_count):
    threshold = compute_threshold_metrics(gt, predictions, conf_threshold, 0.5, class_count)
    precision = threshold["tp"] / max(threshold["tp"] + threshold["fp"], 1)
    recall = threshold["tp"] / max(threshold["tp"] + threshold["fn"], 1)
    f1 = 2.0 * precision * recall / max(precision + recall, 1.0e-12)
    iou_thresholds = [0.5 + 0.05 * i for i in range(10)]
    per_class = []
    ap_rows = []
    for cid, name in enumerate(CLASS_NAMES[:class_count]):
        class_gt = {key: [(c, b) for c, b in rows if c == cid] for key, rows in gt.items()}
        class_predictions = {key: [(c, s, b) for c, s, b in rows if c == cid]
                             for key, rows in predictions.items()}
        class_threshold = compute_threshold_metrics(class_gt, class_predictions,
                                                    conf_threshold, 0.5, class_count)
        ap_values = [average_precision(gt, predictions, cid, threshold, class_count)
                     for threshold in iou_thresholds]
        ap_values = [value for value in ap_values if value is not None]
        ap50 = ap_values[0] if ap_values else None
        ap5095 = float(np.mean(ap_values)) if ap_values else None
        ap_rows.extend(ap_values)
        per_class.append({
            "class_id": cid,
            "class_name": name,
            "gt": sum(len(rows) for rows in class_gt.values()),
            "tp": class_threshold["tp"],
            "fp": class_threshold["fp"],
            "fn": class_threshold["fn"],
            "precision": class_threshold["tp"] / max(class_threshold["tp"] + class_threshold["fp"], 1),
            "recall": class_threshold["tp"] / max(class_threshold["tp"] + class_threshold["fn"], 1),
            "ap50": ap50,
            "map50_95": ap5095,
        })
    return {
        "conf_threshold": conf_threshold,
        "iou_threshold": 0.5,
        "tp": threshold["tp"],
        "fp": threshold["fp"],
        "fn": threshold["fn"],
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "map50": float(np.mean([row["ap50"] for row in per_class if row["ap50"] is not None])) if per_class else 0.0,
        "map50_95": float(np.mean(ap_rows)) if ap_rows else 0.0,
        "per_class": per_class,
    }


def infer_image(image, runtimes, offsets, conf_threshold):
    predictions = []
    tensor, ratio, left, top = viewer.letterbox(image)
    for runtime, offset in zip(runtimes, offsets):
        outputs = runtime.inference(inputs=[tensor])
        raw = np.asarray(outputs[0] if isinstance(outputs, (list, tuple)) else outputs)
        squeezed = np.squeeze(raw)
        if squeezed.ndim == 2 and squeezed.shape[0] < squeezed.shape[1]:
            class_count = max(1, squeezed.shape[0] - 4)
        else:
            class_count = max(1, squeezed.shape[1] - 4)
        names = CLASS_NAMES[:class_count] if class_count > 1 else ["tank"]
        dets = viewer.decode(outputs, ratio, left, top, names,
                             conf_thres=max(0.0001, min(conf_threshold, 0.001)))
        for x1, y1, x2, y2, cid, score in dets:
            predictions.append((cid + offset, score, [x1, y1, x2, y2]))
    return predictions


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--model", action="append", required=True)
    parser.add_argument("--offset", action="append", type=int, default=[])
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--output", required=True)
    parser.add_argument("--predictions", help="optional prediction JSON")
    return parser.parse_args()


def main():
    args = parse_args()
    paths = image_paths(args.dataset)
    if not paths:
        raise RuntimeError("no images under " + args.dataset)
    offsets = list(args.offset)
    if not offsets:
        offsets = [0] * len(args.model)
    if len(offsets) != len(args.model):
        raise RuntimeError("provide one --offset per --model")
    runtimes = []
    for model in args.model:
        runtime = viewer.load_runtime(model)
        if runtime is None:
            return 1
        runtimes.append(runtime)
    gt = {}
    predictions = {}
    timings = []
    started = time.perf_counter()
    try:
        for index, image_path in enumerate(paths, 1):
            image = cv2.imread(str(image_path))
            if image is None:
                raise RuntimeError("failed to read " + str(image_path))
            key = str(image_path.relative_to(Path(args.dataset) / "images"))
            height, width = image.shape[:2]
            gt[key] = [(cid, xywhn_to_xyxy(box, width, height))
                       for cid, box in read_labels(label_path(image_path, args.dataset))]
            t0 = time.perf_counter()
            predictions[key] = infer_image(image, runtimes, offsets, args.conf)
            timings.append((time.perf_counter() - t0) * 1000.0)
            if index % 100 == 0 or index == len(paths):
                print("PROGRESS %d/%d" % (index, len(paths)), flush=True)
    finally:
        for runtime in runtimes:
            runtime.release()
    report = {
        "dataset": str(Path(args.dataset).resolve()),
        "image_count": len(paths),
        "models": [str(Path(model).resolve()) for model in args.model],
        "class_offsets": offsets,
        "preprocess": {"letterbox": viewer.IMGSZ, "rgb": True, "float32_0_1": True,
                       "undistortion": False},
        "performance_ms": {
            "p50": float(np.percentile(timings, 50)),
            "p95": float(np.percentile(timings, 95)),
            "mean": float(np.mean(timings)),
            "effective_fps_p50": 1000.0 / max(float(np.percentile(timings, 50)), 1.0e-9),
        },
        "metrics": metrics_report(gt, predictions, args.conf, len(CLASS_NAMES)),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.predictions:
        Path(args.predictions).write_text(json.dumps(predictions, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report["metrics"], ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
