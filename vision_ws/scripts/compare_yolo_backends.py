#!/usr/bin/env python3
"""Compare two Ultralytics backends on the same image set.

This is a laptop-side export consistency gate.  It does not validate RKNN,
NPU latency, thermals, or OrangePi deployment.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
from ultralytics import YOLO


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-model", required=True)
    parser.add_argument("--candidate-model", required=True)
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--min-match-iou", type=float, default=0.90)
    parser.add_argument("--max-confidence-diff", type=float, default=0.02)
    parser.add_argument("--max-box-diff-px", type=float, default=2.0)
    return parser.parse_args()


def image_paths(source):
    path = Path(source)
    if path.is_file():
        return [path]
    return sorted(
        item for item in path.rglob("*")
        if item.is_file() and item.suffix.lower() in IMAGE_SUFFIXES
    )


def detections(result):
    output = []
    if result.boxes is None:
        return output
    for cls_id, confidence, xyxy in zip(
            result.boxes.cls.cpu().tolist(),
            result.boxes.conf.cpu().tolist(),
            result.boxes.xyxy.cpu().tolist()):
        output.append({
            "class_id": int(cls_id),
            "confidence": float(confidence),
            "box": [float(value) for value in xyxy],
        })
    return output


def iou(left, right):
    lx1, ly1, lx2, ly2 = left
    rx1, ry1, rx2, ry2 = right
    overlap = max(0.0, min(lx2, rx2) - max(lx1, rx1)) * \
        max(0.0, min(ly2, ry2) - max(ly1, ry1))
    left_area = max(0.0, lx2 - lx1) * max(0.0, ly2 - ly1)
    right_area = max(0.0, rx2 - rx1) * max(0.0, ry2 - ry1)
    return overlap / max(left_area + right_area - overlap, 1.0e-9)


def main():
    args = parse_args()
    paths = image_paths(args.source)
    if not paths:
        raise RuntimeError("no images found under %s" % args.source)
    reference = YOLO(args.reference_model)
    candidate = YOLO(args.candidate_model)
    rows = []
    missing = 0
    extra = 0
    matched_ious = []
    confidence_diffs = []
    box_diffs = []

    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError("failed to read %s" % path)
        ref = detections(reference.predict(
            image, imgsz=args.imgsz, conf=args.conf,
            device=args.device, verbose=False)[0])
        cand = detections(candidate.predict(
            image, imgsz=args.imgsz, conf=args.conf,
            device=args.device, verbose=False)[0])
        pairs = []
        for ref_index, ref_detection in enumerate(ref):
            for cand_index, cand_detection in enumerate(cand):
                if ref_detection["class_id"] != cand_detection["class_id"]:
                    continue
                pairs.append((
                    iou(ref_detection["box"], cand_detection["box"]),
                    ref_index, cand_index,
                ))
        used_ref = set()
        used_cand = set()
        image_matches = []
        for overlap, ref_index, cand_index in sorted(pairs, reverse=True):
            if overlap < args.min_match_iou:
                continue
            if ref_index in used_ref or cand_index in used_cand:
                continue
            used_ref.add(ref_index)
            used_cand.add(cand_index)
            ref_detection = ref[ref_index]
            cand_detection = cand[cand_index]
            conf_diff = abs(
                ref_detection["confidence"] - cand_detection["confidence"])
            box_diff = max(
                abs(left - right) for left, right in
                zip(ref_detection["box"], cand_detection["box"]))
            matched_ious.append(overlap)
            confidence_diffs.append(conf_diff)
            box_diffs.append(box_diff)
            image_matches.append({
                "class_id": ref_detection["class_id"],
                "iou": overlap,
                "confidence_diff": conf_diff,
                "max_box_diff_px": box_diff,
            })
        image_missing = len(ref) - len(used_ref)
        image_extra = len(cand) - len(used_cand)
        missing += image_missing
        extra += image_extra
        rows.append({
            "image": str(path.resolve()),
            "reference_count": len(ref),
            "candidate_count": len(cand),
            "missing": image_missing,
            "extra": image_extra,
            "matches": image_matches,
        })

    max_confidence_diff = max(confidence_diffs, default=0.0)
    max_box_diff = max(box_diffs, default=0.0)
    minimum_iou = min(matched_ious, default=1.0)
    checks = {
        "no_missing": missing == 0,
        "no_extra": extra == 0,
        "minimum_iou": minimum_iou >= args.min_match_iou,
        "confidence_diff": max_confidence_diff <= args.max_confidence_diff,
        "box_diff": max_box_diff <= args.max_box_diff_px,
    }
    summary = {
        "scope": "laptop_export_consistency_only",
        "reference_model": str(Path(args.reference_model).resolve()),
        "candidate_model": str(Path(args.candidate_model).resolve()),
        "image_count": len(paths),
        "matched_count": len(matched_ious),
        "missing_count": missing,
        "extra_count": extra,
        "minimum_iou": minimum_iou,
        "mean_iou": sum(matched_ious) / len(matched_ious) if matched_ious else None,
        "max_confidence_diff": max_confidence_diff,
        "max_box_diff_px": max_box_diff,
        "checks": checks,
        "passed": all(checks.values()),
        "images": rows,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        key: summary[key] for key in (
            "scope", "image_count", "matched_count", "missing_count",
            "extra_count", "minimum_iou", "mean_iou",
            "max_confidence_diff", "max_box_diff_px", "passed",
        )
    }, ensure_ascii=False, indent=2))
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
