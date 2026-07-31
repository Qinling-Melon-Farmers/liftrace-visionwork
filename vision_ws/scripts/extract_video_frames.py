#!/usr/bin/env python3
"""Extract full-rate or sampled video frames for manual labeling."""

import argparse
import csv
import json
from pathlib import Path

import cv2


def parse_args():
    parser = argparse.ArgumentParser(description="Extract all video frames to images")
    parser.add_argument("--input", required=True, help="source video path")
    parser.add_argument("--output-dir", required=True, help="output directory root")
    parser.add_argument(
        "--prefix",
        default="frame",
        help="output filename prefix, e.g. redcross",
    )
    parser.add_argument(
        "--format",
        choices=("png", "jpg"),
        default="png",
        help="image format for extracted frames",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=100,
        help="JPEG quality when --format=jpg",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="save one frame every N source frames",
    )
    parser.add_argument(
        "--target-fps",
        type=float,
        default=0.0,
        help="target extracted FPS; if set, derives frame-step from source FPS",
    )
    parser.add_argument(
        "--legacy-layout",
        action="store_true",
        help="write frames/ + manifest.csv like existing video_sources assets",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.frame_step < 1:
        raise ValueError("--frame-step must be >= 1")
    if args.target_fps < 0:
        raise ValueError("--target-fps must be >= 0")
    if args.target_fps > 0 and args.frame_step != 1:
        raise ValueError("use either --target-fps or --frame-step, not both")

    src = Path(args.input).resolve()
    out_root = Path(args.output_dir).resolve()
    images_dir = out_root / "frames" if args.legacy_layout else out_root / f"images_raw_{args.format}"
    images_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        raise RuntimeError(f"failed to open video: {src}")

    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_step = args.frame_step
    if args.target_fps > 0:
        if fps <= 0:
            raise RuntimeError("source FPS unavailable, cannot derive frame-step from --target-fps")
        frame_step = max(1, round(fps / args.target_fps))
    effective_sample_fps = (fps / frame_step) if fps > 0 else None

    if args.format == "jpg":
        write_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
    else:
        write_params = []

    saved = 0
    source_frame_index = 0
    manifest_rows = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if source_frame_index % frame_step == 0:
            timestamp_sec = (source_frame_index / fps) if fps > 0 else 0.0
            if args.legacy_layout:
                filename = f"{args.prefix}_f{source_frame_index:06d}_t{timestamp_sec:07.3f}.{args.format}"
            else:
                filename = f"{args.prefix}_{saved:06d}.{args.format}"
            target = images_dir / filename
            if not cv2.imwrite(str(target), frame, write_params):
                raise RuntimeError(f"failed to write frame: {target}")
            manifest_rows.append(
                {
                    "source_video": str(src),
                    "frame_index": source_frame_index,
                    "timestamp_sec": f"{timestamp_sec:.3f}",
                    "output_image": str(target),
                }
            )
            saved += 1
        source_frame_index += 1

    cap.release()

    if args.legacy_layout:
        with (out_root / "manifest.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["source_video", "frame_index", "timestamp_sec", "output_image"],
            )
            writer.writeheader()
            writer.writerows(manifest_rows)

    meta = {
        "source_video": str(src),
        "output_root": str(out_root),
        "images_dir": str(images_dir),
        "saved_frames": saved,
        "frame_count_reported": frame_count,
        "source_fps": fps,
        "width": width,
        "height": height,
        "duration_sec": (frame_count / fps) if fps > 0 else None,
        "image_format": args.format,
        "filename_prefix": args.prefix,
        "frame_step": frame_step,
        "target_fps_requested": args.target_fps if args.target_fps > 0 else None,
        "effective_sample_fps": effective_sample_fps,
        "legacy_layout": args.legacy_layout,
        "note": (
            "sampled extraction for manual labeling"
            if frame_step > 1
            else "full native-rate extraction for manual labeling"
        ),
    }
    (out_root / "extract_meta.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(json.dumps(meta, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
