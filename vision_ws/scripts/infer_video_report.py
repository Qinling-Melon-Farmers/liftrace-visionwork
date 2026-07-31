#!/usr/bin/env python3
"""对视频做 YOLO 推理，并输出可验证的可视化与结构化记录。"""

import argparse
import csv
import json
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import cv2
from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="视频推理与报告生成")
    parser.add_argument("--video", required=True, help="输入视频路径")
    parser.add_argument("--model", required=True, help="模型路径")
    parser.add_argument("--output-dir", required=True, help="输出目录")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="推理尺寸")
    parser.add_argument("--device", default="0", help="推理设备")
    parser.add_argument(
        "--sample-stride",
        type=int,
        default=120,
        help="抽样保存关键帧的步长（帧）",
    )
    parser.add_argument(
        "--event-gap",
        type=int,
        default=60,
        help="事件帧最小保存间隔（帧）",
    )
    parser.add_argument(
        "--max-event-frames",
        type=int,
        default=200,
        help="最多额外保存多少张有检测的事件帧",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="运行前清空输出目录",
    )
    return parser.parse_args()


def format_timestamp(frame_idx: int, fps: float) -> str:
    seconds = frame_idx / fps if fps > 0 else 0.0
    minutes = int(seconds // 60)
    remain = seconds - minutes * 60
    return f"{minutes:02d}:{remain:06.3f}"


def save_frame(path: Path, frame):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"写入图像失败: {path}")


def draw_boxes(frame, detections):
    drawn = frame.copy()
    for det in detections:
        x1, y1, x2, y2 = det["xyxy"]
        cls_name = det["class_name"]
        conf = det["confidence"]
        cv2.rectangle(drawn, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            drawn,
            f"{cls_name} {conf:.2f}",
            (x1, max(y1 - 5, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    return drawn


def main():
    args = parse_args()
    video_path = Path(args.video).resolve()
    model_path = Path(args.model).resolve()
    output_dir = Path(args.output_dir).resolve()

    if args.clean and output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_dir = output_dir / "samples"
    samples_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

    annotated_video_path = output_dir / "annotated.mp4"
    writer = cv2.VideoWriter(
        str(annotated_video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps if fps > 0 else 25.0,
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"无法创建输出视频: {annotated_video_path}")

    model = YOLO(str(model_path), task="detect")
    class_names = [str(name) for _, name in sorted(model.names.items())]
    detail_csv_path = output_dir / "detections_long.csv"
    frame_csv_path = output_dir / "frame_summary.csv"

    class_counter = Counter()
    frames_with_detection = 0
    per_second_counts = defaultdict(int)
    detection_frames = []
    top_conf_per_class = defaultdict(float)

    last_event_frame = -10**9
    saved_event_frames = 0

    with open(detail_csv_path, "w", newline="", encoding="utf-8") as detail_f, open(
        frame_csv_path, "w", newline="", encoding="utf-8"
    ) as frame_f:
        detail_writer = csv.writer(detail_f)
        detail_writer.writerow(
            [
                "frame_index",
                "timestamp_sec",
                "class_name",
                "confidence",
                "x1",
                "y1",
                "x2",
                "y2",
            ]
        )
        frame_writer = csv.writer(frame_f)
        frame_writer.writerow(
            [
                "frame_index",
                "timestamp_sec",
                "detection_count",
                "classes",
                "top_class",
                "top_confidence",
            ]
        )

        frame_idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            result = model.predict(
                source=frame,
                conf=args.conf,
                imgsz=args.imgsz,
                verbose=False,
                device=args.device,
            )[0]

            detections = []
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                for cls_id, conf_val, xyxy in zip(
                    boxes.cls.cpu().numpy().astype(int),
                    boxes.conf.cpu().numpy(),
                    boxes.xyxy.cpu().numpy().astype(int),
                ):
                    cls_name = model.names[int(cls_id)]
                    det = {
                        "class_name": cls_name,
                        "confidence": float(conf_val),
                        "xyxy": [int(v) for v in xyxy],
                    }
                    detections.append(det)
                    class_counter[cls_name] += 1
                    top_conf_per_class[cls_name] = max(
                        top_conf_per_class[cls_name], float(conf_val)
                    )
                    detail_writer.writerow(
                        [
                            frame_idx,
                            f"{frame_idx / fps:.3f}" if fps > 0 else "0.000",
                            cls_name,
                            f"{conf_val:.6f}",
                            *det["xyxy"],
                        ]
                    )

            if detections:
                frames_with_detection += 1
                detection_frames.append(frame_idx)
                second_key = int(frame_idx / fps) if fps > 0 else 0
                per_second_counts[second_key] += len(detections)

            top_class = ""
            top_conf = 0.0
            if detections:
                top_det = max(detections, key=lambda d: d["confidence"])
                top_class = top_det["class_name"]
                top_conf = top_det["confidence"]

            frame_writer.writerow(
                [
                    frame_idx,
                    f"{frame_idx / fps:.3f}" if fps > 0 else "0.000",
                    len(detections),
                    "|".join(det["class_name"] for det in detections),
                    top_class,
                    f"{top_conf:.6f}",
                ]
            )

            drawn = draw_boxes(frame, detections)
            cv2.putText(
                drawn,
                f"frame={frame_idx} t={format_timestamp(frame_idx, fps)} det={len(detections)}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 0),
                2,
            )
            writer.write(drawn)

            should_save_sample = args.sample_stride > 0 and frame_idx % args.sample_stride == 0
            if should_save_sample:
                save_frame(samples_dir / f"sample_f{frame_idx:06d}.jpg", drawn)

            should_save_event = (
                detections
                and saved_event_frames < args.max_event_frames
                and frame_idx - last_event_frame >= args.event_gap
            )
            if should_save_event:
                save_frame(samples_dir / f"event_f{frame_idx:06d}.jpg", drawn)
                last_event_frame = frame_idx
                saved_event_frames += 1

            frame_idx += 1

    cap.release()
    writer.release()

    summary = {
        "video": str(video_path),
        "model": str(model_path),
        "conf": args.conf,
        "imgsz": args.imgsz,
        "fps": fps,
        "total_frames": total_frames,
        "width": width,
        "height": height,
        "frames_with_detection": frames_with_detection,
        "detection_frame_ratio": (
            frames_with_detection / total_frames if total_frames > 0 else 0.0
        ),
        "class_counts": dict(class_counter),
        "top_conf_per_class": dict(top_conf_per_class),
        "first_detection_frame": detection_frames[0] if detection_frames else None,
        "last_detection_frame": detection_frames[-1] if detection_frames else None,
        "per_second_counts": dict(sorted(per_second_counts.items())),
        "annotated_video": str(annotated_video_path),
        "detail_csv": str(detail_csv_path),
        "frame_csv": str(frame_csv_path),
        "samples_dir": str(samples_dir),
        "notes": [
            f"当前模型类别为: {', '.join(class_names)}。",
            "本报告只统计 YOLO 检测结果，不包含 cross/landing 等传统几何检测链。",
        ],
    }

    summary_path = output_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print(f"video={video_path}")
    print(f"fps={fps:.3f}")
    print(f"frames={total_frames}")
    print(f"resolution={width}x{height}")
    print(f"frames_with_detection={frames_with_detection}")
    print(f"class_counts={dict(class_counter)}")
    print(f"annotated_video={annotated_video_path}")
    print(f"summary={summary_path}")


if __name__ == "__main__":
    main()
