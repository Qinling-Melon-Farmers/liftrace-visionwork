#!/usr/bin/env python3
"""按 liftrace 既有配方训练统一 5 类 YOLO11 模型。"""

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="训练 liftrace 统一 5 类检测模型")
    parser.add_argument(
        "--data",
        default="/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v2_video_20260624/data.yaml",
        help="data.yaml 路径",
    )
    parser.add_argument("--model", default="yolo11n.pt", help="基础模型")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--project",
        default="/home/xhj/liftrace/vision_ws/runs",
        help="输出根目录",
    )
    parser.add_argument(
        "--name",
        default="liftrace_5cls_v2_video_20260624",
        help="本次训练名称",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    data_yaml = Path(args.data).resolve()
    project_dir = Path(args.project).resolve()

    model = YOLO(args.model)
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        patience=20,
        batch=args.batch,
        imgsz=args.imgsz,
        save=True,
        device=args.device,
        workers=args.workers,
        project=str(project_dir),
        name=args.name,
        exist_ok=True,
        pretrained=True,
        verbose=True,
        seed=0,
        deterministic=True,
        close_mosaic=10,
        optimizer="auto",
        lr0=0.01,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3.0,
        warmup_momentum=0.8,
        warmup_bias_lr=0.1,
        box=7.5,
        cls=0.5,
        dfl=1.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=0.0,
        translate=0.1,
        scale=0.5,
        shear=0.0,
        perspective=0.0,
        flipud=0.0,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.0,
        copy_paste=0.0,
        auto_augment="randaugment",
        erasing=0.4,
        plots=True,
        amp=True,
    )


if __name__ == "__main__":
    main()
