#!/usr/bin/env python3
"""按类别统计检出数和置信度分布。
用法: python3 analyze_detections.py --input <dir> [--input <dir2> ...] --model <path> [--conf 0.25]
"""
import argparse
from pathlib import Path

from ultralytics import YOLO


def analyze(input_dirs: list[str], model_path: str, conf: float = 0.25):
    model = YOLO(model_path, task="detect")

    for input_dir in input_dirs:
        input_dir = Path(input_dir)
        exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"}
        image_paths = sorted([p for p in input_dir.iterdir() if p.suffix.lower() in exts])
        results = model(str(input_dir), stream=True, conf=conf, verbose=False)

        print(f"\n{'='*60}")
        print(f"数据集: {input_dir.name}  ({len(image_paths)} 张)")
        print(f"{'='*60}")

        class_confs: dict[str, list[float]] = {}
        class_counts: dict[str, int] = {}
        no_detect = 0
        total = 0

        for r in results:
            total += 1
            boxes = r.boxes
            if boxes is None or len(boxes) == 0:
                no_detect += 1
                continue

            for cls_id, conf_val in zip(
                boxes.cls.cpu().numpy().astype(int),
                boxes.conf.cpu().numpy(),
            ):
                cls_name = model.names[cls_id]
                class_confs.setdefault(cls_name, []).append(float(conf_val))
                class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

        print(f"检出率: {total - no_detect}/{total}  ({100 * (total - no_detect) / total:.1f}%)")
        print(f"\n类别    检出数    平均置信度    最小    最大")
        print("-" * 55)
        for cls_name in sorted(class_counts, key=lambda k: -class_counts[k]):
            confs = class_confs[cls_name]
            print(f"{cls_name:8s}  {class_counts[cls_name]:4d}     "
                  f"{sum(confs) / len(confs):.3f}         "
                  f"{min(confs):.3f}   {max(confs):.3f}")

        if no_detect > 0:
            print(f"\n无检出: {no_detect} 张")


def main():
    parser = argparse.ArgumentParser(description="统计模型检测结果的类别和置信度分布")
    parser.add_argument("--input", action="append", required=True,
                        help="输入图片目录（可多次指定）")
    parser.add_argument("--model", required=True, help="模型路径")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    args = parser.parse_args()

    analyze(input_dirs=args.input, model_path=args.model, conf=args.conf)


if __name__ == "__main__":
    main()
