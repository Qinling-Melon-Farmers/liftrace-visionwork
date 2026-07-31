#!/usr/bin/env python3
"""批量推理脚本：用 best.pt 对两个数据集做检测，输出可视化结果。
用法: python3 inference_eval.py --input <dir> --output <dir> [--model best.pt] [--conf 0.25]
"""
import argparse
import sys
from pathlib import Path

import cv2
from ultralytics import YOLO


def run_inference(input_dir: Path, output_dir: Path, model_path: str, conf: float):
    model = YOLO(model_path, task="detect")
    output_dir.mkdir(parents=True, exist_ok=True)

    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"}
    image_paths = sorted(
        [p for p in input_dir.iterdir() if p.suffix.lower() in exts]
    )

    if not image_paths:
        print(f"[!] {input_dir} 中没有找到图像文件")
        return

    print(f"模型: {model_path}  类别: {model.names}")
    print(f"输入: {input_dir}  ({len(image_paths)} 张)")
    print(f"输出: {output_dir}")
    print(f"置信度阈值: {conf}")
    print("-" * 60)

    results = model(str(input_dir), stream=True, conf=conf, verbose=False)

    class_counts = {}
    total_detections = 0
    images_with_detections = 0

    for r in results:
        img_path = Path(r.path)
        img = r.orig_img.copy()
        boxes = r.boxes

        if boxes is not None and len(boxes) > 0:
            images_with_detections += 1
            for cls_id, conf_val, xyxy in zip(
                boxes.cls.cpu().numpy().astype(int),
                boxes.conf.cpu().numpy(),
                boxes.xyxy.cpu().numpy().astype(int),
            ):
                cls_name = model.names[cls_id]
                class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
                total_detections += 1

                x1, y1, x2, y2 = xyxy
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{cls_name} {conf_val:.2f}"
                cv2.putText(
                    img, label, (x1, max(y1 - 5, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
        else:
            # 无检测，在图上标注
            cv2.putText(
                img, "NO DETECTION", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2,
            )

        out_path = output_dir / img_path.name
        cv2.imwrite(str(out_path), img)

    # 摘要
    print(f"\n===== 推理完成 =====")
    print(f"总图像: {len(image_paths)}")
    print(f"有检测的图像: {images_with_detections}  ({images_with_detections/len(image_paths)*100:.1f}%)")
    print(f"总检测框: {total_detections}")
    print(f"各类别检测数:")
    for cls_name, count in sorted(class_counts.items(), key=lambda x: -x[1]):
        print(f"  {cls_name}: {count}")
    print(f"\n可视化结果已保存到: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="YOLO 批量推理评估")
    parser.add_argument("--input", required=True, help="输入图像目录")
    parser.add_argument("--output", required=True, help="输出可视化目录")
    parser.add_argument("--model", required=True, help="模型路径")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    args = parser.parse_args()

    run_inference(
        input_dir=Path(args.input),
        output_dir=Path(args.output),
        model_path=args.model,
        conf=args.conf,
    )


if __name__ == "__main__":
    main()
