#!/usr/bin/env python3
"""交叉验证：用训练好的学生模型推理，与教师标签对比，找出差异样本。"""
import argparse
from pathlib import Path

import numpy as np
from ultralytics import YOLO


def load_yolo_label(label_path: Path, w: int, h: int) -> list[tuple[int, float, float, float, float]]:
    """加载 YOLO 归一化标签，返回 [(cls_id, cx_n, cy_n, bw_n, bh_n), ...]"""
    dets = []
    if not label_path.exists():
        return dets
    with open(label_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            cls_id = int(float(parts[0]))
            cx, cy, bw, bh = map(float, parts[1:5])
            dets.append((cls_id, cx, cy, bw, bh))
    return dets


def yolo_to_xyxy(cx_n, cy_n, bw_n, bh_n, w, h):
    """归一化 YOLO → 像素 xyxy"""
    x1 = int((cx_n - bw_n / 2) * w)
    y1 = int((cy_n - bh_n / 2) * h)
    x2 = int((cx_n + bw_n / 2) * w)
    y2 = int((cy_n + bh_n / 2) * h)
    return x1, y1, x2, y2


def iou(box_a, box_b):
    """两个 xyxy box 的 IoU"""
    xa = max(box_a[0], box_b[0])
    ya = max(box_a[1], box_b[1])
    xb = min(box_a[2], box_b[2])
    yb = min(box_a[3], box_b[3])
    inter = max(0, xb - xa) * max(0, yb - ya)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    return inter / (area_a + area_b - inter + 1e-6)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--labels-dir", required=True,
                        help="教师标签目录（含 train/ 和 val/）")
    parser.add_argument("--image-dir", required=True)
    parser.add_argument("--rotated-dir", default=None)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--limit", type=int, default=0,
                        help="只处理前 N 张图（0=全量）")
    args = parser.parse_args()

    model = YOLO(args.model, task="detect")
    labels_root = Path(args.labels_dir)
    image_root = Path(args.image_dir)

    # 建立 stem → teacher_label_path 的映射
    teacher_map = {}
    for sub in ["train", "val"]:
        ld = labels_root / "labels" / sub
        if ld.exists():
            for lp in ld.glob("*.txt"):
                teacher_map[lp.stem] = lp

    # 收集所有图片目录，分别推理以避免 r.path 索引名问题
    dirs_to_process = [(image_root, "image")]
    if args.rotated_dir:
        dirs_to_process.append((Path(args.rotated_dir), "rotated"))

    match_ok = 0
    cls_mismatch = 0
    missing_student = 0
    missing_teacher = 0
    extra_det = 0
    total = 0
    mismatches = []
    limit = args.limit

    for img_dir, dir_label in dirs_to_process:
        results = model(str(img_dir), stream=True, conf=args.conf, verbose=False)
        for r in results:
            if limit > 0 and total >= limit:
                break
            total += 1
            img_path = Path(r.path)
            stem = img_path.stem
            h, w = r.orig_img.shape[:2]

            teacher_dets = load_yolo_label(teacher_map.get(stem, Path("/nonexistent")), w, h)

            boxes = r.boxes
            student_dets = []
            if boxes is not None and len(boxes) > 0:
                for cls_id, conf_val, xyxy in zip(
                    boxes.cls.cpu().numpy().astype(int),
                    boxes.conf.cpu().numpy(),
                    boxes.xyxy.cpu().numpy().astype(int),
                ):
                    x1, y1, x2, y2 = xyxy
                    bw, bh = x2 - x1, y2 - y1
                    cx_n = ((x1 + x2) / 2) / w
                    cy_n = ((y1 + y2) / 2) / h
                    bw_n = bw / w
                    bh_n = bh / h
                    student_dets.append((cls_id, conf_val, cx_n, cy_n, bw_n, bh_n, (x1, y1, x2, y2)))

            if not teacher_dets and not student_dets:
                match_ok += 1
                continue
            if not teacher_dets:
                missing_teacher += 1
                student_names = [model.names[d[0]] for d in student_dets]
                mismatches.append((stem, "teacher_missing", "", student_names))
                continue
            if not student_dets:
                missing_student += 1
                teacher_names = [model.names[d[0]] for d in teacher_dets]
                mismatches.append((stem, "student_missing", teacher_names, ""))
                continue

            t_det = teacher_dets[0]
            s_det = student_dets[0]

            t_cls, t_cx, t_cy, t_bw, t_bh = t_det
            s_cls, s_conf, s_cx, s_cy, s_bw, s_bh, s_xyxy = s_det

            t_xyxy = yolo_to_xyxy(t_cx, t_cy, t_bw, t_bh, w, h)
            iou_val = iou(t_xyxy, s_xyxy[:4])

            if iou_val >= 0.5 and t_cls == s_cls:
                match_ok += 1
            elif iou_val >= 0.5 and t_cls != s_cls:
                cls_mismatch += 1
                mismatches.append((stem, f"cls_mismatch(t={model.names[t_cls]},s={model.names[s_cls]})", "", ""))
            else:
                missing_student += 1
                mismatches.append((stem, f"low_iou({iou_val:.2f})",
                                   f"{model.names[t_cls]}", f"{model.names[s_cls]}"))

            if len(student_dets) > 1:
                extra_det += 1
        if limit > 0 and total >= limit:
            break

    print(f"\n总图数: {total}")
    print(f"一致 (cls=Iou≥0.5): {match_ok}")
    print(f"类别不一致 (Iou≥0.5): {cls_mismatch}")
    print(f"学生漏检: {missing_student}")
    print(f"教师标签缺失: {missing_teacher}")
    print(f"学生多检 (≥2框): {extra_det}")

    if mismatches:
        print(f"\n--- 差异样本 ({len(mismatches)} 个) ---")
        for stem, reason, t_label, s_label in mismatches[:50]:
            print(f"  {stem}: {reason}  teacher={t_label}  student={s_label}")
        if len(mismatches) > 50:
            print(f"  ... 还有 {len(mismatches) - 50} 个")


if __name__ == "__main__":
    main()
