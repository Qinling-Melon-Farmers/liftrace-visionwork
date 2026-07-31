#!/usr/bin/env python3
"""统一 5 类数据集自动标注。

支持 4 个数据源：
- image          -> 4 类教师模型，沿用旧 split
- rotated        -> tank 教师模型，沿用旧 split
- bridge_video   -> 4 类教师模型，仅保留 bridge，按时间 8:2 切分
- tank_video     -> tank 教师模型，仅保留 tank，按时间 8:2 切分
"""

import argparse
import json
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set

import cv2
from ultralytics import YOLO

CLASS_NAMES = ["bridge", "panzer", "pillbox", "tent", "tank"]
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"}


@dataclass
class SourceSpec:
    name: str
    input_dir: Path
    model_kind: str
    allowed_classes: Set[str]
    split_mode: str
    conf: float
    source_type: str = "teacher"
    manual_label_dir: Optional[Path] = None


@dataclass
class ImageRecord:
    source_name: str
    image_name: str
    image_path: Path
    label_path: Path
    overlay_path: Path
    class_counts: Dict[str, int] = field(default_factory=dict)
    split: Optional[str] = None


def parse_manual_label_line(line: str, source_name: str, label_path: Path) -> str:
    parts = line.strip().split()
    if len(parts) != 5:
        raise ValueError(f"{source_name}: 标签字段数错误 {label_path}: {line!r}")

    try:
        class_id = int(parts[0])
        values = [float(v) for v in parts[1:]]
    except ValueError as exc:
        raise ValueError(f"{source_name}: 标签数值非法 {label_path}: {line!r}") from exc

    if class_id != 0:
        raise ValueError(f"{source_name}: 期望单类标签 0，实际为 {class_id} -> {label_path}")
    if any(v < 0.0 or v > 1.0 for v in values):
        raise ValueError(f"{source_name}: 标签坐标越界 {label_path}: {values}")

    unified_id = CLASS_NAMES.index("bridge")
    return f"{unified_id} {values[0]:.6f} {values[1]:.6f} {values[2]:.6f} {values[3]:.6f}"


def list_image_paths(input_dir: Path) -> List[Path]:
    if not input_dir.exists():
        return []
    return sorted(
        [p for p in input_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS]
    )


def ensure_split_dirs(output_root: Path):
    for split in ["train", "val"]:
        (output_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_root / "labels" / split).mkdir(parents=True, exist_ok=True)
        (output_root / "overlays" / split).mkdir(parents=True, exist_ok=True)


def build_existing_split_map(existing_dataset_root: Path) -> Dict[str, str]:
    split_map: Dict[str, str] = {}
    for split in ["train", "val"]:
        split_dir = existing_dataset_root / "images" / split
        for img_path in list_image_paths(split_dir):
            prev = split_map.get(img_path.name)
            if prev and prev != split:
                raise ValueError(f"旧 split 冲突: {img_path.name} -> {prev}/{split}")
            split_map[img_path.name] = split
    return split_map


def copy_to_split(record: ImageRecord, output_root: Path, split: str):
    shutil.copy2(str(record.image_path), str(output_root / "images" / split / record.image_name))
    shutil.copy2(
        str(record.label_path),
        str(output_root / "labels" / split / record.label_path.name),
    )
    shutil.copy2(
        str(record.overlay_path),
        str(output_root / "overlays" / split / record.overlay_path.name),
    )


def count_label_lines(lines: Iterable[str]) -> Dict[str, int]:
    counts = {name: 0 for name in CLASS_NAMES}
    for line in lines:
        if not line.strip():
            continue
        class_id = int(line.split()[0])
        counts[CLASS_NAMES[class_id]] += 1
    return counts


def run_model_on_dir(model: YOLO, spec: SourceSpec, output_root: Path) -> List[ImageRecord]:
    labels_dir = output_root / "labels" / spec.name
    imgs_dir = output_root / "images" / spec.name
    overlay_dir = output_root / "overlays" / spec.name
    labels_dir.mkdir(parents=True, exist_ok=True)
    imgs_dir.mkdir(parents=True, exist_ok=True)
    overlay_dir.mkdir(parents=True, exist_ok=True)

    image_paths = list_image_paths(spec.input_dir)
    if not image_paths:
        print(f"[!] {spec.name}: 输入目录为空，跳过 -> {spec.input_dir}")
        return []

    results = model([str(p) for p in image_paths], stream=True, conf=spec.conf, verbose=False)

    records: List[ImageRecord] = []
    empty = 0
    kept_labels = set()

    for img_path, result in zip(image_paths, results):
        img = result.orig_img.copy()
        h, w = img.shape[:2]
        lines: List[str] = []

        if result.boxes is not None and len(result.boxes) > 0:
            for cls_id, conf_val, xyxy in zip(
                result.boxes.cls.cpu().numpy().astype(int),
                result.boxes.conf.cpu().numpy(),
                result.boxes.xyxy.cpu().numpy().astype(int),
            ):
                cls_name = model.names[cls_id]
                if cls_name not in spec.allowed_classes:
                    continue

                unified_id = CLASS_NAMES.index(cls_name)
                kept_labels.add(cls_name)
                x1, y1, x2, y2 = xyxy
                bw, bh = x2 - x1, y2 - y1
                cx_n = ((x1 + x2) / 2) / w
                cy_n = ((y1 + y2) / 2) / h
                bw_n = bw / w
                bh_n = bh / h
                lines.append(
                    f"{unified_id} {cx_n:.6f} {cy_n:.6f} {bw_n:.6f} {bh_n:.6f}"
                )

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{cls_name}({unified_id}) {conf_val:.2f}"
                cv2.putText(
                    img,
                    label,
                    (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        if not lines:
            empty += 1
            cv2.putText(
                img,
                "NO DETECTION",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        label_path = labels_dir / f"{img_path.stem}.txt"
        with open(label_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))

        dest_img_path = imgs_dir / img_path.name
        shutil.copy2(str(img_path), str(dest_img_path))
        overlay_path = overlay_dir / img_path.name
        cv2.imwrite(str(overlay_path), img)

        records.append(
            ImageRecord(
                source_name=spec.name,
                image_name=img_path.name,
                image_path=dest_img_path,
                label_path=label_path,
                overlay_path=overlay_path,
                class_counts=count_label_lines(lines),
            )
        )

    print(
        f"{spec.name}: {len(records)} 张, 空检测 {empty} 张, "
        f"保留标签: {sorted(kept_labels)}"
    )
    return records


def import_manual_bridge_dir(spec: SourceSpec, output_root: Path) -> List[ImageRecord]:
    if spec.manual_label_dir is None:
        raise ValueError(f"{spec.name}: manual_label_dir 未设置")

    labels_dir = output_root / "labels" / spec.name
    imgs_dir = output_root / "images" / spec.name
    overlay_dir = output_root / "overlays" / spec.name
    labels_dir.mkdir(parents=True, exist_ok=True)
    imgs_dir.mkdir(parents=True, exist_ok=True)
    overlay_dir.mkdir(parents=True, exist_ok=True)

    image_paths = list_image_paths(spec.input_dir)
    if not image_paths:
        print(f"[!] {spec.name}: 输入目录为空，跳过 -> {spec.input_dir}")
        return []

    records: List[ImageRecord] = []
    empty = 0
    box_total = 0

    for img_path in image_paths:
        label_path_src = spec.manual_label_dir / f"{img_path.stem}.txt"
        if not label_path_src.exists():
            raise FileNotFoundError(f"{spec.name}: 缺少对应标签 {label_path_src}")

        lines_raw = label_path_src.read_text(encoding="utf-8").splitlines()
        lines = [
            parse_manual_label_line(line, spec.name, label_path_src)
            for line in lines_raw
            if line.strip()
        ]
        if not lines:
            empty += 1
        box_total += len(lines)

        label_path = labels_dir / f"{img_path.stem}.txt"
        label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

        dest_img_path = imgs_dir / img_path.name
        shutil.copy2(str(img_path), str(dest_img_path))

        img = cv2.imread(str(img_path))
        if img is None:
            raise RuntimeError(f"{spec.name}: 无法读取图像 {img_path}")
        h, w = img.shape[:2]
        if lines:
            for line in lines:
                _, cx_n, cy_n, bw_n, bh_n = line.split()
                cx_n, cy_n, bw_n, bh_n = map(float, (cx_n, cy_n, bw_n, bh_n))
                bw = bw_n * w
                bh = bh_n * h
                cx = cx_n * w
                cy = cy_n * h
                x1 = int(round(cx - bw / 2))
                y1 = int(round(cy - bh / 2))
                x2 = int(round(cx + bw / 2))
                y2 = int(round(cy + bh / 2))
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    img,
                    "bridge(0) manual",
                    (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
        else:
            cv2.putText(
                img,
                "NO DETECTION",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )
        overlay_path = overlay_dir / img_path.name
        cv2.imwrite(str(overlay_path), img)

        records.append(
            ImageRecord(
                source_name=spec.name,
                image_name=img_path.name,
                image_path=dest_img_path,
                label_path=label_path,
                overlay_path=overlay_path,
                class_counts=count_label_lines(lines),
            )
        )

    print(
        f"{spec.name}: {len(records)} 张, 空标签 {empty} 张, "
        f"bridge 框总数: {box_total}"
    )
    return records


def assign_reuse_split(
    records: List[ImageRecord], split_map: Dict[str, str]
) -> Dict[str, int]:
    summary = {"train": 0, "val": 0, "skipped": 0}
    for record in records:
        split = split_map.get(record.image_name)
        if split is None:
            record.split = None
            summary["skipped"] += 1
            continue
        record.split = split
        summary[split] += 1
    return summary


def assign_time_split(records: List[ImageRecord]) -> Dict[str, int]:
    summary = {"train": 0, "val": 0, "skipped": 0}
    if not records:
        return summary
    split_idx = int(len(records) * 0.8)
    split_idx = min(max(split_idx, 1), len(records) - 1) if len(records) > 1 else 1
    for idx, record in enumerate(records):
        record.split = "train" if idx < split_idx else "val"
        summary[record.split] += 1
    return summary


def write_dataset_artifacts(
    output_root: Path,
    source_records: Dict[str, List[ImageRecord]],
    source_summaries: Dict[str, Dict[str, int]],
):
    ensure_split_dirs(output_root)

    split_stats = {
        "train": {"images": 0, "boxes": {name: 0 for name in CLASS_NAMES}},
        "val": {"images": 0, "boxes": {name: 0 for name in CLASS_NAMES}},
    }

    for records in source_records.values():
        for record in records:
            if record.split not in {"train", "val"}:
                continue
            copy_to_split(record, output_root, record.split)
            split_stats[record.split]["images"] += 1
            for class_name, count in record.class_counts.items():
                split_stats[record.split]["boxes"][class_name] += count

    generated_at = datetime.now().isoformat(timespec="seconds")
    yaml_content = f"""# liftrace 统一 5 类检测数据集 (自动标注)
# 生成时间: {generated_at}
path: {output_root}
train: images/train
val: images/val
nc: 5
names:
  0: bridge
  1: panzer
  2: pillbox
  3: tent
  4: tank
"""
    with open(output_root / "data.yaml", "w", encoding="utf-8") as f:
        f.write(yaml_content)

    stats = {
        "generated_at": generated_at,
        "class_names": CLASS_NAMES,
        "sources": source_summaries,
        "splits": split_stats,
        "totals": {
            "images": split_stats["train"]["images"] + split_stats["val"]["images"],
            "boxes": {
                name: split_stats["train"]["boxes"][name] + split_stats["val"]["boxes"][name]
                for name in CLASS_NAMES
            },
        },
    }
    with open(output_root / "stats.json", "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)

    print(
        f"train: {split_stats['train']['images']} 张, "
        f"val: {split_stats['val']['images']} 张"
    )
    print(f"data.yaml: {output_root / 'data.yaml'}")
    print(f"stats.json: {output_root / 'stats.json'}")


def main():
    parser = argparse.ArgumentParser(description="统一 5 类 YOLO 数据集自动标注")
    parser.add_argument(
        "--output",
        default="vision_ws/test_data/yolo_dataset_v2_video_20260624",
    )
    parser.add_argument(
        "--reuse-split-from",
        default="vision_ws/test_data/yolo_dataset",
        help="旧 train/val 数据集根目录（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--model-4cls",
        default="Visual/src/yolov5_detect/best.pt",
        help="4 类教师模型路径（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--model-tank",
        default="Visual/src/yolov5_detect/tank.pt",
        help="tank 教师模型路径（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--input-image",
        default="vision_ws/test_data/image",
        help="image 输入目录（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--input-rotated",
        default="vision_ws/test_data/rotated",
        help="rotated 输入目录（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--input-bridge-video",
        default="vision_ws/test_data/video_sources/bridge_20260624/frames",
        help="bridge 视频抽帧目录（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--input-bridge-manual-images",
        default="",
        help="手工 bridge 图像目录（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--input-bridge-manual-labels",
        default="",
        help="手工 bridge 标签目录（相对于 liftrace 根目录）",
    )
    parser.add_argument(
        "--input-tank-video",
        default="vision_ws/test_data/video_sources/tank_20260624/frames",
        help="tank 视频抽帧目录（相对于 liftrace 根目录）",
    )
    parser.add_argument("--conf-4cls", type=float, default=0.25)
    parser.add_argument("--conf-tank", type=float, default=0.25)
    parser.add_argument(
        "--conf-bridge-video",
        type=float,
        default=None,
        help="bridge 视频来源的 4 类教师阈值，默认继承 --conf-4cls",
    )
    parser.add_argument(
        "--conf-tank-video",
        type=float,
        default=None,
        help="tank 视频来源的 tank 教师阈值，默认继承 --conf-tank",
    )
    args = parser.parse_args()

    liftrace = Path(__file__).resolve().parents[2]
    output_root = liftrace / args.output
    reuse_split_root = liftrace / args.reuse_split_from
    if output_root.exists():
        shutil.rmtree(output_root)

    split_map = build_existing_split_map(reuse_split_root)
    model_4cls = YOLO(str(liftrace / args.model_4cls), task="detect")
    model_tank = YOLO(str(liftrace / args.model_tank), task="detect")

    source_specs = [
        SourceSpec(
            name="image",
            input_dir=liftrace / args.input_image,
            model_kind="4cls",
            allowed_classes={"bridge", "panzer", "pillbox", "tent"},
            split_mode="reuse_existing",
            conf=args.conf_4cls,
        ),
        SourceSpec(
            name="rotated",
            input_dir=liftrace / args.input_rotated,
            model_kind="tank",
            allowed_classes={"tank"},
            split_mode="reuse_existing",
            conf=args.conf_tank,
        ),
        SourceSpec(
            name="tank_video",
            input_dir=liftrace / args.input_tank_video,
            model_kind="tank",
            allowed_classes={"tank"},
            split_mode="time_80_20",
            conf=args.conf_tank_video if args.conf_tank_video is not None else args.conf_tank,
        ),
    ]

    if args.input_bridge_manual_images and args.input_bridge_manual_labels:
        source_specs.insert(
            2,
            SourceSpec(
                name="bridge_manual",
                input_dir=liftrace / args.input_bridge_manual_images,
                model_kind="manual",
                allowed_classes={"bridge"},
                split_mode="time_80_20",
                conf=1.0,
                source_type="manual_bridge",
                manual_label_dir=liftrace / args.input_bridge_manual_labels,
            ),
        )
    else:
        source_specs.insert(
            2,
            SourceSpec(
                name="bridge_video",
                input_dir=liftrace / args.input_bridge_video,
                model_kind="4cls",
                allowed_classes={"bridge"},
                split_mode="time_80_20",
                conf=args.conf_bridge_video if args.conf_bridge_video is not None else args.conf_4cls,
            ),
        )

    source_records: Dict[str, List[ImageRecord]] = {}
    source_summaries: Dict[str, Dict[str, object]] = {}

    for idx, spec in enumerate(source_specs, start=1):
        print("\n" + "=" * 60)
        print(f"[{idx}/{len(source_specs)}] {spec.name}")
        if spec.source_type == "manual_bridge":
            records = import_manual_bridge_dir(spec, output_root)
        else:
            model = model_4cls if spec.model_kind == "4cls" else model_tank
            records = run_model_on_dir(model, spec, output_root)
        source_records[spec.name] = records

        if spec.split_mode == "reuse_existing":
            split_summary = assign_reuse_split(records, split_map)
        else:
            split_summary = assign_time_split(records)

        box_counts = {name: 0 for name in CLASS_NAMES}
        empty_labels = 0
        for record in records:
            if sum(record.class_counts.values()) == 0:
                empty_labels += 1
            for class_name, count in record.class_counts.items():
                box_counts[class_name] += count

        source_summaries[spec.name] = {
            "input_dir": str(spec.input_dir),
            "model_kind": spec.model_kind,
            "allowed_classes": sorted(spec.allowed_classes),
            "split_mode": spec.split_mode,
            "conf": spec.conf,
            "images": len(records),
            "empty_labels": empty_labels,
            "assigned": split_summary,
            "boxes": box_counts,
        }

    print("\n" + "=" * 60)
    print(f"[{len(source_specs) + 1}/{len(source_specs) + 1}] 写最终数据集")
    write_dataset_artifacts(output_root, source_records, source_summaries)


if __name__ == "__main__":
    main()
