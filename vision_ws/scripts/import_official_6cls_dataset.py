#!/usr/bin/env python3
"""Import the official target-photo archive into a reproducible v5 extension.

The archive contains five standard target classes (bridge, car, pillbox,
tank, tent) and a landing H marker.  ``car`` is mapped to the project's
``panzer`` class.  H is intentionally kept as a six-class hard negative and
is never relabeled as ``red_cross``.

The existing six-class model supplies the YOLO boxes.  Images for which the
expected class is not detected are kept in the audit directory, but are not
silently added as empty positive samples.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import zipfile
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
from ultralytics import YOLO


CLASS_NAMES = ["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"]
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
PREFIX_TO_CLASS = {
    "bridge": "bridge",
    "car": "panzer",
    "pillbox": "pillbox",
    "tank": "tank",
    "tent": "tent",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="导入比赛方标准图片并生成 v5 六分类扩展数据集")
    parser.add_argument("--archive", default="实物组数据集.zip")
    parser.add_argument(
        "--base-dataset",
        default="vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713",
    )
    parser.add_argument(
        "--output",
        default="vision_ws/test_data/yolo_dataset_v5_6cls_redcross_official_20260713",
    )
    parser.add_argument(
        "--audit-dir",
        default="vision_ws/test_data/official_dataset_audit_20260713",
    )
    parser.add_argument(
        "--model",
        default="vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/weights/best.pt",
    )
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--center-crop-ratio",
        type=float,
        default=0.0,
        help="对官方正上方靶板按中心裁剪后推理；0 表示整图推理，建议 0.50~0.60",
    )
    parser.add_argument("--val-count", type=int, default=2)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def image_members(archive: zipfile.ZipFile) -> List[zipfile.ZipInfo]:
    members = [
        info
        for info in archive.infolist()
        if not info.is_dir() and Path(info.filename).suffix.lower() in IMAGE_EXTS
    ]
    return sorted(members, key=lambda info: Path(info.filename).name.lower())


def source_key(name: str) -> Optional[str]:
    stem = Path(name).stem.lower()
    for prefix in ("bridge", "car", "pillbox", "tank", "tent"):
        if stem.startswith(prefix):
            return prefix
    if stem.startswith("h"):
        return "h"
    return None


def source_index(name: str) -> int:
    match = re.search(r"(\d+)$", Path(name).stem)
    return int(match.group(1)) if match else 0


def write_bytes(member: zipfile.ZipInfo, archive: zipfile.ZipFile, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with archive.open(member, "r") as src, path.open("wb") as dst:
        shutil.copyfileobj(src, dst)


def copy_tree_files(src: Path, dst: Path, suffixes: Iterable[str]) -> int:
    count = 0
    for split in ("train", "val"):
        for kind in ("images", "labels"):
            source_dir = src / kind / split
            target_dir = dst / kind / split
            target_dir.mkdir(parents=True, exist_ok=True)
            for item in sorted(source_dir.iterdir()):
                if item.is_file() and item.suffix.lower() in suffixes:
                    shutil.copy2(item, target_dir / item.name)
                    count += 1
    return count


def ensure_output_root(base: Path, output: Path, overwrite: bool) -> None:
    if output.exists():
        if not overwrite:
            raise FileExistsError(f"输出目录已存在，使用 --overwrite 才允许重建: {output}")
        shutil.rmtree(output)
    output.mkdir(parents=True)
    copied = copy_tree_files(base, output, {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".txt"})
    if copied == 0:
        raise RuntimeError(f"基础数据集为空或结构不符合 v5 约定: {base}")


def xyxy_to_yolo(xyxy, width: int, height: int) -> Tuple[float, float, float, float]:
    x1, y1, x2, y2 = [float(v) for v in xyxy]
    x1 = max(0.0, min(x1, width))
    x2 = max(0.0, min(x2, width))
    y1 = max(0.0, min(y1, height))
    y2 = max(0.0, min(y2, height))
    return (
        ((x1 + x2) / 2.0) / width,
        ((y1 + y2) / 2.0) / height,
        (x2 - x1) / width,
        (y2 - y1) / height,
    )


def draw_overlay(image, detections: List[Dict], expected: Optional[str]):
    overlay = image.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(round(v)) for v in det["xyxy"]]
        is_expected = expected is not None and det["class"] == expected
        color = (0, 200, 0) if is_expected else (0, 0, 255)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 3)
        text = f"{det['class']} {det['confidence']:.2f}"
        cv2.putText(overlay, text, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    if expected is None:
        cv2.putText(overlay, "H / landing hard negative", (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
    return overlay


def main() -> None:
    args = parse_args()
    liftrace = Path(__file__).resolve().parents[2]
    archive_path = (liftrace / args.archive).resolve()
    base_root = (liftrace / args.base_dataset).resolve()
    output_root = (liftrace / args.output).resolve()
    audit_root = (liftrace / args.audit_dir).resolve()
    model_path = (liftrace / args.model).resolve()

    if not archive_path.is_file():
        raise FileNotFoundError(archive_path)
    if not base_root.is_dir():
        raise FileNotFoundError(base_root)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)

    ensure_output_root(base_root, output_root, args.overwrite)
    if audit_root.exists():
        if not args.overwrite:
            raise FileExistsError(f"审计目录已存在，使用 --overwrite 才允许重建: {audit_root}")
        shutil.rmtree(audit_root)
    (audit_root / "overlays").mkdir(parents=True)
    (audit_root / "missed").mkdir(parents=True)

    model = YOLO(str(model_path), task="detect")
    records: List[Dict] = []

    with zipfile.ZipFile(archive_path) as archive:
        members = image_members(archive)
        if len(members) != 60:
            print(f"[WARN] 压缩包图像数量为 {len(members)}，预期为 60")

        prepared: List[Tuple[zipfile.ZipInfo, Path, Path, Tuple[int, int], str, Optional[str], int]] = []
        stage_root = audit_root / "staging"
        stage_root.mkdir(parents=True, exist_ok=True)
        for member in members:
            key = source_key(member.filename)
            if key is None:
                raise ValueError(f"无法识别官方图片类别: {member.filename}")
            index = source_index(member.filename)
            source_path = stage_root / f"{key}_{index:02d}{Path(member.filename).suffix.lower()}"
            write_bytes(member, archive, source_path)
            inference_path = source_path
            crop_offset = (0, 0)
            if args.center_crop_ratio > 0.0:
                if not 0.35 <= args.center_crop_ratio <= 0.80:
                    raise ValueError("--center-crop-ratio 应在 0.35~0.80 之间")
                source_image = cv2.imread(str(source_path))
                if source_image is None:
                    raise RuntimeError(f"无法读取官方图片: {source_path}")
                image_height, image_width = source_image.shape[:2]
                side = int(min(image_height, image_width) * args.center_crop_ratio)
                x0 = max(0, (image_width - side) // 2)
                y0 = max(0, (image_height - side) // 2)
                crop = source_image[y0 : y0 + side, x0 : x0 + side]
                inference_path = stage_root / f"{key}_{index:02d}_center_crop.jpg"
                cv2.imwrite(str(inference_path), crop)
                crop_offset = (x0, y0)
            expected = PREFIX_TO_CLASS.get(key)
            prepared.append((member, source_path, inference_path, crop_offset, key, expected, index))

        prepared.sort(key=lambda item: (item[4], item[6], item[0].filename))
        results = model.predict(
            source=[str(item[2]) for item in prepared],
            stream=True,
            conf=args.conf,
            imgsz=args.imgsz,
            verbose=False,
        )

        counters = {"archive_images": 0, "added_images": 0, "added_boxes": 0, "h_negatives": 0, "missed_positive": 0}
        class_counts = {name: 0 for name in CLASS_NAMES}
        source_counts: Dict[str, Dict[str, int]] = {}

        for item, result in zip(prepared, results):
            member, source_path, inference_path, crop_offset, key, expected, index = item
            counters["archive_images"] += 1
            source_counts.setdefault(key, {"images": 0, "added": 0, "missed": 0, "boxes": 0})
            source_counts[key]["images"] += 1
            image = cv2.imread(str(source_path))
            if image is None:
                raise RuntimeError(f"无法读取官方图片: {source_path}")
            height, width = image.shape[:2]
            detections: List[Dict] = []
            if result.boxes is not None:
                for cls_id, conf_value, xyxy in zip(
                    result.boxes.cls.cpu().numpy().astype(int),
                    result.boxes.conf.cpu().numpy(),
                    result.boxes.xyxy.cpu().numpy(),
                ):
                    class_name = str(model.names[int(cls_id)])
                    x_offset, y_offset = crop_offset
                    xyxy_full = [
                        float(xyxy[0]) + x_offset,
                        float(xyxy[1]) + y_offset,
                        float(xyxy[2]) + x_offset,
                        float(xyxy[3]) + y_offset,
                    ]
                    detections.append({
                        "class": class_name,
                        "confidence": float(conf_value),
                        "xyxy": xyxy_full,
                    })

            kept = [det for det in detections if expected is not None and det["class"] == expected]
            output_name = f"official_{key}_{index:02d}{source_path.suffix.lower()}"
            split = "val" if index > 10 - args.val_count else "train"
            label_lines = []
            if expected is not None:
                class_id = CLASS_NAMES.index(expected)
                for det in kept:
                    cx, cy, bw, bh = xyxy_to_yolo(det["xyxy"], width, height)
                    label_lines.append(f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

            if expected is None:
                include = True
                counters["h_negatives"] += 1
            else:
                include = bool(kept)
                if not include:
                    counters["missed_positive"] += 1
                    source_counts[key]["missed"] += 1

            overlay = draw_overlay(image, detections, expected)
            cv2.imwrite(str(audit_root / "overlays" / f"{Path(output_name).stem}.jpg"), overlay)

            record = {
                "archive_member": member.filename,
                "source_group": key,
                "expected_class": expected,
                "output_name": output_name if include else None,
                "split": split if include else None,
                "included": include,
                "kept_expected_detections": kept,
                "all_model_detections": detections,
            }
            records.append(record)

            if include:
                shutil.copy2(source_path, output_root / "images" / split / output_name)
                (output_root / "labels" / split / f"{Path(output_name).stem}.txt").write_text(
                    "\n".join(label_lines) + ("\n" if label_lines else ""), encoding="utf-8"
                )
                counters["added_images"] += 1
                counters["added_boxes"] += len(label_lines)
                source_counts[key]["added"] += 1
                source_counts[key]["boxes"] += len(label_lines)
                if expected is not None:
                    class_counts[expected] += len(label_lines)
            else:
                shutil.copy2(source_path, audit_root / "missed" / output_name)

    data_yaml = output_root / "data.yaml"
    data_yaml.write_text(
        "# v5 六分类数据集：现有 v5 标准集 + 比赛方官方靶标照片\n"
        f"path: {output_root}\n"
        "train: images/train\n"
        "val: images/val\n"
        "nc: 6\n"
        "names:\n"
        "  0: bridge\n"
        "  1: panzer\n"
        "  2: pillbox\n"
        "  3: tent\n"
        "  4: tank\n"
        "  5: red_cross\n",
        encoding="utf-8",
    )
    dataset_manifest = (
        f"dataset_name: {output_root.name}\n"
        "version: v5_official_extension\n"
        f"base_dataset: {base_root}\n"
        f"official_archive: {archive_path}\n"
        f"official_archive_images: {counters['archive_images']}\n"
        f"official_included_images: {counters['added_images']}\n"
        f"official_included_boxes: {counters['added_boxes']}\n"
        f"official_h_hard_negatives: {counters['h_negatives']}\n"
        f"official_missed_positive: {counters['missed_positive']}\n"
        "car_label_mapping: panzer\n"
        "h_label_mapping: none\n"
        "classes:\n"
        "  - bridge\n"
        "  - panzer\n"
        "  - pillbox\n"
        "  - tent\n"
        "  - tank\n"
        "  - red_cross\n"
        "notes:\n"
        "  - H images are landing-marker hard negatives for the six-class detector.\n"
        "  - Missed positives remain in the audit directory and require manual review.\n"
    )
    (output_root / "dataset_manifest.yaml").write_text(dataset_manifest, encoding="utf-8")
    manifest = {
        "dataset_name": output_root.name,
        "base_dataset": str(base_root),
        "archive": str(archive_path),
        "model": str(model_path),
        "confidence": args.conf,
        "center_crop_ratio": args.center_crop_ratio,
        "class_names": CLASS_NAMES,
        "label_policy": {
            "car_to_panzer": True,
            "H_to_red_cross": False,
            "H_role": "landing hard negative; empty six-class label",
            "missed_positive_policy": "audit only, not included",
        },
        "counters": counters,
        "class_box_counts_added": class_counts,
        "source_counts": source_counts,
        "records": records,
    }
    (audit_root / "import_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps({"output": str(output_root), "audit": str(audit_root), "counters": counters, "class_counts": class_counts}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
