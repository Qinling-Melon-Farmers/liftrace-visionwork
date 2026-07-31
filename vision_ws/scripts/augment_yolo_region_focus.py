#!/usr/bin/env python3
"""Import a YOLO zip and generate flight-style, region-focused variants.

The source zip contains the four standard target classes.  The output keeps
the project's six-class semantic order and only augments pixels around the
annotated target regions for blur, erase, and local crop operations.

The output is an independent training extension.  It does not modify the
existing v5 dataset and uses the canonical v5 validation set by default.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import random
import re
import shutil
import zipfile
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml


SIX_CLASS_NAMES = ["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"]
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
BBox = Tuple[int, float, float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-zip", required=True, type=Path)
    parser.add_argument("--output-dataset", required=True, type=Path)
    parser.add_argument("--variants-per-image", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20260714)
    parser.add_argument(
        "--validation-root",
        type=Path,
        default=Path(
            "/home/xhj/liftrace/vision_ws/test_data/"
            "yolo_dataset_v5_6cls_redcross_standard_20260713/images/val"
        ),
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def safe_stem(name: str) -> str:
    """Make a stable filename while retaining enough source information."""

    stem = Path(name).stem
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", stem)
    return stem.strip("_") or "image"


def read_yolo_labels(text: str, max_class_id: Optional[int] = None) -> List[BBox]:
    """Read normalized YOLO labels.

    ``max_class_id`` is used only when importing the official four-class zip.
    The merged v5 dataset is six-class and must accept IDs 0..5.
    """

    labels: List[BBox] = []
    for line_no, line in enumerate(text.splitlines(), start=1):
        fields = line.split()
        if not fields:
            continue
        if len(fields) != 5:
            raise ValueError(f"invalid YOLO label at line {line_no}: {line!r}")
        class_id, cx, cy, width, height = int(fields[0]), *map(float, fields[1:])
        if class_id < 0 or (max_class_id is not None and class_id > max_class_id):
            if max_class_id is None:
                raise ValueError(f"class id must be non-negative, got {class_id}")
            raise ValueError(f"class id must be 0..{max_class_id}, got {class_id}")
        if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0 and 0.0 < width <= 1.0 and 0.0 < height <= 1.0):
            raise ValueError(f"out-of-range YOLO label: {line!r}")
        labels.append((class_id, cx, cy, width, height))
    return labels


def write_yolo_labels(path: Path, labels: Sequence[BBox]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for class_id, cx, cy, width, height in labels:
            handle.write(
                f"{class_id} {cx:.7f} {cy:.7f} {width:.7f} {height:.7f}\n"
            )


def normalized_to_pixels(labels: Sequence[BBox], width: int, height: int) -> List[Tuple[int, int, int, int, int]]:
    result = []
    for class_id, cx, cy, box_w, box_h in labels:
        x1 = max(0, int(round((cx - box_w / 2.0) * width)))
        y1 = max(0, int(round((cy - box_h / 2.0) * height)))
        x2 = min(width - 1, int(round((cx + box_w / 2.0) * width)))
        y2 = min(height - 1, int(round((cy + box_h / 2.0) * height)))
        if x2 > x1 and y2 > y1:
            result.append((class_id, x1, y1, x2, y2))
    return result


def pixels_to_normalized(
    boxes: Iterable[Tuple[int, float, float, float, float]], width: int, height: int
) -> List[BBox]:
    result: List[BBox] = []
    for class_id, x1, y1, x2, y2 in boxes:
        x1 = max(0.0, min(float(width), x1))
        y1 = max(0.0, min(float(height), y1))
        x2 = max(0.0, min(float(width), x2))
        y2 = max(0.0, min(float(height), y2))
        if x2 <= x1 or y2 <= y1:
            continue
        result.append(
            (
                int(class_id),
                ((x1 + x2) / 2.0) / width,
                ((y1 + y2) / 2.0) / height,
                (x2 - x1) / width,
                (y2 - y1) / height,
            )
        )
    return result


def feathered_mask(shape: Tuple[int, int], boxes: Sequence[Tuple[int, int, int, int, int]], pad: float = 0.08) -> np.ndarray:
    height, width = shape
    mask = np.zeros((height, width), dtype=np.float32)
    for _, x1, y1, x2, y2 in boxes:
        box_w, box_h = x2 - x1, y2 - y1
        px = int(round(box_w * pad))
        py = int(round(box_h * pad))
        xa, ya = max(0, x1 - px), max(0, y1 - py)
        xb, yb = min(width, x2 + px), min(height, y2 + py)
        mask[ya:yb, xa:xb] = 1.0
    kernel_size = max(3, int(round(min(height, width) * 0.025)) | 1)
    return cv2.GaussianBlur(mask, (kernel_size, kernel_size), 0)


def blend_with_mask(original: np.ndarray, transformed: np.ndarray, mask: np.ndarray) -> np.ndarray:
    alpha = mask[..., None]
    return np.clip(original.astype(np.float32) * (1.0 - alpha) + transformed.astype(np.float32) * alpha, 0, 255).astype(np.uint8)


def apply_region_blur(image: np.ndarray, boxes: Sequence[Tuple[int, int, int, int, int]], rng: random.Random) -> np.ndarray:
    kernel = rng.choice([5, 7, 9, 11])
    transformed = cv2.GaussianBlur(image, (kernel, kernel), 0)
    return blend_with_mask(image, transformed, feathered_mask(image.shape[:2], boxes, pad=0.04))


def apply_region_erase(image: np.ndarray, boxes: Sequence[Tuple[int, int, int, int, int]], rng: random.Random) -> np.ndarray:
    result = image.copy()
    height, width = image.shape[:2]
    for _, x1, y1, x2, y2 in boxes:
        box_w, box_h = max(2, x2 - x1), max(2, y2 - y1)
        erase_w = max(1, int(box_w * rng.uniform(0.18, 0.38)))
        erase_h = max(1, int(box_h * rng.uniform(0.18, 0.38)))
        left = rng.randint(x1, max(x1, x2 - erase_w))
        top = rng.randint(y1, max(y1, y2 - erase_h))
        right, bottom = min(width, left + erase_w), min(height, top + erase_h)
        border = image[max(0, top - 2):min(height, bottom + 2), max(0, left - 2):min(width, right + 2)]
        fill = tuple(int(value) for value in np.median(border.reshape(-1, 3), axis=0)) if border.size else (128, 128, 128)
        result[top:bottom, left:right] = fill
    return result


def apply_region_crop(
    image: np.ndarray,
    labels: Sequence[BBox],
    boxes: Sequence[Tuple[int, int, int, int, int]],
    rng: random.Random,
) -> Tuple[np.ndarray, List[BBox]]:
    height, width = image.shape[:2]
    x1 = min(box[1] for box in boxes)
    y1 = min(box[2] for box in boxes)
    x2 = max(box[3] for box in boxes)
    y2 = max(box[4] for box in boxes)
    target_w, target_h = max(2, x2 - x1), max(2, y2 - y1)
    crop_w = min(width, max(target_w + 2, int(target_w * rng.uniform(1.45, 2.0))))
    crop_h = min(height, max(target_h + 2, int(target_h * rng.uniform(1.45, 2.0))))
    center_x = (x1 + x2) / 2.0 + rng.uniform(-0.08, 0.08) * target_w
    center_y = (y1 + y2) / 2.0 + rng.uniform(-0.08, 0.08) * target_h
    left = int(round(center_x - crop_w / 2.0))
    top = int(round(center_y - crop_h / 2.0))
    left = max(0, min(width - crop_w, left))
    top = max(0, min(height - crop_h, top))
    right, bottom = left + crop_w, top + crop_h

    cropped = image[top:bottom, left:right]
    resized = cv2.resize(cropped, (width, height), interpolation=cv2.INTER_LINEAR)
    scale_x, scale_y = width / crop_w, height / crop_h
    transformed_boxes = []
    for class_id, old_x1, old_y1, old_x2, old_y2 in normalized_to_pixels(labels, width, height):
        transformed_boxes.append(
            (
                class_id,
                (old_x1 - left) * scale_x,
                (old_y1 - top) * scale_y,
                (old_x2 - left) * scale_x,
                (old_y2 - top) * scale_y,
            )
        )
    return resized, pixels_to_normalized(transformed_boxes, width, height)


def extract_source(zip_path: Path, temp_root: Path) -> List[Tuple[str, bytes, str]]:
    """Return (image_name, image_bytes, label_text) from the zip train split."""

    with zipfile.ZipFile(zip_path) as archive:
        members = [member for member in archive.namelist() if not member.endswith("/")]
        image_members = [
            member for member in members
            if "/images/" in f"/{member}" and Path(member).suffix.lower() in IMAGE_EXTENSIONS
        ]
        label_by_stem = {
            Path(member).stem: member
            for member in members
            if "/labels/" in f"/{member}" and Path(member).suffix.lower() == ".txt"
        }
        records = []
        for image_member in sorted(image_members):
            stem = Path(image_member).stem
            label_member = label_by_stem.get(stem)
            if label_member is None:
                raise RuntimeError(f"missing label for {image_member}")
            records.append((Path(image_member).name, archive.read(image_member), archive.read(label_member).decode("utf-8")))
    return records


def write_data_yaml(output: Path, validation_root: Path) -> None:
    data = {
        "path": str(output),
        "train": "images/train",
        "val": str(validation_root),
        "nc": len(SIX_CLASS_NAMES),
        "names": {index: name for index, name in enumerate(SIX_CLASS_NAMES)},
    }
    with (output / "data.yaml").open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)


def main() -> int:
    args = parse_args()
    args.input_zip = args.input_zip.resolve()
    args.output_dataset = args.output_dataset.resolve()
    args.validation_root = args.validation_root.resolve()
    if args.variants_per_image < 0:
        raise ValueError("--variants-per-image must be non-negative")
    if args.output_dataset.exists():
        if not args.overwrite:
            raise FileExistsError(f"output exists; pass --overwrite: {args.output_dataset}")
        shutil.rmtree(args.output_dataset)
    (args.output_dataset / "images/train").mkdir(parents=True, exist_ok=True)
    (args.output_dataset / "labels/train").mkdir(parents=True, exist_ok=True)

    records = extract_source(args.input_zip, args.output_dataset / ".source_extract")
    modes = ["region_blur", "region_erase", "region_crop"]
    counts = {"source": 0, **{mode: 0 for mode in modes}}
    label_counts = {name: 0 for name in SIX_CLASS_NAMES}
    digest = hashlib.sha256()

    for record_index, (source_name, image_bytes, label_text) in enumerate(records):
        source_stem = safe_stem(source_name)
        image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"cannot decode {source_name} from {args.input_zip}")
        labels = read_yolo_labels(label_text, max_class_id=3)
        boxes = normalized_to_pixels(labels, image.shape[1], image.shape[0])
        if not boxes:
            raise RuntimeError(f"empty label for {source_name}")
        digest.update(image_bytes)

        source_image_path = args.output_dataset / "images/train" / f"{source_stem}.jpg"
        source_label_path = args.output_dataset / "labels/train" / f"{source_stem}.txt"
        if not cv2.imwrite(str(source_image_path), image, [cv2.IMWRITE_JPEG_QUALITY, 97]):
            raise RuntimeError(f"cannot write {source_image_path}")
        write_yolo_labels(source_label_path, labels)
        counts["source"] += 1
        for class_id, *_ in labels:
            label_counts[SIX_CLASS_NAMES[class_id]] += 1

        for variant_index in range(args.variants_per_image):
            mode = modes[(record_index * args.variants_per_image + variant_index) % len(modes)]
            rng = random.Random(args.seed + record_index * 1009 + variant_index * 9176)
            if mode == "region_blur":
                variant_image = apply_region_blur(image, boxes, rng)
                variant_labels = labels
            elif mode == "region_erase":
                variant_image = apply_region_erase(image, boxes, rng)
                variant_labels = labels
            else:
                variant_image, variant_labels = apply_region_crop(image, labels, boxes, rng)
            variant_stem = f"{source_stem}__{mode}__v{variant_index:02d}"
            variant_image_path = args.output_dataset / "images/train" / f"{variant_stem}.jpg"
            variant_label_path = args.output_dataset / "labels/train" / f"{variant_stem}.txt"
            if not cv2.imwrite(str(variant_image_path), variant_image, [cv2.IMWRITE_JPEG_QUALITY, 97]):
                raise RuntimeError(f"cannot write {variant_image_path}")
            write_yolo_labels(variant_label_path, variant_labels)
            counts[mode] += 1

    write_data_yaml(args.output_dataset, args.validation_root)
    manifest = {
        "source_zip": str(args.input_zip),
        "source_sha256_image_bytes": digest.hexdigest(),
        "source_images": counts["source"],
        "generated_images": sum(counts[mode] for mode in modes),
        "counts_by_mode": counts,
        "label_counts_source": label_counts,
        "class_names": SIX_CLASS_NAMES,
        "region_focus": {
            "region_blur": "Gaussian blur blended only over each annotated box and a small feathered margin",
            "region_erase": "partial rectangular occlusion sampled inside each annotated box; labels remain because the target is not fully erased",
            "region_crop": "crop centered on the union of annotated boxes, resize back, and transform all labels",
        },
        "validation": {
            "policy": "use the existing canonical v5 validation images; do not split rotated siblings randomly",
            "root": str(args.validation_root),
        },
        "seed": args.seed,
    }
    with (args.output_dataset / "dataset_manifest.json").open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)
    shutil.rmtree(args.output_dataset / ".source_extract", ignore_errors=True)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
