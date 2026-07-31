#!/usr/bin/env python3
"""Build a region-focused augmentation set from the merged v5 dataset."""

from __future__ import annotations

import argparse
import random
import shutil
import sys
from pathlib import Path

import cv2
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from augment_yolo_region_focus import (  # noqa: E402
    IMAGE_EXTENSIONS,
    apply_region_blur,
    apply_region_crop,
    apply_region_erase,
    normalized_to_pixels,
    read_yolo_labels,
    write_yolo_labels,
)


ROOT = Path("/home/xhj/liftrace").resolve()
DEFAULT_INPUT = ROOT / "vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713"
DEFAULT_OUTPUT = ROOT / "vision_ws/test_data/yolo_dataset_v5_region_focus_aug_20260714"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dataset", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dataset", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--variants-per-image", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20260714)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def copy_split(input_root: Path, output_root: Path, split: str) -> int:
    count = 0
    source_images = input_root / "images" / split
    source_labels = input_root / "labels" / split
    for image_path in sorted(source_images.iterdir()):
        if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        label_path = source_labels / f"{image_path.stem}.txt"
        if not label_path.is_file():
            raise FileNotFoundError(label_path)
        destination_image = output_root / "images" / split / image_path.name
        destination_label = output_root / "labels" / split / label_path.name
        destination_image.parent.mkdir(parents=True, exist_ok=True)
        destination_label.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image_path, destination_image)
        shutil.copy2(label_path, destination_label)
        count += 1
    return count


def main() -> int:
    args = parse_args()
    input_root = args.input_dataset.resolve()
    output_root = args.output_dataset.resolve()
    if args.variants_per_image < 1:
        raise ValueError("--variants-per-image must be >= 1")
    if output_root.exists():
        if not args.overwrite:
            raise FileExistsError(f"output exists: {output_root}")
        if output_root.parent != (ROOT / "vision_ws/test_data").resolve():
            raise RuntimeError(f"refusing to delete unexpected output: {output_root}")
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)
    with (input_root / "data.yaml").open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    val_count = copy_split(input_root, output_root, "val")
    train_images = sorted(
        path for path in (input_root / "images/train").iterdir()
        if path.suffix.lower() in IMAGE_EXTENSIONS
    )
    mode_names = ["region_blur", "region_erase", "region_crop"]
    mode_counts = {mode: 0 for mode in mode_names}
    empty_label_images = 0
    for index, image_path in enumerate(train_images):
        label_path = input_root / "labels/train" / f"{image_path.stem}.txt"
        if not label_path.is_file():
            raise FileNotFoundError(label_path)
        image_out = output_root / "images/train" / image_path.name
        label_out = output_root / "labels/train" / label_path.name
        image_out.parent.mkdir(parents=True, exist_ok=True)
        label_out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image_path, image_out)
        shutil.copy2(label_path, label_out)

        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"cannot decode {image_path}")
        labels = read_yolo_labels(
            label_path.read_text(encoding="utf-8"), max_class_id=5
        )
        boxes = normalized_to_pixels(labels, image.shape[1], image.shape[0])
        if not boxes:
            # Keep hard-negative images unchanged.  There is no annotated
            # region on which a region-focused transform can be based.
            empty_label_images += 1
            continue
        for variant in range(args.variants_per_image):
            mode = mode_names[(index * args.variants_per_image + variant) % len(mode_names)]
            rng = random.Random(args.seed + index * 1009 + variant * 9176)
            if mode == "region_blur":
                augmented, augmented_labels = apply_region_blur(image, boxes, rng), labels
            elif mode == "region_erase":
                augmented, augmented_labels = apply_region_erase(image, boxes, rng), labels
            else:
                augmented, augmented_labels = apply_region_crop(image, labels, boxes, rng)
            stem = f"{image_path.stem}__{mode}__v{variant:02d}"
            augmented_path = output_root / "images/train" / f"{stem}.jpg"
            augmented_label_path = output_root / "labels/train" / f"{stem}.txt"
            if not cv2.imwrite(str(augmented_path), augmented, [cv2.IMWRITE_JPEG_QUALITY, 97]):
                raise RuntimeError(f"cannot write {augmented_path}")
            write_yolo_labels(augmented_label_path, augmented_labels)
            mode_counts[mode] += 1

    output_data = dict(data)
    output_data["path"] = str(output_root)
    output_data.pop("test", None)
    (output_root / "data.yaml").write_text(
        yaml.safe_dump(output_data, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    manifest = {
        "dataset_name": "yolo_dataset_v5_region_focus_aug_20260714",
        "source_dataset": str(input_root),
        "source_train_images": len(train_images),
        "source_val_images": val_count,
        "source_train_images_with_regions": len(train_images) - empty_label_images,
        "source_train_empty_label_images": empty_label_images,
        "variants_per_image": args.variants_per_image,
        "generated_augmented_images": sum(mode_counts.values()),
        "train_images_total": len(train_images) + sum(mode_counts.values()),
        "validation_policy": "copied unchanged from merged v5 validation split",
        "mode_counts": mode_counts,
        "region_focus": {
            "blur": "inside each annotated box plus a small feathered margin",
            "erase": "partial occlusion sampled inside each annotated box",
            "crop": "crop around the union of annotated boxes and transform labels",
        },
        "classes": output_data.get("names"),
        "seed": args.seed,
    }
    (output_root / "dataset_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    print(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
