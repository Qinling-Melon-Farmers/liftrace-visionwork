#!/usr/bin/env python3
"""Merge the official four-class YOLO zip into the v5 training split.

The v5 directory is the canonical dataset.  The official zip contributes
training images only; the existing v5 validation split is left unchanged.
The previously generated independent region-augmentation directory is removed
only after its target path is checked to be the known workspace directory.
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
import zipfile
from pathlib import Path

import yaml


ROOT = Path("/home/xhj/liftrace").resolve()
DEFAULT_BASE = ROOT / "vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713"
DEFAULT_ZIP = ROOT / "vision_ws/test_data/officIal-JPG.yolov11.zip"
WRONG_DATASET = ROOT / "vision_ws/test_data/yolo_dataset_v5_official_rotated_region_aug_20260714"
SIX_CLASS_NAMES = ["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-dataset", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--official-zip", type=Path, default=DEFAULT_ZIP)
    parser.add_argument("--wrong-dataset", type=Path, default=WRONG_DATASET)
    parser.add_argument("--remove-wrong-dataset", action="store_true")
    return parser.parse_args()


def read_labels(path: Path) -> list[str]:
    lines = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        fields = raw.split()
        if not fields:
            continue
        if len(fields) != 5 or not fields[0].isdigit() or int(fields[0]) not in range(4):
            raise ValueError(f"official label is not one of classes 0..3: {path}: {raw}")
        values = [float(value) for value in fields[1:]]
        if not (0.0 <= values[0] <= 1.0 and 0.0 <= values[1] <= 1.0 and 0.0 < values[2] <= 1.0 and 0.0 < values[3] <= 1.0):
            raise ValueError(f"official label is out of range: {path}: {raw}")
        lines.append(raw.rstrip())
    if not lines:
        raise ValueError(f"official label is empty: {path}")
    return lines


def check_safe_delete(target: Path) -> Path:
    target = target.resolve()
    allowed_root = (ROOT / "vision_ws/test_data").resolve()
    if target.parent != allowed_root or target.name != WRONG_DATASET.name:
        raise RuntimeError(f"refusing to delete unexpected path: {target}")
    return target


def main() -> int:
    args = parse_args()
    base = args.base_dataset.resolve()
    official_zip = args.official_zip.resolve()
    wrong_dataset = check_safe_delete(args.wrong_dataset)

    if args.remove_wrong_dataset and wrong_dataset.exists():
        shutil.rmtree(wrong_dataset)
    if not base.is_dir():
        raise FileNotFoundError(base)
    if not official_zip.is_file():
        raise FileNotFoundError(official_zip)

    train_images = base / "images/train"
    train_labels = base / "labels/train"
    train_images.mkdir(parents=True, exist_ok=True)
    train_labels.mkdir(parents=True, exist_ok=True)

    added = []
    with tempfile.TemporaryDirectory(prefix="official_v5_merge_") as temp_dir:
        extract_root = Path(temp_dir)
        with zipfile.ZipFile(official_zip) as archive:
            archive.extractall(extract_root)
        source_yaml = extract_root / "data.yaml"
        source_data = yaml.safe_load(source_yaml.read_text(encoding="utf-8"))
        if source_data.get("names") != ["bridge", "panzer", "pillbox", "tent"]:
            raise ValueError(f"unexpected official class order: {source_data.get('names')}")

        source_images = sorted((extract_root / "train/images").glob("*"))
        if len(source_images) != 240:
            raise ValueError(f"expected 240 official images, found {len(source_images)}")
        for source_image in source_images:
            if source_image.suffix.lower() not in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}:
                continue
            source_label = extract_root / "train/labels" / f"{source_image.stem}.txt"
            if not source_label.is_file():
                raise FileNotFoundError(source_label)
            read_labels(source_label)
            destination_image = train_images / source_image.name
            destination_label = train_labels / source_label.name
            if destination_image.exists() or destination_label.exists():
                raise FileExistsError(f"v5 name collision: {source_image.name}")
            shutil.copy2(source_image, destination_image)
            shutil.copy2(source_label, destination_label)
            added.append(source_image.name)

    manifest = {
        "base_dataset": str(base),
        "official_source_zip": str(official_zip),
        "official_source_images_added_to_train": len(added),
        "official_class_order": ["bridge", "panzer", "pillbox", "tent"],
        "output_class_order": SIX_CLASS_NAMES,
        "validation_policy": "existing v5 validation split unchanged",
        "removed_wrong_independent_dataset": str(wrong_dataset) if args.remove_wrong_dataset else None,
        "sample_names": added[:5],
    }
    (base / "official_merge_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    print(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
