#!/usr/bin/env python3
"""Validate YOLO image/label pairing and class/range constraints."""

from __future__ import annotations

import argparse
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def validate_split(root: Path, split: str) -> dict:
    image_dir = root / "images" / split
    label_dir = root / "labels" / split
    images = {path.stem: path for path in image_dir.iterdir() if path.suffix.lower() in IMAGE_EXTENSIONS}
    labels = {path.stem: path for path in label_dir.glob("*.txt")}
    missing_labels = sorted(set(images) - set(labels))
    orphan_labels = sorted(set(labels) - set(images))
    invalid = []
    empty = []
    class_counts = {class_id: 0 for class_id in range(6)}
    for stem, label_path in labels.items():
        lines = [line for line in label_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if not lines:
            empty.append(stem)
            continue
        for line_no, line in enumerate(lines, start=1):
            fields = line.split()
            if len(fields) != 5:
                invalid.append(f"{label_path}:{line_no}: {line}")
                continue
            try:
                class_id = int(fields[0])
                values = [float(value) for value in fields[1:]]
            except ValueError:
                invalid.append(f"{label_path}:{line_no}: {line}")
                continue
            if class_id not in class_counts or not (
                0.0 <= values[0] <= 1.0
                and 0.0 <= values[1] <= 1.0
                and 0.0 < values[2] <= 1.0
                and 0.0 < values[3] <= 1.0
            ):
                invalid.append(f"{label_path}:{line_no}: {line}")
                continue
            class_counts[class_id] += 1
    return {
        "split": split,
        "images": len(images),
        "labels": len(labels),
        "missing_labels": missing_labels,
        "orphan_labels": orphan_labels,
        "invalid": invalid,
        "empty_labels": empty,
        "class_counts": class_counts,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()
    reports = [validate_split(args.dataset.resolve(), split) for split in ("train", "val")]
    for report in reports:
        print(report)
    if any(report["missing_labels"] or report["orphan_labels"] or report["invalid"] for report in reports):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
