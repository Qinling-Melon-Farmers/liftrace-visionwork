#!/usr/bin/env python3
"""Generate a controlled flight/ground-view augmentation set for YOLO detection.

Only the training split is augmented. Validation images and labels are copied
unchanged so that model comparisons remain meaningful.
"""

import argparse
import os
import random
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args():
    parser = argparse.ArgumentParser(description="生成飞行对地场景 YOLO 增强数据集")
    parser.add_argument(
        "--input-dataset",
        default="/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713",
    )
    parser.add_argument(
        "--output-dataset",
        default="/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v5_flight_aug_20260713",
    )
    parser.add_argument("--variants-per-image", type=int, default=1)
    parser.add_argument("--seed", type=int, default=20260713)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def link_or_copy(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def read_labels(path: Path):
    labels = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        parts = raw.split()
        if len(parts) != 5:
            continue
        try:
            labels.append([int(parts[0])] + [float(value) for value in parts[1:]])
        except ValueError:
            continue
    return labels


def write_labels(path: Path, labels):
    lines = []
    for cls_id, cx, cy, width, height in labels:
        lines.append(
            f"{int(cls_id)} {cx:.6f} {cy:.6f} {width:.6f} {height:.6f}"
        )
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def labels_to_corners(labels, width, height):
    corners = []
    for cls_id, cx, cy, box_w, box_h in labels:
        x1 = (cx - box_w / 2.0) * width
        y1 = (cy - box_h / 2.0) * height
        x2 = (cx + box_w / 2.0) * width
        y2 = (cy + box_h / 2.0) * height
        corners.append((cls_id, np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32), (x2 - x1) * (y2 - y1)))
    return corners


def corners_to_labels(corners, width, height):
    output = []
    for cls_id, points, old_area in corners:
        x1 = float(np.clip(np.min(points[:, 0]), 0, width - 1))
        y1 = float(np.clip(np.min(points[:, 1]), 0, height - 1))
        x2 = float(np.clip(np.max(points[:, 0]), 0, width - 1))
        y2 = float(np.clip(np.max(points[:, 1]), 0, height - 1))
        new_area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if new_area < 4.0 or (old_area > 1.0 and new_area / old_area < 0.12):
            continue
        output.append(
            [
                cls_id,
                ((x1 + x2) / 2.0) / width,
                ((y1 + y2) / 2.0) / height,
                (x2 - x1) / width,
                (y2 - y1) / height,
            ]
        )
    return output


def transform_points(points, matrix):
    homogeneous = np.concatenate([points, np.ones((len(points), 1), dtype=np.float32)], axis=1)
    transformed = homogeneous @ matrix.T
    transformed[:, :2] /= np.maximum(transformed[:, 2:3], 1e-6)
    return transformed[:, :2]


def apply_geometry(image, labels, rng):
    height, width = image.shape[:2]
    angle = rng.uniform(-12.0, 12.0)
    scale = rng.uniform(0.92, 1.08)
    matrix = cv2.getRotationMatrix2D((width / 2.0, height / 2.0), angle, scale)
    matrix[0, 2] += rng.uniform(-0.06, 0.06) * width
    matrix[1, 2] += rng.uniform(-0.06, 0.06) * height
    warped = cv2.warpAffine(
        image,
        matrix,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT101,
    )
    matrix3 = np.vstack([matrix, [0.0, 0.0, 1.0]])
    corners = labels_to_corners(labels, width, height)
    transformed = [(cls_id, transform_points(points, matrix3), area) for cls_id, points, area in corners]
    return warped, corners_to_labels(transformed, width, height), "geometry"


def apply_motion_blur(image, labels, rng):
    length = rng.choice([5, 7, 9, 13])
    kernel = np.zeros((length, length), dtype=np.float32)
    direction = rng.choice(["horizontal", "vertical", "diag_a", "diag_b"])
    if direction == "horizontal":
        kernel[length // 2, :] = 1.0
    elif direction == "vertical":
        kernel[:, length // 2] = 1.0
    elif direction == "diag_a":
        np.fill_diagonal(kernel, 1.0)
    else:
        np.fill_diagonal(np.fliplr(kernel), 1.0)
    kernel /= kernel.sum()
    return cv2.filter2D(image, -1, kernel), labels, "motion_blur"


def apply_crop(image, labels, rng):
    height, width = image.shape[:2]
    max_x = int(width * 0.12)
    max_y = int(height * 0.12)
    left = rng.randint(0, max_x)
    right = rng.randint(0, max_x)
    top = rng.randint(0, max_y)
    bottom = rng.randint(0, max_y)
    if width - left - right < width * 0.72 or height - top - bottom < height * 0.72:
        return image, labels, "crop_skipped"
    cropped = image[top : height - bottom, left : width - right]
    cropped = cv2.resize(cropped, (width, height), interpolation=cv2.INTER_LINEAR)
    scale_x = width / float(width - left - right)
    scale_y = height / float(height - top - bottom)
    transformed = []
    for cls_id, cx, cy, box_w, box_h in labels:
        x1 = (cx - box_w / 2.0) * width
        y1 = (cy - box_h / 2.0) * height
        x2 = (cx + box_w / 2.0) * width
        y2 = (cy + box_h / 2.0) * height
        points = np.array(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32
        )
        points[:, 0] = (points[:, 0] - left) * scale_x
        points[:, 1] = (points[:, 1] - top) * scale_y
        transformed.append((cls_id, points, (x2 - x1) * (y2 - y1)))
    return cropped, corners_to_labels(transformed, width, height), "crop"


def apply_occlusion(image, labels, rng):
    output = image.copy()
    height, width = output.shape[:2]
    for _ in range(rng.randint(1, 3)):
        box_w = rng.randint(max(8, width // 20), max(12, width // 7))
        box_h = rng.randint(max(8, height // 20), max(12, height // 7))
        x = rng.randint(0, max(0, width - box_w))
        y = rng.randint(0, max(0, height - box_h))
        color = tuple(int(value) for value in output.mean(axis=(0, 1)))
        cv2.rectangle(output, (x, y), (x + box_w, y + box_h), color, -1)
    return output, labels, "occlusion"


def apply_photometric(image, labels, rng):
    output = image.astype(np.float32)
    alpha = rng.uniform(0.78, 1.22)
    beta = rng.uniform(-24.0, 24.0)
    output = np.clip(output * alpha + beta, 0, 255).astype(np.uint8)
    if rng.random() < 0.65:
        hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV).astype(np.int16)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] + rng.randint(-18, 19), 0, 255)
        hsv[:, :, 2] = np.clip(hsv[:, :, 2] + rng.randint(-12, 13), 0, 255)
        output = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    if rng.random() < 0.35:
        output = cv2.GaussianBlur(output, (3, 3), 0)
    return output, labels, "photometric"


def augment_one(image, labels, rng):
    mode = rng.choice(["geometry", "motion_blur", "crop", "occlusion", "photometric"])
    if mode == "geometry":
        return apply_geometry(image, labels, rng)
    if mode == "motion_blur":
        return apply_motion_blur(image, labels, rng)
    if mode == "crop":
        return apply_crop(image, labels, rng)
    if mode == "occlusion":
        return apply_occlusion(image, labels, rng)
    return apply_photometric(image, labels, rng)


def copy_split(input_root, output_root, split):
    image_dir = input_root / "images" / split
    label_dir = input_root / "labels" / split
    for image_path in sorted(image_dir.iterdir()):
        if image_path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label_path = label_dir / f"{image_path.stem}.txt"
        if not label_path.exists():
            raise RuntimeError(f"missing label for {image_path}")
        link_or_copy(image_path, output_root / "images" / split / image_path.name)
        link_or_copy(label_path, output_root / "labels" / split / label_path.name)


def main():
    args = parse_args()
    input_root = Path(args.input_dataset).resolve()
    output_root = Path(args.output_dataset).resolve()
    if output_root.exists():
        if not args.overwrite:
            raise RuntimeError(f"output exists: {output_root}; use --overwrite after inspection")
        shutil.rmtree(output_root)
    if args.variants_per_image < 1:
        raise ValueError("--variants-per-image must be >= 1")

    with (input_root / "data.yaml").open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    output_root.mkdir(parents=True)
    copy_split(input_root, output_root, "val")
    (output_root / "images" / "train").mkdir(parents=True, exist_ok=True)
    (output_root / "labels" / "train").mkdir(parents=True, exist_ok=True)

    mode_counts = {}
    source_images = sorted(
        path for path in (input_root / "images" / "train").iterdir()
        if path.suffix.lower() in IMAGE_SUFFIXES
    )
    for index, image_path in enumerate(source_images):
        label_path = input_root / "labels" / "train" / f"{image_path.stem}.txt"
        if not label_path.exists():
            raise RuntimeError(f"missing label for {image_path}")
        link_or_copy(image_path, output_root / "images" / "train" / image_path.name)
        link_or_copy(label_path, output_root / "labels" / "train" / label_path.name)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"could not decode {image_path}")
        labels = read_labels(label_path)
        for variant in range(args.variants_per_image):
            rng = random.Random(args.seed + index * 1009 + variant * 9176)
            augmented, augmented_labels, mode = augment_one(image, labels, rng)
            mode_counts[mode] = mode_counts.get(mode, 0) + 1
            stem = f"{image_path.stem}__flightaug_{mode}_{variant:02d}"
            image_out = output_root / "images" / "train" / f"{stem}.jpg"
            label_out = output_root / "labels" / "train" / f"{stem}.txt"
            if not cv2.imwrite(str(image_out), augmented, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f"could not write {image_out}")
            write_labels(label_out, augmented_labels)

    data = dict(data)
    data["path"] = str(output_root)
    data.pop("test", None)
    (output_root / "data.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    manifest = {
        "dataset_name": "yolo_dataset_v5_flight_aug_20260713",
        "source_dataset": str(input_root),
        "variants_per_image": args.variants_per_image,
        "seed": args.seed,
        "source_train_images": len(source_images),
        "generated_augmented_images": len(source_images) * args.variants_per_image,
        "validation_policy": "unchanged v5 val split",
        "mode_counts": mode_counts,
        "classes": data.get("names"),
    }
    (output_root / "dataset_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    print(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True))


if __name__ == "__main__":
    main()
