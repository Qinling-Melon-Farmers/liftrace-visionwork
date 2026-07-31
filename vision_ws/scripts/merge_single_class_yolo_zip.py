#!/usr/bin/env python3
"""Merge a single-class YOLO zip dataset into an existing multi-class YOLO dataset."""

import argparse
import os
import random
import shutil
import tempfile
import zipfile
from collections import defaultdict
from pathlib import Path

import yaml


IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
SPLIT_KEYS = ("train", "val", "test")
SPLIT_ALIASES = {
    "train": "train",
    "training": "train",
    "val": "val",
    "valid": "val",
    "validation": "val",
    "test": "test",
}


def parse_args():
    parser = argparse.ArgumentParser(description="Merge a single-class YOLO zip into an existing dataset")
    parser.add_argument(
        "--base-dataset",
        default="/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v3_bridge_manual_20260703",
        help="existing multi-class YOLO dataset root",
    )
    parser.add_argument(
        "--import-zip",
        default="/home/xhj/liftrace/vision_ws/test_data/redcross.yolov11.zip",
        help="single-class YOLO zip to import",
    )
    parser.add_argument(
        "--output-dataset",
        default="/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713",
        help="merged dataset output root",
    )
    parser.add_argument(
        "--class-names",
        nargs="+",
        default=["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"],
        help="merged dataset class names in order",
    )
    parser.add_argument(
        "--import-class-name",
        default="red_cross",
        help="class name to assign to every imported label",
    )
    parser.add_argument(
        "--import-prefix",
        default="redcross_manual_20260712",
        help="filename prefix for imported samples in merged dataset",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="deterministic seed when imported dataset needs synthetic split",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="explicitly replace an existing output dataset",
    )
    return parser.parse_args()


def link_or_copy(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        return
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def write_yaml(path: Path, data):
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")


def read_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def resolve_dataset_paths_from_yaml(dataset_root: Path):
    yaml_candidates = list(dataset_root.rglob("data.yaml")) + list(dataset_root.rglob("dataset.yaml"))
    for yaml_path in yaml_candidates:
        data = read_yaml(yaml_path)
        mapping = {}
        valid = True
        for split in SPLIT_KEYS:
            split_value = data.get(split)
            if not split_value:
                continue
            split_path = (yaml_path.parent / split_value).resolve() if not Path(split_value).is_absolute() else Path(split_value)
            if split_path.is_dir():
                images_dir = split_path
            else:
                valid = False
                break
            labels_dir = Path(str(images_dir).replace("/images/", "/labels/"))
            if not labels_dir.exists():
                valid = False
                break
            mapping[split] = (images_dir, labels_dir)
        if valid and mapping:
            return mapping, yaml_path
    return None, None


def fallback_dataset_paths(dataset_root: Path):
    patterns = [
        ("images/train", "labels/train", "train"),
        ("images/val", "labels/val", "val"),
        ("images/test", "labels/test", "test"),
        ("train/images", "train/labels", "train"),
        ("valid/images", "valid/labels", "val"),
        ("val/images", "val/labels", "val"),
        ("test/images", "test/labels", "test"),
    ]
    mapping = {}
    for image_rel, label_rel, split in patterns:
        images_dir = dataset_root / image_rel
        labels_dir = dataset_root / label_rel
        if images_dir.exists() and labels_dir.exists():
            mapping[split] = (images_dir.resolve(), labels_dir.resolve())
    return mapping


def find_image(images_dir: Path, stem: str):
    for suffix in IMAGE_SUFFIXES:
        candidate = images_dir / f"{stem}{suffix}"
        if candidate.exists():
            return candidate
    return None


def collect_samples(dataset_paths):
    samples = defaultdict(list)
    for split, (images_dir, labels_dir) in dataset_paths.items():
        for label_path in sorted(labels_dir.glob("*.txt")):
            image_path = find_image(images_dir, label_path.stem)
            if image_path is None:
                continue
            samples[split].append((image_path, label_path))
    return samples


def synthetic_split(samples, seed: int):
    flat = []
    for split_samples in samples.values():
        flat.extend(split_samples)
    if not flat:
        return {"train": [], "val": []}
    rng = random.Random(seed)
    rng.shuffle(flat)
    val_count = max(1, int(round(len(flat) * 0.2))) if len(flat) > 4 else 1 if len(flat) > 1 else 0
    val_set = flat[:val_count]
    train_set = flat[val_count:]
    return {"train": train_set, "val": val_set}


def normalize_import_samples(dataset_root: Path, seed: int):
    paths, yaml_path = resolve_dataset_paths_from_yaml(dataset_root)
    if paths is None:
        paths = fallback_dataset_paths(dataset_root)
    if not paths:
        raise RuntimeError(f"could not detect YOLO layout under {dataset_root}")
    samples = collect_samples(paths)
    if not samples.get("train") and (samples.get("val") or samples.get("test")):
        merged = synthetic_split(samples, seed)
        return merged, yaml_path
    if not samples.get("val") and not samples.get("test"):
        return synthetic_split(samples, seed), yaml_path
    return samples, yaml_path


def remap_label(label_src: Path, label_dst: Path, target_class_id: int):
    lines = []
    if label_src.exists():
        for raw in label_src.read_text(encoding="utf-8").splitlines():
            raw = raw.strip()
            if not raw:
                continue
            parts = raw.split()
            if len(parts) < 5:
                continue
            parts[0] = str(target_class_id)
            lines.append(" ".join(parts))
    label_dst.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def copy_base_dataset(base_root: Path, out_root: Path, summary):
    base_paths, _ = resolve_dataset_paths_from_yaml(base_root)
    if base_paths is None:
        base_paths = fallback_dataset_paths(base_root)
    if not base_paths:
        raise RuntimeError(f"could not detect base dataset layout under {base_root}")
    base_samples = collect_samples(base_paths)
    for split, pairs in base_samples.items():
        for image_src, label_src in pairs:
            image_dst = out_root / "images" / split / image_src.name
            label_dst = out_root / "labels" / split / label_src.name
            link_or_copy(image_src, image_dst)
            link_or_copy(label_src, label_dst)
            summary["base"][split] += 1


def extract_zip(zip_path: Path, out_dir: Path):
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as zf:
        zf.extractall(out_dir)


def merge_import_dataset(import_root: Path, out_root: Path, class_names, import_class_name, import_prefix, seed, summary):
    target_class_id = class_names.index(import_class_name)
    samples, yaml_path = normalize_import_samples(import_root, seed)
    summary["import"]["source_yaml"] = str(yaml_path) if yaml_path else None
    summary["import"]["target_class_id"] = target_class_id
    for split, pairs in samples.items():
        for idx, (image_src, label_src) in enumerate(sorted(pairs)):
            stem = f"{import_prefix}_{split}_{idx:05d}"
            image_dst = out_root / "images" / split / f"{stem}{image_src.suffix.lower()}"
            label_dst = out_root / "labels" / split / f"{stem}.txt"
            link_or_copy(image_src, image_dst)
            remap_label(label_src, label_dst, target_class_id)
            summary["import"][split] += 1


def write_data_yaml(out_root: Path, class_names):
    data = {
        "path": str(out_root),
        "train": "images/train",
        "val": "images/val",
        "nc": len(class_names),
        "names": class_names,
    }
    write_yaml(out_root / "data.yaml", data)


def main():
    args = parse_args()
    base_root = Path(args.base_dataset).resolve()
    zip_path = Path(args.import_zip).resolve()
    out_root = Path(args.output_dataset).resolve()

    if args.import_class_name not in args.class_names:
        raise ValueError(f"{args.import_class_name} not found in class names")
    if out_root.exists():
        if not args.overwrite:
            raise RuntimeError(
                f"output dataset already exists: {out_root}; "
                "use --overwrite only after verifying the replacement"
            )
        shutil.rmtree(out_root)
    (out_root / "images").mkdir(parents=True, exist_ok=True)
    (out_root / "labels").mkdir(parents=True, exist_ok=True)

    summary = {
        "base_dataset": str(base_root),
        "import_zip": str(zip_path),
        "output_dataset": str(out_root),
        "class_names": args.class_names,
        "base": defaultdict(int),
        "import": defaultdict(int),
    }

    copy_base_dataset(base_root, out_root, summary)

    with tempfile.TemporaryDirectory(prefix="liftrace_redcross_import_") as temp_dir:
        extracted_root = Path(temp_dir) / "dataset"
        extract_zip(zip_path, extracted_root)

        # If the zip contains a single top-level folder, use it as the dataset root.
        top_level = [p for p in extracted_root.iterdir()]
        import_dataset_root = extracted_root
        if len(top_level) == 1 and top_level[0].is_dir():
            import_dataset_root = top_level[0]

        merge_import_dataset(
            import_dataset_root,
            out_root,
            args.class_names,
            args.import_class_name,
            args.import_prefix,
            args.seed,
            summary,
        )
    write_data_yaml(out_root, args.class_names)

    manifest = {
        "base_dataset": summary["base_dataset"],
        "import_zip": summary["import_zip"],
        "output_dataset": summary["output_dataset"],
        "class_names": summary["class_names"],
        "base_counts": {k: int(v) for k, v in summary["base"].items()},
        "import_counts": {
            k: (int(v) if isinstance(v, int) else v) for k, v in summary["import"].items()
        },
    }
    write_yaml(out_root / "dataset_manifest.yaml", manifest)


if __name__ == "__main__":
    main()
