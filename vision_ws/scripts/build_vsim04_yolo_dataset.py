#!/usr/bin/env python3
"""Convert a validated V-SIM-04 schema-v2 capture into a YOLO dataset.

The converter groups splits by trial, never by frame, and labels the union of
the active truth target and every fully visible co-scene target.  Classes not
present in the supplied detector metadata (for example ``landing_pad``) are
reported and ignored instead of being assigned an invented class id.
"""

import argparse
import hashlib
import json
import math
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

import yaml

VISION_WS_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VISION_WS_ROOT / "src/uav_vision_eval/src"))

from uav_vision_eval.failure_capture import validate_capture_manifest


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--metadata", type=Path,
        default=(VISION_WS_ROOT / "src/uav_vision/config/"
                 "merged_standard_6cls_metadata.yaml"))
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--test-fraction", type=float, default=0.0)
    parser.add_argument("--min-box-px", type=float, default=2.0)
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def detector_classes(metadata_path):
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    raw = metadata.get("names") if isinstance(metadata, dict) else None
    if not isinstance(raw, dict) or not raw:
        raise ValueError("detector metadata names must be a non-empty mapping")
    names = {int(index): str(name) for index, name in raw.items()}
    if sorted(names) != list(range(len(names))):
        raise ValueError("detector class ids must be contiguous from zero")
    if len(set(names.values())) != len(names):
        raise ValueError("detector class names must be unique")
    return names


def validate_capture(payload, capture_root):
    validate_capture_manifest(payload, str(capture_root))
    if payload.get("schema_version") != 2:
        raise ValueError("capture manifest schema_version must be 2")
    if payload.get("dataset_kind") != "sim-small-target":
        raise ValueError("unexpected capture dataset_kind")
    if payload.get("status") != "DIAGNOSTIC":
        raise ValueError("capture status must be DIAGNOSTIC")
    if payload.get("run_complete") is not True:
        raise ValueError("capture run_complete must be true")
    records = payload.get("records")
    if not isinstance(records, list) or not records:
        raise ValueError("capture records must be a non-empty list")
    if int(payload.get("captured_frames", -1)) != len(records):
        raise ValueError("captured_frames does not match records")
    if int(payload.get("max_frames", -1)) != len(records):
        raise ValueError("capture is incomplete")
    expected_indices = list(range(1, len(records) + 1))
    if [int(record.get("capture_index", -1)) for record in records] != \
            expected_indices:
        raise ValueError("capture indices are not contiguous")
    for record in records:
        for field in ("image_file", "metadata_file"):
            value = Path(str(record.get(field, "")))
            if not value.name or value.is_absolute() or value.name != str(value):
                raise ValueError("unsafe {} in capture record".format(field))
            path = capture_root / value
            if not path.is_file() or path.stat().st_size <= 0:
                raise ValueError("missing or empty capture file: {}".format(path))
        image = record.get("image", {})
        if (int(image.get("width", 0)) <= 0 or
                int(image.get("height", 0)) <= 0 or
                image.get("saved_encoding") != "bgr8"):
            raise ValueError("invalid saved image contract")
        if not isinstance(record.get("scene_targets"), list):
            raise ValueError("scene_targets must be a list")
    return records


def split_trials(trial_ids, seed, val_fraction, test_fraction):
    if seed <= 0:
        raise ValueError("seed must be positive")
    for name, value in (("val_fraction", val_fraction),
                        ("test_fraction", test_fraction)):
        if not math.isfinite(value) or value < 0.0 or value >= 1.0:
            raise ValueError("{} must be in [0, 1)".format(name))
    if val_fraction + test_fraction >= 1.0:
        raise ValueError("val_fraction + test_fraction must be below 1")
    trial_ids = sorted(set(trial_ids), key=lambda trial_id: hashlib.sha256(
        "{}:{}".format(seed, trial_id).encode("utf-8")).hexdigest())
    count = len(trial_ids)
    test_count = int(round(count * test_fraction))
    val_count = int(round(count * val_fraction))
    if test_fraction > 0.0 and count >= 3:
        test_count = max(1, test_count)
    if val_fraction > 0.0 and count - test_count >= 2:
        val_count = max(1, val_count)
    while test_count + val_count >= count and (test_count or val_count):
        if val_count >= test_count and val_count:
            val_count -= 1
        else:
            test_count -= 1
    assignment = {}
    for trial_id in trial_ids[:test_count]:
        assignment[trial_id] = "test"
    for trial_id in trial_ids[test_count:test_count + val_count]:
        assignment[trial_id] = "val"
    for trial_id in trial_ids[test_count + val_count:]:
        assignment[trial_id] = "train"
    return assignment


def clipped_label(target, width, height, class_to_id, min_box_px):
    class_name = str(target.get("class_name", ""))
    if class_name not in class_to_id:
        return None, "class_not_in_detector_metadata"
    roi = target.get("roi")
    if not isinstance(roi, dict):
        return None, "roi_missing"
    try:
        x1 = max(0.0, min(float(width), float(roi["x_offset"])))
        y1 = max(0.0, min(float(height), float(roi["y_offset"])))
        x2 = max(0.0, min(float(width),
                          float(roi["x_offset"]) + float(roi["width"])))
        y2 = max(0.0, min(float(height),
                          float(roi["y_offset"]) + float(roi["height"])))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None, "roi_invalid"
    box_width = x2 - x1
    box_height = y2 - y1
    if (not all(math.isfinite(value) for value in
                (x1, y1, x2, y2, box_width, box_height)) or
            box_width < min_box_px or box_height < min_box_px):
        return None, "roi_too_small"
    values = (
        (x1 + x2) * 0.5 / width,
        (y1 + y2) * 0.5 / height,
        box_width / width,
        box_height / height,
    )
    if not all(0.0 <= value <= 1.0 for value in values):
        return None, "normalized_roi_invalid"
    return (class_to_id[class_name], values), ""


def record_targets(record):
    # Co-scene entries are inserted first; exact-stamp active truth wins when
    # both describe the same physical target.
    targets = {}
    for target in record.get("scene_targets", []):
        target_id = str(target.get("target_id", ""))
        if target_id:
            targets[target_id] = dict(target)
    active = record.get("truth", {})
    target_id = str(active.get("target_id", ""))
    if target_id:
        targets[target_id] = dict(active)
    return [targets[target_id] for target_id in sorted(targets)]


def prepare_output(output):
    if output.exists() and any(output.iterdir()):
        raise ValueError("output directory must be absent or empty: {}".format(
            output))
    output.mkdir(parents=True, exist_ok=True)
    for split in ("train", "val", "test"):
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)


def main():
    args = parse_args()
    if not math.isfinite(args.min_box_px) or args.min_box_px <= 0.0:
        raise ValueError("min_box_px must be positive and finite")
    manifest_path = args.manifest.resolve()
    capture_root = manifest_path.parent
    payload = load_json(manifest_path)
    records = validate_capture(payload, capture_root)
    names = detector_classes(args.metadata.resolve())
    class_to_id = {name: index for index, name in names.items()}
    assignments = split_trials(
        [record["trial"]["trial_id"] for record in records],
        args.seed, args.val_fraction, args.test_fraction)
    output = args.output.resolve()
    prepare_output(output)

    converted = []
    class_counts = Counter()
    ignored_counts = Counter()
    split_counts = Counter()
    trial_split = defaultdict(set)
    for record in records:
        trial_id = str(record["trial"]["trial_id"])
        split = assignments[trial_id]
        trial_split[trial_id].add(split)
        source_image = capture_root / record["image_file"]
        target_image = output / "images" / split / source_image.name
        target_label = output / "labels" / split / (
            source_image.stem + ".txt")
        shutil.copy2(source_image, target_image)
        image = record["image"]
        width = int(image["width"])
        height = int(image["height"])
        labels = []
        label_meta = []
        for target in record_targets(record):
            label, reason = clipped_label(
                target, width, height, class_to_id, args.min_box_px)
            if label is None:
                ignored_counts["{}:{}".format(
                    target.get("class_name", ""), reason)] += 1
                continue
            class_id, values = label
            labels.append((class_id, str(target.get("target_id", "")), values))
            class_counts[names[class_id]] += 1
            label_meta.append({
                "target_id": str(target.get("target_id", "")),
                "class_name": names[class_id],
                "class_id": class_id,
                "fully_in_frame": bool(target.get("fully_in_frame", False)),
            })
        labels.sort(key=lambda value: (value[0], value[1]))
        target_label.write_text("".join(
            "{} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(
                class_id, *values)
            for class_id, _, values in labels), encoding="utf-8")
        split_counts[split] += 1
        converted.append({
            "capture_index": int(record["capture_index"]),
            "trial_id": trial_id,
            "split": split,
            "image": str(target_image.relative_to(output)),
            "label": str(target_label.relative_to(output)),
            "labels": label_meta,
        })

    if any(len(values) != 1 for values in trial_split.values()):
        raise RuntimeError("a trial was split across dataset partitions")
    dataset_yaml = {
        "path": ".",
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": names,
    }
    with (output / "dataset.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(dataset_yaml, stream, allow_unicode=True,
                       sort_keys=False)
    result = {
        "schema_version": 1,
        "source_manifest": str(manifest_path),
        "source_schema_version": payload["schema_version"],
        "detector_metadata": str(args.metadata.resolve()),
        "seed": args.seed,
        "split_policy": "sha256(seed:trial_id), trial-grouped",
        "split_fractions": {
            "val": args.val_fraction,
            "test": args.test_fraction,
        },
        "trial_assignments": dict(sorted(assignments.items())),
        "frame_counts": dict(sorted(split_counts.items())),
        "class_label_counts": dict(sorted(class_counts.items())),
        "ignored_target_counts": dict(sorted(ignored_counts.items())),
        "records": converted,
        "training_readiness": {
            "ready": False,
            "reason": (
                "diagnostic captures require held-out diversity review and "
                "must not be promoted to training solely because conversion "
                "succeeded"),
        },
    }
    (output / "conversion_manifest.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "frames": len(converted),
        "frame_counts": result["frame_counts"],
        "class_label_counts": result["class_label_counts"],
        "ignored_target_counts": result["ignored_target_counts"],
        "training_ready": False,
    }, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
