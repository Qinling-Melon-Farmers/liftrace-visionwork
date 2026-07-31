#!/usr/bin/env python3
"""Generate class-organized backbone activation maps for a trained YOLO model.

This is a diagnostic visualization, not a detector attribution proof. It uses
the last 4-D backbone/FPN tensor, aggregates activation maps for validation
images containing each class, and saves per-class overlays and channel grids.
"""

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args():
    parser = argparse.ArgumentParser(description="生成 YOLO 六类训练特征图")
    parser.add_argument(
        "--model",
        default="/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/weights/best.pt",
    )
    parser.add_argument(
        "--dataset",
        default="/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713",
    )
    parser.add_argument(
        "--output",
        default="/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/feature_maps",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--max-samples-per-class", type=int, default=12)
    return parser.parse_args()


def load_labels(path: Path):
    labels = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        labels.append([int(parts[0])] + [float(value) for value in parts[1:]])
    return labels


def letterbox(image, size):
    height, width = image.shape[:2]
    scale = min(size / float(width), size / float(height))
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    left = (size - new_width) // 2
    top = (size - new_height) // 2
    canvas[top : top + new_height, left : left + new_width] = resized
    return canvas, scale, left, top


def normalize_heatmap(array):
    array = np.asarray(array, dtype=np.float32)
    low = float(np.percentile(array, 2.0))
    high = float(np.percentile(array, 98.0))
    if high <= low:
        return np.zeros_like(array, dtype=np.uint8)
    normalized = (array - low) / (high - low)
    return np.clip(normalized * 255.0, 0, 255).astype(np.uint8)


def overlay_heatmap(image, heatmap):
    color = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
    return cv2.addWeighted(image, 0.55, color, 0.45, 0.0)


def draw_boxes(image, labels, names, scale, left, top, size):
    output = image.copy()
    for cls_id, cx, cy, box_w, box_h in labels:
        x1 = int(left + (cx - box_w / 2.0) * size)
        y1 = int(top + (cy - box_h / 2.0) * size)
        x2 = int(left + (cx + box_w / 2.0) * size)
        y2 = int(top + (cy + box_h / 2.0) * size)
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            output,
            names.get(cls_id, str(cls_id)),
            (x1, max(18, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    return output


def save_channel_grid(feature, path):
    channels = feature.shape[0]
    scores = feature.reshape(channels, -1).mean(axis=1)
    selected = np.argsort(scores)[-16:][::-1]
    tiles = []
    for channel in selected:
        tile = normalize_heatmap(feature[channel])
        tile = cv2.applyColorMap(tile, cv2.COLORMAP_JET)
        cv2.putText(tile, f"ch {channel}", (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        tiles.append(tile)
    if not tiles:
        return
    tile_h, tile_w = tiles[0].shape[:2]
    canvas = np.zeros((tile_h * 4, tile_w * 4, 3), dtype=np.uint8)
    for index, tile in enumerate(tiles):
        row, col = divmod(index, 4)
        canvas[row * tile_h : (row + 1) * tile_h, col * tile_w : (col + 1) * tile_w] = tile
    cv2.imwrite(str(path), canvas)


def main():
    args = parse_args()
    model = YOLO(str(Path(args.model).resolve()), task="detect")
    model.model.eval()
    if str(args.device).lower() != "cpu" and torch.cuda.is_available():
        model.model.to(torch.device(f"cuda:{args.device}"))
        device = torch.device(f"cuda:{args.device}")
    else:
        model.model.to(torch.device("cpu"))
        device = torch.device("cpu")

    modules = list(model.model.model)
    captures = {}

    def make_hook(index):
        def hook(_module, _inputs, output):
            if isinstance(output, torch.Tensor) and output.ndim == 4:
                captures[index] = output.detach().float().cpu()

        return hook

    handles = [module.register_forward_hook(make_hook(index)) for index, module in enumerate(modules)]
    dataset_root = Path(args.dataset).resolve()
    image_root = dataset_root / "images" / "val"
    label_root = dataset_root / "labels" / "val"
    output_root = Path(args.output).resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    names = {int(index): str(name) for index, name in model.names.items()}
    class_images = defaultdict(list)
    for image_path in sorted(image_root.iterdir()):
        if image_path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label_path = label_root / f"{image_path.stem}.txt"
        if not label_path.exists():
            continue
        labels = load_labels(label_path)
        for cls_id in sorted({label[0] for label in labels}):
            if len(class_images[cls_id]) < args.max_samples_per_class:
                class_images[cls_id].append((image_path, labels))

    aggregate = {}
    counts = defaultdict(int)
    sample_records = defaultdict(list)
    with torch.no_grad():
        for cls_id in sorted(class_images):
            class_name = names.get(cls_id, f"class_{cls_id}")
            class_dir = output_root / class_name
            class_dir.mkdir(parents=True, exist_ok=True)
            for image_path, labels in class_images[cls_id]:
                image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                boxed, scale, left, top = letterbox(image, args.imgsz)
                tensor = torch.from_numpy(cv2.cvtColor(boxed, cv2.COLOR_BGR2RGB)).permute(2, 0, 1).float()[None] / 255.0
                tensor = tensor.to(device)
                captures.clear()
                model.model(tensor)
                valid_layers = [index for index, value in captures.items() if value.ndim == 4]
                if not valid_layers:
                    continue
                layer_index = max(valid_layers)
                feature = captures[layer_index][0].numpy()
                activation = np.mean(np.abs(feature), axis=0)
                if layer_index not in aggregate:
                    aggregate[layer_index] = {}
                if cls_id not in aggregate[layer_index]:
                    aggregate[layer_index][cls_id] = np.zeros_like(activation, dtype=np.float64)
                aggregate[layer_index][cls_id] += activation
                counts[cls_id] += 1
                heatmap = normalize_heatmap(cv2.resize(activation, (args.imgsz, args.imgsz)))
                visual = overlay_heatmap(boxed, heatmap)
                visual = draw_boxes(visual, labels, names, scale, left, top, args.imgsz)
                sample_path = class_dir / f"{image_path.stem}_layer{layer_index}.jpg"
                cv2.imwrite(str(sample_path), visual)
                if len(sample_records[cls_id]) < 1:
                    save_channel_grid(feature, class_dir / f"channels_layer{layer_index}.jpg")
                sample_records[cls_id].append(str(sample_path))

    for handle in handles:
        handle.remove()

    summary = {"model": str(Path(args.model).resolve()), "dataset": str(dataset_root), "classes": {}}
    for cls_id in sorted(counts):
        class_name = names.get(cls_id, f"class_{cls_id}")
        layer_index = max(aggregate)
        mean_activation = aggregate[layer_index][cls_id] / max(counts[cls_id], 1)
        mean_heatmap = normalize_heatmap(cv2.resize(mean_activation.astype(np.float32), (args.imgsz, args.imgsz)))
        cv2.imwrite(str(output_root / class_name / f"mean_activation_layer{layer_index}.jpg"), overlay_heatmap(np.full((args.imgsz, args.imgsz, 3), 114, dtype=np.uint8), mean_heatmap))
        summary["classes"][class_name] = {
            "class_id": cls_id,
            "sample_count": counts[cls_id],
            "layer": layer_index,
            "sample_outputs": sample_records[cls_id],
        }
    (output_root / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
