#!/usr/bin/env python3
"""Generate the 6-degree rotation set for the official non-tank first images.

The existing ``vision_ws/test_data/rotated`` set contains one image every
6 degrees (000..354).  This script applies the same angular coverage to the
first official image of bridge, car, H, pillbox and tent.  Tank is deliberately
excluded because its existing rotation set is already available.

This is an image-only preparation step.  The official first images are not
trusted as automatically labelled training data yet, so no YOLO label files
are fabricated.  A manifest records the semantic mapping for the later manual
annotation pass.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import zipfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SOURCE_KEYS = ("bridge", "car", "H", "pillbox", "tent")
CLASS_MAPPING = {
    "bridge": "bridge (YOLO id 0)",
    "car": "panzer (YOLO id 1)",
    "pillbox": "pillbox (YOLO id 2)",
    "tent": "tent (YOLO id 3)",
    "H": "landing marker; no six-class YOLO label",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成官方非 tank 首图的 6° 旋转图像集")
    parser.add_argument("--archive", default="实物组数据集.zip")
    parser.add_argument(
        "--output",
        default="vision_ws/test_data/official_first_non_tank_rotated_20260713",
    )
    parser.add_argument("--angle-step", type=int, default=6)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def source_key(member_name: str) -> Optional[str]:
    """Return the official class from an ASCII basename despite ZIP encoding."""

    stem = Path(member_name).stem
    match = re.match(r"^(bridge|car|pillbox|tent|H|h|tank)(\d+)$", stem, re.IGNORECASE)
    if match is None:
        return None
    key = match.group(1)
    return "H" if key.lower() == "h" else key.lower()


def source_index(member_name: str) -> int:
    match = re.search(r"(\d+)$", Path(member_name).stem)
    return int(match.group(1)) if match else 0


def first_members(archive: zipfile.ZipFile) -> Dict[str, zipfile.ZipInfo]:
    candidates: Dict[str, List[zipfile.ZipInfo]] = {key: [] for key in SOURCE_KEYS}
    for member in archive.infolist():
        if member.is_dir() or Path(member.filename).suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        key = source_key(member.filename)
        if key in candidates:
            candidates[key].append(member)

    selected: Dict[str, zipfile.ZipInfo] = {}
    for key in SOURCE_KEYS:
        items = sorted(candidates[key], key=lambda item: (source_index(item.filename), item.filename))
        if not items or source_index(items[0].filename) != 1:
            raise RuntimeError(f"未找到 {key}1 首图；候选数量={len(items)}")
        selected[key] = items[0]
    return selected


def decode_member(archive: zipfile.ZipFile, member: zipfile.ZipInfo) -> np.ndarray:
    encoded = np.frombuffer(archive.read(member), dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"无法解码官方图片: {member.filename}")
    return image


def border_fill(image: np.ndarray) -> Tuple[int, int, int]:
    border = np.concatenate(
        [image[0, :, :], image[-1, :, :], image[:, 0, :], image[:, -1, :]],
        axis=0,
    )
    median = np.median(border, axis=0).astype(np.uint8)
    return tuple(int(value) for value in median)


def pad_to_square(image: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int, int]]:
    height, width = image.shape[:2]
    side = max(height, width)
    fill = border_fill(image)
    canvas = np.empty((side, side, 3), dtype=np.uint8)
    canvas[:, :] = fill
    x0 = (side - width) // 2
    y0 = (side - height) // 2
    canvas[y0 : y0 + height, x0 : x0 + width] = image
    return canvas, fill


def rotate_square(image: np.ndarray, angle: int, fill: Tuple[int, int, int]) -> np.ndarray:
    side = image.shape[0]
    matrix = cv2.getRotationMatrix2D((side / 2.0, side / 2.0), float(angle), 1.0)
    return cv2.warpAffine(
        image,
        matrix,
        (side, side),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=fill,
    )


def write_readme(output_root: Path, angle_step: int, total_images: int) -> None:
    lines = [
        "# 官方标准靶标首图旋转集（非 tank）",
        "",
        "本目录参照 `vision_ws/test_data/rotated/` 生成：每个角度间隔 6°，从 000° 到 354°，共 60 个角度。",
        f"本批次包含 {total_images} 张图像，来源为比赛方 `实物组数据集.zip` 中各类别编号为 1 的首图。",
        "",
        "## 当前内容",
        "",
        "- `images/`：已生成旋转图像；文件名中的 `rotNNN` 是逆时针旋转角度。",
        "- `labels/`：暂不写入 YOLO 标签；后续手工标注后再放入同名 `.txt`。",
        "- `manifest.json`：源文件、类别语义和生成参数记录。",
        "",
        "## 类别语义",
        "",
        "- `bridge` → 六分类 `bridge`，YOLO id 0。",
        "- `car` → 六分类 `panzer`，YOLO id 1。",
        "- `pillbox` → 六分类 `pillbox`，YOLO id 2。",
        "- `tent` → 六分类 `tent`，YOLO id 3。",
        "- `H` → 降落标志，不属于六分类目标，不得标成 `red_cross`。",
        "- `tank` 未生成，本目录只处理非 tank 首图。",
        "",
        "## 标注注意",
        "",
        "官方首图当前只作为旋转素材，不把模型自动框当作真值。手工标注时请统一标注靶板目标框；圆环中心后续由视觉几何链单独精修。",
        "由于 bridge/car/pillbox/tent 首图为 800×600，生成前补齐为不拉伸的正方形画布，以避免旋转时裁掉圆形靶板；H 首图保持其正方形尺寸。",
        "",
        f"angle_step: {angle_step} degrees",
        "label_status: pending_manual_annotation",
    ]
    (output_root / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    if args.angle_step <= 0 or 360 % args.angle_step != 0:
        raise ValueError("--angle-step 必须是能整除 360 的正整数")

    workspace = Path(__file__).resolve().parents[2]
    archive_path = (workspace / args.archive).resolve()
    output_root = (workspace / args.output).resolve()
    if not archive_path.is_file():
        raise FileNotFoundError(archive_path)
    if output_root.exists():
        if not args.overwrite:
            raise FileExistsError(f"输出目录已存在，使用 --overwrite 才允许重建: {output_root}")
        shutil.rmtree(output_root)

    image_root = output_root / "images"
    label_root = output_root / "labels"
    image_root.mkdir(parents=True)
    label_root.mkdir()
    (label_root / "README.md").write_text(
        "此目录预留给后续手工生成的 YOLO 标签；当前旋转集不含自动标签。\n",
        encoding="utf-8",
    )

    angles = list(range(0, 360, args.angle_step))
    records = []
    with zipfile.ZipFile(archive_path) as archive:
        selected = first_members(archive)
        for key in SOURCE_KEYS:
            member = selected[key]
            original = decode_member(archive, member)
            square, fill = pad_to_square(original)
            source_name = f"{key}1{Path(member.filename).suffix.lower()}"
            source_dir = output_root / "source_images"
            source_dir.mkdir(exist_ok=True)
            cv2.imwrite(str(source_dir / source_name), original, [cv2.IMWRITE_JPEG_QUALITY, 98])
            for angle in angles:
                rotated = rotate_square(square, angle, fill)
                filename = f"official_{key}_first_rot{angle:03d}.jpg"
                output_path = image_root / filename
                if not cv2.imwrite(str(output_path), rotated, [cv2.IMWRITE_JPEG_QUALITY, 98]):
                    raise RuntimeError(f"无法写出旋转图片: {output_path}")
                records.append(
                    {
                        "file": f"images/{filename}",
                        "source_member": member.filename,
                        "source_basename": Path(member.filename).name,
                        "source_key": key,
                        "semantic_mapping": CLASS_MAPPING[key],
                        "angle_degrees_ccw": angle,
                        "label_status": "pending_manual_annotation",
                    }
                )

    manifest = {
        "dataset_name": output_root.name,
        "source_archive": str(archive_path),
        "source_policy": "official class first image only; tank excluded",
        "classes_generated": list(SOURCE_KEYS),
        "class_mapping": CLASS_MAPPING,
        "angle_step_degrees": args.angle_step,
        "angles_degrees": angles,
        "images_per_source": len(angles),
        "total_images": len(records),
        "geometry": {
            "rotation": "cv2 warpAffine around image center",
            "canvas": "pad to square before rotation; no resize distortion",
            "border_fill": "median of source image border pixels",
        },
        "labels": {
            "status": "pending_manual_annotation",
            "generated": False,
            "h_policy": "landing marker only; never red_cross",
        },
        "records": records,
    }
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_readme(output_root, args.angle_step, len(records))
    print(json.dumps({"output": str(output_root), "total_images": len(records), "sources": list(SOURCE_KEYS)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
