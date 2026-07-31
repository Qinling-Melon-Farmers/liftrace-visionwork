#!/usr/bin/env python3
"""离线完整视觉链视频评测。

目标：
1. 用五分类 YOLO 跑标准目标检测
2. 用与 uav_vision 当前实现一致的传统视觉逻辑补充 red_cross / landing_pad
3. 生成统一的可视化视频、结构化 CSV/JSON 和样本帧，便于核验
"""

import argparse
import csv
import json
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import yaml
from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="完整视觉链视频评测")
    parser.add_argument("--video", required=True, help="输入视频路径")
    parser.add_argument("--model", required=True, help="五分类模型路径")
    parser.add_argument("--output-dir", required=True, help="输出目录")
    parser.add_argument(
        "--cross-config",
        default="vision_ws/src/uav_vision/config/cross_detector.yaml",
        help="cross_detector 参数文件",
    )
    parser.add_argument(
        "--landing-config",
        default="vision_ws/src/uav_vision/config/landing_detector.yaml",
        help="landing_detector 参数文件",
    )
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO 置信度阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO 推理尺寸")
    parser.add_argument("--device", default="0", help="YOLO 推理设备")
    parser.add_argument(
        "--detector-mode",
        choices=["unified5", "legacy_split"],
        default="unified5",
        help="标准目标检测器模式：五分类统一模型，或旧 4 类+单独 tank",
    )
    parser.add_argument(
        "--tank-model",
        default="vision_ws/src/yolov5_detect/tank.pt",
        help="legacy_split 模式下的单独 tank 模型路径",
    )
    parser.add_argument(
        "--output-fps",
        type=float,
        default=0.0,
        help="输出标记视频 FPS，0 表示自动按输入 FPS / frame_stride",
    )
    parser.add_argument(
        "--cross-mode",
        choices=["current", "legacy_simple"],
        default="current",
        help="red_cross 检测模式",
    )
    parser.add_argument(
        "--landing-mode",
        choices=["current", "legacy_old"],
        default="current",
        help="landing_pad 检测模式",
    )
    parser.add_argument(
        "--suppress-bridge-on-red-cross",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="若 red_cross 通过几何验证，则抑制同帧 bridge",
    )
    parser.add_argument(
        "--suppress-bridge-on-landing-pad",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="若 landing_pad 通过几何验证，则抑制同帧 bridge",
    )
    parser.add_argument(
        "--aux-geometry-threshold",
        type=float,
        default=0.85,
        help="辅助几何链触发 bridge 抑制的 geometry_confidence 阈值",
    )
    parser.add_argument("--frame-stride", type=int, default=1, help="视频处理步长")
    parser.add_argument("--sample-stride", type=int, default=180, help="周期样本保存步长")
    parser.add_argument("--event-gap", type=int, default=45, help="事件样本最小间隔（帧）")
    parser.add_argument("--max-event-frames", type=int, default=300, help="最多保存事件样本数")
    parser.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧，0 表示全部")
    parser.add_argument("--clean", action="store_true", help="运行前清空输出目录")
    return parser.parse_args()


def load_yaml(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def format_timestamp(frame_idx: int, fps: float) -> str:
    seconds = frame_idx / fps if fps > 0 else 0.0
    minutes = int(seconds // 60)
    remain = seconds - minutes * 60
    return f"{minutes:02d}:{remain:06.3f}"


def save_frame(path: Path, frame):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"写入图像失败: {path}")


def calculate_solidity(contour):
    contour_area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    return contour_area / hull_area if hull_area > 0 else 0.0


def calculate_extent(contour):
    x, y, w, h = cv2.boundingRect(contour)
    rect_area = float(w * h)
    return cv2.contourArea(contour) / rect_area if rect_area > 0 else 0.0


def count_concave_points(contour, depth_threshold):
    hull_indices = cv2.convexHull(contour, returnPoints=False)
    defects = cv2.convexityDefects(contour, hull_indices) if hull_indices is not None and len(hull_indices) >= 3 else None
    concave_points = 0
    if defects is not None:
        for d in defects[:, 0]:
            depth = d[3] / 256.0
            if depth > depth_threshold:
                concave_points += 1
    return concave_points


def calculate_center_coverage(mask, contour):
    br = cv2.boundingRect(contour)
    if br[2] <= 0 or br[3] <= 0:
        return 0.0, 0.0
    m = cv2.moments(contour)
    cx = (m["m10"] / m["m00"]) if m["m00"] > 0 else (br[0] + br[2] * 0.5)
    cy = (m["m01"] / m["m00"]) if m["m00"] > 0 else (br[1] + br[3] * 0.5)
    cxi = max(br[0], min(br[0] + br[2] - 1, int(round(cx))))
    cyi = max(br[1], min(br[1] + br[3] - 1, int(round(cy))))
    h_hit = sum(1 for x in range(br[0], br[0] + br[2]) if mask[cyi, x] > 0)
    v_hit = sum(1 for y in range(br[1], br[1] + br[3]) if mask[y, cxi] > 0)
    return (
        h_hit / float(br[2]) if br[2] > 0 else 0.0,
        v_hit / float(br[3]) if br[3] > 0 else 0.0,
    )


def is_cross_like_shape(contour):
    hull = cv2.convexHull(contour)
    if len(hull) < 8:
        return False
    r = cv2.boundingRect(contour)
    rc_x = r[0] + r[2] / 2.0
    rc_y = r[1] + r[3] / 2.0
    top = bottom = left = right = False
    for pt in contour.reshape(-1, 2):
        x, y = pt
        if y < rc_y - r[3] * 0.3:
            top = True
        if y > rc_y + r[3] * 0.3:
            bottom = True
        if x < rc_x - r[2] * 0.3:
            left = True
        if x > rc_x + r[2] * 0.3:
            right = True
    return top and bottom and left and right


def check_black_outer_ring(bgr, center, roi_radius):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    cx = int(center[0])
    cy = int(center[1])
    r = int(roi_radius)
    x0 = max(cx - r, 0)
    y0 = max(cy - r, 0)
    x1 = min(cx + r, gray.shape[1])
    y1 = min(cy + r, gray.shape[0])
    if x1 - x0 < 10 or y1 - y0 < 10:
        return True
    patch = gray[y0:y1, x0:x1]
    dark = int((patch < 60).sum())
    ratio = dark / float(patch.size)
    return ratio > 0.15


def detect_red_cross(image, cfg):
    red_s_min = int(cfg.get("red_s_min", 50))
    red_v_min = int(cfg.get("red_v_min", 50))
    red_s_max = int(cfg.get("red_s_max", 255))
    red_v_max = int(cfg.get("red_v_max", 255))
    cross_aspect_ratio_min = float(cfg.get("cross_aspect_ratio_min", 0.6))
    cross_min_contour_points = int(cfg.get("cross_min_contour_points", 20))
    cross_area_min = float(cfg.get("cross_area_min", 200.0))
    cross_solidity_min = float(cfg.get("cross_solidity_min", 0.6))
    cross_solidity_max = float(cfg.get("cross_solidity_max", 0.85))
    cross_enable_relaxed_scoring = bool(cfg.get("cross_enable_relaxed_scoring", True))
    cross_relaxed_min_score = float(cfg.get("cross_relaxed_min_score", 3.0))
    cross_relaxed_depth_threshold = float(cfg.get("cross_relaxed_depth_threshold", 10.0))
    cross_relaxed_min_solidity = float(cfg.get("cross_relaxed_min_solidity", 0.4))
    cross_relaxed_max_solidity = float(cfg.get("cross_relaxed_max_solidity", 0.9))
    cross_relaxed_min_extent = float(cfg.get("cross_relaxed_min_extent", 0.2))
    cross_relaxed_max_extent = float(cfg.get("cross_relaxed_max_extent", 0.75))
    cross_relaxed_max_aspect_ratio = float(cfg.get("cross_relaxed_max_aspect_ratio", 2.0))
    cross_relaxed_prefer_aspect_ratio = float(cfg.get("cross_relaxed_prefer_aspect_ratio", 1.4))
    cross_relaxed_min_cover = float(cfg.get("cross_relaxed_min_cover", 0.3))
    cross_relaxed_good_cover = float(cfg.get("cross_relaxed_good_cover", 0.5))
    cross_relaxed_min_concave_points = int(cfg.get("cross_relaxed_min_concave_points", 2))
    cross_relaxed_max_concave_points = int(cfg.get("cross_relaxed_max_concave_points", 6))
    cross_relaxed_prefer_concave_points = int(cfg.get("cross_relaxed_prefer_concave_points", 4))
    enable_gaussian_blur = bool(cfg.get("enable_gaussian_blur", True))
    blur_kernel_size = int(cfg.get("blur_kernel_size", 5))
    enable_black_ring_check = bool(cfg.get("enable_black_ring_check", False))
    black_ring_roi_radius = int(cfg.get("black_ring_roi_radius", 80))

    processed = image
    if enable_gaussian_blur:
        ks = blur_kernel_size + 1 if blur_kernel_size % 2 == 0 else blur_kernel_size
        processed = cv2.GaussianBlur(image, (ks, ks), 0)

    hsv = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(
        hsv,
        (0, red_s_min, red_v_min),
        (10, red_s_max, red_v_max),
    )
    mask2 = cv2.inRange(
        hsv,
        (170, red_s_min, red_v_min),
        (180, red_s_max, red_v_max),
    )
    red_mask = cv2.bitwise_or(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        if len(contour) < cross_min_contour_points:
            continue
        area = cv2.contourArea(contour)
        if area < cross_area_min:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue
        ar = min(w, h) / float(max(w, h))
        solidity = calculate_solidity(contour)
        m = cv2.moments(contour)
        if m["m00"] <= 0:
            continue
        center = (float(m["m10"] / m["m00"]), float(m["m01"] / m["m00"]))
        score = None
        geometry_confidence = None

        strict_ok = (
            ar >= cross_aspect_ratio_min
            and cross_solidity_min <= solidity <= cross_solidity_max
            and is_cross_like_shape(contour)
        )
        if strict_ok:
            score = max(0.0, min(1.0, solidity))
            geometry_confidence = score
        elif cross_enable_relaxed_scoring:
            extent = calculate_extent(contour)
            aspect_ratio = max(w, h) / float(max(1, min(w, h)))
            concave_points = count_concave_points(contour, cross_relaxed_depth_threshold)
            h_cover, v_cover = calculate_center_coverage(red_mask, contour)
            relaxed_score = 0.0
            if cross_relaxed_min_concave_points <= concave_points <= cross_relaxed_max_concave_points:
                relaxed_score += 2.0
            if concave_points == cross_relaxed_prefer_concave_points:
                relaxed_score += 1.0
            if aspect_ratio <= cross_relaxed_max_aspect_ratio:
                relaxed_score += 1.0
            if aspect_ratio <= cross_relaxed_prefer_aspect_ratio:
                relaxed_score += 1.0
            if cross_relaxed_min_solidity <= solidity <= cross_relaxed_max_solidity:
                relaxed_score += 1.0
            if cross_relaxed_min_extent <= extent <= cross_relaxed_max_extent:
                relaxed_score += 1.0
            if h_cover > cross_relaxed_min_cover and v_cover > cross_relaxed_min_cover:
                relaxed_score += 1.0
            if h_cover > cross_relaxed_good_cover and v_cover > cross_relaxed_good_cover:
                relaxed_score += 0.5
            relaxed_score += min(area / 5000.0, 2.0)
            if relaxed_score >= cross_relaxed_min_score:
                score = max(0.0, min(1.0, relaxed_score / 8.5))
                geometry_confidence = score

        if score is None or geometry_confidence is None:
            continue
        candidate = {
            "center": center,
            "area": float(m["m00"]),
            "bbox": [int(x), int(y), int(x + w), int(y + h)],
            "solidity": float(solidity),
            "contour": contour,
            "mask": red_mask,
            "score": score,
            "geometry_confidence": geometry_confidence,
        }
        if best is None or candidate["area"] > best["area"]:
            best = candidate

    if best is None:
        return None

    if enable_black_ring_check and not check_black_outer_ring(image, best["center"], black_ring_roi_radius):
        return None

    return {
        "class_name": "red_cross",
        "confidence": best["score"],
        "geometry_confidence": best["geometry_confidence"],
        "center": best["center"],
        "bbox": best["bbox"],
        "area": best["area"],
        "score": best["score"],
    }


def detect_red_cross_legacy_simple(image, cfg):
    s_min = int(cfg.get("red_s_min", 50))
    s_max = int(cfg.get("red_s_max", 255))
    v_min = int(cfg.get("red_v_min", 50))
    v_max = int(cfg.get("red_v_max", 255))
    depth_threshold = float(cfg.get("cross_relaxed_depth_threshold", 10.0))
    contours_area_threshold = float(cfg.get("legacy_simple_contours_area_threshold", 500.0))
    morphology_kernel_size = int(cfg.get("legacy_simple_morphology_kernel_size", 15))
    morphology_kernel_size = morphology_kernel_size if morphology_kernel_size % 2 == 1 else morphology_kernel_size + 1

    src_h, src_w = image.shape[:2]
    resized = cv2.resize(image, (640, 512), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, (0, s_min, v_min), (8, s_max, v_max))
    mask2 = cv2.inRange(hsv, (172, s_min, v_min), (180, s_max, v_max))
    mask = cv2.bitwise_or(mask1, mask2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morphology_kernel_size, morphology_kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_score = -1e9
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < contours_area_threshold:
            continue
        br = cv2.boundingRect(contour)
        rect_area = float(br[2] * br[3])
        if rect_area <= 0:
            continue
        extent = area / rect_area
        hull = cv2.convexHull(contour, returnPoints=True)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0.0
        rrect = cv2.minAreaRect(contour)
        w = max(1.0, float(rrect[1][0]))
        h = max(1.0, float(rrect[1][1]))
        ar = (w / h) if w > h else (h / w)
        hull_indices = cv2.convexHull(contour, returnPoints=False)
        defects = cv2.convexityDefects(contour, hull_indices) if hull_indices is not None and len(hull_indices) >= 3 else None
        concave_points = 0
        if defects is not None:
            for d in defects[:, 0]:
                depth = d[3] / 256.0
                if depth > depth_threshold:
                    concave_points += 1
        m = cv2.moments(contour)
        cx = (m["m10"] / m["m00"]) if m["m00"] > 0 else (br[0] + br[2] * 0.5)
        cy = (m["m01"] / m["m00"]) if m["m00"] > 0 else (br[1] + br[3] * 0.5)
        cxi = max(br[0], min(br[0] + br[2] - 1, int(round(cx))))
        cyi = max(br[1], min(br[1] + br[3] - 1, int(round(cy))))
        h_hit = sum(1 for x in range(br[0], br[0] + br[2]) if mask[cyi, x] > 0)
        v_hit = sum(1 for y in range(br[1], br[1] + br[3]) if mask[y, cxi] > 0)
        h_cover = h_hit / float(br[2]) if br[2] > 0 else 0.0
        v_cover = v_hit / float(br[3]) if br[3] > 0 else 0.0

        score = 0.0
        if 2 <= concave_points <= 6:
            score += 2.0
        if concave_points == 4:
            score += 1.0
        if ar <= 2.0:
            score += 1.0
        if ar <= 1.4:
            score += 1.0
        if 0.4 <= solidity <= 0.9:
            score += 1.0
        if 0.2 <= extent <= 0.75:
            score += 1.0
        if h_cover > 0.3 and v_cover > 0.3:
            score += 1.0
        if h_cover > 0.5 and v_cover > 0.5:
            score += 0.5
        score += min(area / 5000.0, 2.0)
        if score >= 3.0 and score > best_score:
            best_score = score
            best = {
                "center": (cx, cy),
                "bbox": [br[0], br[1], br[0] + br[2], br[1] + br[3]],
                "score": score,
                "area": area,
            }

    if best is None:
        return None

    sx = src_w / 640.0
    sy = src_h / 512.0
    bbox = [
        int(best["bbox"][0] * sx),
        int(best["bbox"][1] * sy),
        int(best["bbox"][2] * sx),
        int(best["bbox"][3] * sy),
    ]
    center = (best["center"][0] * sx, best["center"][1] * sy)
    score_norm = max(0.0, min(1.0, best["score"] / 8.5))
    return {
        "class_name": "red_cross",
        "confidence": score_norm,
        "geometry_confidence": score_norm,
        "center": center,
        "bbox": bbox,
        "area": best["area"] * sx * sy,
        "score": score_norm,
    }


def detect_landing_pad(image, cfg):
    blur_kernel_size = int(cfg.get("landing_blur_kernel_size", 5))
    adaptive_block_size = int(cfg.get("landing_adaptive_block_size", 31))
    adaptive_c = float(cfg.get("landing_adaptive_c", 10.0))
    morphology_kernel_size = int(cfg.get("landing_morphology_kernel_size", 7))
    min_contour_points = int(cfg.get("landing_min_contour_points", 15))
    aspect_ratio_threshold = float(cfg.get("landing_aspect_ratio_threshold", 0.85))
    radius_min = float(cfg.get("landing_radius_min", 15.0))
    radius_max = float(cfg.get("landing_radius_max", 300.0))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    bks = blur_kernel_size + 1 if blur_kernel_size % 2 == 0 else blur_kernel_size
    gray = cv2.GaussianBlur(gray, (bks, bks), 0)

    absz = adaptive_block_size + 1 if adaptive_block_size % 2 == 0 else adaptive_block_size
    binary = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        absz,
        adaptive_c,
    )

    mks = morphology_kernel_size + 1 if morphology_kernel_size % 2 == 0 else morphology_kernel_size
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mks, mks))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        if len(contour) < min_contour_points:
            continue
        if len(contour) < 5:
            continue
        try:
            ellipse = cv2.fitEllipse(contour)
        except cv2.error:
            continue
        area = cv2.contourArea(contour)
        w, h = ellipse[1]
        if w < 1e-3 or h < 1e-3:
            continue
        ar = min(w, h) / max(w, h)
        if ar < aspect_ratio_threshold:
            continue
        radius = (w + h) / 4.0
        if radius < radius_min or radius > radius_max:
            continue
        candidate = {
            "center": (float(ellipse[0][0]), float(ellipse[0][1])),
            "radius": float(radius),
            "bbox": list(cv2.boundingRect(contour)),
            "area": float(area),
            "ellipse": ellipse,
            "contour_points": int(len(contour)),
        }
        if best is None or candidate["area"] > best["area"]:
            best = candidate

    if best is None:
        return None

    area_score = max(0.0, min(1.0, best["area"] / 5000.0))
    radius_score = max(0.0, min(1.0, best["radius"] / max(radius_min, 1.0)))
    aspect_ratio = min(best["ellipse"][1]) / max(best["ellipse"][1])
    geom_conf = max(0.0, min(1.0, 0.5 * aspect_ratio + 0.3 * area_score + 0.2 * radius_score))
    x, y, w, h = best["bbox"]
    return {
        "class_name": "landing_pad",
        "confidence": max(0.85, geom_conf),
        "geometry_confidence": max(0.85, geom_conf),
        "center": best["center"],
        "bbox": [int(x), int(y), int(x + w), int(y + h)],
        "radius": best["radius"],
        "area": best["area"],
        "aspect_ratio": aspect_ratio,
        "contour_points": best["contour_points"],
    }


def detect_landing_pad_legacy_old(image, cfg):
    src_h, src_w = image.shape[:2]
    resized = cv2.resize(image, (640, 512), interpolation=cv2.INTER_AREA)
    det = detect_landing_pad(resized, cfg)
    if det is None:
        return None
    sx = src_w / 640.0
    sy = src_h / 512.0
    det["bbox"] = [
        int(det["bbox"][0] * sx),
        int(det["bbox"][1] * sy),
        int(det["bbox"][2] * sx),
        int(det["bbox"][3] * sy),
    ]
    det["center"] = (det["center"][0] * sx, det["center"][1] * sy)
    det["radius"] *= 0.5 * (sx + sy)
    det["area"] *= sx * sy
    return det


def draw_yolo_boxes(frame, detections):
    out = frame.copy()
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        cls_name = det["class_name"]
        conf = det["confidence"]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            out,
            f"YOLO {cls_name} {conf:.2f}",
            (x1, max(y1 - 5, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    return out


def draw_suppressed_boxes(frame, detections):
    out = frame
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        conf = det["confidence"]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 165, 255), 2)
        cv2.putText(
            out,
            f"SUP {det['class_name']} {conf:.2f}",
            (x1, min(y2 + 22, out.shape[0] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 165, 255),
            2,
        )
    return out


def draw_marker(frame, det, color):
    out = frame
    cx = int(det["center"][0])
    cy = int(det["center"][1])
    cv2.circle(out, (cx, cy), 8, color, -1)
    if det["class_name"] == "landing_pad":
        radius = int(det.get("radius", 0))
        if radius > 0:
            cv2.circle(out, (cx, cy), radius, color, 3)
    else:
        x1, y1, x2, y2 = det["bbox"]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        out,
        f"{det['class_name']} {det['confidence']:.2f}",
        (cx + 10, max(cy - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
    )
    return out


def run_detector_unified5(frame, model, args, counters):
    result = model.predict(
        source=frame,
        conf=args.conf,
        imgsz=args.imgsz,
        verbose=False,
        device=args.device,
    )[0]
    detections = []
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return detections

    for cls_id, conf_val, xyxy in zip(
        boxes.cls.cpu().numpy().astype(int),
        boxes.conf.cpu().numpy(),
        boxes.xyxy.cpu().numpy().astype(int),
    ):
        cls_name = model.names[int(cls_id)]
        bbox = [int(v) for v in xyxy]
        det = {
            "detector": "yolo5cls",
            "class_name": cls_name,
            "confidence": float(conf_val),
            "geometry_confidence": 0.0,
            "bbox": bbox,
            "center": ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0),
            "radius": 0.0,
        }
        detections.append(det)
        counters["yolo_class_counter"][cls_name] += 1
        counters["detector_counter"]["yolo5cls"] += 1
        counters["top_conf_per_class"][cls_name] = max(
            counters["top_conf_per_class"][cls_name], float(conf_val)
        )
    return detections


def run_detector_legacy_split(frame, model_4cls, model_tank, args, counters):
    detections = []

    result_4cls = model_4cls.predict(
        source=frame,
        conf=args.conf,
        imgsz=args.imgsz,
        verbose=False,
        device=args.device,
    )[0]
    boxes = result_4cls.boxes
    if boxes is not None and len(boxes) > 0:
        for cls_id, conf_val, xyxy in zip(
            boxes.cls.cpu().numpy().astype(int),
            boxes.conf.cpu().numpy(),
            boxes.xyxy.cpu().numpy().astype(int),
        ):
            cls_name = model_4cls.names[int(cls_id)]
            bbox = [int(v) for v in xyxy]
            det = {
                "detector": "yolo4cls_old",
                "class_name": cls_name,
                "confidence": float(conf_val),
                "geometry_confidence": 0.0,
                "bbox": bbox,
                "center": ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0),
                "radius": 0.0,
            }
            detections.append(det)
            counters["yolo_class_counter"][cls_name] += 1
            counters["detector_counter"]["yolo4cls_old"] += 1
            counters["top_conf_per_class"][cls_name] = max(
                counters["top_conf_per_class"][cls_name], float(conf_val)
            )

    result_tank = model_tank.predict(
        source=frame,
        conf=args.conf,
        imgsz=args.imgsz,
        verbose=False,
        device=args.device,
    )[0]
    boxes = result_tank.boxes
    if boxes is not None and len(boxes) > 0:
        for cls_id, conf_val, xyxy in zip(
            boxes.cls.cpu().numpy().astype(int),
            boxes.conf.cpu().numpy(),
            boxes.xyxy.cpu().numpy().astype(int),
        ):
            cls_name = model_tank.names[int(cls_id)]
            bbox = [int(v) for v in xyxy]
            det = {
                "detector": "yolo_tank_old",
                "class_name": cls_name,
                "confidence": float(conf_val),
                "geometry_confidence": 0.0,
                "bbox": bbox,
                "center": ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0),
                "radius": 0.0,
            }
            detections.append(det)
            counters["yolo_class_counter"][cls_name] += 1
            counters["detector_counter"]["yolo_tank_old"] += 1
            counters["top_conf_per_class"][cls_name] = max(
                counters["top_conf_per_class"][cls_name], float(conf_val)
            )

    return detections


def main():
    args = parse_args()
    video_path = Path(args.video).resolve()
    model_path = Path(args.model).resolve()
    tank_model_path = Path(args.tank_model).resolve()
    output_dir = Path(args.output_dir).resolve()
    cross_cfg_path = Path(args.cross_config).resolve()
    landing_cfg_path = Path(args.landing_config).resolve()

    if args.clean and output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_dir = output_dir / "samples"
    samples_dir.mkdir(parents=True, exist_ok=True)

    cross_cfg = load_yaml(cross_cfg_path)
    landing_cfg = load_yaml(landing_cfg_path)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    if args.output_fps > 0:
        output_fps = args.output_fps
    elif fps > 0 and args.frame_stride > 1:
        output_fps = fps / float(args.frame_stride)
    else:
        output_fps = fps if fps > 0 else 25.0

    annotated_video_path = output_dir / "annotated_full_chain.mp4"
    writer = cv2.VideoWriter(
        str(annotated_video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        output_fps,
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"无法创建输出视频: {annotated_video_path}")

    model = YOLO(str(model_path), task="detect")
    tank_model = None
    if args.detector_mode == "legacy_split":
        tank_model = YOLO(str(tank_model_path), task="detect")
    detail_csv_path = output_dir / "detections_long.csv"
    frame_csv_path = output_dir / "frame_summary.csv"

    yolo_class_counter = Counter()
    effective_yolo_class_counter = Counter()
    detector_counter = Counter()
    frames_with_any_detection = 0
    red_cross_frames = 0
    landing_pad_frames = 0
    raw_bridge_with_red_cross_frames = 0
    raw_bridge_with_landing_pad_frames = 0
    bridge_with_red_cross_frames = 0
    bridge_with_landing_pad_frames = 0
    aux_only_frames = 0
    suppressed_bridge_frames = 0
    top_conf_per_class = defaultdict(float)
    last_event_frame = -10**9
    saved_event_frames = 0
    processed_frames = 0

    with open(detail_csv_path, "w", newline="", encoding="utf-8") as detail_f, open(
        frame_csv_path, "w", newline="", encoding="utf-8"
    ) as frame_f:
        detail_writer = csv.writer(detail_f)
        detail_writer.writerow(
            [
                "frame_index",
                "timestamp_sec",
                "detector",
                "class_name",
                "confidence",
                "geometry_confidence",
                "x1",
                "y1",
                "x2",
                "y2",
                "center_x",
                "center_y",
                "radius",
                "effective",
            ]
        )
        frame_writer = csv.writer(frame_f)
        frame_writer.writerow(
            [
                "frame_index",
                "timestamp_sec",
                "yolo_classes",
                "has_red_cross",
                "has_landing_pad",
                "bridge_present",
                "event_flags",
            ]
        )

        frame_idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if args.max_frames > 0 and processed_frames >= args.max_frames:
                break
            if args.frame_stride > 1 and frame_idx % args.frame_stride != 0:
                frame_idx += 1
                continue

            counters = {
                "yolo_class_counter": yolo_class_counter,
                "detector_counter": detector_counter,
                "top_conf_per_class": top_conf_per_class,
            }
            if args.detector_mode == "legacy_split":
                yolo_dets = run_detector_legacy_split(frame, model, tank_model, args, counters)
            else:
                yolo_dets = run_detector_unified5(frame, model, args, counters)

            if args.cross_mode == "legacy_simple":
                cross_det = detect_red_cross_legacy_simple(frame, cross_cfg)
            else:
                cross_det = detect_red_cross(frame, cross_cfg)
            if cross_det is not None:
                detector_counter["cross_detector"] += 1
                red_cross_frames += 1
                top_conf_per_class["red_cross"] = max(
                    top_conf_per_class["red_cross"], cross_det["confidence"]
                )
                detail_writer.writerow(
                    [
                        frame_idx,
                        f"{frame_idx / fps:.3f}" if fps > 0 else "0.000",
                        "cross_detector",
                        cross_det["class_name"],
                        f"{cross_det['confidence']:.6f}",
                        f"{cross_det['geometry_confidence']:.6f}",
                        *cross_det["bbox"],
                        f"{cross_det['center'][0]:.3f}",
                        f"{cross_det['center'][1]:.3f}",
                        "0.000",
                        "1",
                    ]
                )

            if args.landing_mode == "legacy_old":
                landing_det = detect_landing_pad_legacy_old(frame, landing_cfg)
            else:
                landing_det = detect_landing_pad(frame, landing_cfg)
            if landing_det is not None:
                detector_counter["landing_detector"] += 1
                landing_pad_frames += 1
                top_conf_per_class["landing_pad"] = max(
                    top_conf_per_class["landing_pad"], landing_det["confidence"]
                )
                detail_writer.writerow(
                    [
                        frame_idx,
                        f"{frame_idx / fps:.3f}" if fps > 0 else "0.000",
                        "landing_detector",
                        landing_det["class_name"],
                        f"{landing_det['confidence']:.6f}",
                        f"{landing_det['geometry_confidence']:.6f}",
                        *landing_det["bbox"],
                        f"{landing_det['center'][0]:.3f}",
                        f"{landing_det['center'][1]:.3f}",
                        f"{landing_det['radius']:.3f}",
                        "1",
                    ]
                )

            yolo_classes = [det["class_name"] for det in yolo_dets]
            raw_bridge_present = "bridge" in yolo_classes
            if raw_bridge_present and cross_det is not None:
                raw_bridge_with_red_cross_frames += 1
            if raw_bridge_present and landing_det is not None:
                raw_bridge_with_landing_pad_frames += 1

            suppress_bridge = False
            if args.suppress_bridge_on_red_cross and cross_det is not None and \
               cross_det["geometry_confidence"] >= args.aux_geometry_threshold:
                suppress_bridge = True
            if args.suppress_bridge_on_landing_pad and landing_det is not None and \
               landing_det["geometry_confidence"] >= args.aux_geometry_threshold:
                suppress_bridge = True

            effective_yolo_dets = []
            suppressed_yolo_dets = []
            for det in yolo_dets:
                effective = not (suppress_bridge and det["class_name"] == "bridge")
                if effective:
                    effective_yolo_dets.append(det)
                    effective_yolo_class_counter[det["class_name"]] += 1
                else:
                    suppressed_yolo_dets.append(det)
                    suppressed_bridge_frames += 1
                detail_writer.writerow(
                    [
                        frame_idx,
                        f"{frame_idx / fps:.3f}" if fps > 0 else "0.000",
                        det["detector"],
                        det["class_name"],
                        f"{det['confidence']:.6f}",
                        f"{det['geometry_confidence']:.6f}",
                        *det["bbox"],
                        f"{det['center'][0]:.3f}",
                        f"{det['center'][1]:.3f}",
                        f"{det['radius']:.3f}",
                        "1" if effective else "0",
                    ]
                )

            bridge_present = any(det["class_name"] == "bridge" for det in effective_yolo_dets)
            if bridge_present and cross_det is not None:
                bridge_with_red_cross_frames += 1
            if bridge_present and landing_det is not None:
                bridge_with_landing_pad_frames += 1
            if not effective_yolo_dets and (cross_det is not None or landing_det is not None):
                aux_only_frames += 1

            has_any = bool(effective_yolo_dets or cross_det is not None or landing_det is not None)
            if has_any:
                frames_with_any_detection += 1

            event_flags = []
            if bridge_present:
                event_flags.append("bridge_yolo")
            if raw_bridge_present and not bridge_present:
                event_flags.append("bridge_suppressed")
            if cross_det is not None:
                event_flags.append("red_cross")
            if landing_det is not None:
                event_flags.append("landing_pad")

            frame_writer.writerow(
                [
                    frame_idx,
                    f"{frame_idx / fps:.3f}" if fps > 0 else "0.000",
                    "|".join(det["class_name"] for det in effective_yolo_dets),
                    str(cross_det is not None).lower(),
                    str(landing_det is not None).lower(),
                    str(bridge_present).lower(),
                    "|".join(event_flags),
                ]
            )

            drawn = draw_yolo_boxes(frame, effective_yolo_dets)
            if suppressed_yolo_dets:
                drawn = draw_suppressed_boxes(drawn, suppressed_yolo_dets)
            if cross_det is not None:
                drawn = draw_marker(drawn, cross_det, (0, 0, 255))
            if landing_det is not None:
                drawn = draw_marker(drawn, landing_det, (255, 255, 0))

            cv2.putText(
                drawn,
                (
                    f"mode: cross={args.cross_mode} landing={args.landing_mode} "
                    f"detector={args.detector_mode} "
                    f"resolve={'on' if (args.suppress_bridge_on_red_cross or args.suppress_bridge_on_landing_pad) else 'off'}"
                ),
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                drawn,
                (
                    f"frame={frame_idx} t={format_timestamp(frame_idx, fps)} "
                    f"yolo={len(effective_yolo_dets)} cross={'1' if cross_det else '0'} "
                    f"landing={'1' if landing_det else '0'} bridge_sup={'1' if (raw_bridge_present and not bridge_present) else '0'}"
                ),
                (20, 72),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            writer.write(drawn)

            if args.sample_stride > 0 and frame_idx % args.sample_stride == 0:
                save_frame(samples_dir / f"sample_f{frame_idx:06d}.jpg", drawn)

            should_save_event = (
                (cross_det is not None or landing_det is not None)
                and saved_event_frames < args.max_event_frames
                and frame_idx - last_event_frame >= args.event_gap
            )
            if should_save_event:
                save_frame(samples_dir / f"event_f{frame_idx:06d}.jpg", drawn)
                last_event_frame = frame_idx
                saved_event_frames += 1

            processed_frames += 1
            frame_idx += 1

    cap.release()
    writer.release()

    summary = {
        "video": str(video_path),
        "model": str(model_path),
        "cross_config": str(cross_cfg_path),
        "landing_config": str(landing_cfg_path),
        "detector_mode": args.detector_mode,
        "tank_model": str(tank_model_path) if args.detector_mode == "legacy_split" else "",
        "conf": args.conf,
        "imgsz": args.imgsz,
        "device": args.device,
        "output_fps": output_fps,
        "cross_mode": args.cross_mode,
        "landing_mode": args.landing_mode,
        "frame_stride": args.frame_stride,
        "fps": fps,
        "total_frames": total_frames,
        "processed_frames": processed_frames,
        "width": width,
        "height": height,
        "frames_with_any_detection": frames_with_any_detection,
        "raw_yolo_class_counts": dict(yolo_class_counter),
        "yolo_class_counts": dict(effective_yolo_class_counter),
        "detector_counts": dict(detector_counter),
        "red_cross_frames": red_cross_frames,
        "landing_pad_frames": landing_pad_frames,
        "raw_bridge_with_red_cross_frames": raw_bridge_with_red_cross_frames,
        "raw_bridge_with_landing_pad_frames": raw_bridge_with_landing_pad_frames,
        "bridge_with_red_cross_frames": bridge_with_red_cross_frames,
        "bridge_with_landing_pad_frames": bridge_with_landing_pad_frames,
        "aux_only_frames": aux_only_frames,
        "suppressed_bridge_frames": suppressed_bridge_frames,
        "top_conf_per_class": dict(top_conf_per_class),
        "annotated_video": str(annotated_video_path),
        "detail_csv": str(detail_csv_path),
        "frame_csv": str(frame_csv_path),
        "samples_dir": str(samples_dir),
        "notes": [
            "landing_pad 表示降落标识外圈/H 相关传统视觉检测链，而不是独立 YOLO 类别。",
            "raw_bridge_with_* 表示抑制前冲突；bridge_with_* 表示抑制后仍残留的冲突。",
            f"suppress_bridge_on_red_cross={args.suppress_bridge_on_red_cross}, "
            f"suppress_bridge_on_landing_pad={args.suppress_bridge_on_landing_pad}",
            "本脚本是完整视觉链的离线等效评测，不直接替代 ROS 实时链路。"
        ],
    }

    summary_path = output_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print(f"video={video_path}")
    print(f"model={model_path}")
    print(f"processed_frames={processed_frames}")
    print(f"red_cross_frames={red_cross_frames}")
    print(f"landing_pad_frames={landing_pad_frames}")
    print(f"raw_bridge_with_red_cross_frames={raw_bridge_with_red_cross_frames}")
    print(f"raw_bridge_with_landing_pad_frames={raw_bridge_with_landing_pad_frames}")
    print(f"bridge_with_red_cross_frames={bridge_with_red_cross_frames}")
    print(f"bridge_with_landing_pad_frames={bridge_with_landing_pad_frames}")
    print(f"summary={summary_path}")


if __name__ == "__main__":
    main()
