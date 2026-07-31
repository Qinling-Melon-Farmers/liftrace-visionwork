#!/usr/bin/env python3
"""真实视频上的圆环关联与地图投影离线评估。

本工具不重新运行 YOLO，而是读取已有完整链 ``detections_long.csv``，在同一
视频帧上复现 ``circle_detector_node`` 的蓝色圆环几何检测，再按
``target_refiner`` 的 ROI/距离规则完成类别--圆环关联。

它输出两类不同性质的指标：

* 不需要人工真值：圆环候选数、关联率、中心抖动、质量分布，以及可视化回放；
* 需要外部真值：圆环中心像素误差和地图投影绝对误差。没有同步的相机标定、
  位姿和目标地图坐标时，绝不把估计结果冒充为误差。

可选输入约定：

``ground_truth_csv``::

    frame_index,class_name,center_x_px,center_y_px,target_id

``pose_csv``（位姿必须是 camera frame 到 map frame 的变换，按帧同步）::

    frame_index,tx,ty,tz,qx,qy,qz,qw

``target_map_csv``::

    target_id,class_name,map_x,map_y,map_z

相机 YAML 使用 ROS ``camera_info`` 常见格式，优先读取 ``camera_matrix``，图像
必须是与该内参匹配的去畸变/校正图像。地图投影只使用精修后的圆环中心。
"""

import argparse
import csv
import json
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}


def parse_args():
    parser = argparse.ArgumentParser(description="真实视频圆环关联/地图投影评估")
    parser.add_argument("--video", required=True, help="真实视频路径")
    parser.add_argument(
        "--detections-csv",
        required=True,
        help="已有完整视觉链 detections_long.csv，坐标必须是原图坐标",
    )
    parser.add_argument("--output-dir", required=True, help="评估输出目录")
    parser.add_argument(
        "--circle-config",
        default="vision_ws/src/uav_vision/config/circle_detector.yaml",
        help="circle_detector 参数文件",
    )
    parser.add_argument(
        "--detector",
        default="yolo5cls",
        help="关联使用的检测器名称；填 all 表示使用所有来源",
    )
    parser.add_argument(
        "--detection-coordinate-transform",
        choices=["auto", "none", "portrait_to_landscape_ccw", "portrait_to_landscape_cw"],
        default="auto",
        help="检测 CSV 到当前视频解码坐标的变换；auto 根据坐标范围判断",
    )
    parser.add_argument("--frame-stride", type=int, default=4, help="处理视频帧步长")
    parser.add_argument("--sample-stride", type=int, default=120, help="保存样本帧步长")
    parser.add_argument("--event-gap", type=int, default=60, help="关联事件样本最小间隔")
    parser.add_argument("--max-event-frames", type=int, default=200)
    parser.add_argument("--max-frames", type=int, default=0, help="0 表示处理全部视频")
    parser.add_argument("--roi-margin-px", type=float, default=40.0)
    parser.add_argument("--max-center-distance-ratio", type=float, default=1.25)
    parser.add_argument("--min-ring-quality", type=float, default=0.70)
    parser.add_argument(
        "--ground-truth-csv",
        default="",
        help="可选人工圆环中心真值 CSV",
    )
    parser.add_argument(
        "--camera-info-yaml",
        default="",
        help="可选 ROS CameraInfo YAML",
    )
    parser.add_argument(
        "--pose-csv",
        default="",
        help="可选逐帧 camera->map 位姿 CSV",
    )
    parser.add_argument(
        "--target-map-csv",
        default="",
        help="可选目标地图坐标 CSV",
    )
    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument(
        "--clean",
        action="store_true",
        help="运行前清空输出目录；仅作用于明确指定的 output-dir",
    )
    return parser.parse_args()


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value, default=0):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def as_bool(value, default=True):
    if value is None or value == "":
        return default
    return str(value).strip().lower() not in {"0", "false", "no", "off"}


def bool_text(value):
    return "true" if value else "false"


def timestamp(frame_index, fps):
    return frame_index / fps if fps > 0 else 0.0


def load_detection_index(path, detector):
    detections = defaultdict(list)
    with open(path, "r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            class_name = str(row.get("class_name", "")).strip()
            if class_name not in STANDARD_CLASSES:
                continue
            source = str(row.get("detector", "")).strip()
            if detector != "all" and source != detector:
                continue
            if not as_bool(row.get("effective", "1"), True):
                continue
            x1 = as_float(row.get("x1"))
            y1 = as_float(row.get("y1"))
            x2 = as_float(row.get("x2"))
            y2 = as_float(row.get("y2"))
            detections[as_int(row.get("frame_index"))].append(
                {
                    "class_name": class_name,
                    "confidence": as_float(row.get("confidence")),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "center_x": as_float(row.get("center_x"), (x1 + x2) / 2.0),
                    "center_y": as_float(row.get("center_y"), (y1 + y2) / 2.0),
                    "detector": source,
                }
            )
    return detections


def _transform_point(x, y, mode, width, height):
    if mode == "portrait_to_landscape_ccw":
        # old portrait image = current landscape image rotated CCW:
        # x_old=y_new, y_old=(width-1)-x_new.
        return width - 1.0 - y, x
    if mode == "portrait_to_landscape_cw":
        # old portrait image = current landscape image rotated CW:
        # x_old=(height-1)-y_new, y_old=x_new.
        return y, height - 1.0 - x
    return x, y


def transform_detection_coordinates(detections, requested_mode, width, height):
    """Convert old portrait detections to the current decoded video direction."""
    max_x = 0.0
    max_y = 0.0
    for rows in detections.values():
        for row in rows:
            max_x = max(max_x, row["x2"])
            max_y = max(max_y, row["y2"])
    mode = requested_mode
    if mode == "auto":
        # The recorded v5 CSV is 1080x2560, while OpenCV decodes this MP4 as
        # 2560x1080.  The distinctive y extent makes this unambiguous.
        if max_x <= height * 1.05 and max_y > height * 1.05 and max_y <= width * 1.05:
            mode = "portrait_to_landscape_ccw"
        else:
            mode = "none"
    if mode == "none":
        return detections, mode
    for rows in detections.values():
        for row in rows:
            points = [
                _transform_point(row["x1"], row["y1"], mode, width, height),
                _transform_point(row["x1"], row["y2"], mode, width, height),
                _transform_point(row["x2"], row["y1"], mode, width, height),
                _transform_point(row["x2"], row["y2"], mode, width, height),
            ]
            row["x1"] = max(0.0, min(point[0] for point in points))
            row["y1"] = max(0.0, min(point[1] for point in points))
            row["x2"] = min(float(width - 1), max(point[0] for point in points))
            row["y2"] = min(float(height - 1), max(point[1] for point in points))
            row["center_x"], row["center_y"] = _transform_point(
                row["center_x"], row["center_y"], mode, width, height
            )
    return detections, mode


def load_ground_truth(path):
    if not path:
        return defaultdict(list)
    truth = defaultdict(list)
    with open(path, "r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if not row.get("center_x_px") or not row.get("center_y_px"):
                continue
            truth[as_int(row.get("frame_index"))].append(
                {
                    "class_name": str(row.get("class_name", "")).strip(),
                    "center_x": as_float(row.get("center_x_px")),
                    "center_y": as_float(row.get("center_y_px")),
                    "target_id": str(row.get("target_id", "")).strip(),
                }
            )
    return truth


def load_pose(path):
    if not path:
        return {}
    poses = {}
    with open(path, "r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            required = {"tx", "ty", "tz", "qx", "qy", "qz", "qw"}
            if not required.issubset(row):
                raise ValueError(
                    "pose CSV 必须包含 frame_index,tx,ty,tz,qx,qy,qz,qw"
                )
            poses[as_int(row.get("frame_index"))] = {
                "translation": np.array(
                    [as_float(row.get("tx")), as_float(row.get("ty")), as_float(row.get("tz"))],
                    dtype=np.float64,
                ),
                "quaternion": np.array(
                    [
                        as_float(row.get("qx")),
                        as_float(row.get("qy")),
                        as_float(row.get("qz")),
                        as_float(row.get("qw")),
                    ],
                    dtype=np.float64,
                ),
            }
    return poses


def load_target_map(path):
    if not path:
        return {}
    targets = {}
    with open(path, "r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            target_id = str(row.get("target_id", "")).strip()
            if not target_id:
                continue
            x = row.get("map_x", row.get("x"))
            y = row.get("map_y", row.get("y"))
            z = row.get("map_z", row.get("z"))
            if x is None or y is None or z is None:
                raise ValueError("target map CSV 必须包含 target_id,map_x,map_y,map_z")
            targets[target_id] = np.array(
                [as_float(x), as_float(y), as_float(z)], dtype=np.float64
            )
    return targets


def load_camera_info(path):
    if not path:
        return None
    data = load_yaml(path)
    matrix = data.get("camera_matrix", {})
    values = matrix.get("data", []) if isinstance(matrix, dict) else []
    if len(values) < 9:
        projection = data.get("projection_matrix", {})
        values = projection.get("data", []) if isinstance(projection, dict) else []
    if len(values) < 9:
        raise ValueError("CameraInfo YAML 缺少 camera_matrix/projection_matrix.data")
    return {
        "width": as_int(data.get("image_width")),
        "height": as_int(data.get("image_height")),
        "K": np.array(values[:9], dtype=np.float64).reshape(3, 3),
        "distortion_model": data.get("distortion_model", ""),
        "D": data.get("distortion_coefficients", {}).get("data", []),
        "path": str(Path(path).resolve()),
    }


def detect_blue_circles(image, cfg):
    """Mirror circle_detector_node.cpp and return original-image coordinates."""
    h_min = as_int(cfg.get("circle_h_min"), 90)
    s_min = as_int(cfg.get("circle_s_min"), 80)
    v_min = as_int(cfg.get("circle_v_min"), 80)
    h_max = as_int(cfg.get("circle_h_max"), 130)
    s_max = as_int(cfg.get("circle_s_max"), 255)
    v_max = as_int(cfg.get("circle_v_max"), 255)
    min_points = as_int(cfg.get("circle_min_contour_points"), 15)
    aspect_threshold = as_float(cfg.get("circle_aspect_ratio_threshold"), 0.85)
    radius_min = as_float(cfg.get("circle_radius_min"), 10.0)
    radius_max = as_float(cfg.get("circle_radius_max"), 300.0)
    min_quality = as_float(cfg.get("circle_min_quality"), 0.70)
    duplicate_ratio = as_float(cfg.get("circle_duplicate_center_ratio"), 0.45)
    max_candidates = as_int(cfg.get("circle_max_candidates"), 12)
    reject_border = as_bool(cfg.get("circle_reject_border_clipped"), True)
    blur_size = as_int(cfg.get("circle_blur_kernel_size"), 5)
    enable_morphology = as_bool(cfg.get("circle_enable_morphology"), True)
    morphology_size = as_int(cfg.get("circle_morphology_kernel_size"), 15)
    enable_resize = as_bool(cfg.get("circle_enable_resize"), False)
    preserve_aspect = as_bool(cfg.get("circle_preserve_aspect_ratio"), True)
    resize_width = as_int(cfg.get("circle_resize_width"), 640)
    resize_height = as_int(cfg.get("circle_resize_height"), 512)

    original_height, original_width = image.shape[:2]
    work = image
    offset_x = 0.0
    offset_y = 0.0
    if enable_resize and preserve_aspect:
        scale = min(
            resize_width / float(original_width),
            resize_height / float(original_height),
        )
        scaled_width = max(1, int(round(original_width * scale)))
        scaled_height = max(1, int(round(original_height * scale)))
        resized = cv2.resize(
            image, (scaled_width, scaled_height), interpolation=cv2.INTER_AREA)
        work = np.zeros((resize_height, resize_width, image.shape[2]), dtype=image.dtype)
        offset_x = float((resize_width - scaled_width) // 2)
        offset_y = float((resize_height - scaled_height) // 2)
        work[
            int(offset_y):int(offset_y) + scaled_height,
            int(offset_x):int(offset_x) + scaled_width,
        ] = resized
        scale_x = 1.0 / scale
        scale_y = 1.0 / scale
    elif enable_resize:
        work = cv2.resize(image, (resize_width, resize_height), interpolation=cv2.INTER_AREA)
        scale_x = original_width / float(work.shape[1])
        scale_y = original_height / float(work.shape[0])
    else:
        scale_x = 1.0
        scale_y = 1.0

    filtered = work
    if blur_size > 1:
        filtered = cv2.GaussianBlur(work, (blur_size | 1, blur_size | 1), 0.0)
    hsv = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
    if enable_morphology:
        kernel_size = morphology_size if morphology_size % 2 else morphology_size + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        if len(contour) < max(5, min_points):
            continue
        try:
            (cx, cy), (ew, eh), angle = cv2.fitEllipse(contour)
        except cv2.error:
            continue
        if ew < 1e-3 or eh < 1e-3:
            continue
        aspect = min(ew, eh) / max(ew, eh)
        if aspect < aspect_threshold:
            continue
        radius = (ew + eh) / 4.0
        if radius < radius_min or radius > radius_max:
            continue
        clipped = (
            cx - radius < 1.0
            or cy - radius < 1.0
            or cx + radius >= work.shape[1] - 1
            or cy + radius >= work.shape[0] - 1
        )
        if clipped and reject_border:
            continue
        perimeter = max(1.0, 2.0 * math.pi * radius)
        contour_density = min(1.0, len(contour) / (perimeter * 0.8))
        aspect_quality = min(1.0, aspect)
        border_quality = 0.55 if clipped else 1.0
        quality = max(
            0.0,
            min(
                1.0,
                0.45 * aspect_quality
                + 0.40 * contour_density
                + 0.15 * border_quality,
            ),
        )
        if quality < min_quality:
            continue
        duplicate = False
        for kept in candidates:
            distance = math.hypot(kept["work_cx"] - cx, kept["work_cy"] - cy)
            duplicate_radius = max(kept["work_ew"], kept["work_eh"]) * duplicate_ratio
            if distance < duplicate_radius:
                duplicate = True
                break
        if duplicate:
            continue
        candidates.append(
            {
                "work_cx": float(cx),
                "work_cy": float(cy),
                "work_ew": float(ew),
                "work_eh": float(eh),
                "angle": float(angle),
                "center_x": max(0.0, min((cx - offset_x) * scale_x, original_width - 1.0)),
                "center_y": max(0.0, min((cy - offset_y) * scale_y, original_height - 1.0)),
                "radius": float((ew * scale_x + eh * scale_y) / 4.0),
                "aspect_ratio": float(aspect),
                "quality": float(quality),
                "geometry_verified": bool(quality >= min_quality),
                "clipped": bool(clipped),
                "contour_points": int(len(contour)),
            }
        )

    candidates.sort(key=lambda item: item["quality"], reverse=True)
    return candidates[:max_candidates]


def in_roi(x, y, x1, y1, x2, y2, margin):
    return x1 - margin <= x <= x2 + margin and y1 - margin <= y <= y2 + margin


def associate(targets, circles, roi_margin, max_distance_ratio, min_quality):
    usable = [
        circle
        for circle in circles
        if circle["quality"] >= min_quality and circle["geometry_verified"]
    ]
    pair_scores = []
    for target_index, target in enumerate(targets):
        for circle_index, circle in enumerate(usable):
            circle_in_target = in_roi(
                circle["center_x"], circle["center_y"],
                target["x1"], target["y1"], target["x2"], target["y2"], roi_margin
            )
            target_in_circle = in_roi(
                target["center_x"], target["center_y"],
                circle["center_x"] - circle["radius"],
                circle["center_y"] - circle["radius"],
                circle["center_x"] + circle["radius"],
                circle["center_y"] + circle["radius"],
                roi_margin,
            )
            if not (circle_in_target or target_in_circle):
                continue
            distance = math.hypot(
                target["center_x"] - circle["center_x"],
                target["center_y"] - circle["center_y"],
            )
            target_scale = max(
                target["x2"] - target["x1"],
                target["y2"] - target["y1"],
                circle["radius"] * 2.0,
                1.0,
            )
            if distance > max_distance_ratio * target_scale:
                continue
            score = distance / target_scale + (1.0 - circle["quality"])
            pair_scores.append((score, target_index, circle_index, distance))

    # Runtime target_refiner uses the same global lowest-cost one-to-one
    # assignment; target iteration order must not steal a ring from a better
    # match later in the frame.
    assigned = {}
    used_targets = set()
    used_circles = set()
    for score, target_index, circle_index, distance in sorted(pair_scores):
        if target_index in used_targets or circle_index in used_circles:
            continue
        used_targets.add(target_index)
        used_circles.add(circle_index)
        assigned[target_index] = (circle_index, score, distance)

    associations = []
    for target_index, target in enumerate(targets):
        assignment = assigned.get(target_index)
        if assignment is None:
            associations.append(
                {
                    "target_index": target_index,
                    "circle_index": -1,
                    "associated": False,
                    "score": "",
                    "center_shift_px": "",
                    "refined_center_x": target["center_x"],
                    "refined_center_y": target["center_y"],
                }
            )
            continue
        circle_index, best_score, center_shift = assignment
        circle = usable[circle_index]
        associations.append(
            {
                "target_index": target_index,
                "circle_index": circles.index(circle),
                "associated": True,
                "score": best_score,
                "center_shift_px": center_shift,
                "refined_center_x": circle["center_x"],
                "refined_center_y": circle["center_y"],
            }
        )
    return associations


def quaternion_matrix(quaternion):
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(float(np.dot(quaternion, quaternion)))
    if norm < 1e-9:
        return None
    qx, qy, qz, qw = quaternion / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def project_pixel(center_x, center_y, camera_info, pose, ground_z):
    K = camera_info["K"]
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    if fx <= 0 or fy <= 0:
        return None, "invalid_intrinsics"
    ray_camera = np.array([(center_x - cx) / fx, (center_y - cy) / fy, 1.0])
    rotation = quaternion_matrix(pose["quaternion"])
    if rotation is None:
        return None, "invalid_pose_quaternion"
    origin = pose["translation"]
    ray_map = rotation.dot(ray_camera)
    if abs(ray_map[2]) < 1e-9:
        return None, "ray_parallel_ground"
    distance = (ground_z - origin[2]) / ray_map[2]
    if distance <= 0:
        return None, "ground_behind_camera"
    return origin + distance * ray_map, "ok"


def match_truth(target, truth_rows, used):
    candidates = []
    for index, truth in enumerate(truth_rows):
        if index in used or truth["class_name"] != target["class_name"]:
            continue
        distance = math.hypot(
            target["center_x"] - truth["center_x"],
            target["center_y"] - truth["center_y"],
        )
        candidates.append((distance, index, truth))
    if not candidates:
        return None
    distance, index, truth = min(candidates, key=lambda item: item[0])
    used.add(index)
    return truth


def draw_frame(frame, targets, circles, associations, frame_index, fps, map_errors):
    drawn = frame.copy()
    for index, circle in enumerate(circles):
        center = (int(round(circle["center_x"])), int(round(circle["center_y"])))
        radius = max(2, int(round(circle["radius"])))
        cv2.circle(drawn, center, radius, (255, 180, 0), 2)
        cv2.circle(drawn, center, 5, (255, 0, 255), -1)
        cv2.putText(
            drawn,
            "R%d q=%.2f" % (index, circle["quality"]),
            (center[0] + 6, max(20, center[1] - radius)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 180, 0),
            2,
        )
    for association in associations:
        target = targets[association["target_index"]]
        raw = (int(round(target["center_x"])), int(round(target["center_y"])))
        cv2.rectangle(
            drawn,
            (int(target["x1"]), int(target["y1"])),
            (int(target["x2"]), int(target["y2"])),
            (0, 220, 0),
            2,
        )
        if association["associated"]:
            refined = (
                int(round(association["refined_center_x"])),
                int(round(association["refined_center_y"])),
            )
            cv2.line(drawn, raw, refined, (0, 165, 255), 2)
            cv2.circle(drawn, refined, 8, (0, 0, 255), -1)
            label = "%s refined shift=%.1f" % (target["class_name"], association["center_shift_px"])
        else:
            cv2.circle(drawn, raw, 7, (0, 255, 255), 2)
            label = "%s raw_only" % target["class_name"]
        cv2.putText(
            drawn,
            label,
            (int(target["x1"]), max(25, int(target["y1"]) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 220, 0),
            2,
        )
    status = "map_err=%.3fm" % map_errors[0] if map_errors else "map_err=NA"
    cv2.putText(
        drawn,
        "frame=%d t=%.3fs circles=%d assoc=%d %s" %
        (frame_index, timestamp(frame_index, fps), len(circles),
         sum(1 for item in associations if item["associated"]), status),
        (20, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 0),
        2,
    )
    return drawn


def write_template(path, detection_index, frame_stride):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["frame_index", "class_name", "center_x_px", "center_y_px", "target_id", "notes"]
        )
        for frame_index in sorted(detection_index):
            if frame_index % frame_stride != 0:
                continue
            for target in detection_index[frame_index]:
                writer.writerow([frame_index, target["class_name"], "", "", "", "fill_me"])


def main():
    args = parse_args()
    if args.frame_stride <= 0:
        raise ValueError("--frame-stride 必须大于 0")
    video_path = Path(args.video).resolve()
    detection_path = Path(args.detections_csv).resolve()
    output_dir = Path(args.output_dir).resolve()
    if args.clean and output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_dir = output_dir / "samples"
    samples_dir.mkdir(parents=True, exist_ok=True)

    cfg = load_yaml(args.circle_config)
    detection_index = load_detection_index(args.detections_csv, args.detector)
    ground_truth = load_ground_truth(args.ground_truth_csv)
    poses = load_pose(args.pose_csv)
    target_map = load_target_map(args.target_map_csv)
    camera_info = load_camera_info(args.camera_info_yaml)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError("无法打开视频: %s" % video_path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    detection_index, coordinate_transform = transform_detection_coordinates(
        detection_index, args.detection_coordinate_transform, width, height
    )
    write_template(output_dir / "ring_ground_truth_template.csv", detection_index, args.frame_stride)
    writer = cv2.VideoWriter(
        str(output_dir / "annotated_ring_association.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps if fps > 0 else 25.0,
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError("无法创建标注视频")

    ring_csv_path = output_dir / "circle_detections.csv"
    association_csv_path = output_dir / "association_long.csv"
    frame_csv_path = output_dir / "frame_summary.csv"
    ring_counter = Counter()
    class_counter = Counter()
    associated_class_counter = Counter()
    frame_count = 0
    processed_count = 0
    frames_with_circle = 0
    frames_with_standard = 0
    frames_with_association = 0
    association_count = 0
    standard_count = 0
    map_valid_count = 0
    raw_errors = []
    refined_errors = []
    map_errors = []
    map_failure_reasons = Counter()
    jitter_by_class = defaultdict(list)
    last_center_by_class = {}
    last_event_frame = -10**9
    saved_event_frames = 0

    with open(ring_csv_path, "w", newline="", encoding="utf-8") as ring_file, \
            open(association_csv_path, "w", newline="", encoding="utf-8") as assoc_file, \
            open(frame_csv_path, "w", newline="", encoding="utf-8") as frame_file:
        ring_writer = csv.writer(ring_file)
        ring_writer.writerow(
            [
                "frame_index", "timestamp_sec", "candidate_index", "center_x", "center_y",
                "radius", "ellipse_width_work", "ellipse_height_work", "aspect_ratio",
                "quality", "geometry_verified", "clipped", "contour_points",
            ]
        )
        assoc_writer = csv.writer(assoc_file)
        assoc_writer.writerow(
            [
                "frame_index", "timestamp_sec", "target_index", "class_name", "confidence",
                "raw_center_x", "raw_center_y", "refined_center_x", "refined_center_y",
                "associated", "circle_index", "association_score", "center_shift_px",
                "gt_center_x", "gt_center_y", "raw_center_error_px", "refined_center_error_px",
                "target_id", "map_valid", "map_x", "map_y", "map_z", "map_error_m",
                "map_failure_reason",
            ]
        )
        frame_writer = csv.writer(frame_file)
        frame_writer.writerow(
            [
                "frame_index", "timestamp_sec", "standard_count", "circle_count",
                "association_count", "association_rate", "map_valid_count", "map_error_mean_m",
            ]
        )

        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if args.max_frames and frame_count >= args.max_frames:
                break
            targets = detection_index.get(frame_count, []) if frame_count % args.frame_stride == 0 else []
            if frame_count % args.frame_stride != 0:
                writer.write(frame)
                frame_count += 1
                continue
            processed_count += 1
            circles = detect_blue_circles(frame, cfg)
            associations = associate(
                targets,
                circles,
                args.roi_margin_px,
                args.max_center_distance_ratio,
                args.min_ring_quality,
            )
            frames_with_circle += bool(circles)
            frames_with_standard += bool(targets)
            associated_items = [item for item in associations if item["associated"]]
            frames_with_association += bool(associated_items)
            association_count += len(associated_items)
            standard_count += len(targets)
            for circle_index, circle in enumerate(circles):
                ring_counter["all"] += 1
                ring_writer.writerow(
                    [
                        frame_count, "%.6f" % timestamp(frame_count, fps), circle_index,
                        "%.3f" % circle["center_x"], "%.3f" % circle["center_y"],
                        "%.3f" % circle["radius"], "%.3f" % circle["work_ew"],
                        "%.3f" % circle["work_eh"], "%.6f" % circle["aspect_ratio"],
                        "%.6f" % circle["quality"], bool_text(circle["geometry_verified"]),
                        bool_text(circle["clipped"]), circle["contour_points"],
                    ]
                )

            truth_rows = ground_truth.get(frame_count, [])
            used_truth = set()
            frame_map_errors = []
            pose = poses.get(frame_count)
            for association in associations:
                target = targets[association["target_index"]]
                class_counter[target["class_name"]] += 1
                truth = match_truth(target, truth_rows, used_truth)
                raw_error = ""
                refined_error = ""
                target_id = ""
                if truth is not None:
                    target_id = truth["target_id"]
                    raw_error_value = math.hypot(
                        target["center_x"] - truth["center_x"],
                        target["center_y"] - truth["center_y"],
                    )
                    raw_error = "%.6f" % raw_error_value
                    raw_errors.append(raw_error_value)
                    if association["associated"]:
                        refined_error_value = math.hypot(
                            association["refined_center_x"] - truth["center_x"],
                            association["refined_center_y"] - truth["center_y"],
                        )
                        refined_error = "%.6f" % refined_error_value
                        refined_errors.append(refined_error_value)

                map_valid = False
                map_point = None
                map_error = ""
                failure_reason = "missing_camera_info_or_pose"
                if association["associated"] and camera_info is not None and pose is not None:
                    map_point, failure_reason = project_pixel(
                        association["refined_center_x"],
                        association["refined_center_y"],
                        camera_info,
                        pose,
                        args.ground_z,
                    )
                    map_valid = map_point is not None
                    if not map_valid:
                        map_failure_reasons[failure_reason] += 1
                    else:
                        map_valid_count += 1
                        if target_id and target_id in target_map:
                            error_value = float(np.linalg.norm(map_point - target_map[target_id]))
                            map_errors.append(error_value)
                            frame_map_errors.append(error_value)
                            map_error = "%.6f" % error_value

                if association["associated"]:
                    associated_class_counter[target["class_name"]] += 1
                    previous = last_center_by_class.get(target["class_name"])
                    if previous is not None:
                        jitter_by_class[target["class_name"]].append(
                            math.hypot(
                                association["refined_center_x"] - previous[0],
                                association["refined_center_y"] - previous[1],
                            )
                        )
                    last_center_by_class[target["class_name"]] = (
                        association["refined_center_x"], association["refined_center_y"]
                    )

                assoc_writer.writerow(
                    [
                        frame_count, "%.6f" % timestamp(frame_count, fps),
                        association["target_index"], target["class_name"],
                        "%.6f" % target["confidence"], "%.3f" % target["center_x"],
                        "%.3f" % target["center_y"], "%.3f" % association["refined_center_x"],
                        "%.3f" % association["refined_center_y"],
                        bool_text(association["associated"]), association["circle_index"],
                        "%.6f" % association["score"] if association["associated"] else "",
                        "%.6f" % association["center_shift_px"] if association["associated"] else "",
                        "%.3f" % truth["center_x"] if truth is not None else "",
                        "%.3f" % truth["center_y"] if truth is not None else "",
                        raw_error, refined_error, target_id, bool_text(map_valid),
                        "%.6f" % map_point[0] if map_valid else "",
                        "%.6f" % map_point[1] if map_valid else "",
                        "%.6f" % map_point[2] if map_valid else "",
                        map_error, "" if map_valid else failure_reason,
                    ]
                )

            frame_writer.writerow(
                [
                    frame_count, "%.6f" % timestamp(frame_count, fps), len(targets), len(circles),
                    len(associated_items),
                    "%.6f" % (len(associated_items) / float(len(targets))) if targets else "",
                    sum(1 for item in associations if item["associated"] and pose is not None and camera_info is not None),
                    "%.6f" % (sum(frame_map_errors) / len(frame_map_errors)) if frame_map_errors else "",
                ]
            )
            drawn = draw_frame(frame, targets, circles, associations, frame_count, fps, frame_map_errors)
            writer.write(drawn)
            should_save = args.sample_stride > 0 and frame_count % args.sample_stride == 0
            should_event = (
                associated_items
                and saved_event_frames < args.max_event_frames
                and frame_count - last_event_frame >= args.event_gap
            )
            if should_save or should_event:
                prefix = "sample" if should_save else "event"
                sample_name = "%s_f%06d.jpg" % (prefix, frame_count)
                cv2.imwrite(str(samples_dir / sample_name), drawn)
                if should_event:
                    last_event_frame = frame_count
                    saved_event_frames += 1
            frame_count += 1

    cap.release()
    writer.release()

    def mean(values):
        return float(sum(values) / len(values)) if values else None

    def percentile(values, q):
        return float(np.percentile(values, q)) if values else None

    association_rate = association_count / float(standard_count) if standard_count else 0.0
    summary = {
        "video": str(video_path),
        "detections_csv": str(detection_path),
        "circle_config": str(Path(args.circle_config).resolve()),
        "detector_filter": args.detector,
        "detection_coordinate_transform_requested": args.detection_coordinate_transform,
        "detection_coordinate_transform_applied": coordinate_transform,
        "fps": fps,
        "total_frames": total_frames,
        "processed_frames": processed_count,
        "width": width,
        "height": height,
        "frame_stride": args.frame_stride,
        "frames_with_circles": int(frames_with_circle),
        "frames_with_standard_targets": int(frames_with_standard),
        "frames_with_association": int(frames_with_association),
        "standard_detection_count": standard_count,
        "circle_candidate_count": ring_counter["all"],
        "association_count": association_count,
        "association_rate": association_rate,
        "standard_detection_count_by_class": dict(class_counter),
        "association_count_by_class": dict(associated_class_counter),
        "unassociated_count_by_class": {
            class_name: count - associated_class_counter[class_name]
            for class_name, count in sorted(class_counter.items())
        },
        "center_jitter_px_mean_by_class": {
            key: mean(values) for key, values in sorted(jitter_by_class.items())
        },
        "center_jitter_px_p95_by_class": {
            key: percentile(values, 95) for key, values in sorted(jitter_by_class.items())
        },
        "ground_truth": {
            "provided": bool(args.ground_truth_csv),
            "raw_center_error_px_mean": mean(raw_errors),
            "raw_center_error_px_p95": percentile(raw_errors, 95),
            "refined_center_error_px_mean": mean(refined_errors),
            "refined_center_error_px_p95": percentile(refined_errors, 95),
            "matched_rows": len(refined_errors),
        },
        "map_projection": {
            "camera_info_provided": bool(camera_info),
            "pose_provided": bool(args.pose_csv),
            "target_map_provided": bool(args.target_map_csv),
            "camera_info_path": camera_info["path"] if camera_info else "",
            "camera_resolution": [camera_info["width"], camera_info["height"]] if camera_info else [],
            "video_resolution_matches_camera_info": (
                bool(camera_info) and camera_info["width"] == width and camera_info["height"] == height
            ),
            "projected_point_count": map_valid_count,
            "map_error_m_mean": mean(map_errors),
            "map_error_m_p95": percentile(map_errors, 95),
            "map_failure_reasons": dict(map_failure_reasons),
        },
        "outputs": {
            "annotated_video": str(output_dir / "annotated_ring_association.mp4"),
            "circle_csv": str(ring_csv_path),
            "association_csv": str(association_csv_path),
            "frame_csv": str(frame_csv_path),
            "ground_truth_template": str(output_dir / "ring_ground_truth_template.csv"),
            "samples_dir": str(samples_dir),
        },
        "measurement_status": {
            "ring_candidate_and_association": "available_from_real_video",
            "ring_center_absolute_error": "available_with_ground_truth_csv" if args.ground_truth_csv else "unavailable_without_ground_truth_csv",
            "map_projection_point": "available_with_camera_info_and_pose" if camera_info and args.pose_csv else "unavailable_without_camera_info_and_pose",
            "map_projection_absolute_error": (
                "available_with_camera_info_pose_and_target_map_csv"
                if camera_info and args.pose_csv and args.target_map_csv and args.ground_truth_csv
                else "unavailable_without_camera_info_pose_target_map_and_ground_truth"
            ),
        },
        "limitations": [
            "MP4 本身不包含同步 TF/odom；普通视频不能独立给出地图绝对误差。",
            "圆环关联率是检测链自洽性指标，不等于真实中心准确率。",
            "若提供 CameraInfo，必须确认其分辨率、裁剪/旋转和去畸变状态与视频一致。",
            "地图投影假定 pose_csv 的四元数表示 camera frame 到 map frame 的旋转，ground_z 使用 map frame。",
        ],
    }
    with open(output_dir / "summary.json", "w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
