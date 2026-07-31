#!/usr/bin/env python3
"""用上一届旧视觉链路在真实视频上做离线推演。

旧链的关键行为按 ``patrol_control_visual``/``Visual`` 中的实现保留：

* 旧 4 类 YOLO ``best.pt`` + 独立 ``tank.pt``；
* 旧 red_cross / landing 几何检测；
* 旧 ``circle_detector_node``：图像先缩放到 640x512，蓝色分割后从所有
  合格轮廓中选面积最大的椭圆，不做类别关联；
* 旧中心投影使用 ``best_ellipse.center * 2``，相机内参和 map TF 仍只做
  记录，不在没有 TF 的 MP4 上伪造地图点；
* 旧 YOLO 节点的三帧类别累计输出单独记录为 ``legacy_yolo_output``。

这是旧链复盘工具，不是新 ``uav_vision`` 运行时入口。它的输出会明确标出
旧硬编码内参、全图最大圆、中心乘 2 和缺少类别关联等历史行为。
"""

import argparse
import csv
import json
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from eval_full_vision_chain_video import (
    detect_landing_pad_legacy_old,
    detect_red_cross_legacy_simple,
    draw_marker,
    draw_yolo_boxes,
    format_timestamp,
    load_yaml,
    run_detector_legacy_split,
    save_frame,
)


OLD_CIRCLE_K = np.array(
    [[998.743048, 0.0, 662.188350],
     [0.0, 997.846645, 523.650663],
     [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
OLD_SERVICE_K = np.array(
    [[997.634832, 0.0, 625.164605],
     [0.0, 998.885312, 509.350168],
     [0.0, 0.0, 1.0]],
    dtype=np.float64,
)


def legacy_ray(u, v, camera_matrix):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    return ((u - cx) / fx, (v - cy) / fy, 1.0)


def parse_args():
    parser = argparse.ArgumentParser(description="旧完整视觉链真实视频推演")
    parser.add_argument("--video", required=True)
    parser.add_argument(
        "--model",
        default="Visual/src/yolov5_detect/best.pt",
        help="旧四类模型",
    )
    parser.add_argument(
        "--tank-model",
        default="Visual/src/yolov5_detect/tank.pt",
        help="旧独立 tank 模型",
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--cross-config",
        default="vision_ws/src/uav_vision/config/cross_detector.yaml",
    )
    parser.add_argument(
        "--landing-config",
        default="vision_ws/src/uav_vision/config/landing_detector.yaml",
    )
    parser.add_argument(
        "--circle-config",
        default="vision_ws/migration_refs/patrol_control_visual/config/circle_detection_params.yaml",
        help="旧 patrol_control 圆环参数；默认使用旧实机链参数",
    )
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--frame-stride", type=int, default=4)
    parser.add_argument("--sample-stride", type=int, default=120)
    parser.add_argument("--event-gap", type=int, default=60)
    parser.add_argument("--max-event-frames", type=int, default=200)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def legacy_circle_config(data):
    circle = data.get("circle_detection", {}) or {}
    color = data.get("color_segmentation", {}) or {}
    quality = data.get("quality_assessment", {}) or {}
    preprocess = data.get("image_preprocessing", {}) or {}
    return {
        "min_radius": float(circle.get("min_radius", 50.0)),
        "max_radius": float(circle.get("max_radius", 450.0)),
        "h_min": int(color.get("h_min", 70)),
        "h_max": int(color.get("h_max", 150)),
        "s_min": int(color.get("s_min", 20)),
        "s_max": int(color.get("s_max", 255)),
        "v_min": int(color.get("v_min", 30)),
        "v_max": int(color.get("v_max", 255)),
        "min_contour_points": int(quality.get("min_contour_points", 15)),
        "aspect_ratio_threshold": float(quality.get("aspect_ratio_threshold", 0.7)),
        "morphology_kernel_size": int(preprocess.get("morphology_kernel_size", 15)),
        # The old C++ has GaussianBlur commented out. Keep the value only for the report.
        "blur_enabled_in_config": bool(preprocess.get("enable_gaussian_blur", True)),
    }


def detect_legacy_circle(image, cfg):
    """Mirror the old circle_detector_node.cpp before map/TF publishing."""
    src_height, src_width = image.shape[:2]
    resized = cv2.resize(image, (640, 512), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        (cfg["h_min"], cfg["s_min"], cfg["v_min"]),
        (cfg["h_max"], cfg["s_max"], cfg["v_max"]),
    )
    kernel_size = cfg["morphology_kernel_size"]
    kernel_size = kernel_size if kernel_size % 2 else kernel_size + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        if len(contour) < cfg["min_contour_points"]:
            continue
        try:
            ellipse = cv2.fitEllipse(contour)
        except cv2.error:
            continue
        ew, eh = ellipse[1]
        if ew < 1e-3 or eh < 1e-3:
            continue
        aspect = min(ew, eh) / max(ew, eh)
        if aspect < cfg["aspect_ratio_threshold"]:
            continue
        radius = (ew + eh) / 4.0
        if radius < cfg["min_radius"] or radius > cfg["max_radius"]:
            continue
        area = cv2.contourArea(contour)
        if best is None or area > best["contour_area"]:
            best = {
                "ellipse": ellipse,
                "contour_area": float(area),
                "aspect_ratio": float(aspect),
                "radius_work": float(radius),
                "contour_points": int(len(contour)),
            }

    if best is None:
        return None

    center_work = best["ellipse"][0]
    scale_x = src_width / 640.0
    scale_y = src_height / 512.0
    # The old node's image is 640x512, but the published/projected ray uses center*2.
    center_projected = (center_work[0] * 2.0, center_work[1] * 2.0)
    center_original = (center_work[0] * scale_x, center_work[1] * scale_y)
    radius_original = best["radius_work"] * 0.5 * (scale_x + scale_y)
    ray = legacy_ray(center_projected[0], center_projected[1], OLD_CIRCLE_K)
    return {
        "class_name": "legacy_circle",
        "confidence": 1.0,
        "geometry_confidence": 0.0,
        "center_work_x": float(center_work[0]),
        "center_work_y": float(center_work[1]),
        "center_projected_x": float(center_projected[0]),
        "center_projected_y": float(center_projected[1]),
        "center_original_x": float(center_original[0]),
        "center_original_y": float(center_original[1]),
        "ray_x": ray[0],
        "ray_y": ray[1],
        "ray_z": ray[2],
        "radius_work": best["radius_work"],
        "radius_original": float(radius_original),
        "aspect_ratio": best["aspect_ratio"],
        "contour_area": best["contour_area"],
        "contour_points": best["contour_points"],
    }


def circle_as_detection(circle):
    radius = circle["radius_original"]
    cx = circle["center_original_x"]
    cy = circle["center_original_y"]
    return {
        "class_name": "legacy_circle",
        "confidence": 1.0,
        "bbox": [
            int(cx - radius), int(cy - radius),
            int(cx + radius), int(cy + radius),
        ],
        "center": (cx, cy),
        "radius": int(max(1.0, radius)),
    }


def draw_legacy_circle(frame, circle):
    if circle is None:
        return frame
    out = frame
    cx = int(round(circle["center_original_x"]))
    cy = int(round(circle["center_original_y"]))
    radius = int(max(1.0, circle["radius_original"]))
    cv2.ellipse(
        out,
        ((cx, cy), (radius * 2, radius * 2), 0.0),
        (255, 0, 255),
        3,
    )
    cv2.circle(out, (cx, cy), 9, (255, 0, 255), -1)
    cv2.putText(
        out,
        "OLD_CENTER (%.0f,%.0f) r=%.0f" % (cx, cy, circle["radius_original"]),
        (cx + 12, max(25, cy - 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 255),
        2,
    )
    return out


def main():
    args = parse_args()
    if args.frame_stride <= 0:
        raise ValueError("--frame-stride 必须大于 0")
    video_path = Path(args.video).resolve()
    output_dir = Path(args.output_dir).resolve()
    if args.clean and output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_dir = output_dir / "samples"
    samples_dir.mkdir(parents=True, exist_ok=True)

    circle_cfg_raw = load_yaml(Path(args.circle_config).resolve())
    circle_cfg = legacy_circle_config(circle_cfg_raw)
    cross_cfg = load_yaml(Path(args.cross_config).resolve())
    landing_cfg = load_yaml(Path(args.landing_config).resolve())

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError("无法打开视频: %s" % video_path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    output_fps = fps / float(args.frame_stride) if fps > 0 else 25.0
    writer = cv2.VideoWriter(
        str(output_dir / "annotated_legacy_full_chain.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"),
        output_fps,
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError("无法创建旧链标注视频")

    model = YOLO(str(Path(args.model).resolve()), task="detect")
    tank_model = YOLO(str(Path(args.tank_model).resolve()), task="detect")

    detail_path = output_dir / "detections_long.csv"
    frame_path = output_dir / "frame_summary.csv"
    circle_path = output_dir / "legacy_circle_centers.csv"
    projection_path = output_dir / "legacy_projection_inputs.csv"
    class_counts = Counter()
    detector_counts = Counter()
    top_conf = defaultdict(float)
    legacy_output_counts = Counter()
    circle_count = 0
    circle_frames = 0
    standard_detection_count = 0
    tank_service_calls = 0
    cross_frames = 0
    landing_frames = 0
    frames_with_any = 0
    processed = 0
    legacy_seq = 0
    legacy_accumulated = Counter()
    last_event_frame = -10**9
    saved_events = 0

    with open(detail_path, "w", newline="", encoding="utf-8") as detail_file, \
            open(frame_path, "w", newline="", encoding="utf-8") as frame_file, \
            open(circle_path, "w", newline="", encoding="utf-8") as circle_file, \
            open(projection_path, "w", newline="", encoding="utf-8") as projection_file:
        detail_writer = csv.writer(detail_file)
        detail_writer.writerow(
            [
                "frame_index", "timestamp_sec", "detector", "class_name", "confidence",
                "geometry_confidence", "x1", "y1", "x2", "y2", "center_x", "center_y",
                "radius", "effective",
            ]
        )
        frame_writer = csv.writer(frame_file)
        frame_writer.writerow(
            [
                "frame_index", "timestamp_sec", "legacy_yolo_output", "yolo_classes",
                "standard_detection_count", "tank_service_call_count", "has_legacy_circle",
                "legacy_circle_center_x", "legacy_circle_center_y", "legacy_circle_radius",
                "has_red_cross", "has_landing_pad", "event_flags",
            ]
        )
        circle_writer = csv.writer(circle_file)
        circle_writer.writerow(
            [
                "frame_index", "timestamp_sec", "center_work_x", "center_work_y",
                "center_projected_x", "center_projected_y", "center_original_x",
                "center_original_y", "radius_work", "radius_original", "aspect_ratio",
                "contour_area", "contour_points", "ray_x", "ray_y", "ray_z", "map_status",
            ]
        )
        projection_writer = csv.writer(projection_file)
        projection_writer.writerow(
            [
                "frame_index", "timestamp_sec", "source", "class_name", "input_u",
                "input_v", "ray_x", "ray_y", "ray_z", "map_status",
            ]
        )

        frame_index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if args.max_frames > 0 and processed >= args.max_frames:
                break
            if frame_index % args.frame_stride != 0:
                frame_index += 1
                continue

            counters = {
                "yolo_class_counter": class_counts,
                "detector_counter": detector_counts,
                "top_conf_per_class": top_conf,
            }
            class_args = argparse.Namespace(
                conf=args.conf, imgsz=args.imgsz, device=args.device
            )
            yolo_dets = run_detector_legacy_split(
                frame, model, tank_model, class_args, counters
            )
            standard_dets = [
                det for det in yolo_dets if det["class_name"] != "tank"
            ]
            tank_dets = [det for det in yolo_dets if det["class_name"] == "tank"]
            standard_detection_count += len(standard_dets)
            tank_service_calls += len(tank_dets)
            for det in yolo_dets:
                detail_writer.writerow(
                    [
                        frame_index, "%.6f" % (frame_index / fps if fps else 0.0),
                        det["detector"], det["class_name"], "%.6f" % det["confidence"],
                        "%.6f" % det["geometry_confidence"], *det["bbox"],
                        "%.3f" % det["center"][0], "%.3f" % det["center"][1],
                        "0.000", "1",
                    ]
                )

            # Old yolo_detect.py accumulates class counts for three enabled frames.
            legacy_seq += 1
            for det in standard_dets:
                legacy_accumulated[det["class_name"]] += 1
            if legacy_seq >= 3:
                legacy_output = (
                    max(legacy_accumulated, key=legacy_accumulated.get)
                    if sum(legacy_accumulated.values()) > 0
                    else "Nothing"
                )
                legacy_output_counts[legacy_output] += 1
                legacy_accumulated.clear()
                legacy_seq = 0
            else:
                legacy_output = ""

            circle = detect_legacy_circle(frame, circle_cfg)
            if circle is not None:
                circle_count += 1
                circle_frames += 1
                circle_writer.writerow(
                    [
                        frame_index, "%.6f" % (frame_index / fps if fps else 0.0),
                        "%.3f" % circle["center_work_x"], "%.3f" % circle["center_work_y"],
                        "%.3f" % circle["center_projected_x"], "%.3f" % circle["center_projected_y"],
                        "%.3f" % circle["center_original_x"], "%.3f" % circle["center_original_y"],
                        "%.3f" % circle["radius_work"], "%.3f" % circle["radius_original"],
                        "%.6f" % circle["aspect_ratio"], "%.3f" % circle["contour_area"],
                        circle["contour_points"], "%.8f" % circle["ray_x"],
                        "%.8f" % circle["ray_y"], "%.8f" % circle["ray_z"],
                        "unavailable_without_map_tf",
                    ]
                )
                projection_writer.writerow(
                    [
                        frame_index, "%.6f" % (frame_index / fps if fps else 0.0),
                        "legacy_circle_detector", "legacy_circle",
                        "%.3f" % circle["center_projected_x"],
                        "%.3f" % circle["center_projected_y"],
                        "%.8f" % circle["ray_x"], "%.8f" % circle["ray_y"],
                        "%.8f" % circle["ray_z"], "unavailable_without_map_tf",
                    ]
                )

            for tank_det in tank_dets:
                ray = legacy_ray(
                    tank_det["center"][0], tank_det["center"][1], OLD_SERVICE_K
                )
                projection_writer.writerow(
                    [
                        frame_index, "%.6f" % (frame_index / fps if fps else 0.0),
                        "image2center_service", "tank",
                        "%.3f" % tank_det["center"][0],
                        "%.3f" % tank_det["center"][1],
                        "%.8f" % ray[0], "%.8f" % ray[1], "%.8f" % ray[2],
                        "unavailable_without_map_tf",
                    ]
                )

            cross = detect_red_cross_legacy_simple(frame, cross_cfg)
            landing = detect_landing_pad_legacy_old(frame, landing_cfg)
            cross_frames += bool(cross)
            landing_frames += bool(landing)
            if cross is not None:
                detail_writer.writerow(
                    [
                        frame_index, "%.6f" % (frame_index / fps if fps else 0.0),
                        "legacy_cross", "red_cross", "%.6f" % cross["confidence"],
                        "%.6f" % cross["geometry_confidence"], *cross["bbox"],
                        "%.3f" % cross["center"][0], "%.3f" % cross["center"][1],
                        "0.000", "1",
                    ]
                )
            if landing is not None:
                detail_writer.writerow(
                    [
                        frame_index, "%.6f" % (frame_index / fps if fps else 0.0),
                        "legacy_landing", "landing_pad", "%.6f" % landing["confidence"],
                        "%.6f" % landing["geometry_confidence"], *landing["bbox"],
                        "%.3f" % landing["center"][0], "%.3f" % landing["center"][1],
                        "%.3f" % landing.get("radius", 0.0), "1",
                    ]
                )

            flags = []
            if standard_dets:
                flags.append("yolo")
            if tank_dets:
                flags.append("tank_service")
            if circle is not None:
                flags.append("legacy_circle")
            if cross is not None:
                flags.append("red_cross")
            if landing is not None:
                flags.append("landing_pad")
            frames_with_any += bool(flags)
            frame_writer.writerow(
                [
                    frame_index, "%.6f" % (frame_index / fps if fps else 0.0),
                    legacy_output, "|".join(det["class_name"] for det in yolo_dets),
                    len(standard_dets), len(tank_dets), bool_text(circle is not None),
                    "%.3f" % circle["center_original_x"] if circle else "",
                    "%.3f" % circle["center_original_y"] if circle else "",
                    "%.3f" % circle["radius_original"] if circle else "",
                    bool_text(cross is not None), bool_text(landing is not None),
                    "|".join(flags),
                ]
            )

            drawn = draw_yolo_boxes(frame, yolo_dets)
            if circle is not None:
                draw_legacy_circle(drawn, circle)
            if cross is not None:
                draw_marker(drawn, cross, (0, 0, 255))
            if landing is not None:
                draw_marker(drawn, landing, (255, 255, 0))
            cv2.putText(
                drawn,
                "OLD full chain: 4cls+tank + legacy center + legacy geometry",
                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
            )
            cv2.putText(
                drawn,
                "frame=%d t=%s yolo=%d old_yolo=%s center=%s cross=%d land=%d" % (
                    frame_index,
                    format_timestamp(frame_index, fps),
                    len(yolo_dets), legacy_output or "pending",
                    "1" if circle else "0", bool(cross), bool(landing),
                ),
                (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
            )
            writer.write(drawn)

            if args.sample_stride > 0 and frame_index % args.sample_stride == 0:
                save_frame(samples_dir / ("sample_f%06d.jpg" % frame_index), drawn)
            should_event = (
                flags and saved_events < args.max_event_frames
                and frame_index - last_event_frame >= args.event_gap
            )
            if should_event:
                save_frame(samples_dir / ("event_f%06d.jpg" % frame_index), drawn)
                last_event_frame = frame_index
                saved_events += 1
            processed += 1
            frame_index += 1

    cap.release()
    writer.release()
    summary = {
        "video": str(video_path),
        "model": str(Path(args.model).resolve()),
        "tank_model": str(Path(args.tank_model).resolve()),
        "circle_config": str(Path(args.circle_config).resolve()),
        "cross_config": str(Path(args.cross_config).resolve()),
        "landing_config": str(Path(args.landing_config).resolve()),
        "fps": fps,
        "total_frames": total_frames,
        "processed_frames": processed,
        "frame_stride": args.frame_stride,
        "width": width,
        "height": height,
        "standard_detection_count": standard_detection_count,
        "tank_service_call_count": tank_service_calls,
        "legacy_circle_count": circle_count,
        "legacy_circle_frame_rate": circle_frames / float(processed) if processed else 0.0,
        "legacy_cross_frames": cross_frames,
        "legacy_landing_pad_frames": landing_frames,
        "frames_with_any_output": frames_with_any,
        "detector_counts": dict(detector_counts),
        "class_counts": dict(class_counts),
        "top_conf_per_class": dict(top_conf),
        "legacy_yolo_output_counts": dict(legacy_output_counts),
        "legacy_center_semantics": {
            "resize": "640x512",
            "published_center_coordinate": "old node uses resized ellipse center for display; projection ray uses center*2",
            "projected_coordinate_size_assumption": "1280x1024",
            "map_status": "unavailable_without_runtime_map_tf",
            "category_circle_association": "not implemented in old chain; global largest qualified blue ellipse",
        },
        "legacy_projection": {
            "circle_camera_model": "hardcoded K from old circle_detector_node.cpp, 1280x1024 assumption",
            "tank_service_camera_model": "hardcoded K from old image_process.cpp, 1280x1024 assumption",
            "ray_outputs": "available in legacy_projection_inputs.csv",
            "map_point": "unavailable_without_runtime_map_tf",
        },
        "known_legacy_risks": [
            "旧圆环检测全图选择面积最大的合格椭圆，不与 YOLO 类别框绑定。",
            "旧投影使用硬编码 1280x1024 相机模型，并把 640x512 中心乘 2。",
            "旧 image_process/image2center 也使用硬编码相机内参并依赖 map TF。",
            "旧视频回放没有 /detect/control、/detect/class_control、odom 和 TF，因此本报告按检测常开处理，不能等价声称已复现主控时序。",
        ],
        "outputs": {
            "annotated_video": str(output_dir / "annotated_legacy_full_chain.mp4"),
            "detections_csv": str(detail_path),
            "frame_csv": str(frame_path),
            "circle_csv": str(circle_path),
            "projection_csv": str(projection_path),
            "samples_dir": str(samples_dir),
        },
    }
    with open(output_dir / "summary.json", "w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def bool_text(value):
    return "true" if value else "false"


if __name__ == "__main__":
    main()
