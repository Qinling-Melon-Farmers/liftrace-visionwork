#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OrangePi 5 Plus 板载显示器实时 RKNN 推理查看器/视频评测器。

- 相机、分辨率、帧率和标定 YAML 均由命令行指定
- 模型: 默认 merged_standard_fp32.rknn (六分类, RKNNLite 单核)
- 预处理: 畸变校正 + letterbox 640 + RGB + f32/255
- 仅做显示与推理, 不涉及任何执行机构/ROS 话题
- 退出: 窗口按 q 或 ESC; 或远程 pkill -f board_realtime_rknn_viewer

用法(板上):
    DISPLAY=:0 XAUTHORITY=/home/orangepi/.Xauthority \
        python3 board_realtime_rknn_viewer.py [rknn模型路径]

视频评测(不占用相机，可与实时窗口同时运行):
    python3 board_realtime_rknn_viewer.py --video real_target.mp4 \
        --json real_target_raw.json --output-video raw_result.mp4 \
        --no-window [rknn模型路径]
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

try:
    from rknnlite.api import RKNNLite
except ImportError:  # 允许本机只运行离线契约测试
    RKNNLite = None

DEFAULT_MODEL = os.environ.get(
    "UAV_VISION_RKNN_MODEL_PATH",
    str(Path(__file__).resolve().parents[1] /
        "vision_ws/src/uav_vision/models/merged_standard_fp32.rknn"))
NAMES = ["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"]
COLORS = [(255, 128, 0), (0, 200, 255), (0, 255, 0), (255, 0, 255), (0, 128, 255), (0, 0, 255)]
CONF_THRES = 0.5
IOU_THRES = 0.45
IMGSZ = 640
CAM_DEV = "/dev/video0"
CAM_W, CAM_H, CAM_FPS = 1280, 720, 30.0
WINDOW = "liftrace RKNN realtime"

# 默认使用本仓新相机 1280x720 标定；可用 --calibration 覆盖。
DEFAULT_CALIBRATION = str(
    Path(__file__).resolve().parents[1] /
    "vision_ws/src/camera_sdk/param/calibration_1280x720.yaml")
CALIBRATION_SOURCE = DEFAULT_CALIBRATION
CALIBRATION_WIDTH, CALIBRATION_HEIGHT = 1280, 720
CAMERA_K = np.array(
    [[725.3510059644434, 0.0, 631.67186313702575],
     [0.0, 723.34035628450874, 397.56638133116269],
     [0.0, 0.0, 1.0]],
    dtype=np.float32,
)
CAMERA_D = np.array(
    [0.0058668600963917095, 0.017910549546758369,
     -0.0010064115869294274, 0.0014715593681005204,
     -0.026485100937585344], dtype=np.float32)


def load_calibration(path):
    """Load one ROS CameraInfo YAML without re-solving or rescaling it."""
    with open(path, "r", encoding="utf-8") as handle:
        profile = yaml.safe_load(handle) or {}
    width = int(profile["image_width"])
    height = int(profile["image_height"])
    matrix = np.asarray(
        profile["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
    distortion = np.asarray(
        profile["distortion_coefficients"]["data"],
        dtype=np.float32).reshape(-1)
    if width <= 0 or height <= 0 or distortion.size < 4:
        raise ValueError("invalid camera calibration: %s" % path)
    return width, height, matrix, distortion


def configure_calibration(path):
    global CALIBRATION_SOURCE, CALIBRATION_WIDTH, CALIBRATION_HEIGHT
    global CAMERA_K, CAMERA_D
    (CALIBRATION_WIDTH, CALIBRATION_HEIGHT,
     CAMERA_K, CAMERA_D) = load_calibration(path)
    CALIBRATION_SOURCE = os.path.abspath(path)


def letterbox(frame):
    h, w = frame.shape[:2]
    r = min(IMGSZ / w, IMGSZ / h)
    nw, nh = round(w * r), round(h * r)
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((IMGSZ, IMGSZ, 3), 114, dtype=np.uint8)
    top, left = (IMGSZ - nh) // 2, (IMGSZ - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    tensor = (rgb.astype(np.float32) / 255.0)[None, ...]
    return tensor, r, left, top


def build_undistort_maps(width, height, allow_scale=False):
    """Build maps from the loaded profile; live capture requires exact size."""
    width, height = int(width), int(height)
    if width <= 0 or height <= 0:
        raise ValueError("invalid image size %sx%s" % (width, height))
    if (width, height) != (CALIBRATION_WIDTH, CALIBRATION_HEIGHT) and not allow_scale:
        raise ValueError(
            "capture %dx%d does not match calibration %dx%d" %
            (width, height, CALIBRATION_WIDTH, CALIBRATION_HEIGHT))
    sx = float(width) / CALIBRATION_WIDTH
    sy = float(height) / CALIBRATION_HEIGHT
    scaled_k = CAMERA_K.copy()
    scaled_k[0, :] *= sx
    scaled_k[1, :] *= sy
    new_k, _ = cv2.getOptimalNewCameraMatrix(
        scaled_k, CAMERA_D, (width, height), 0.0, (width, height)
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        scaled_k, CAMERA_D, None, new_k, (width, height), cv2.CV_16SC2
    )
    return map1, map2, new_k


def undistort_frame(frame, map1, map2):
    if map1 is None or map2 is None:
        return frame
    return cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)


def decode(outputs, r, left, top, class_names=None, conf_thres=CONF_THRES,
           iou_thres=IOU_THRES):
    class_names = NAMES if class_names is None else class_names
    arr = np.asarray(outputs[0] if isinstance(outputs, (list, tuple)) else outputs)
    arr = np.squeeze(arr)
    if arr.ndim == 2 and arr.shape[0] < arr.shape[1]:
        arr = arr.T  # -> (8400, 4+nc)
    if arr.ndim != 2 or arr.shape[1] < 4 + len(class_names):
        return []
    scores = arr[:, 4:4 + len(class_names)]
    best = scores.max(axis=1)
    mask = best >= conf_thres
    if not np.any(mask):
        return []
    boxes_xywh = arr[mask, :4]
    cls_ids = scores[mask].argmax(axis=1)
    confs = best[mask]
    # cx,cy,w,h (letterbox 像素) -> 原图 x1,y1,x2,y2
    nms_boxes = []
    for cx, cy, bw, bh in boxes_xywh:
        x = (cx - bw / 2 - left) / r
        y = (cy - bh / 2 - top) / r
        nms_boxes.append([float(x), float(y), float(bw / r), float(bh / r)])
    keep = cv2.dnn.NMSBoxes(nms_boxes, confs.astype(float).tolist(), conf_thres, iou_thres)
    dets = []
    for i in np.asarray(keep).flatten():
        x, y, bw, bh = nms_boxes[int(i)]
        dets.append((int(x), int(y), int(x + bw), int(y + bh), int(cls_ids[int(i)]), float(confs[int(i)])))
    return dets


def load_runtime(model_path):
    if RKNNLite is None:
        print("FATAL: rknnlite is not installed", flush=True)
        return None
    rt = RKNNLite()
    if rt.load_rknn(model_path) != 0 or rt.init_runtime() != 0:
        print("FATAL: rknn load/init failed", model_path, flush=True)
        return None
    print("MODEL_READY", model_path, flush=True)
    return rt


def percentile(values, p):
    return float(np.percentile(np.asarray(values, dtype=np.float64), p)) if values else None


def process_frame(frame, rt, map1=None, map2=None):
    total_start = time.perf_counter()
    rectify_start = total_start
    rectified = undistort_frame(frame, map1, map2)
    rectify_ms = (time.perf_counter() - rectify_start) * 1000.0
    prep_start = time.perf_counter()
    tensor, r, left, top = letterbox(rectified)
    prep_ms = (time.perf_counter() - prep_start) * 1000.0
    infer_start = time.perf_counter()
    outputs = rt.inference(inputs=[tensor])
    infer_ms = (time.perf_counter() - infer_start) * 1000.0
    post_start = time.perf_counter()
    dets = decode(outputs, r, left, top)
    post_ms = (time.perf_counter() - post_start) * 1000.0
    total_ms = (time.perf_counter() - total_start) * 1000.0
    return rectified, dets, {
        "rectify_ms": rectify_ms,
        "prep_ms": prep_ms,
        "infer_ms": infer_ms,
        "post_ms": post_ms,
        "total_ms": total_ms,
    }


def annotate(frame, dets, fps_ema=None, timing=None):
    for x1, y1, x2, y2, cid, conf in dets:
        color = COLORS[cid % len(COLORS)]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, "%s %.2f" % (NAMES[cid], conf), (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    if timing is not None:
        text = "fps %.1f  total %.0f ms  infer %.0f ms  dets %d" % (
            fps_ema or 0.0, timing["total_ms"], timing["infer_ms"], len(dets)
        )
    else:
        text = "dets %d" % len(dets)
    cv2.putText(frame, text, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(frame, "undistort: %s" % CALIBRATION_SOURCE, (12, 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)


def run_camera(rt, args):

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
    cap.set(cv2.CAP_PROP_FPS, args.camera_fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("FATAL: camera open failed %s" % args.camera, flush=True)
        return 1
    ok, first_frame = cap.read()
    if not ok or first_frame is None:
        print("FATAL: first camera frame unavailable %s" % args.camera,
              flush=True)
        cap.release()
        return 1
    actual_h, actual_w = first_frame.shape[:2]
    if (actual_w, actual_h) != (args.camera_width, args.camera_height):
        print(
            "FATAL: first frame %dx%d does not match requested %dx%d" %
            (actual_w, actual_h, args.camera_width, args.camera_height),
            flush=True)
        cap.release()
        return 1
    actual_fps = float(cap.get(5))
    if not args.apply_rectify:
        map1 = map2 = None
    else:
        map1, map2, _ = build_undistort_maps(actual_w, actual_h)
        print("CALIBRATION_READY source=%s size=%dx%d" %
              (CALIBRATION_SOURCE, actual_w, actual_h), flush=True)
    print("CAMERA_READY device=%s %dx%d@%s" %
          (args.camera, actual_w, actual_h, actual_fps), flush=True)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 720)

    fps_ema, last = None, time.perf_counter()
    frames = 0
    pending_frame = first_frame
    while True:
        if pending_frame is not None:
            frame = pending_frame
            pending_frame = None
        else:
            ok, frame = cap.read()
            if not ok:
                print("WARN: frame grab failed", flush=True)
                time.sleep(0.05)
                continue
        frame, dets, timing = process_frame(frame, rt, map1, map2)

        now = time.perf_counter()
        inst = 1.0 / max(now - last, 1e-6)
        last = now
        fps_ema = inst if fps_ema is None else fps_ema * 0.9 + inst * 0.1
        annotate(frame, dets, fps_ema, timing)
        cv2.imshow(WINDOW, frame)
        frames += 1
        if frames % 100 == 0:
            print("STATS frames=%d fps=%.1f total_ms=%.1f infer_ms=%.1f rectify_ms=%.1f dets=%d" %
                  (frames, fps_ema, timing["total_ms"], timing["infer_ms"],
                   timing["rectify_ms"], len(dets)), flush=True)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("EXIT_OK frames=%d" % frames, flush=True)
    return 0


def run_video(rt, args):
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print("FATAL: video open failed", args.video, flush=True)
        return 1
    # The board GStreamer backend auto-rotates this MP4 according to its
    # 90-degree metadata. Disable that so the decoded frame stays in the
    # camera/video pixel coordinate system (W=2560,H=1080), matching local PT.
    orientation_meta = float(cap.get(getattr(cv2, "CAP_PROP_ORIENTATION_META", 48)))
    orientation_auto_before = float(cap.get(getattr(cv2, "CAP_PROP_ORIENTATION_AUTO", 49)))
    cap.set(getattr(cv2, "CAP_PROP_ORIENTATION_AUTO", 49), 0)
    orientation_auto_after = float(cap.get(getattr(cv2, "CAP_PROP_ORIENTATION_AUTO", 49)))
    video_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    # Use the decoded frame shape rather than CAP_PROP metadata. On the board
    # OpenCV build this file reports 1080x2560 while decoded frames are HxW
    # = 1080x2560, so metadata width/height would build transposed maps.
    width = height = 0
    map1 = map2 = new_k = None

    timings = {name: [] for name in ("rectify_ms", "prep_ms", "infer_ms", "post_ms", "total_ms")}
    class_hist = [0] * len(NAMES)
    det_counts = []
    max_confs = []
    first_detections = None
    frame_idx = -1
    sampled = 0
    measured = 0
    started = time.perf_counter()
    writer = None
    output_size = None
    output_frames = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if width == 0:
            height, width = frame.shape[:2]
            if args.apply_rectify:
                map1, map2, new_k = build_undistort_maps(
                    width, height, allow_scale=True)
                print("CALIBRATION_READY source=%s decoded=%dx%d" %
                      (CALIBRATION_SOURCE, width, height), flush=True)
            if args.output_video:
                output_dir = os.path.dirname(args.output_video)
                if output_dir:
                    os.makedirs(output_dir, exist_ok=True)
                output_fps = video_fps / max(1, args.stride) if video_fps > 0 else 5.0
                output_width = min(width, max(1, args.output_width))
                output_height = int(round(height * output_width / width))
                output_size = (output_width, output_height)
                writer = cv2.VideoWriter(
                    args.output_video, cv2.VideoWriter_fourcc(*"mp4v"),
                    output_fps, (output_width, output_height)
                )
                if not writer.isOpened():
                    raise RuntimeError("video writer open failed: " + args.output_video)
                print("OUTPUT_VIDEO_READY %s %.3f FPS %dx%d" %
                      (args.output_video, output_fps, output_width, output_height), flush=True)
        frame_idx += 1
        if frame_idx % max(1, args.stride) != 0:
            continue
        sampled += 1
        rectified, dets, timing = process_frame(frame, rt, map1, map2)
        if sampled <= args.warmup:
            continue
        measured += 1
        for key in timings:
            timings[key].append(timing[key])
        det_counts.append(len(dets))
        max_confs.append(max((d[5] for d in dets), default=0.0))
        for det in dets:
            class_hist[det[4]] += 1
        if first_detections is None:
            first_detections = [
                {"xyxy": list(det[:4]), "class_id": det[4], "class_name": NAMES[det[4]], "confidence": det[5]}
                for det in dets
            ]
        if args.show or writer is not None:
            rendered = rectified.copy()
            annotate(rendered, dets, timing=timing)
            if writer is not None:
                output_width, output_height = output_size
                if rendered.shape[1] != output_width or rendered.shape[0] != output_height:
                    rendered = cv2.resize(rendered, (output_width, output_height), interpolation=cv2.INTER_AREA)
                writer.write(rendered)
                output_frames += 1
            if args.show:
                cv2.imshow(WINDOW, rendered)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    cap.release()
    if writer is not None:
        writer.release()
    if args.show:
        cv2.destroyAllWindows()
    elapsed = time.perf_counter() - started
    report = {
        "model": args.model,
        "source": args.video,
        "input": {"width": width, "height": height, "fps": video_fps},
        "video_orientation": {"metadata_degrees": orientation_meta,
                              "auto_before": orientation_auto_before,
                              "auto_after": orientation_auto_after},
        "preprocess": {"letterbox": IMGSZ, "rgb": True, "float32_0_1": True},
        "calibration": {
            "enabled": args.apply_rectify,
            "source": CALIBRATION_SOURCE,
            "source_size": [CALIBRATION_WIDTH, CALIBRATION_HEIGHT],
            "camera_matrix": CAMERA_K.tolist(),
            "distortion": CAMERA_D.tolist(),
            "effective_camera_matrix": new_k.tolist() if new_k is not None else None,
        },
        "sampling": {"stride": args.stride, "warmup": args.warmup,
                     "frames_sampled": sampled, "frames_measured": measured,
                     "output_video": args.output_video,
                     "output_frames": output_frames},
        "metrics_ms": {
            key: {"p50": percentile(values, 50), "p95": percentile(values, 95),
                  "mean": float(np.mean(values)) if values else None}
            for key, values in timings.items()
        },
        "derived": {
            "throughput_from_total_p50_fps":
                1000.0 / percentile(timings["total_ms"], 50) if timings["total_ms"] else None,
            "wall_fps": measured / elapsed if elapsed > 0 else None,
            "median_detections": float(np.median(det_counts)) if det_counts else None,
            "median_max_confidence": float(np.median(max_confs)) if max_confs else None,
            "class_hist_total": class_hist,
            "first_detections": first_detections or [],
        },
    }
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", nargs="?", default=DEFAULT_MODEL,
                        help="RKNN model path (default: %(default)s)")
    parser.add_argument("--camera", default=CAM_DEV,
                        help="V4L2 device path; prefer /dev/v4l/by-id/... path")
    parser.add_argument("--camera-width", type=int, default=CAM_W)
    parser.add_argument("--camera-height", type=int, default=CAM_H)
    parser.add_argument("--camera-fps", type=float, default=CAM_FPS)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION,
                        help="ROS CameraInfo YAML for live undistortion")
    parser.add_argument("--video", help="run a sampled video benchmark instead of camera")
    parser.add_argument("--json", help="write benchmark JSON report")
    parser.add_argument("--output-video", help="write annotated sampled video")
    parser.add_argument("--output-width", type=int, default=1280,
                        help="annotated video width, default 1280")
    parser.add_argument("--stride", type=int, default=12, help="sample every Nth video frame")
    parser.add_argument("--warmup", type=int, default=5, help="discard first N sampled frames")
    parser.add_argument("--show", action="store_true", help="show video benchmark on board display")
    parser.add_argument("--no-window", action="store_true", help="disable display window")
    parser.add_argument("--no-rectify", action="store_true", help="disable fixed camera undistortion")
    parser.add_argument("--rectify-video", action="store_true",
                        help="apply fixed camera undistortion to video mode (off by default)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if len(args.fourcc) != 4:
        raise SystemExit("--fourcc must contain exactly four characters")
    if args.video and args.no_rectify and args.rectify_video:
        raise SystemExit("--no-rectify and --rectify-video are mutually exclusive")
    # File replay stays in the original pixel coordinate system by default;
    # only live camera input uses the fixed calibration automatically.
    args.apply_rectify = (not args.no_rectify) and (not args.video or args.rectify_video)
    if args.apply_rectify:
        try:
            configure_calibration(args.calibration)
        except (OSError, KeyError, TypeError, ValueError) as error:
            print("FATAL: calibration load failed %s: %s" %
                  (args.calibration, error), flush=True)
            return 2
    if args.video and args.no_window:
        args.show = False
    elif args.video and not args.show:
        args.show = False
    else:
        args.show = not args.no_window
    rt = load_runtime(args.model)
    if rt is None:
        return 1
    try:
        if args.video:
            return run_video(rt, args)
        return run_camera(rt, args)
    finally:
        rt.release()


if __name__ == "__main__":
    sys.exit(main())
