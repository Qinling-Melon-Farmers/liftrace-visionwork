#!/usr/bin/env python3
"""target_detector_rknn: OrangePi/RK3588 板端标准目标检测入口。

设计目标：
- 优先使用显式配置的 unified 6-class RKNN
- 历史 split assets 仅在调用方显式提供路径时启用
- 当前环境无 RKNNLite 或没有可用模型时启动失败，不发布伪完成空检测

说明：
- 本节点不依赖 PyTorch / ultralytics 运行时
- 板端真实推理仍需在 OrangePi 5 Plus 上做最终验证
"""
import logging
import os
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image, RegionOfInterest

from uav_vision.msg import TargetDetection, TargetDetectionArray


def _restore_standard_logging_levels():
    """Undo RKNNLite 2.3.x's process-wide one-letter logging names."""
    # rknn_log replaces DEBUG/INFO/... with D/I/... during import.  rospy reads
    # its logging config afterwards and requires the standard names.
    aliases = (
        (logging.CRITICAL, "FATAL"),
        (logging.WARNING, "WARN"),
    )
    standard = (
        (logging.CRITICAL, "CRITICAL"),
        (logging.ERROR, "ERROR"),
        (logging.WARNING, "WARNING"),
        (logging.INFO, "INFO"),
        (logging.DEBUG, "DEBUG"),
        (logging.NOTSET, "NOTSET"),
    )
    for level, name in aliases + standard:
        logging.addLevelName(level, name)

try:
    from rknnlite.api import RKNNLite
    _restore_standard_logging_levels()
except ImportError:
    RKNNLite = None


def _load_metadata(metadata_path):
    if not metadata_path or not os.path.exists(metadata_path):
        return {}
    try:
        with open(metadata_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception as exc:  # pragma: no cover - best effort
        rospy.logwarn("[TargetDetectorRKNN] failed to parse metadata %s: %s",
                      metadata_path, exc)
        return {}


def _load_names(metadata_path):
    meta = _load_metadata(metadata_path)
    names = meta.get("names", {})
    return {int(k): str(v) for k, v in names.items()}


def _clip(value, low, high):
    return max(low, min(high, value))


def _letterbox_bgr(img, new_shape=640, color=(114, 114, 114)):
    """YOLO 常见 letterbox，返回 padded 图、缩放比例和 padding。"""
    shape = img.shape[:2]  # (h, w)
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    new_h, new_w = int(new_shape[0]), int(new_shape[1])

    r = min(new_h / shape[0], new_w / shape[1])
    resized_w = int(round(shape[1] * r))
    resized_h = int(round(shape[0] * r))

    dw = new_w - resized_w
    dh = new_h - resized_h
    dw /= 2.0
    dh /= 2.0

    if (shape[1], shape[0]) != (resized_w, resized_h):
        img = cv2.resize(img, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=color)
    return img, r, (dw, dh)


def _to_model_input(img_bgr, imgsz, layout="NHWC", color_space="RGB",
                     input_dtype="float32", normalize=True):
    """构造 RKNN 输入，并保留 letterbox 逆变换参数。

    旧 split RKNN 默认使用 float32/255；由 Toolkit2 量化导出的统一模型
    可通过配置切换为原始 uint8。不要把 uint8 输入再次除以 255，否则
    RKNNLite 会收到错误的字节/尺度契约。
    """
    padded, scale, pad = _letterbox_bgr(img_bgr, imgsz)
    if color_space.upper() == "RGB":
        padded = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)

    dtype_name = str(input_dtype).lower()
    if dtype_name in ("uint8", "u8"):
        tensor = padded.astype(np.uint8)
    elif dtype_name in ("int8", "i8"):
        tensor = padded.astype(np.int16).clip(-128, 127).astype(np.int8)
    else:
        tensor = padded.astype(np.float32)
        if normalize:
            tensor /= 255.0
    if layout.upper() == "NCHW":
        tensor = np.transpose(tensor, (2, 0, 1))
    tensor = np.expand_dims(tensor, axis=0)
    return tensor, scale, pad, padded.shape[:2]


def _xywh_to_xyxy(boxes):
    out = np.empty_like(boxes)
    out[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    out[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    out[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    out[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
    return out


def _scale_boxes(boxes, orig_shape, scale, pad):
    """将输入尺度上的框还原到原图尺寸。"""
    out = boxes.copy()
    out[:, [0, 2]] -= pad[0]
    out[:, [1, 3]] -= pad[1]
    out[:, :4] /= max(scale, 1e-6)
    h, w = orig_shape[:2]
    out[:, 0] = np.clip(out[:, 0], 0, w - 1)
    out[:, 1] = np.clip(out[:, 1], 0, h - 1)
    out[:, 2] = np.clip(out[:, 2], 0, w - 1)
    out[:, 3] = np.clip(out[:, 3], 0, h - 1)
    return out


def _iou_xyxy(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area1 = np.maximum(0.0, box[2] - box[0]) * np.maximum(0.0, box[3] - box[1])
    area2 = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0.0, boxes[:, 3] - boxes[:, 1])
    union = area1 + area2 - inter + 1e-6
    return inter / union


def _nms_classwise(boxes, scores, classes, iou_threshold):
    if len(boxes) == 0:
        return np.array([], dtype=np.int32)

    keep = []
    for cls in np.unique(classes):
        cls_mask = (classes == cls)
        cls_indices = np.where(cls_mask)[0]
        cls_boxes = boxes[cls_indices]
        cls_scores = scores[cls_indices]
        order = np.argsort(-cls_scores)

        while order.size > 0:
            i = order[0]
            keep.append(cls_indices[i])
            if order.size == 1:
                break
            ious = _iou_xyxy(cls_boxes[i], cls_boxes[order[1:]])
            order = order[1:][ious < iou_threshold]
    return np.array(sorted(keep), dtype=np.int32)


def _candidate_arrays(outputs):
    """尽量把各类 RKNN 输出规整成 [N, C] 形式。"""
    tensors = []
    for out in outputs:
        arr = np.asarray(out)
        arr = np.squeeze(arr)
        if arr.size == 0:
            continue
        if arr.ndim == 1:
            continue
        if arr.ndim == 2:
            # Ultralytics/RKNN 常见输出为 (1, C, 8400)，squeeze 后为
            # (C, 8400)。统一先变成候选框在行、通道在列的形式。
            # 8400 不是硬编码的输入尺寸，只用“较小的通道维”判定。
            if arr.shape[0] >= 4 and arr.shape[1] > arr.shape[0] and arr.shape[0] < 128:
                arr = arr.T
        if arr.ndim > 2:
            # 尝试把 channel 维挪到最后，再展平空间维
            c_first = arr.shape[0]
            c_last = arr.shape[-1]
            if c_last >= 5 and c_last >= c_first:
                arr = arr.reshape(-1, c_last)
            elif c_first >= 5:
                arr = np.moveaxis(arr, 0, -1).reshape(-1, c_first)
            else:
                arr = arr.reshape(arr.shape[0], -1)
        if arr.ndim != 2:
            continue
        if arr.shape[1] < arr.shape[0] and arr.shape[0] >= 5 and arr.shape[1] < 5:
            continue
        tensors.append(arr)
    return tensors


def _decode_outputs(outputs, num_classes, conf_threshold, imgsz, orig_shape, scale, pad,
                    iou_threshold=0.45, box_format="auto"):
    """通用解码：
    - 支持 [N, 4+nc]、[N, 5+nc]、[4+nc, N]、[5+nc, N] 等常见形式
    - 若格式无法识别则返回空
    """
    boxes_all = []
    scores_all = []
    classes_all = []

    for arr in _candidate_arrays(outputs):
        if arr.shape[1] < 4 + num_classes and arr.shape[0] >= 4 + num_classes:
            arr = arr.T
        channels = arr.shape[1]
        if channels not in (4 + num_classes, 5 + num_classes):
            continue

        coords = arr[:, :4].astype(np.float32)
        if channels == 5 + num_classes:
            obj = arr[:, 4].astype(np.float32)
            cls_scores = arr[:, 5:].astype(np.float32)
        else:
            obj = np.ones((arr.shape[0],), dtype=np.float32)
            cls_scores = arr[:, 4:].astype(np.float32)

        cls_id = np.argmax(cls_scores, axis=1).astype(np.int32)
        cls_conf = cls_scores[np.arange(cls_scores.shape[0]), cls_id]
        conf = obj * cls_conf
        valid = conf >= conf_threshold
        if not np.any(valid):
            continue

        coords = coords[valid]
        conf = conf[valid]
        cls_id = cls_id[valid]

        # 归一化输出时放大到输入尺寸
        if np.max(coords) <= 2.0:
            coords[:, [0, 2]] *= float(imgsz)
            coords[:, [1, 3]] *= float(imgsz)

        use_xyxy = False
        if box_format == "xyxy":
            use_xyxy = True
        elif box_format == "xywh":
            use_xyxy = False
        else:
            # auto：若绝大多数框满足 x2>x1 且 y2>y1，则按 xyxy 解释
            xyxy_ratio = np.mean((coords[:, 2] > coords[:, 0]) & (coords[:, 3] > coords[:, 1]))
            use_xyxy = xyxy_ratio > 0.8

        boxes = coords if use_xyxy else _xywh_to_xyxy(coords)
        boxes = _scale_boxes(boxes, orig_shape, scale, pad)

        keep = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
        if not np.any(keep):
            continue
        boxes_all.append(boxes[keep])
        scores_all.append(conf[keep])
        classes_all.append(cls_id[keep])

    if not boxes_all:
        return []

    boxes = np.concatenate(boxes_all, axis=0)
    scores = np.concatenate(scores_all, axis=0)
    classes = np.concatenate(classes_all, axis=0)
    keep = _nms_classwise(boxes, scores, classes, iou_threshold)
    dets = []
    for idx in keep:
        dets.append({
            "bbox": boxes[idx],
            "score": float(scores[idx]),
            "class_id": int(classes[idx]),
        })
    return dets


class _RknnHandle:
    def __init__(self, model_path, metadata_path, tag):
        self.model_path = model_path
        self.metadata_path = metadata_path
        self.tag = tag
        self.meta = _load_metadata(metadata_path)
        self.names = _load_names(metadata_path)
        self.runtime = None
        self.num_classes = len(self.names)

        if RKNNLite is None:
            return
        if not model_path:
            return
        if not os.path.exists(model_path):
            rospy.logwarn("[TargetDetectorRKNN] %s model not found: %s",
                          tag, model_path)
            return

        try:
            self.runtime = RKNNLite()
            if self.runtime.load_rknn(model_path) != 0:
                rospy.logwarn("[TargetDetectorRKNN] failed to load %s RKNN model: %s",
                              tag, model_path)
                self.runtime = None
                return
            if self.runtime.init_runtime() != 0:
                rospy.logwarn("[TargetDetectorRKNN] failed to init %s RKNN runtime",
                              tag)
                self.runtime = None
                return
            rospy.loginfo("[TargetDetectorRKNN] %s runtime ready: %s (%d classes)",
                          tag, model_path, self.num_classes)
        except Exception as exc:  # pragma: no cover - board runtime only
            rospy.logwarn("[TargetDetectorRKNN] failed to init %s runtime: %s",
                          tag, exc)
            self.runtime = None

    def available(self):
        return self.runtime is not None and self.num_classes > 0


class TargetDetectorRKNN:
    def __init__(self):
        rospy.init_node("target_detector_rknn")

        self._image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self._conf_threshold = float(rospy.get_param("~conf_threshold", 0.5))
        self._iou_threshold = float(rospy.get_param("~iou_threshold", 0.45))
        self._imgsz = int(rospy.get_param("~imgsz", 640))
        self._layout = str(rospy.get_param("~input_layout", "NHWC"))
        self._color_space = str(rospy.get_param("~input_color_space", "RGB"))
        self._input_dtype = str(rospy.get_param("~input_dtype", "float32"))
        self._input_normalize = bool(rospy.get_param("~input_normalize", True))
        self._box_format = str(rospy.get_param("~box_format", "auto"))
        self._perf_topic = rospy.get_param("~perf_topic", "/uav_vision/perf")

        self._model_path = rospy.get_param("~model_path", "")
        self._metadata_path = rospy.get_param("~metadata_path", "")
        self._std_model_path = rospy.get_param(
            "~std_model_path",
            "",
        )
        self._std_metadata_path = rospy.get_param(
            "~std_metadata_path",
            "",
        )
        self._tank_model_path = rospy.get_param(
            "~tank_model_path",
            "",
        )
        self._tank_metadata_path = rospy.get_param(
            "~tank_metadata_path",
            "",
        )

        self._bridge = CvBridge()
        self._detections_pub = rospy.Publisher("/uav_vision/detections",
                                               TargetDetectionArray, queue_size=1)
        self._perf_pub = rospy.Publisher(self._perf_topic, DiagnosticArray, queue_size=1)

        self._unified = _RknnHandle(self._model_path, self._metadata_path, "unified")
        self._std = _RknnHandle(self._std_model_path, self._std_metadata_path, "standard")
        self._tank = _RknnHandle(self._tank_model_path, self._tank_metadata_path, "tank")
        self._warned_decode = set()
        self._frames = 0
        self._last_frame_time = None
        self._fps_ema = 0.0

        if self._unified.available():
            rospy.loginfo("[TargetDetectorRKNN] unified model selected")
        elif self._std.available() or self._tank.available():
            rospy.loginfo("[TargetDetectorRKNN] split RKNN assets selected (std + tank)")
        else:
            if RKNNLite is None:
                raise RuntimeError(
                    "RKNNLite is unavailable in the board ROS Python")
            else:
                raise RuntimeError("no usable RKNN runtime/model found")

        # Advertise the image subscription only after every callback-visible
        # runtime handle and counter is ready. A replay publisher can emit its
        # first frame synchronously as soon as it observes this subscriber.
        self._image_sub = rospy.Subscriber(self._image_topic, Image,
                                           self._on_image, queue_size=1,
                                           buff_size=2**24)

    def _empty_publish(self, header):
        arr = TargetDetectionArray()
        arr.header = header
        arr.source = "target_detector"
        arr.completed_sources = [arr.source]
        self._detections_pub.publish(arr)

    def _warn_once(self, key, msg, *args):
        if key in self._warned_decode:
            return
        self._warned_decode.add(key)
        rospy.logwarn(msg, *args)

    def _backend_name(self):
        if self._unified.available():
            return "rknn_unified"
        if self._std.available() or self._tank.available():
            return "rknn_split"
        return "unavailable"

    def _publish_perf(self, header, detections_count, total_ms, inference_ms):
        now = time.perf_counter()
        if self._last_frame_time is not None:
            dt = max(now - self._last_frame_time, 1e-6)
            inst_fps = 1.0 / dt
            if self._fps_ema <= 0.0:
                self._fps_ema = inst_fps
            else:
                self._fps_ema = 0.8 * self._fps_ema + 0.2 * inst_fps
        self._last_frame_time = now
        self._frames += 1

        degraded = not (self._unified.available() or self._std.available() or self._tank.available())
        status = DiagnosticStatus()
        status.name = "uav_vision/target_detector_rknn"
        status.hardware_id = "board_path"
        status.level = DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK
        status.message = self._backend_name()
        status.values = [
            KeyValue("backend", self._backend_name()),
            KeyValue("image_topic", self._image_topic),
            KeyValue("frames", str(self._frames)),
            KeyValue("detections", str(int(detections_count))),
            KeyValue("processing_ms", f"{total_ms:.3f}"),
            KeyValue("inference_ms", f"{inference_ms:.3f}"),
            KeyValue("fps_ema", f"{self._fps_ema:.3f}"),
            KeyValue("input_dtype", self._input_dtype),
            KeyValue("input_normalize", str(self._input_normalize)),
            KeyValue("unified_model", self._model_path),
            KeyValue("std_model", self._std_model_path),
            KeyValue("tank_model", self._tank_model_path),
        ]
        msg = DiagnosticArray()
        msg.header = header
        msg.status = [status]
        self._perf_pub.publish(msg)

    def _infer_handle(self, handle, img_bgr):
        if not handle.available():
            return [], 0.0
        tensor, scale, pad, _ = _to_model_input(
            img_bgr,
            self._imgsz,
            layout=self._layout,
            color_space=self._color_space,
            input_dtype=self._input_dtype,
            normalize=self._input_normalize,
        )
        try:
            t_infer = time.perf_counter()
            outputs = handle.runtime.inference(inputs=[tensor])
            infer_ms = (time.perf_counter() - t_infer) * 1000.0
        except Exception as exc:  # pragma: no cover - board runtime only
            self._warn_once(
                (handle.tag, "infer"),
                "[TargetDetectorRKNN] %s inference failed: %s",
                handle.tag, exc,
            )
            return [], 0.0

        detections = _decode_outputs(
            outputs=outputs,
            num_classes=handle.num_classes,
            conf_threshold=self._conf_threshold,
            imgsz=self._imgsz,
            orig_shape=img_bgr.shape,
            scale=scale,
            pad=pad,
            iou_threshold=self._iou_threshold,
            box_format=self._box_format,
        )
        if not detections and outputs:
            shapes = [tuple(np.asarray(out).shape) for out in outputs]
            self._warn_once(
                (handle.tag, "decode"),
                "[TargetDetectorRKNN] %s produced no detections after decode/threshold; "
                "shapes=%s threshold=%.3f",
                handle.tag, shapes, self._conf_threshold,
            )
        return detections, infer_ms

    def _build_msg(self, header, detections, handle):
        arr = TargetDetectionArray()
        arr.header = header
        arr.source = "target_detector"
        arr.completed_sources = [arr.source]
        for det in detections:
            cls_id = det["class_id"]
            x1, y1, x2, y2 = det["bbox"]
            msg = TargetDetection()
            msg.header = header
            msg.class_name = handle.names.get(cls_id, "class_%d" % cls_id)
            msg.class_confidence = float(det["score"])
            msg.geometry_confidence = float(det["score"])
            msg.geometry_verified = False
            msg.center_refined = False
            msg.center_source = "bbox"
            msg.association_valid = False
            msg.reject_reason = "geometry_not_refined"
            msg.transform_age_sec = -1.0
            x1 = int(round(x1))
            y1 = int(round(y1))
            x2 = int(round(x2))
            y2 = int(round(y2))
            msg.roi = RegionOfInterest(
                x_offset=max(0, x1),
                y_offset=max(0, y1),
                width=max(0, x2 - x1),
                height=max(0, y2 - y1),
                do_rectify=False,
            )
            msg.center_px.x = (x1 + x2) / 2.0
            msg.center_px.y = (y1 + y2) / 2.0
            msg.center_px.z = 0.0
            arr.detections.append(msg)
        return arr

    def _on_image(self, msg):
        t0 = time.perf_counter()
        try:
            img_bgr = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            rospy.logerr_throttle(5, "[TargetDetectorRKNN] cv_bridge: %s", exc)
            return

        if self._unified.available():
            detections, infer_ms = self._infer_handle(self._unified, img_bgr)
            arr = self._build_msg(msg.header, detections, self._unified)
            self._detections_pub.publish(arr)
            total_ms = (time.perf_counter() - t0) * 1000.0
            self._publish_perf(msg.header, len(arr.detections), total_ms, infer_ms)
            return

        merged = []
        infer_ms = 0.0
        if self._std.available():
            std_dets, std_ms = self._infer_handle(self._std, img_bgr)
            infer_ms += std_ms
            merged.extend((self._std, det) for det in std_dets)
        if self._tank.available():
            tank_dets, tank_ms = self._infer_handle(self._tank, img_bgr)
            infer_ms += tank_ms
            merged.extend((self._tank, det) for det in tank_dets)

        if not merged:
            self._empty_publish(msg.header)
            total_ms = (time.perf_counter() - t0) * 1000.0
            self._publish_perf(msg.header, 0, total_ms, infer_ms)
            return

        arr = TargetDetectionArray()
        arr.header = msg.header
        arr.source = "target_detector"
        arr.completed_sources = [arr.source]
        for handle, det in merged:
            partial = self._build_msg(msg.header, [det], handle)
            arr.detections.extend(partial.detections)
        self._detections_pub.publish(arr)
        total_ms = (time.perf_counter() - t0) * 1000.0
        self._publish_perf(msg.header, len(arr.detections), total_ms, infer_ms)


def main():
    TargetDetectorRKNN()
    rospy.spin()


if __name__ == "__main__":
    main()
