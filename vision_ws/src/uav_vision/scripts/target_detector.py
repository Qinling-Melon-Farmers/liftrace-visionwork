#!/usr/bin/env python3
"""target_detector: YOLO 标准目标检测（dev/sim 路径，板端使用 RKNN 替代）。

订阅相机图像，运行 YOLO 推理，输出 TargetDetectionArray 到 /uav_vision/detections。
当前 dev/sim 默认支持类别：bridge, panzer, pillbox, tent, tank, red_cross。
"""
import rospy
import cv2
import numpy as np
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image
from uav_vision.msg import TargetDetection, TargetDetectionArray
from sensor_msgs.msg import RegionOfInterest

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class TargetDetector:
    def __init__(self):
        rospy.init_node("target_detector")

        # 模型选择由 launch/yaml 层负责，使源码可在不同笔记本工作区和未来部署设备间移植。
        self._model_path = rospy.get_param("~model_path", "")
        self._conf_threshold = rospy.get_param("~conf_threshold", 0.5)
        self._image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self._imgsz = rospy.get_param("~imgsz", 640)
        self._device = rospy.get_param("~device", "")
        self._perf_topic = rospy.get_param("~perf_topic", "/uav_vision/perf")

        self._model = YOLO(self._model_path) if YOLO is not None and self._model_path else None
        self._class_names = self._model.names if self._model is not None else {}
        self._frames = 0
        self._last_frame_time = None
        self._fps_ema = 0.0

        self._detections_pub = rospy.Publisher("/uav_vision/detections",
                                               TargetDetectionArray, queue_size=1)
        self._perf_pub = rospy.Publisher(self._perf_topic, DiagnosticArray, queue_size=1)
        self._image_sub = rospy.Subscriber(self._image_topic, Image,
                                            self._on_image, queue_size=1,
                                            buff_size=2**24)

        if self._model is None:
            rospy.logwarn("[TargetDetector] model unavailable (ultralytics=%s model_path=%r); "
                          "publishing empty detections for dev/sim launch compatibility",
                          YOLO is not None, self._model_path)
        else:
            rospy.loginfo("[TargetDetector] ready  model=%s  conf=%.2f  device=%s  classes=%s",
                          self._model_path, self._conf_threshold,
                          self._device if self._device != "" else "ultralytics_default",
                          list(self._class_names.values()))

    def _publish_perf(self, header, detections_count, total_ms, inference_ms,
                      callback_start, callback_end):
        now = callback_end
        if self._last_frame_time is not None:
            dt = max(now - self._last_frame_time, 1e-6)
            inst_fps = 1.0 / dt
            if self._fps_ema <= 0.0:
                self._fps_ema = inst_fps
            else:
                self._fps_ema = 0.8 * self._fps_ema + 0.2 * inst_fps
        self._last_frame_time = now
        self._frames += 1

        status = DiagnosticStatus()
        status.name = "uav_vision/target_detector"
        status.hardware_id = "dev_sim"
        degraded = self._model is None
        status.level = DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK
        status.message = "ultralytics_missing" if degraded else "ok"
        status.values = [
            KeyValue("backend", "ultralytics" if self._model is not None else "empty"),
            KeyValue("image_topic", self._image_topic),
            KeyValue("model_path", self._model_path),
            KeyValue("device", str(self._device)),
            KeyValue("frames", str(self._frames)),
            KeyValue("detections", str(int(detections_count))),
            KeyValue("processing_ms", f"{total_ms:.3f}"),
            KeyValue("inference_ms", f"{inference_ms:.3f}"),
            KeyValue("fps_ema", f"{self._fps_ema:.3f}"),
            KeyValue("callback_start_monotonic_sec",
                     f"{callback_start:.9f}"),
            KeyValue("callback_end_monotonic_sec",
                     f"{callback_end:.9f}"),
        ]
        msg = DiagnosticArray()
        msg.header = header
        msg.status = [status]
        self._perf_pub.publish(msg)

    @staticmethod
    def _image_to_bgr(msg):
        """Decode common 8-bit ROS encodings without the cv_bridge binary.

        The dev/sim detector runs in the ML Conda environment. Loading the
        system cv_bridge extension there can mix incompatible libffi builds,
        while these camera encodings need no compiled conversion bridge.
        """
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "mono8": 1,
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError("unsupported image encoding: %s" % msg.encoding)
        expected_row_bytes = int(msg.width) * channels
        if msg.step < expected_row_bytes:
            raise ValueError("invalid image step %d for width %d" % (msg.step, msg.width))
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = int(msg.step) * int(msg.height)
        if raw.size < expected_size:
            raise ValueError("short image buffer: %d < %d" % (raw.size, expected_size))
        rows = raw[:expected_size].reshape((msg.height, msg.step))
        pixels = rows[:, :expected_row_bytes]
        if channels == 1:
            return cv2.cvtColor(pixels.reshape((msg.height, msg.width)), cv2.COLOR_GRAY2BGR)
        image = pixels.reshape((msg.height, msg.width, channels))
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image.copy()

    def _on_image(self, msg):
        t0 = time.monotonic()
        try:
            img = self._image_to_bgr(msg)
        except Exception as e:
            rospy.logerr_throttle(5, f"[TargetDetector] image decode: {e}")
            return

        arr = TargetDetectionArray()
        arr.header = msg.header
        arr.source = "target_detector"
        arr.completed_sources = [arr.source]

        if self._model is None:
            self._detections_pub.publish(arr)
            callback_end = time.monotonic()
            total_ms = (callback_end - t0) * 1000.0
            self._publish_perf(
                msg.header, 0, total_ms, 0.0, t0, callback_end)
            return

        t_infer = time.monotonic()
        predict_kwargs = {
            "imgsz": self._imgsz,
            "conf": self._conf_threshold,
            "verbose": False,
        }
        if self._device != "":
            predict_kwargs["device"] = self._device
        results = self._model.predict(img, **predict_kwargs)
        infer_ms = (time.monotonic() - t_infer) * 1000.0

        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)

                det = TargetDetection()
                det.header = msg.header
                det.class_name = self._class_names.get(cls_id, f"class_{cls_id}")
                det.class_confidence = conf
                det.geometry_confidence = conf
                det.geometry_verified = False
                det.center_refined = False
                det.center_source = "bbox"
                det.association_valid = False
                det.reject_reason = "geometry_not_refined"
                det.transform_age_sec = -1.0
                det.roi = RegionOfInterest(
                    x_offset=int(x1), y_offset=int(y1),
                    width=int(x2 - x1), height=int(y2 - y1),
                    do_rectify=False)
                det.center_px.x = (x1 + x2) / 2.0
                det.center_px.y = (y1 + y2) / 2.0
                det.center_px.z = 0

                arr.detections.append(det)

        self._detections_pub.publish(arr)
        callback_end = time.monotonic()
        total_ms = (callback_end - t0) * 1000.0
        self._publish_perf(
            msg.header, len(arr.detections), total_ms, infer_ms,
            t0, callback_end)


def main():
    TargetDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
