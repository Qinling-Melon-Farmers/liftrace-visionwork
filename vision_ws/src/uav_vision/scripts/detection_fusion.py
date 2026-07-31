#!/usr/bin/env python3
"""detection_fusion: 合并多路检测结果并执行简单冲突裁决。

输入默认来自各检测器共享发布的 /uav_vision/detections。
本节点按 header.stamp 聚合同一图像帧的多条 TargetDetectionArray，
在输出前可选执行 bridge ↔ red_cross/landing_pad 冲突抑制，然后发布到
/uav_vision/detections_resolved，供 target_memory / compat_bridge 使用。
"""
from collections import OrderedDict

import rospy
from std_msgs.msg import String

from uav_vision.msg import TargetDetectionArray


VALID_ALIGN_MODES = {"disabled", "drop_circle", "drop_cross", "landing"}
STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}


class DetectionFusion:
    def __init__(self):
        rospy.init_node("detection_fusion")

        self._input_topic = rospy.get_param("~input_topic", "/uav_vision/detections")
        self._output_topic = rospy.get_param("~output_topic", "/uav_vision/detections_resolved")
        # 聚合窗口从第一条分支结果到达本节点时开始；若从源图像时间戳开始计时，较慢检测器
        # 尚未给出结果时，该帧的聚合桶就可能已经过期。
        self._flush_delay = rospy.get_param("~flush_delay", 0.8)
        self._source_sync_slop = float(
            rospy.get_param("~source_sync_slop_sec", 0.05))
        self._early_flush_complete_evidence = bool(
            rospy.get_param("~early_flush_complete_evidence", True))
        self._aux_geometry_confidence = rospy.get_param("~aux_geometry_confidence", 0.85)
        self._suppress_bridge_on_red_cross = rospy.get_param(
            "~suppress_bridge_on_red_cross", False
        )
        self._suppress_bridge_on_landing_pad = rospy.get_param(
            "~suppress_bridge_on_landing_pad", False
        )
        self._deduplicate_same_class = rospy.get_param("~deduplicate_same_class", True)
        self._require_red_cross_dual_confirmation = bool(
            rospy.get_param("~require_red_cross_dual_confirmation", True))
        self._dedup_iou_threshold = float(rospy.get_param("~dedup_iou_threshold", 0.30))
        self._dedup_center_ratio = float(rospy.get_param("~dedup_center_ratio", 0.35))
        self._align_mode_topic = rospy.get_param(
            "~align_mode_topic", "/uav_vision/align_mode")
        default_mode = rospy.get_param("~default_align_mode", "disabled")
        self._align_mode = (
            default_mode if default_mode in VALID_ALIGN_MODES else "disabled")
        self._buckets = OrderedDict()
        self._flushed_keys = OrderedDict()

        self._publisher = rospy.Publisher(self._output_topic, TargetDetectionArray, queue_size=4)
        self._subscriber = rospy.Subscriber(
            self._input_topic, TargetDetectionArray, self._on_detections, queue_size=16
        )
        self._mode_subscriber = rospy.Subscriber(
            self._align_mode_topic, String, self._on_align_mode, queue_size=2)
        self._timer = rospy.Timer(rospy.Duration(0.02), self._flush_ready)

        rospy.loginfo(
            "[DetectionFusion] ready  input=%s  output=%s  flush_delay=%.3fs  "
            "suppress_bridge_on_red_cross=%s  suppress_bridge_on_landing_pad=%s",
            self._input_topic,
            self._output_topic,
            self._flush_delay,
            self._suppress_bridge_on_red_cross,
            self._suppress_bridge_on_landing_pad,
        )

    def _on_align_mode(self, message):
        mode = message.data.strip()
        self._align_mode = mode if mode in VALID_ALIGN_MODES else "disabled"

    def _allowed_in_current_mode(self, class_name):
        if self._align_mode == "landing":
            return class_name == "landing_pad"
        if self._align_mode == "drop_cross":
            return class_name == "red_cross"
        if self._align_mode == "drop_circle":
            return class_name == "circle" or class_name in STANDARD_CLASSES
        # disabled 表示搜索/观察阶段：保留标准靶、红十字和圆环证据，但 H 在降落阶段前
        # 不得进入可操作链路。
        return class_name != "landing_pad"

    def _stamp_key(self, msg):
        stamp = msg.header.stamp
        if stamp.to_sec() <= 0:
            stamp = rospy.Time.now()
        return (stamp.secs, stamp.nsecs)

    def _on_detections(self, msg):
        key = self._stamp_key(msg)
        if key in self._flushed_keys:
            return
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        bucket = self._buckets.get(key)
        bucket_key = key
        if bucket is None and msg.source:
            candidates = []
            for candidate_key, candidate in self._buckets.items():
                if msg.source in candidate["sources"]:
                    continue
                delta = abs((candidate["stamp"] - stamp).to_sec())
                if delta <= self._source_sync_slop:
                    candidates.append((delta, candidate_key, candidate))
            if candidates:
                _delta, bucket_key, bucket = min(candidates, key=lambda item: item[0])
        if bucket is None:
            bucket = {
                "header": msg.header,
                "stamp": stamp,
                "received_at": rospy.Time.now(),
                "detections": [],
                "sources": set(),
                "member_keys": {key},
            }
            self._buckets[bucket_key] = bucket
        else:
            bucket["member_keys"].add(key)
            # 标准靶/红十字的类别判断属于分类器图像帧；仅降落模式属于降落检测器图像帧。
            if msg.source == "target_detector" or (
                    self._align_mode == "landing" and
                    msg.source == "landing_detector"):
                bucket["header"] = msg.header
                bucket["stamp"] = stamp

        bucket["detections"].extend(msg.detections)
        if msg.source:
            bucket["sources"].add(msg.source)
        if self._early_flush_complete_evidence and self._bucket_complete(bucket):
            self._buckets.pop(bucket_key, None)
            self._publish_bucket(bucket)
            self._mark_flushed(bucket, rospy.Time.now())

    def _mark_flushed(self, bucket, now):
        for member_key in bucket["member_keys"]:
            self._flushed_keys[member_key] = now

    def _bucket_complete(self, bucket):
        """Check whether every detector required by the active mode replied."""
        if self._align_mode == "landing":
            required = {"landing_detector"}
        elif self._align_mode == "drop_cross":
            required = {"target_detector", "cross_detector"}
        elif self._align_mode == "drop_circle":
            required = {"target_detector", "circle_detector"}
        else:
            required = {
                "target_detector", "circle_detector", "cross_detector",
            }
        return required.issubset(bucket["sources"])

    def _flush_ready(self, _event):
        if not self._buckets:
            return

        now = rospy.Time.now()
        expired_flushed = [
            key for key, flushed_at in self._flushed_keys.items()
            if (now - flushed_at).to_sec() > 5.0
        ]
        for key in expired_flushed:
            self._flushed_keys.pop(key, None)
        ready_keys = []
        for key, bucket in self._buckets.items():
            age = (now - bucket["received_at"]).to_sec()
            if age >= self._flush_delay:
                ready_keys.append(key)

        for key in ready_keys:
            bucket = self._buckets.pop(key, None)
            if bucket is not None:
                self._publish_bucket(bucket)
                self._mark_flushed(bucket, now)

    def _publish_bucket(self, bucket):
        detections = list(bucket["detections"])
        if self._require_red_cross_dual_confirmation:
            geometry_crosses = [
                detection for detection in detections
                if detection.class_name == "red_cross" and
                detection.geometry_verified and detection.center_refined
            ]
            classifier_crosses = [
                detection for detection in detections
                if detection.class_name == "red_cross" and
                not detection.geometry_verified
            ]
            confirmed_geometry = [
                geometry for geometry in geometry_crosses
                if any(self._overlaps(geometry, classifier)
                       for classifier in classifier_crosses)
            ]
            detections = [
                detection for detection in detections
                if detection.class_name != "red_cross"
            ] + confirmed_geometry
        if self._deduplicate_same_class:
            detections = self._deduplicate(detections)
        suppress_bridge = False
        for det in detections:
            if det.class_name == "red_cross" and det.geometry_verified and \
               det.geometry_confidence >= self._aux_geometry_confidence and \
               self._suppress_bridge_on_red_cross:
                suppress_bridge = True
            if det.class_name == "landing_pad" and det.geometry_verified and \
               det.geometry_confidence >= self._aux_geometry_confidence and \
               self._suppress_bridge_on_landing_pad:
                suppress_bridge = True

        out = TargetDetectionArray()
        out.header = bucket["header"]
        out.source = "detection_fusion"
        out.completed_sources = sorted(bucket["sources"])
        for det in detections:
            if not self._allowed_in_current_mode(det.class_name):
                continue
            if suppress_bridge and det.class_name == "bridge":
                continue
            out.detections.append(det)
        self._publisher.publish(out)

    @staticmethod
    def _roi(det):
        x1 = float(det.roi.x_offset)
        y1 = float(det.roi.y_offset)
        width = max(float(det.roi.width), float(det.center_px.z) * 2.0, 1.0)
        height = max(float(det.roi.height), float(det.center_px.z) * 2.0, 1.0)
        if det.roi.width == 0 and det.center_px.z > 0:
            x1 = float(det.center_px.x) - float(det.center_px.z)
        if det.roi.height == 0 and det.center_px.z > 0:
            y1 = float(det.center_px.y) - float(det.center_px.z)
        return x1, y1, x1 + width, y1 + height

    @classmethod
    def _iou(cls, left, right):
        lx1, ly1, lx2, ly2 = cls._roi(left)
        rx1, ry1, rx2, ry2 = cls._roi(right)
        overlap = max(0.0, min(lx2, rx2) - max(lx1, rx1)) * \
            max(0.0, min(ly2, ry2) - max(ly1, ry1))
        left_area = max(1.0, (lx2 - lx1) * (ly2 - ly1))
        right_area = max(1.0, (rx2 - rx1) * (ry2 - ry1))
        return overlap / max(left_area + right_area - overlap, 1.0)

    def _overlaps(self, left, right):
        if self._iou(left, right) >= self._dedup_iou_threshold:
            return True
        dx = float(left.center_px.x) - float(right.center_px.x)
        dy = float(left.center_px.y) - float(right.center_px.y)
        scale = max(
            float(left.roi.width), float(left.roi.height),
            float(right.roi.width), float(right.roi.height),
            float(left.center_px.z) * 2.0, float(right.center_px.z) * 2.0, 1.0,
        )
        return (dx * dx + dy * dy) ** 0.5 <= self._dedup_center_ratio * scale

    @staticmethod
    def _preference(det):
        return (
            int(bool(det.geometry_verified)),
            int(bool(det.center_refined)),
            float(det.geometry_confidence),
            float(det.class_confidence),
        )

    def _deduplicate(self, detections):
        kept = []
        for detection in sorted(detections, key=self._preference, reverse=True):
            duplicate = any(
                existing.class_name == detection.class_name and
                self._overlaps(existing, detection)
                for existing in kept
            )
            if not duplicate:
                kept.append(detection)
        return kept


def main():
    DetectionFusion()
    rospy.spin()


if __name__ == "__main__":
    main()
