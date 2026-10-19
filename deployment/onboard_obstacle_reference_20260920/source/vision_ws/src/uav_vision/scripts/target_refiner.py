#!/usr/bin/env python3
"""将标准靶检测与蓝色圆环几何观测关联。

检测器仍负责类别识别。本节点仅在两类观测属于同一帧且 ROI 匹配时，才用外围蓝环中心
替换标准靶的图像中心。未匹配的类别检测仍可用于搜索，但会明确标记为“中心未精修”，
避免下游投递逻辑把它误当成可用投放中心。
"""
import copy
import math

import rospy

from uav_vision.msg import TargetDetection, TargetDetectionArray
from uav_vision.target_selection_policy import resolve_class_profile


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}


def _point_in_roi(x, y, roi, margin):
    return (roi.x_offset - margin <= x <= roi.x_offset + roi.width + margin and
            roi.y_offset - margin <= y <= roi.y_offset + roi.height + margin)


def _center_in_roi(det, margin):
    return _point_in_roi(det.center_px.x, det.center_px.y, det.roi, margin)


class TargetRefiner:
    def __init__(self):
        rospy.init_node("target_refiner")
        self._input_topic = rospy.get_param(
            "~input_topic", "/uav_vision/detections_resolved")
        self._output_topic = rospy.get_param(
            "~output_topic", "/uav_vision/detections_refined")
        self._roi_margin_px = float(rospy.get_param("~roi_margin_px", 40.0))
        self._max_center_distance_ratio = float(
            rospy.get_param("~max_center_distance_ratio", 1.25))
        self._min_ring_quality = float(rospy.get_param("~min_ring_quality", 0.70))
        self._class_profile, selectable_classes = resolve_class_profile(
            rospy.get_param("~class_profile", "full"))
        self._selectable_standard_classes = (
            STANDARD_CLASSES.intersection(selectable_classes))

        self._pub = rospy.Publisher(self._output_topic,
                                    TargetDetectionArray, queue_size=2)
        rospy.Subscriber(self._input_topic, TargetDetectionArray,
                         self._on_detections, queue_size=4)
        rospy.loginfo(
            "[TargetRefiner] ready input=%s output=%s margin=%.1fpx profile=%s selectable_standard=%s",
            self._input_topic, self._output_topic, self._roi_margin_px,
            self._class_profile,
            ",".join(sorted(self._selectable_standard_classes)))

    def _association_score(self, target, circle):
        circle_in_target = _center_in_roi(circle, self._roi_margin_px)
        target_in_circle = _point_in_roi(
            target.center_px.x, target.center_px.y,
            circle.roi, self._roi_margin_px)
        if not (circle_in_target or target_in_circle):
            return None
        tx = target.center_px.x
        ty = target.center_px.y
        cx = circle.center_px.x
        cy = circle.center_px.y
        distance = math.hypot(tx - cx, ty - cy)
        target_scale = max(float(target.roi.width), float(target.roi.height),
                           float(circle.center_px.z) * 2.0, 1.0)
        if distance > self._max_center_distance_ratio * target_scale:
            return None
        # 代价越低越好：先选择高质量圆环，再选择中心误差较小的关联。
        return distance / target_scale + (1.0 - circle.geometry_confidence)

    def _on_detections(self, msg):
        circles = [d for d in msg.detections
                   if d.class_name == "circle" and
                   d.geometry_verified and d.center_refined and
                   d.geometry_confidence >= self._min_ring_quality]
        target_indices = [
            index for index, detection in enumerate(msg.detections)
            if detection.class_name in self._selectable_standard_classes
        ]
        pair_scores = []
        for target_index in target_indices:
            target = msg.detections[target_index]
            for circle_index, circle in enumerate(circles):
                score = self._association_score(target, circle)
                if score is not None:
                    pair_scores.append((score, target_index, circle_index))
        used_targets = set()
        used_circles = set()
        association_by_target = {}
        for _score, target_index, circle_index in sorted(pair_scores):
            if target_index in used_targets or circle_index in used_circles:
                continue
            used_targets.add(target_index)
            used_circles.add(circle_index)
            association_by_target[target_index] = circle_index

        output = TargetDetectionArray()
        output.header = msg.header
        output.source = "target_refiner"
        output.completed_sources = msg.completed_sources

        for detection_index, det in enumerate(msg.detections):
            if det.class_name not in STANDARD_CLASSES:
                if det.class_name == "circle":
                    output.detections.append(det)
                else:
                    output.detections.append(det)
                continue

            refined = copy.deepcopy(det)
            refined.center_refined = False
            refined.center_source = "bbox"
            refined.association_valid = False
            refined.reject_reason = (
                "class_profile_disallowed"
                if det.class_name not in self._selectable_standard_classes
                else "circle_association_missing")
            refined.map_valid = False
            refined.map_frame = ""
            refined.map_quality = 0.0
            refined.transform_age_sec = -1.0

            circle_index = association_by_target.get(detection_index)
            if circle_index is not None:
                circle = circles[circle_index]
                refined.center_px = circle.center_px
                refined.center_refined = bool(circle.geometry_verified)
                refined.center_source = "circle_geometry"
                refined.association_valid = bool(circle.geometry_verified)
                refined.reject_reason = (
                    "" if circle.geometry_verified else "geometry_quality_low")
                refined.geometry_confidence = min(
                    float(det.geometry_confidence),
                    float(circle.geometry_confidence))
                refined.geometry_verified = bool(circle.geometry_verified)

            output.detections.append(refined)

        self._pub.publish(output)


def main():
    TargetRefiner()
    rospy.spin()


if __name__ == "__main__":
    main()
