#!/usr/bin/env python3
"""隔离的辅助粗候选记忆。

该记忆只发布 TargetCandidateArray，不做优先级选择，也不发布 selected_target。标准五类
在地图近邻内允许类别投票，目的是抑制斜视透视导致的单帧类别抖动。
"""

import math

import rospy
from std_srvs.srv import Empty, EmptyResponse

from uav_vision.msg import (
    TargetCandidate, TargetCandidateArray, TargetDetectionArray)


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}
ST_DETECTED = 0
ST_OBSERVING = 1
ST_CONFIRMED = 2


class Record:
    def __init__(self, candidate_id, detection, stamp):
        self.id = candidate_id
        self.class_votes = {detection.class_name: float(detection.class_confidence)}
        self.detection = detection
        self.first_seen = stamp
        self.last_seen = stamp
        self.observe_count = 1
        self.consecutive_count = 1
        self.map_weight = max(float(detection.map_quality), 0.01)

    @staticmethod
    def _same_group(left, right):
        return (left in STANDARD_CLASSES and right in STANDARD_CLASSES) or left == right

    def matches(self, detection, distance_limit):
        if not self._same_group(self.detection.class_name, detection.class_name):
            return False
        if self.detection.map_frame != detection.map_frame:
            return False
        dx = self.detection.map_point.x - detection.map_point.x
        dy = self.detection.map_point.y - detection.map_point.y
        return math.hypot(dx, dy) <= distance_limit

    def update(self, detection, stamp):
        weight = max(float(detection.map_quality), 0.01)
        total = self.map_weight + weight
        detection.map_point.x = (
            self.detection.map_point.x * self.map_weight +
            detection.map_point.x * weight) / total
        detection.map_point.y = (
            self.detection.map_point.y * self.map_weight +
            detection.map_point.y * weight) / total
        detection.map_point.z = (
            self.detection.map_point.z * self.map_weight +
            detection.map_point.z * weight) / total
        self.map_weight = total
        self.class_votes[detection.class_name] = (
            self.class_votes.get(detection.class_name, 0.0) +
            float(detection.class_confidence))
        winning_class = max(self.class_votes, key=self.class_votes.get)
        detection.class_name = winning_class
        self.detection = detection
        self.last_seen = stamp
        self.observe_count += 1
        self.consecutive_count += 1

    def missed(self):
        self.consecutive_count = 0

    def to_message(self, stamp, confirm_frames):
        message = TargetCandidate()
        message.header.stamp = stamp
        message.header.frame_id = self.detection.map_frame
        message.id = self.id
        message.class_name = self.detection.class_name
        message.class_confidence = self.detection.class_confidence
        message.geometry_confidence = self.detection.geometry_confidence
        message.roi = self.detection.roi
        message.center_px = self.detection.center_px
        message.center_refined = False
        message.center_source = "aux_bbox"
        message.association_valid = False
        message.reject_reason = ""
        message.map_valid = True
        message.map_point = self.detection.map_point
        message.map_frame = self.detection.map_frame
        message.map_quality = self.detection.map_quality
        message.transform_age_sec = self.detection.transform_age_sec
        if self.consecutive_count >= confirm_frames:
            message.state = ST_CONFIRMED
        elif self.consecutive_count >= max(1, confirm_frames - 1):
            message.state = ST_OBSERVING
        else:
            message.state = ST_DETECTED
        message.observe_count = self.observe_count
        message.consecutive_observe_count = self.consecutive_count
        message.first_seen = self.first_seen
        message.last_seen = self.last_seen
        return message


class AuxCandidateMemory:
    def __init__(self):
        rospy.init_node("aux_candidate_memory")
        self._input_topic = rospy.get_param(
            "~input_topic", "/uav_vision/aux/detections_mapped")
        self._output_topic = rospy.get_param(
            "~output_topic", "/uav_vision/aux/targets")
        self._reset_service_name = rospy.get_param(
            "~reset_service", "/uav_vision/aux/reset_memory")
        self._confirm_frames = max(1, int(rospy.get_param("~confirm_frames", 2)))
        self._min_class_confidence = float(
            rospy.get_param("~min_class_confidence", 0.45))
        self._min_map_quality = float(rospy.get_param("~min_map_quality", 0.10))
        self._match_distance = float(rospy.get_param("~match_distance_m", 1.0))
        self._ttl = float(rospy.get_param("~candidate_ttl_sec", 5.0))
        self._records = {}
        self._next_id = 0
        self._publisher = rospy.Publisher(
            self._output_topic, TargetCandidateArray, queue_size=1)
        rospy.Subscriber(self._input_topic, TargetDetectionArray,
                         self._on_detections, queue_size=2)
        self._reset_service = rospy.Service(
            self._reset_service_name, Empty, self._on_reset)
        self._publish(rospy.Time.now())
        rospy.loginfo(
            "[AuxCandidateMemory] input=%s output=%s confirm=%d",
            self._input_topic, self._output_topic, self._confirm_frames)

    def _on_reset(self, _request):
        self._records.clear()
        self._next_id = 0
        self._publish(rospy.Time.now())
        return EmptyResponse()

    def _find_match(self, detection, matched_ids):
        candidates = []
        for candidate_id, record in self._records.items():
            if candidate_id in matched_ids or not record.matches(
                    detection, self._match_distance):
                continue
            dx = record.detection.map_point.x - detection.map_point.x
            dy = record.detection.map_point.y - detection.map_point.y
            candidates.append((math.hypot(dx, dy), candidate_id))
        return min(candidates)[1] if candidates else None

    def _on_detections(self, message):
        stamp = message.header.stamp
        if stamp.to_sec() <= 0.0:
            stamp = rospy.Time.now()
        matched_ids = set()
        for detection in sorted(
                message.detections,
                key=lambda item: float(item.class_confidence) *
                float(item.map_quality), reverse=True):
            if (not detection.map_valid or
                    detection.class_confidence < self._min_class_confidence or
                    detection.map_quality < self._min_map_quality):
                continue
            candidate_id = self._find_match(detection, matched_ids)
            if candidate_id is None:
                candidate_id = self._next_id
                self._next_id += 1
                self._records[candidate_id] = Record(
                    candidate_id, detection, stamp)
            else:
                self._records[candidate_id].update(detection, stamp)
            matched_ids.add(candidate_id)

        expired = []
        for candidate_id, record in self._records.items():
            if candidate_id not in matched_ids:
                record.missed()
            if self._ttl > 0.0 and (stamp - record.last_seen).to_sec() > self._ttl:
                expired.append(candidate_id)
        for candidate_id in expired:
            del self._records[candidate_id]
        self._publish(stamp)

    def _publish(self, stamp):
        array = TargetCandidateArray()
        array.header.stamp = stamp
        array.targets = [
            record.to_message(stamp, self._confirm_frames)
            for _, record in sorted(self._records.items())]
        self._publisher.publish(array)


if __name__ == "__main__":
    AuxCandidateMemory()
    rospy.spin()
