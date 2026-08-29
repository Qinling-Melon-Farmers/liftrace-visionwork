#!/usr/bin/env python3
"""Verify persistent map memory does not republish a stale selected target."""

import sys
import time

import rospy
from uav_vision.msg import (
    TargetCandidate, TargetCandidateArray, TargetDetection, TargetDetectionArray,
)


class FreshnessAssertion:
    def __init__(self):
        self.publisher = rospy.Publisher("/freshness_test/detections", TargetDetectionArray, queue_size=1)
        self.selected_times = []
        self.latest_targets = None
        self.last_detection_time = None
        rospy.Subscriber("/uav_vision/selected_target", TargetCandidate, self._selected, queue_size=10)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray, self._targets, queue_size=10)

    def _selected(self, _message):
        self.selected_times.append(rospy.Time.now())

    def _targets(self, message):
        self.latest_targets = message

    @staticmethod
    def _detection(stamp):
        detection = TargetDetection()
        detection.header.stamp = stamp
        detection.header.frame_id = "camera"
        detection.class_name = "tent"
        detection.class_confidence = 0.95
        detection.geometry_confidence = 0.9
        detection.geometry_verified = True
        detection.center_refined = True
        detection.center_source = "circle_geometry"
        detection.association_valid = True
        detection.reject_reason = ""
        detection.center_px.x = 320.0
        detection.center_px.y = 240.0
        detection.roi.x_offset = 280
        detection.roi.y_offset = 200
        detection.roi.width = 80
        detection.roi.height = 80
        detection.map_valid = True
        detection.map_frame = "world"
        detection.map_point.x = 1.0
        detection.map_point.y = 2.0
        detection.map_quality = 0.9
        return detection

    def run(self):
        deadline = time.monotonic() + 8.0
        while self.publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        for index in range(16):
            stamp = rospy.Time.now()
            message = TargetDetectionArray()
            message.header.stamp = stamp
            message.header.frame_id = "camera"
            if index < 5:
                message.detections = [self._detection(stamp)]
                self.last_detection_time = stamp
            self.publisher.publish(message)
            time.sleep(0.1)
        time.sleep(0.3)

        if self.last_detection_time is None or not self.selected_times:
            rospy.logerr("freshness assertion never observed a confirmed selection")
            return 6
        stale_deadline = self.last_detection_time + rospy.Duration(0.45)
        late_selections = [stamp for stamp in self.selected_times if stamp > stale_deadline]
        persisted_targets = [
            target for target in (self.latest_targets.targets
                                  if self.latest_targets is not None else [])
            if target.class_name == "tent" and target.state == 2 and
            target.map_frame == "world" and
            abs(target.map_point.x - 1.0) < 1.0e-6 and
            abs(target.map_point.y - 2.0) < 1.0e-6]
        memory_persisted = bool(persisted_targets)
        current_map_invalid = bool(persisted_targets) and all(
            not target.map_valid for target in persisted_targets)
        if late_selections or not memory_persisted or not current_map_invalid:
            rospy.logerr(
                "freshness assertion failed: late_selected=%d "
                "memory_persisted=%s current_map_invalid=%s",
                len(late_selections), memory_persisted, current_map_invalid,
            )
            return 6
        rospy.loginfo("V-ALG target-memory freshness PASS")
        return 0


if __name__ == "__main__":
    rospy.init_node("target_memory_freshness_assertion")
    sys.exit(FreshnessAssertion().run())
