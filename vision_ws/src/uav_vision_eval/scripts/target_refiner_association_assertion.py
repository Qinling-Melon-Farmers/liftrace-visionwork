#!/usr/bin/env python3
"""Exercise profile-aware global one-to-one class-to-ring association."""

import sys
import time

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest
from uav_vision.msg import TargetDetection, TargetDetectionArray


class AssociationAssertion:
    def __init__(self):
        self.r2026_publisher = rospy.Publisher(
            "/association_test/r2026/input", TargetDetectionArray, queue_size=1)
        self.full_publisher = rospy.Publisher(
            "/association_test/full/input", TargetDetectionArray, queue_size=1)
        self.r2026_passed = False
        self.full_passed = False
        rospy.Subscriber(
            "/association_test/r2026/output", TargetDetectionArray,
            self._r2026_result, queue_size=1)
        rospy.Subscriber(
            "/association_test/full/output", TargetDetectionArray,
            self._full_result, queue_size=1)

    @staticmethod
    def _detection(stamp, class_name, x, radius=0.0):
        detection = TargetDetection()
        detection.header.stamp = stamp
        detection.class_name = class_name
        detection.class_confidence = 0.95
        detection.geometry_confidence = 0.95
        detection.geometry_verified = class_name == "circle"
        detection.center_refined = class_name == "circle"
        detection.center_px = Point(x, 240.0, radius)
        detection.roi = RegionOfInterest(
            x_offset=max(0, int(x - 100)), y_offset=140, width=200, height=200)
        return detection

    def _r2026_result(self, message):
        targets = {item.class_name: item for item in message.detections}
        tent = targets.get("tent")
        panzer = targets.get("panzer")
        tank = targets.get("tank")
        pillbox = targets.get("pillbox")
        red_cross = targets.get("red_cross")
        if any(item is None for item in (
                tent, panzer, tank, pillbox, red_cross)):
            return
        self.r2026_passed = (
            tent.center_refined and panzer.center_refined and
            abs(tent.center_px.x - 60.0) < 0.01 and
            abs(panzer.center_px.x - 135.0) < 0.01 and
            pillbox.center_refined and
            abs(pillbox.center_px.x - 420.0) < 0.01 and
            not tank.center_refined and
            not tank.association_valid and
            tank.reject_reason == "class_profile_disallowed" and
            red_cross.center_refined and
            red_cross.association_valid and
            red_cross.reject_reason == "red_cross_passthrough" and
            abs(red_cross.center_px.x - 777.0) < 0.01
        )

    def _full_result(self, message):
        targets = {item.class_name: item for item in message.detections}
        tank = targets.get("tank")
        self.full_passed = bool(
            tank is not None and tank.center_refined and
            tank.association_valid and not tank.reject_reason and
            abs(tank.center_px.x - 520.0) < 0.01)

    def _r2026_message(self, stamp):
        message = TargetDetectionArray()
        message.header.stamp = stamp
        red_cross = self._detection(stamp, "red_cross", 777.0)
        red_cross.center_refined = True
        red_cross.association_valid = True
        red_cross.reject_reason = "red_cross_passthrough"
        message.detections = [
            self._detection(stamp, "tent", 100.0),
            self._detection(stamp, "panzer", 140.0),
            # Put the disallowed class first so the old global greedy matcher
            # would steal the shared ring from pillbox.
            self._detection(stamp, "tank", 420.0),
            self._detection(stamp, "pillbox", 420.0),
            self._detection(stamp, "circle", 135.0, 80.0),
            self._detection(stamp, "circle", 60.0, 80.0),
            self._detection(stamp, "circle", 420.0, 80.0),
            red_cross,
        ]
        return message

    def _full_message(self, stamp):
        message = TargetDetectionArray()
        message.header.stamp = stamp
        message.detections = [
            self._detection(stamp, "tank", 520.0),
            self._detection(stamp, "circle", 520.0, 80.0),
        ]
        return message

    def run(self):
        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            stamp = rospy.Time.now()
            self.r2026_publisher.publish(self._r2026_message(stamp))
            self.full_publisher.publish(self._full_message(stamp))
            time.sleep(0.1)
            if self.r2026_passed and self.full_passed:
                rospy.loginfo(
                    "V-ALG global ring association PASS (profile-aware r2026/full)")
                return 0
        rospy.logerr(
            "profile-aware global ring association assertion failed "
            "r2026=%s full=%s", self.r2026_passed, self.full_passed)
        return 7


if __name__ == "__main__":
    rospy.init_node("target_refiner_association_assertion")
    sys.exit(AssociationAssertion().run())
