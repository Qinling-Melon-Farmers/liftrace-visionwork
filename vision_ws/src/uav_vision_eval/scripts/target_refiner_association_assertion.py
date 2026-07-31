#!/usr/bin/env python3
"""Exercise global one-to-one class-to-ring association."""

import sys
import time

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest
from uav_vision.msg import TargetDetection, TargetDetectionArray


class AssociationAssertion:
    def __init__(self):
        self.publisher = rospy.Publisher("/association_test/input", TargetDetectionArray, queue_size=1)
        self.passed = False
        rospy.Subscriber("/association_test/output", TargetDetectionArray, self._result, queue_size=1)

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

    def _result(self, message):
        targets = {item.class_name: item for item in message.detections}
        tent = targets.get("tent")
        panzer = targets.get("panzer")
        if tent is None or panzer is None:
            return
        self.passed = (
            tent.center_refined and panzer.center_refined and
            abs(tent.center_px.x - 60.0) < 0.01 and
            abs(panzer.center_px.x - 135.0) < 0.01
        )

    def run(self):
        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            stamp = rospy.Time.now()
            message = TargetDetectionArray()
            message.header.stamp = stamp
            message.detections = [
                self._detection(stamp, "tent", 100.0),
                self._detection(stamp, "panzer", 140.0),
                self._detection(stamp, "circle", 135.0, 80.0),
                self._detection(stamp, "circle", 60.0, 80.0),
            ]
            self.publisher.publish(message)
            time.sleep(0.1)
            if self.passed:
                rospy.loginfo("V-ALG global ring association PASS")
                return 0
        rospy.logerr("global ring association assertion failed")
        return 7


if __name__ == "__main__":
    rospy.init_node("target_refiner_association_assertion")
    sys.exit(AssociationAssertion().run())
