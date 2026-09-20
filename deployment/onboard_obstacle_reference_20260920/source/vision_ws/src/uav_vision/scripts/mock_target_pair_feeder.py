#!/usr/bin/env python3
"""发布一个标准靶及其外围蓝色圆环观测。"""
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest

from uav_vision.msg import TargetDetection, TargetDetectionArray


class MockTargetPairFeeder:
    def __init__(self):
        rospy.init_node("mock_target_pair_feeder")
        self._topic = rospy.get_param("~topic", "/uav_vision/detections")
        self._class_name = rospy.get_param("~class_name", "panzer")
        self._frame_id = rospy.get_param("~frame_id", "camera")
        self._cx = float(rospy.get_param("~center_x", 640.0))
        self._cy = float(rospy.get_param("~center_y", 480.0))
        self._radius = float(rospy.get_param("~radius_px", 180.0))
        self._rate = float(rospy.get_param("~rate", 10.0))
        self._emit_unmatched = bool(
            rospy.get_param("~emit_unmatched_standard", False))
        self._pub = rospy.Publisher(self._topic, TargetDetectionArray, queue_size=1)

    def _make(self, stamp, class_name, confidence, geometry, refined,
              roi, center):
        det = TargetDetection()
        det.header.stamp = stamp
        det.header.frame_id = self._frame_id
        det.class_name = class_name
        det.class_confidence = confidence
        det.geometry_confidence = geometry
        det.geometry_verified = refined
        det.center_refined = refined
        det.center_source = "circle_geometry" if class_name == "circle" else "bbox"
        det.association_valid = refined
        det.reject_reason = "" if refined else "geometry_not_refined"
        det.transform_age_sec = -1.0
        det.roi = roi
        det.center_px = center
        return det

    def spin(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            target_roi = RegionOfInterest(x_offset=500, y_offset=340,
                                          width=280, height=280)
            ring_roi = RegionOfInterest(x_offset=420, y_offset=260,
                                        width=440, height=440)
            target = self._make(
                stamp, self._class_name, 0.92, 0.92, False,
                target_roi, Point(self._cx, self._cy, 0.0))
            ring = self._make(
                stamp, "circle", 0.95, 0.95, True,
                ring_roi, Point(self._cx, self._cy, self._radius))
            arr = TargetDetectionArray()
            arr.header.stamp = stamp
            arr.header.frame_id = self._frame_id
            arr.detections = [target, ring]
            if self._emit_unmatched:
                unmatched_roi = RegionOfInterest(
                    x_offset=40, y_offset=40, width=120, height=120)
                unmatched = self._make(
                    stamp, "tent", 0.88, 0.88, False,
                    unmatched_roi, Point(100.0, 100.0, 0.0))
                arr.detections.append(unmatched)
            self._pub.publish(arr)
            rate.sleep()


if __name__ == "__main__":
    MockTargetPairFeeder().spin()
