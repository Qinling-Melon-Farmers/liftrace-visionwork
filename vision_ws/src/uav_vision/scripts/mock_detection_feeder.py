#!/usr/bin/env python3
"""发布固定检测结果，用于验证 target_memory / drop_aligner / patrol_control 接线。"""
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest

from uav_vision.msg import TargetDetection, TargetDetectionArray


class MockDetectionFeeder:
    def __init__(self):
        rospy.init_node("mock_detection_feeder")
        self._topic = rospy.get_param("~topic", "/uav_vision/detections")
        self._frame_id = rospy.get_param("~frame_id", "camera")
        self._class_name = rospy.get_param("~class_name", "red_cross")
        self._class_conf = rospy.get_param("~class_confidence", 0.95)
        self._geom_conf = rospy.get_param("~geometry_confidence", 0.95)
        self._center_x = rospy.get_param("~center_x", 640.0)
        self._center_y = rospy.get_param("~center_y", 480.0)
        self._radius_px = rospy.get_param("~radius_px", 120.0)
        self._roi_x = rospy.get_param("~roi_x", 560)
        self._roi_y = rospy.get_param("~roi_y", 400)
        self._roi_w = rospy.get_param("~roi_w", 160)
        self._roi_h = rospy.get_param("~roi_h", 160)
        self._rate = rospy.get_param("~rate", 10.0)
        self._publisher = rospy.Publisher(self._topic, TargetDetectionArray, queue_size=1)

        rospy.loginfo("[MockDetectionFeeder] topic=%s class=%s center=(%.1f, %.1f) rate=%.1f",
                      self._topic, self._class_name, self._center_x, self._center_y, self._rate)

    def spin(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            arr = TargetDetectionArray()
            arr.header.stamp = now
            arr.header.frame_id = self._frame_id

            det = TargetDetection()
            det.header = arr.header
            det.class_name = self._class_name
            det.class_confidence = self._class_conf
            det.geometry_confidence = self._geom_conf
            det.geometry_verified = True
            det.center_refined = True
            det.roi = RegionOfInterest(
                x_offset=int(self._roi_x),
                y_offset=int(self._roi_y),
                width=int(self._roi_w),
                height=int(self._roi_h),
                do_rectify=False,
            )
            det.center_px = Point(x=self._center_x, y=self._center_y, z=self._radius_px)
            arr.detections.append(det)

            self._publisher.publish(arr)
            rate.sleep()


def main():
    MockDetectionFeeder().spin()


if __name__ == "__main__":
    main()
