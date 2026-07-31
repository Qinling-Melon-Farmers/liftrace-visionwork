#!/usr/bin/env python3
"""验证多圆环输出及其到原始图像坐标的恢复结果。"""
import rospy
from uav_vision.msg import TargetDetectionArray


class CircleGeometryAssertion:
    def __init__(self):
        rospy.init_node("circle_geometry_assertion")
        self.exit_code = 1
        self._timeout = float(rospy.get_param("~timeout", 8.0))
        self._sub = rospy.Subscriber("/uav_vision/detections",
                                     TargetDetectionArray,
                                     self._on_detections, queue_size=4)
        rospy.Timer(rospy.Duration(self._timeout), self._on_timeout, oneshot=True)

    def _on_detections(self, msg):
        circles = sorted([d for d in msg.detections if d.class_name == "circle"],
                         key=lambda d: d.center_px.x)
        if len(circles) < 2:
            return
        expected = [(320.0, 300.0), (960.0, 700.0)]
        for det, (ex, ey) in zip(circles[:2], expected):
            if abs(det.center_px.x - ex) > 12.0 or abs(det.center_px.y - ey) > 12.0:
                rospy.logerr("[CircleGeometryAssertion] bad center got=(%.1f,%.1f) expected=(%.1f,%.1f)",
                             det.center_px.x, det.center_px.y, ex, ey)
                self.exit_code = 6
                rospy.signal_shutdown("circle center regression failed")
                return
            if not det.center_refined or det.center_px.z <= 0.0:
                rospy.logerr("[CircleGeometryAssertion] missing refined center/radius")
                self.exit_code = 6
                rospy.signal_shutdown("circle metadata regression failed")
                return
        rospy.loginfo("[CircleGeometryAssertion] success circles=%d", len(circles))
        self.exit_code = 0
        rospy.signal_shutdown("circle geometry regression passed")

    def _on_timeout(self, _event):
        rospy.logerr("[CircleGeometryAssertion] timeout waiting for two circles")
        self.exit_code = 6
        rospy.signal_shutdown("circle geometry regression timeout")


if __name__ == "__main__":
    import sys
    assertion = CircleGeometryAssertion()
    rospy.spin()
    sys.exit(assertion.exit_code)
