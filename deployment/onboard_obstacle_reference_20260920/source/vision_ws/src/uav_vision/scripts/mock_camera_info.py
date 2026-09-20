#!/usr/bin/env python3
"""为视觉投影测试发布确定性的 CameraInfo。"""
import rospy
from sensor_msgs.msg import CameraInfo


class MockCameraInfo:
    def __init__(self):
        rospy.init_node("mock_camera_info")
        self._topic = rospy.get_param("~topic", "/camera/camera_info")
        self._frame_id = rospy.get_param("~frame_id", "camera")
        self._width = int(rospy.get_param("~width", 1280))
        self._height = int(rospy.get_param("~height", 1024))
        self._fx = float(rospy.get_param("~fx", 500.0))
        self._fy = float(rospy.get_param("~fy", 500.0))
        self._cx = float(rospy.get_param("~cx", 640.0))
        self._cy = float(rospy.get_param("~cy", 480.0))
        self._pub = rospy.Publisher(self._topic, CameraInfo, queue_size=1,
                                    latch=True)

    def spin(self):
        msg = CameraInfo()
        msg.header.frame_id = self._frame_id
        msg.width = self._width
        msg.height = self._height
        msg.distortion_model = "plumb_bob"
        msg.K = [self._fx, 0.0, self._cx,
                 0.0, self._fy, self._cy,
                 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.P = [self._fx, 0.0, self._cx, 0.0,
                 0.0, self._fy, self._cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            self._pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    MockCameraInfo().spin()
