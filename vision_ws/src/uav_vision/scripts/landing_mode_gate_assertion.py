#!/usr/bin/env python3
"""Verify that landing H processing only runs in landing align mode."""

import threading
import time
import unittest

import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from uav_vision.msg import TargetDetectionArray


class LandingModeGateAssertion:
    def __init__(self):
        rospy.init_node("landing_mode_gate_assertion")
        self._lock = threading.Lock()
        self._debug_count = 0
        self._heartbeat_count = 0
        self._sequence = 0
        self._image_pub = rospy.Publisher(
            "/landing_gate/image", Image, queue_size=1)
        self._camera_pub = rospy.Publisher(
            "/landing_gate/camera_info", CameraInfo, queue_size=1,
            latch=True)
        self._mode_pub = rospy.Publisher(
            "/landing_gate/align_mode", String, queue_size=1, latch=True)
        rospy.Subscriber(
            "/landing_gate/debug", Image, self._on_debug, queue_size=10)
        rospy.Subscriber(
            "/uav_vision/detections", TargetDetectionArray,
            self._on_detections, queue_size=20)

    def _on_debug(self, _message):
        with self._lock:
            self._debug_count += 1

    def _on_detections(self, message):
        if message.source != "landing_detector":
            return
        with self._lock:
            self._heartbeat_count += 1

    def _counts(self):
        with self._lock:
            return self._debug_count, self._heartbeat_count

    def _publish_camera_info(self):
        message = CameraInfo()
        message.header.frame_id = "landing_gate_camera"
        message.width = 320
        message.height = 240
        message.K = [230.0, 0.0, 160.0,
                     0.0, 230.0, 120.0,
                     0.0, 0.0, 1.0]
        message.P = [230.0, 0.0, 160.0, 0.0,
                     0.0, 230.0, 120.0, 0.0,
                     0.0, 0.0, 1.0, 0.0]
        self._camera_pub.publish(message)

    def _publish_image(self):
        self._sequence += 1
        message = Image()
        message.header.seq = self._sequence
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "landing_gate_camera"
        message.width = 320
        message.height = 240
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = 320 * 3
        message.data = bytes([127]) * (message.step * message.height)
        self._image_pub.publish(message)

    def _publish_stage(self, mode, frame_count=8):
        self._mode_pub.publish(String(data=mode))
        rospy.sleep(0.15)
        for _ in range(frame_count):
            self._publish_camera_info()
            self._publish_image()
            rospy.sleep(0.05)
        rospy.sleep(0.2)

    def run(self):
        deadline = time.monotonic() + 4.0
        while (self._image_pub.get_num_connections() == 0 or
               self._camera_pub.get_num_connections() == 0 or
               self._mode_pub.get_num_connections() == 0):
            if time.monotonic() >= deadline:
                raise RuntimeError("landing gate publishers did not connect")
            rospy.sleep(0.05)

        self._publish_stage("disabled")
        disabled_debug, disabled_heartbeat = self._counts()
        if disabled_debug != 0 or disabled_heartbeat != 0:
            raise AssertionError(
                "disabled mode must be silent and skip debug processing")

        self._publish_stage("landing")
        landing_debug, landing_heartbeat = self._counts()
        if (landing_debug <= disabled_debug or
                landing_heartbeat <= disabled_heartbeat):
            raise AssertionError("landing mode did not activate processing")

        self._mode_pub.publish(String(data="drop_circle"))
        rospy.sleep(0.2)
        gated_debug, gated_heartbeat = self._counts()
        self._publish_stage("drop_circle")
        final_debug, final_heartbeat = self._counts()
        if (final_debug != gated_debug or
                final_heartbeat != gated_heartbeat):
            raise AssertionError(
                "non-landing mode did not restore the silent gate")
        rospy.loginfo("V-DEPLOY landing mode processing gate PASS")


class LandingModeGateTest(unittest.TestCase):
    def test_processing_is_landing_mode_gated(self):
        LandingModeGateAssertion().run()


if __name__ == "__main__":
    import rostest

    rostest.rosrun(
        "uav_vision", "landing_mode_gate", LandingModeGateTest)
