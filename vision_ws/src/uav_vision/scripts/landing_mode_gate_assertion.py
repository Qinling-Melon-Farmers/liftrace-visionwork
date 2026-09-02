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
        self._detections_by_stamp = {}
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
            self._detections_by_stamp[message.header.stamp.to_nsec()] = [
                detection.class_name for detection in message.detections]

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

    @staticmethod
    def _image_data(pattern):
        width = 320
        height = 240
        if pattern == "background":
            return bytes([127]) * (width * height * 3)

        pixels = bytearray([255]) * (width * height * 3)

        def paint_black(x_value, y_value):
            if not (0 <= x_value < width and 0 <= y_value < height):
                return
            offset = (y_value * width + x_value) * 3
            pixels[offset:offset + 3] = b"\x00\x00\x00"

        center_x = 160
        center_y = 120
        if pattern in ("landing_h", "ring_only", "partial_h",
                       "broken_ring_h"):
            for y_value in range(center_y - 72, center_y + 73):
                for x_value in range(center_x - 72, center_x + 73):
                    dx = x_value - center_x
                    dy = y_value - center_y
                    radius_squared = dx * dx + dy * dy
                    in_ring = 60 * 60 <= radius_squared <= 70 * 70
                    if in_ring and (pattern != "broken_ring_h" or dy <= 0):
                        paint_black(x_value, y_value)

        def paint_rectangle(x_min, y_min, x_max, y_max):
            for y_value in range(y_min, y_max):
                for x_value in range(x_min, x_max):
                    paint_black(x_value, y_value)

        if pattern in ("landing_h", "partial_h", "broken_ring_h"):
            paint_rectangle(130, 82, 142, 158)
            paint_rectangle(178, 82, 190, 158)
        if pattern in ("landing_h", "broken_ring_h"):
            paint_rectangle(130, 114, 190, 126)

        if pattern not in ("landing_h", "ring_only", "partial_h",
                           "broken_ring_h"):
            raise ValueError("unknown landing test pattern: {}".format(pattern))
        return bytes(pixels)

    def _publish_image(self, pattern="background"):
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
        message.data = self._image_data(pattern)
        self._image_pub.publish(message)
        return self._sequence, message.header.stamp.to_nsec()

    def _wait_for_stamp(self, sequence, stamp_key, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if stamp_key in self._detections_by_stamp:
                    return list(self._detections_by_stamp[stamp_key])
            rospy.sleep(0.02)
        raise AssertionError(
            "landing detector did not publish heartbeat for seq={} stamp={}".format(
                sequence, stamp_key))

    def _publish_stage(self, mode, frame_count=8):
        self._mode_pub.publish(String(data=mode))
        rospy.sleep(0.15)
        for _ in range(frame_count):
            self._publish_camera_info()
            self._publish_image()
            rospy.sleep(0.05)
        rospy.sleep(0.2)

    def _assert_active_pattern(self, pattern, expect_detection, repeats=3):
        for _ in range(repeats):
            self._publish_camera_info()
            sequence, stamp_key = self._publish_image(pattern)
            classes = self._wait_for_stamp(sequence, stamp_key)
            detected = "landing_pad" in classes
            if detected != expect_detection:
                raise AssertionError(
                    "pattern={} seq={} expected landing_pad={} got {}".format(
                        pattern, sequence, expect_detection, classes))

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

        # The operational H contract requires both the outer ring and the
        # internal concave H.  Background, a plain black ring, a ring with two
        # uncoupled bars, and a half-ring remnant must remain negative even
        # while landing mode is active.
        self._assert_active_pattern("landing_h", True)
        self._assert_active_pattern("background", False)
        self._assert_active_pattern("ring_only", False)
        self._assert_active_pattern("partial_h", False)
        self._assert_active_pattern("broken_ring_h", False)

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
