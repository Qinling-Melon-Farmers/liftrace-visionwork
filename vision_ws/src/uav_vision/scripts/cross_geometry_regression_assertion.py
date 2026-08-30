#!/usr/bin/env python3
"""用可生成的正负图像验证红十字几何发布边界。"""
import threading
import time
import unittest

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

from uav_vision.msg import TargetDetectionArray


class CrossGeometryRegressionAssertion:
    def __init__(self):
        rospy.init_node("cross_geometry_regression_assertion")
        self._image_topic = rospy.get_param("~image_topic", "/cross_test/image")
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/cross_test/camera_info")
        self._timeout = float(rospy.get_param("~case_timeout", 3.0))
        self._condition = threading.Condition()
        self._stamp = None
        self._response = None
        self._image_pub = rospy.Publisher(
            self._image_topic, Image, queue_size=1)
        self._camera_pub = rospy.Publisher(
            self._camera_info_topic, CameraInfo, queue_size=1, latch=True)
        self._sub = rospy.Subscriber(
            "/uav_vision/detections", TargetDetectionArray,
            self._on_detections, queue_size=8)

    @staticmethod
    def _blank():
        return np.zeros((512, 640, 3), dtype=np.uint8)

    @staticmethod
    def _plus(image, center=(320, 256), outer=96, thickness=30):
        cx, cy = center
        half = outer // 2
        arm = thickness // 2
        cv2.rectangle(image, (cx - half, cy - arm),
                      (cx + half, cy + arm), (0, 0, 255), -1)
        cv2.rectangle(image, (cx - arm, cy - half),
                      (cx + arm, cy + half), (0, 0, 255), -1)

    def _cases(self):
        cases = []

        normal = self._blank()
        self._plus(normal)
        cases.append(("positive_standard", normal, True))

        high_altitude = self._blank()
        # 0.35 m 靶在约 3.6 m、fx~500 px 时的约 49 px 尺度。
        self._plus(high_altitude, outer=50, thickness=16)
        cases.append(("positive_3p6m_scale", high_altitude, True))

        square = self._blank()
        cv2.rectangle(square, (285, 221), (355, 291), (0, 0, 255), -1)
        cases.append(("negative_square", square, False))

        circle = self._blank()
        cv2.circle(circle, (320, 256), 38, (0, 0, 255), -1)
        cases.append(("negative_circle", circle, False))

        bar = self._blank()
        cv2.rectangle(bar, (260, 246), (380, 266), (0, 0, 255), -1)
        cases.append(("negative_bar", bar, False))

        tee = self._blank()
        cv2.rectangle(tee, (270, 215), (370, 239), (0, 0, 255), -1)
        cv2.rectangle(tee, (308, 215), (332, 306), (0, 0, 255), -1)
        cases.append(("negative_t", tee, False))

        ell = self._blank()
        cv2.rectangle(ell, (285, 205), (309, 305), (0, 0, 255), -1)
        cv2.rectangle(ell, (285, 281), (380, 305), (0, 0, 255), -1)
        cases.append(("negative_l", ell, False))

        diagonal = self._blank()
        cv2.line(diagonal, (275, 211), (365, 301), (0, 0, 255), 19)
        cv2.line(diagonal, (365, 211), (275, 301), (0, 0, 255), 19)
        cases.append(("negative_x", diagonal, False))

        broken = self._blank()
        cv2.rectangle(broken, (270, 246), (304, 266), (0, 0, 255), -1)
        cv2.rectangle(broken, (336, 246), (370, 266), (0, 0, 255), -1)
        cv2.rectangle(broken, (310, 206), (330, 240), (0, 0, 255), -1)
        cv2.rectangle(broken, (310, 272), (330, 306), (0, 0, 255), -1)
        cases.append(("negative_broken", broken, False))

        clipped = self._blank()
        self._plus(clipped, center=(12, 256), outer=80, thickness=26)
        cases.append(("negative_border_clipped", clipped, False))
        return cases

    def _publish_camera_info(self):
        msg = CameraInfo()
        msg.header.frame_id = "cross_test_camera"
        msg.width = 640
        msg.height = 512
        msg.distortion_model = "plumb_bob"
        msg.K = [500.0, 0.0, 320.0,
                 0.0, 500.0, 256.0,
                 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.P = [500.0, 0.0, 320.0, 0.0,
                 0.0, 500.0, 256.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.header.stamp = rospy.Time.now()
        self._camera_pub.publish(msg)

    def _on_detections(self, msg):
        with self._condition:
            if self._stamp is None or msg.header.stamp != self._stamp:
                return
            self._response = msg
            self._condition.notify_all()

    def _run_case(self, name, image, expected_positive):
        message = Image()
        message.header.frame_id = "cross_test_camera"
        message.height, message.width = image.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = image.tobytes()

        deadline = time.monotonic() + self._timeout
        response = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._condition:
                self._stamp = rospy.Time.now()
                self._response = None
                message.header.stamp = self._stamp
                self._image_pub.publish(message)
                self._condition.wait(timeout=0.15)
                response = self._response
            if response is not None:
                break
        if response is None:
            raise AssertionError("{}: detector response timeout".format(name))

        verified = [
            detection for detection in response.detections
            if detection.class_name == "red_cross" and
            detection.geometry_verified
        ]
        if not expected_positive:
            if verified:
                raise AssertionError(
                    "{}: negative published geometry_verified quality={:.3f}".format(
                        name, verified[0].geometry_confidence))
            rospy.loginfo("[CrossGeometryRegression] PASS %s rejected", name)
            return

        if len(verified) != 1:
            raise AssertionError(
                "{}: expected one verified detection, got {}".format(
                    name, len(verified)))
        detection = verified[0]
        if detection.center_source != "red_cross_geometry":
            raise AssertionError(
                "{}: center_source changed to {}".format(
                    name, detection.center_source))
        if not 0.70 <= detection.geometry_confidence <= 1.0:
            raise AssertionError(
                "{}: geometry quality {:.3f} outside [0.70,1]".format(
                    name, detection.geometry_confidence))
        rospy.loginfo(
            "[CrossGeometryRegression] PASS %s quality=%.3f",
            name, detection.geometry_confidence)

    def run(self):
        wait_deadline = time.monotonic() + self._timeout
        while (self._image_pub.get_num_connections() < 1 and
               not rospy.is_shutdown() and time.monotonic() < wait_deadline):
            self._publish_camera_info()
            rospy.sleep(0.05)
        if self._image_pub.get_num_connections() < 1:
            raise AssertionError("cross detector image subscriber unavailable")
        for _ in range(3):
            self._publish_camera_info()
            rospy.sleep(0.05)

        for name, image, expected_positive in self._cases():
            self._run_case(name, image, expected_positive)
        rospy.loginfo("[CrossGeometryRegression] all cases passed")


class CrossGeometryRegressionTest(unittest.TestCase):
    def test_generated_positive_and_negative_shapes(self):
        CrossGeometryRegressionAssertion().run()


if __name__ == "__main__":
    import rostest

    rostest.rosrun(
        "uav_vision", "cross_geometry_regression",
        CrossGeometryRegressionTest)
