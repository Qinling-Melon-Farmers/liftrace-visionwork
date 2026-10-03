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
        self._debug_response = None
        self._image_pub = rospy.Publisher(
            self._image_topic, Image, queue_size=1)
        self._camera_pub = rospy.Publisher(
            self._camera_info_topic, CameraInfo, queue_size=1, latch=True)
        self._sub = rospy.Subscriber(
            "/uav_vision/detections", TargetDetectionArray,
            self._on_detections, queue_size=8)
        self._debug_sub = rospy.Subscriber(
            "/uav_vision/cross_debug", Image,
            self._on_debug, queue_size=2)

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

    @classmethod
    def _rotated_plus(cls, angle_deg, outer, thickness):
        image = cls._blank()
        cls._plus(image, outer=outer, thickness=thickness)
        transform = cv2.getRotationMatrix2D((320, 256), angle_deg, 1.0)
        return cv2.warpAffine(
            image, transform, (image.shape[1], image.shape[0]),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0))

    def _cases(self):
        cases = []

        # 覆盖相机/目标 yaw 的离散角度，并在每个角度同时覆盖标准尺度和
        # 0.35 m 靶在约 3.6 m、fx~500 px 时的约 49 px 小尺度。
        for angle_deg in (0, 15, 30, 45, 60, 75, 90):
            cases.append((
                "positive_yaw_{:02d}_scale96".format(angle_deg),
                self._rotated_plus(angle_deg, outer=96, thickness=30),
                True))
            cases.append((
                "positive_yaw_{:02d}_scale50".format(angle_deg),
                self._rotated_plus(angle_deg, outer=50, thickness=16),
                True))

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

        thin_x = self._blank()
        # 45° 等比正十字本质上就是 X，不能再将它当负样本。这个负样本用
        # 约 8% 的细臂宽度，与正靶约 31% 的等比臂宽在几何上可分。
        cv2.line(thin_x, (275, 211), (365, 301), (0, 0, 255), 8)
        cv2.line(thin_x, (365, 211), (275, 301), (0, 0, 255), 8)
        cases.append(("negative_thin_x", thin_x, False))

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

    def _on_debug(self, msg):
        with self._condition:
            if self._stamp is None or msg.header.stamp != self._stamp:
                return
            self._debug_response = msg
            self._condition.notify_all()

    def _assert_camera_info_debug_reference(self):
        image = self._blank()
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
                self._debug_response = None
                message.header.stamp = self._stamp
                self._image_pub.publish(message)
                self._condition.wait_for(
                    lambda: self._debug_response is not None,
                    timeout=0.15)
                response = self._debug_response
            if response is not None:
                break
        if response is None:
            raise AssertionError("cross debug response timeout")
        pixels = np.frombuffer(response.data, dtype=np.uint8)
        pixels = pixels.reshape(response.height, response.step)
        pixels = pixels[:, :response.width * 3].reshape(
            response.height, response.width, 3)
        # CameraInfo K publishes principal point (320, 256).  The binary debug
        # image draws its alignment reference as a white filled circle.
        if not np.all(pixels[256, 320] >= 250):
            raise AssertionError(
                "cross debug reference did not follow CameraInfo principal point")

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

        self._assert_camera_info_debug_reference()

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
