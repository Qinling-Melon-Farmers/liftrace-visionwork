#!/usr/bin/env python3
"""为斜下粗定位 L0 回归发布确定性 CameraInfo、TF、深度和检测框。"""

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest

from uav_vision.msg import TargetDetection, TargetDetectionArray


class MockInputs:
    def __init__(self):
        self._camera_info_pub = rospy.Publisher(
            "/mock/aux/camera_info", CameraInfo, queue_size=1)
        self._depth_pub = rospy.Publisher(
            "/mock/aux/depth", Image, queue_size=1)
        self._detections_pub = rospy.Publisher(
            "/mock/aux/detections", TargetDetectionArray, queue_size=1)
        self._tf = tf2_ros.StaticTransformBroadcaster()
        self._publish_transform()
        self._timer = rospy.Timer(rospy.Duration(0.10), self._publish)

    def _publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "aux_camera_optical_frame"
        transform.transform.translation.z = 2.0
        # 光学 z 轴朝地面；光学 x 轴与 world x 同向。
        transform.transform.rotation.x = 1.0
        transform.transform.rotation.w = 0.0
        self._tf.sendTransform(transform)

    @staticmethod
    def _detection(header, class_name, confidence, center_x, center_y, size):
        detection = TargetDetection()
        detection.header = header
        detection.class_name = class_name
        detection.class_confidence = confidence
        detection.geometry_confidence = confidence
        detection.geometry_verified = False
        detection.roi = RegionOfInterest(
            x_offset=int(center_x - size / 2),
            y_offset=int(center_y - size / 2),
            width=size, height=size, do_rectify=False)
        detection.center_px.x = center_x
        detection.center_px.y = center_y
        detection.center_source = "bbox"
        detection.center_refined = False
        detection.association_valid = False
        detection.reject_reason = "geometry_not_refined"
        detection.transform_age_sec = -1.0
        return detection

    def _publish(self, _event):
        stamp = rospy.Time.now()
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = "aux_camera_optical_frame"
        camera_info.width = 640
        camera_info.height = 480
        camera_info.K = [400.0, 0.0, 320.0,
                         0.0, 400.0, 240.0,
                         0.0, 0.0, 1.0]
        camera_info.P = [400.0, 0.0, 320.0, 0.0,
                         0.0, 400.0, 240.0, 0.0,
                         0.0, 0.0, 1.0, 0.0]
        self._camera_info_pub.publish(camera_info)

        depth = np.zeros((480, 640), dtype=np.float32)
        depth[220:260, 300:340] = 2.0
        depth_message = Image()
        depth_message.header = camera_info.header
        depth_message.width = 640
        depth_message.height = 480
        depth_message.encoding = "32FC1"
        depth_message.is_bigendian = False
        depth_message.step = 640 * 4
        depth_message.data = depth.tobytes()
        self._depth_pub.publish(depth_message)

        array = TargetDetectionArray()
        array.header = camera_info.header
        array.source = "mock_aux_detector"
        array.completed_sources = ["mock_aux_detector"]
        array.detections = [
            self._detection(camera_info.header, "tent", 0.90, 320.0, 240.0, 80),
            # 此 ROI 无有效深度，depth 模式必须安全回退到单目。
            self._detection(camera_info.header, "red_cross", 0.85, 560.0, 240.0, 60),
        ]
        self._detections_pub.publish(array)


if __name__ == "__main__":
    rospy.init_node("aux_projection_mock_inputs")
    MockInputs()
    rospy.spin()
