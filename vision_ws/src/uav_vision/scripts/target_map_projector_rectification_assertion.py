#!/usr/bin/env python3
"""Regression check: raw-image centers are rectified before ray projection."""

import sys

import rospy
from geometry_msgs.msg import TransformStamped

from target_map_projector import TargetMapProjector
from uav_vision.msg import TargetDetection


class _CameraModel:
    def __init__(self):
        self.rectified = None
        self.projected = None

    def rectifyPoint(self, pixel):
        self.rectified = tuple(pixel)
        return pixel[0] + 4.0, pixel[1] - 3.0

    def projectPixelTo3dRay(self, pixel):
        self.projected = tuple(pixel)
        return 0.0, 0.0, -1.0


class _TfBuffer:
    @staticmethod
    def lookup_transform(_target, _source, stamp, _timeout):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera"
        transform.transform.translation.z = 1.0
        transform.transform.rotation.w = 1.0
        return transform


def _project(rectify_input_pixels, camera_has_distortion=True):
    projector = TargetMapProjector.__new__(TargetMapProjector)
    projector._camera_ready = True
    projector._camera_model = _CameraModel()
    projector._tf_buffer = _TfBuffer()
    projector._map_frame = "map"
    projector._ground_z = 0.0
    projector._ray_epsilon = 1e-5
    projector._tf_timeout = 0.05
    projector._allow_latest_tf_fallback = False
    projector._max_latest_tf_age = 0.1
    projector._rectify_input_pixels = rectify_input_pixels
    projector._camera_has_distortion = camera_has_distortion

    detection = TargetDetection()
    detection.center_refined = True
    detection.association_valid = True
    detection.geometry_confidence = 0.9
    detection.center_px.x = 800.0
    detection.center_px.y = 420.0
    ok, reason = projector._project(
        detection, rospy.Time.from_sec(5.0), "camera")
    assert ok, reason
    return projector._camera_model


def main():
    raw_model = _project(True)
    assert raw_model.rectified == (800.0, 420.0)
    assert raw_model.projected == (804.0, 417.0)

    rectified_model = _project(False)
    assert rectified_model.rectified is None
    assert rectified_model.projected == (800.0, 420.0)

    zero_distortion_model = _project(True, camera_has_distortion=False)
    assert zero_distortion_model.rectified is None
    assert zero_distortion_model.projected == (800.0, 420.0)
    print("target_map_projector rectification PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
