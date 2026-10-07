#!/usr/bin/env python3
"""把精修后的图像中心投影到可配置的地面平面。

本节点严格属于视觉观测链：只发布地图坐标和有效性，绝不控制飞机。相机内参只来自
sensor_msgs/CameraInfo，位姿只使用源图像时间戳对应的 TF。
"""
import copy
import math

import rospy
import tf2_geometry_msgs  # noqa: F401 - 注册 geometry 消息的 TF 变换
import tf2_ros
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

from uav_vision.msg import TargetDetectionArray


class TargetMapProjector:
    def __init__(self):
        rospy.init_node("target_map_projector")
        self._input_topic = rospy.get_param(
            "~input_topic", "/uav_vision/detections_refined")
        self._output_topic = rospy.get_param(
            "~output_topic", "/uav_vision/detections_mapped")
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/camera_info")
        self._map_frame = rospy.get_param("~map_frame", "map")
        self._ground_z = float(rospy.get_param("~ground_z", 0.0))
        self._ray_epsilon = float(rospy.get_param("~ray_z_epsilon", 1e-5))
        self._tf_timeout = float(rospy.get_param("~tf_timeout", 0.05))
        self._allow_latest_tf_fallback = bool(
            rospy.get_param("~allow_latest_tf_fallback", False))
        self._max_latest_tf_age = float(
            rospy.get_param("~max_latest_tf_age_sec", 0.10))
        # projectPixelTo3dRay expects rectified pixel coordinates.  The
        # project camera and board camera publish image_raw, so their refined
        # centers must be rectified with the matching CameraInfo first.
        self._rectify_input_pixels = bool(
            rospy.get_param("~rectify_input_pixels", True))

        self._coarse_enabled = bool(rospy.get_param("~coarse_navigation_enabled", False))
        self._coarse_min_confidence = float(rospy.get_param("~coarse_min_confidence", 0.60))
        if not math.isfinite(self._coarse_min_confidence) or not 0.0 <= self._coarse_min_confidence <= 1.0:
            raise ValueError("invalid coarse_min_confidence")
        self._coarse_classes = frozenset(rospy.get_param(
            "~coarse_classes", ["bridge", "panzer", "red_cross", "pillbox", "tent"]))
        self._camera_model = PinholeCameraModel()
        self._camera_ready = False
        self._camera_has_distortion = False
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._pub = rospy.Publisher(self._output_topic,
                                    TargetDetectionArray, queue_size=2)
        if self._coarse_enabled:
            self._coarse_pub = rospy.Publisher(
                rospy.get_param("~coarse_output_topic", "/uav_vision/navigation_hints"),
                TargetDetectionArray, queue_size=1)
            rospy.Subscriber(rospy.get_param("~coarse_input_topic", "/uav_vision/detections"),
                             TargetDetectionArray, self._on_coarse, queue_size=1)
        rospy.Subscriber(self._camera_info_topic, CameraInfo,
                         self._on_camera_info, queue_size=1)
        rospy.Subscriber(self._input_topic, TargetDetectionArray,
                         self._on_detections, queue_size=4)
        rospy.loginfo("[TargetMapProjector] ready input=%s output=%s map=%s ground_z=%.3f",
                      self._input_topic, self._output_topic,
                      self._map_frame, self._ground_z)

    def _on_camera_info(self, msg):
        self._camera_model.fromCameraInfo(msg)
        self._camera_has_distortion = any(
            abs(float(value)) > 1.0e-12 for value in msg.D)
        self._camera_ready = True

    def _invalidate(self, det, reason):
        det.map_valid = False
        det.map_point = Point()
        det.map_frame = self._map_frame
        det.map_quality = 0.0
        det.transform_age_sec = -1.0
        if reason:
            det.reject_reason = reason
            rospy.logdebug_throttle(5.0, "[TargetMapProjector] %s", reason)

    def _project(self, det, stamp, source_frame, require_refined=True):
        if not self._camera_ready:
            return False, "camera_info_unavailable"
        if require_refined and not det.center_refined:
            return False, det.reject_reason or "center_not_refined"
        if require_refined and not det.association_valid:
            return False, det.reject_reason or "association_invalid"
        if not source_frame:
            return False, "image_frame_empty"

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, source_frame, stamp,
                rospy.Duration(self._tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            if not self._allow_latest_tf_fallback:
                rospy.logdebug_throttle(
                    5.0, "[TargetMapProjector] exact TF unavailable: %s", exc)
                return False, "tf_unavailable"
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._map_frame, source_frame, rospy.Time(0),
                    rospy.Duration(self._tf_timeout))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as fallback_exc:
                rospy.logdebug_throttle(
                    5.0, "[TargetMapProjector] exact/latest TF unavailable: %s; %s",
                    exc, fallback_exc)
                return False, "tf_unavailable"
            transform_stamp = transform.header.stamp
            if stamp.to_sec() > 0.0 and transform_stamp.to_sec() > 0.0:
                transform_age = abs((stamp - transform_stamp).to_sec())
                if transform_age > self._max_latest_tf_age:
                    rospy.logdebug_throttle(
                        5.0, "[TargetMapProjector] latest TF age %.3fs exceeds %.3fs",
                        transform_age, self._max_latest_tf_age)
                    return False, "tf_too_old"

        transform_stamp = transform.header.stamp
        if stamp.to_sec() > 0.0 and transform_stamp.to_sec() > 0.0:
            det.transform_age_sec = abs((stamp - transform_stamp).to_sec())
        else:
            # 静态变换的时间戳合法地为零。
            det.transform_age_sec = 0.0
        pixel = (float(det.center_px.x), float(det.center_px.y))
        if self._rectify_input_pixels and self._camera_has_distortion:
            pixel = self._camera_model.rectifyPoint(pixel)
        if not all(math.isfinite(value) for value in pixel):
            return False, "pixel_nonfinite"
        ray = self._camera_model.projectPixelTo3dRay(pixel)
        origin = PointStamped()
        origin.header.stamp = stamp
        origin.header.frame_id = source_frame
        origin.point = Point(0.0, 0.0, 0.0)
        endpoint = PointStamped()
        endpoint.header = origin.header
        endpoint.point = Point(float(ray[0]), float(ray[1]), float(ray[2]))

        map_origin = tf2_geometry_msgs.do_transform_point(origin, transform)
        map_endpoint = tf2_geometry_msgs.do_transform_point(endpoint, transform)
        direction = (
            map_endpoint.point.x - map_origin.point.x,
            map_endpoint.point.y - map_origin.point.y,
            map_endpoint.point.z - map_origin.point.z,
        )
        if not all(math.isfinite(value) for value in direction + (
                map_origin.point.x, map_origin.point.y, map_origin.point.z, self._ground_z)):
            return False, "projection_nonfinite"
        if abs(direction[2]) < self._ray_epsilon:
            return False, "ray_parallel_ground"

        scale = (self._ground_z - map_origin.point.z) / direction[2]
        if scale <= 0.0:
            return False, "intersection_behind_camera"

        det.map_point = Point(
            map_origin.point.x + scale * direction[0],
            map_origin.point.y + scale * direction[1],
            self._ground_z,
        )
        det.map_valid = True
        det.map_frame = self._map_frame
        det.map_quality = max(0.0, min(1.0, float(det.geometry_confidence)))
        det.reject_reason = ""
        return True, ""

    def _on_coarse(self, msg):
        # Navigation hypotheses only: never publish on the refined/memory input.
        if msg.source != "target_detector":
            return
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if msg.header.stamp.to_sec() <= 0.0 or not 0.0 <= age <= 0.5:
            return
        out = TargetDetectionArray()
        out.header = msg.header
        out.source = "coarse_navigation_projector"
        out.completed_sources = list(msg.completed_sources)
        for original in msg.detections:
            confidence = float(original.class_confidence)
            if (original.class_name not in self._coarse_classes or
                    not math.isfinite(confidence) or
                    not self._coarse_min_confidence <= confidence <= 1.0 or
                    original.roi.width <= 0 or original.roi.height <= 0):
                continue
            det = copy.deepcopy(original)
            det.center_px = Point(det.roi.x_offset + det.roi.width / 2.0,
                                  det.roi.y_offset + det.roi.height / 2.0, 0.0)
            det.center_source = "bbox_navigation_only"
            det.center_refined = det.association_valid = det.geometry_verified = False
            det.geometry_confidence = 0.0
            self._invalidate(det, "")
            ok, reason = self._project(det, msg.header.stamp, msg.header.frame_id,
                                       require_refined=False)
            if not ok:
                self._invalidate(det, reason)
            # map_quality remains zero: no geometry quality was measured.
            out.detections.append(det)
        self._coarse_pub.publish(out)

    def _on_detections(self, msg):
        out = TargetDetectionArray()
        out.header = msg.header
        out.source = "target_map_projector"
        out.completed_sources = msg.completed_sources
        source_frame = msg.header.frame_id
        if not source_frame and self._camera_ready:
            source_frame = self._camera_model.tfFrame()

        for original in msg.detections:
            det = copy.deepcopy(original)
            self._invalidate(det, "")
            ok, reason = self._project(det, msg.header.stamp, source_frame)
            if not ok:
                self._invalidate(det, reason)
            out.detections.append(det)
        self._pub.publish(out)


def main():
    TargetMapProjector()
    rospy.spin()


if __name__ == "__main__":
    main()
