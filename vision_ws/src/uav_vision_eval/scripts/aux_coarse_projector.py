#!/usr/bin/env python3
"""斜下辅助相机粗定位：单目地面求交或对齐深度 ROI 反投影。

本节点只产生 ``/uav_vision/aux/*`` 视觉观测。它接受 YOLO 框中心，不要求圆环或
投递级几何证据，也不会发布 selected_target、drop_ready 或任何控制/释放话题。
"""

import copy
import math
from collections import deque

import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401 - 注册 geometry 消息的 TF 变换
import tf2_ros
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image

from uav_vision.msg import TargetDetectionArray
from uav_vision_eval.msg import AuxProjection, AuxProjectionArray


class AuxCoarseProjector:
    def __init__(self):
        rospy.init_node("aux_coarse_projector")
        self._input_topic = rospy.get_param(
            "~input_topic", "/uav_vision/aux/detections")
        self._output_topic = rospy.get_param(
            "~output_topic", "/uav_vision/aux/detections_mapped")
        self._diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/uav_vision/aux/projection_diagnostics")
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/aux_camera/camera_info")
        self._depth_topic = rospy.get_param(
            "~depth_topic", "/aux_camera/depth/image_raw")
        self._mode = str(rospy.get_param("~projection_mode", "mono")).lower()
        if self._mode not in ("mono", "depth"):
            raise ValueError("projection_mode must be mono or depth")

        self._map_frame = rospy.get_param("~map_frame", "camera_init")
        self._ground_z = float(rospy.get_param("~ground_z", 0.0))
        self._min_class_confidence = float(
            rospy.get_param("~min_class_confidence", 0.35))
        self._ray_z_epsilon = float(rospy.get_param("~ray_z_epsilon", 0.08))
        self._max_ground_range = float(
            rospy.get_param("~max_ground_range_m", 12.0))
        self._field_min_x = float(rospy.get_param("~field_min_x", -1.0e6))
        self._field_max_x = float(rospy.get_param("~field_max_x", 1.0e6))
        self._field_min_y = float(rospy.get_param("~field_min_y", -1.0e6))
        self._field_max_y = float(rospy.get_param("~field_max_y", 1.0e6))

        self._tf_timeout = float(rospy.get_param("~tf_timeout", 0.05))
        self._allow_latest_tf_fallback = bool(
            rospy.get_param("~allow_latest_tf_fallback", False))
        self._max_latest_tf_age = float(
            rospy.get_param("~max_latest_tf_age_sec", 0.10))

        self._depth_roi_scale = float(rospy.get_param("~depth_roi_scale", 0.35))
        self._depth_scale_16u = float(
            rospy.get_param("~depth_scale_16u", 0.001))
        self._min_depth = float(rospy.get_param("~min_depth_m", 0.20))
        self._max_depth = float(rospy.get_param("~max_depth_m", 10.0))
        self._min_depth_fraction = float(
            rospy.get_param("~min_depth_valid_fraction", 0.25))
        self._max_depth_mad = float(
            rospy.get_param("~max_depth_mad_m", 0.50))
        self._max_depth_age = float(
            rospy.get_param("~max_depth_age_sec", 0.12))
        self._max_depth_ground_residual = float(
            rospy.get_param("~max_depth_ground_residual_m", 0.75))
        self._fallback_to_mono = bool(
            rospy.get_param("~fallback_to_mono", True))

        self._mono_uncertainty_base = float(
            rospy.get_param("~mono_uncertainty_base_m", 0.15))
        self._mono_uncertainty_per_meter = float(
            rospy.get_param("~mono_uncertainty_per_meter", 0.08))
        self._ground_height_uncertainty = float(
            rospy.get_param("~ground_height_uncertainty_m", 0.05))
        self._depth_uncertainty_base = float(
            rospy.get_param("~depth_uncertainty_base_m", 0.05))
        self._depth_mad_scale = float(
            rospy.get_param("~depth_mad_scale", 1.5))
        self._depth_bbox_scale = float(
            rospy.get_param("~depth_bbox_scale", 0.25))
        self._quality_uncertainty_scale = max(
            1.0e-6, float(rospy.get_param("~quality_uncertainty_scale_m", 1.0)))

        self._camera_model = PinholeCameraModel()
        self._camera_ready = False
        self._depth_messages = deque(maxlen=max(
            2, int(rospy.get_param("~depth_cache_size", 12))))
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._mapped_pub = rospy.Publisher(
            self._output_topic, TargetDetectionArray, queue_size=2)
        self._diagnostics_pub = rospy.Publisher(
            self._diagnostics_topic, AuxProjectionArray, queue_size=2)
        rospy.Subscriber(self._camera_info_topic, CameraInfo,
                         self._on_camera_info, queue_size=1)
        if self._mode == "depth":
            rospy.Subscriber(self._depth_topic, Image,
                             self._on_depth, queue_size=1,
                             buff_size=2 ** 24)
        rospy.Subscriber(self._input_topic, TargetDetectionArray,
                         self._on_detections, queue_size=2)
        rospy.loginfo(
            "[AuxCoarseProjector] mode=%s input=%s output=%s map=%s",
            self._mode, self._input_topic, self._output_topic, self._map_frame)

    def _on_camera_info(self, message):
        self._camera_model.fromCameraInfo(message)
        self._camera_ready = True

    def _on_depth(self, message):
        self._depth_messages.append(message)

    @staticmethod
    def _stamp_age(left, right):
        if left.to_sec() <= 0.0 or right.to_sec() <= 0.0:
            return 0.0
        return abs((left - right).to_sec())

    def _lookup_transform(self, source_frame, stamp):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, source_frame, stamp,
                rospy.Duration(self._tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            if not self._allow_latest_tf_fallback:
                return None, -1.0, "tf_unavailable"
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._map_frame, source_frame, rospy.Time(0),
                    rospy.Duration(self._tf_timeout))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None, -1.0, "tf_unavailable"
            age = self._stamp_age(stamp, transform.header.stamp)
            if age > self._max_latest_tf_age:
                return None, age, "tf_too_old"
            return transform, age, ""
        return transform, self._stamp_age(stamp, transform.header.stamp), ""

    @staticmethod
    def _point_stamped(frame, stamp, xyz):
        point = PointStamped()
        point.header.frame_id = frame
        point.header.stamp = stamp
        point.point = Point(float(xyz[0]), float(xyz[1]), float(xyz[2]))
        return point

    def _camera_ray_and_origin(self, center, source_frame, stamp, transform):
        ray = self._camera_model.projectPixelTo3dRay(
            (float(center.x), float(center.y)))
        origin = self._point_stamped(source_frame, stamp, (0.0, 0.0, 0.0))
        endpoint = self._point_stamped(source_frame, stamp, ray)
        map_origin = tf2_geometry_msgs.do_transform_point(origin, transform)
        map_endpoint = tf2_geometry_msgs.do_transform_point(endpoint, transform)
        direction = np.array([
            map_endpoint.point.x - map_origin.point.x,
            map_endpoint.point.y - map_origin.point.y,
            map_endpoint.point.z - map_origin.point.z,
        ], dtype=float)
        return np.asarray(ray, dtype=float), map_origin, direction

    def _in_field(self, point):
        return (self._field_min_x <= point.x <= self._field_max_x and
                self._field_min_y <= point.y <= self._field_max_y)

    def _mono_projection(self, ray, map_origin, direction):
        norm = float(np.linalg.norm(direction))
        if norm <= 1.0e-9:
            return None, 0.0, 0.0, "invalid_ray"
        unit = direction / norm
        if abs(float(unit[2])) < self._ray_z_epsilon:
            return None, 0.0, 0.0, "ray_near_horizontal"
        scale = (self._ground_z - map_origin.point.z) / float(direction[2])
        if scale <= 0.0:
            return None, 0.0, 0.0, "intersection_behind_camera"
        point = Point(
            map_origin.point.x + scale * float(direction[0]),
            map_origin.point.y + scale * float(direction[1]),
            self._ground_z)
        ground_range = math.hypot(
            point.x - map_origin.point.x, point.y - map_origin.point.y)
        if ground_range > self._max_ground_range:
            return None, ground_range, 0.0, "intersection_out_of_range"
        if not self._in_field(point):
            return None, ground_range, 0.0, "intersection_out_of_field"
        uncertainty = (
            self._mono_uncertainty_base +
            self._mono_uncertainty_per_meter * ground_range +
            self._ground_height_uncertainty / max(abs(float(unit[2])), 1.0e-6))
        return point, ground_range, uncertainty, ""

    @staticmethod
    def _depth_array(message, scale_16u):
        encoding = message.encoding.lower()
        if encoding == "32fc1":
            item_size = 4
            dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
            scale = 1.0
        elif encoding in ("16uc1", "mono16"):
            item_size = 2
            dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
            scale = scale_16u
        else:
            raise ValueError("unsupported depth encoding: %s" % message.encoding)
        row_items = int(message.step) // item_size
        expected = row_items * int(message.height)
        raw = np.frombuffer(message.data, dtype=dtype, count=expected)
        if raw.size != expected or row_items < int(message.width):
            raise ValueError("invalid depth buffer")
        return raw.reshape((message.height, row_items))[:, :message.width].astype(
            np.float32) * scale

    def _nearest_depth(self, stamp):
        if not self._depth_messages:
            return None, "depth_unavailable"
        message = min(
            self._depth_messages,
            key=lambda item: self._stamp_age(item.header.stamp, stamp))
        if self._stamp_age(message.header.stamp, stamp) > self._max_depth_age:
            return None, "depth_too_old"
        return message, ""

    def _depth_sample(self, detection, stamp):
        message, reason = self._nearest_depth(stamp)
        if message is None:
            return None, 0.0, 0.0, 0.0, reason
        try:
            depth = self._depth_array(message, self._depth_scale_16u)
        except ValueError:
            return None, 0.0, 0.0, 0.0, "depth_encoding_invalid"

        width = max(3, int(round(max(1, detection.roi.width) * self._depth_roi_scale)))
        height = max(3, int(round(max(1, detection.roi.height) * self._depth_roi_scale)))
        center_x = int(round(float(detection.center_px.x)))
        center_y = int(round(float(detection.center_px.y)))
        x0 = max(0, center_x - width // 2)
        x1 = min(depth.shape[1], center_x + (width + 1) // 2)
        y0 = max(0, center_y - height // 2)
        y1 = min(depth.shape[0], center_y + (height + 1) // 2)
        if x1 <= x0 or y1 <= y0:
            return None, 0.0, 0.0, 0.0, "depth_roi_out_of_frame"
        values = depth[y0:y1, x0:x1].reshape(-1)
        valid = values[
            np.isfinite(values) &
            (values >= self._min_depth) &
            (values <= self._max_depth)]
        fraction = float(valid.size) / float(max(1, values.size))
        if valid.size == 0 or fraction < self._min_depth_fraction:
            return None, 0.0, 0.0, fraction, "depth_valid_fraction_low"
        median = float(np.median(valid))
        mad = float(np.median(np.abs(valid - median)))
        if mad > self._max_depth_mad:
            return None, median, mad, fraction, "depth_dispersion_high"
        return median, median, mad, fraction, ""

    def _depth_projection(self, detection, ray, source_frame, stamp, transform):
        depth, median, mad, fraction, reason = self._depth_sample(detection, stamp)
        if depth is None:
            return None, 0.0, 0.0, median, mad, fraction, reason
        camera_point = self._point_stamped(
            source_frame, stamp,
            (float(ray[0]) * depth, float(ray[1]) * depth, float(ray[2]) * depth))
        mapped = tf2_geometry_msgs.do_transform_point(camera_point, transform)
        if abs(mapped.point.z - self._ground_z) > self._max_depth_ground_residual:
            return (None, float(np.linalg.norm(ray) * depth), 0.0,
                    median, mad, fraction, "depth_ground_residual_high")
        point = Point(mapped.point.x, mapped.point.y, self._ground_z)
        if not self._in_field(point):
            return (None, float(np.linalg.norm(ray) * depth), 0.0,
                    median, mad, fraction, "depth_point_out_of_field")
        range_m = float(np.linalg.norm(ray) * depth)
        fx = max(float(self._camera_model.fx()), 1.0)
        fy = max(float(self._camera_model.fy()), 1.0)
        bbox_angle = max(
            float(detection.roi.width) / fx,
            float(detection.roi.height) / fy)
        uncertainty = (
            self._depth_uncertainty_base + self._depth_mad_scale * mad +
            self._depth_bbox_scale * range_m * bbox_angle)
        return point, range_m, uncertainty, median, mad, fraction, ""

    @staticmethod
    def _invalidate(detection, map_frame, reason):
        detection.map_valid = False
        detection.map_point = Point()
        detection.map_frame = map_frame
        detection.map_quality = 0.0
        detection.transform_age_sec = -1.0
        detection.reject_reason = reason

    def _observation(self, detection, index, ray):
        observation = AuxProjection()
        observation.header = detection.header
        observation.detection_index = index
        observation.class_name = detection.class_name
        observation.range_source = "invalid"
        observation.map_frame = self._map_frame
        observation.bearing_rad = math.atan2(float(ray[0]), float(ray[2]))
        observation.elevation_down_rad = math.atan2(
            float(ray[1]), math.hypot(float(ray[0]), float(ray[2])))
        return observation

    def _on_detections(self, message):
        output = TargetDetectionArray()
        output.header = message.header
        output.source = "aux_coarse_projector"
        output.completed_sources = list(message.completed_sources)
        if "aux_coarse_projector" not in output.completed_sources:
            output.completed_sources.append("aux_coarse_projector")
        diagnostics = AuxProjectionArray()
        diagnostics.header = message.header

        source_frame = message.header.frame_id
        if not source_frame and self._camera_ready:
            source_frame = self._camera_model.tfFrame()

        for index, original in enumerate(message.detections):
            detection = copy.deepcopy(original)
            self._invalidate(detection, self._map_frame, "")
            if not self._camera_ready:
                ray = np.array((0.0, 0.0, 1.0))
                reason = "camera_info_unavailable"
                observation = self._observation(detection, index, ray)
            elif not source_frame:
                ray = np.array((0.0, 0.0, 1.0))
                reason = "image_frame_empty"
                observation = self._observation(detection, index, ray)
            elif detection.class_confidence < self._min_class_confidence:
                ray = self._camera_model.projectPixelTo3dRay(
                    (float(detection.center_px.x), float(detection.center_px.y)))
                reason = "class_confidence_low"
                observation = self._observation(detection, index, ray)
            else:
                transform, transform_age, reason = self._lookup_transform(
                    source_frame, message.header.stamp)
                if transform is None:
                    ray = self._camera_model.projectPixelTo3dRay(
                        (float(detection.center_px.x), float(detection.center_px.y)))
                    observation = self._observation(detection, index, ray)
                else:
                    ray, map_origin, direction = self._camera_ray_and_origin(
                        detection.center_px, source_frame,
                        message.header.stamp, transform)
                    observation = self._observation(detection, index, ray)
                    point = None
                    range_m = 0.0
                    uncertainty = 0.0
                    depth_median = 0.0
                    depth_mad = 0.0
                    depth_fraction = 0.0
                    source = "mono_ground"
                    if self._mode == "depth":
                        (point, range_m, uncertainty, depth_median, depth_mad,
                         depth_fraction, reason) = self._depth_projection(
                            detection, ray, source_frame,
                            message.header.stamp, transform)
                        source = "depth_roi"
                        if point is None and self._fallback_to_mono:
                            point, range_m, uncertainty, mono_reason = \
                                self._mono_projection(ray, map_origin, direction)
                            if point is not None:
                                reason = ""
                                source = "mono_fallback"
                            else:
                                reason = reason + "+" + mono_reason
                    else:
                        point, range_m, uncertainty, reason = \
                            self._mono_projection(ray, map_origin, direction)
                    observation.depth_median_m = depth_median
                    observation.depth_mad_m = depth_mad
                    observation.depth_valid_fraction = depth_fraction
                    if point is not None:
                        detection.map_valid = True
                        detection.map_point = point
                        detection.map_frame = self._map_frame
                        detection.transform_age_sec = transform_age
                        detection.map_quality = max(
                            0.0, min(1.0,
                                     float(detection.class_confidence) *
                                     math.exp(-uncertainty /
                                              self._quality_uncertainty_scale)))
                        detection.reject_reason = ""
                        observation.valid = True
                        observation.reason = ""
                        observation.range_source = source
                        observation.range_m = range_m
                        observation.position_uncertainty_m = uncertainty
                        observation.map_point = point
                        observation.map_frame = self._map_frame
                    else:
                        observation.reason = reason

            if not detection.map_valid:
                self._invalidate(detection, self._map_frame, reason)
                observation.valid = False
                observation.reason = reason
            output.detections.append(detection)
            diagnostics.observations.append(observation)

        self._mapped_pub.publish(output)
        self._diagnostics_pub.publish(diagnostics)


if __name__ == "__main__":
    AuxCoarseProjector()
    rospy.spin()
