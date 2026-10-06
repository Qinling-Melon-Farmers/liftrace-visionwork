#!/usr/bin/env python3
"""drop_aligner: 计算像素偏差，判定对准条件，发布 DropOffset + DropReady。"""
import math
import copy
import threading
from collections import deque

import rospy
import cv2
import tf2_ros
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

from uav_vision.msg import (
    AlignmentTargetContext, DropOffset, DropReady, ReleaseEvidence,
    ReleaseEvidenceContext, TargetCandidate, TargetCandidateArray, TargetDetectionArray,
)
from uav_vision.ground_projection import intersect_ground
from uav_vision.drop_geometry_policy import (
    ObservationFence, observation_age, calibrated_camera_info,
    match_source_frame, slot_goal, pixel_equivalent_limit,
)
from uav_vision.alignment_context_policy import (
    COMMAND_ALIGN, associate_geometry, context_frozen_key,
    geometry_identity_key, validate_alignment_context,
)
from uav_vision.target_selection_policy import resolve_class_profile

CONFIRMED_STATE = 2
VALID_ALIGN_MODES = {"disabled", "drop_circle", "drop_cross", "landing"}
MODE_CLASS_MAP = {
    "drop_circle": {"circle"},
    "drop_cross": {"red_cross"},
    "landing": {"landing_pad"},
}


class DropAligner:
    def __init__(self):
        rospy.init_node("drop_aligner")

        self._align_mode_topic = rospy.get_param("~align_mode_topic", "/uav_vision/align_mode")
        self._default_mode = self._sanitize_mode(rospy.get_param("~default_mode", "disabled"))
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/camera_info")
        self._use_camera_info = bool(rospy.get_param("~use_camera_info", True))
        self._alignment_context_topic = rospy.get_param(
            "~alignment_context_topic", "/uav_vision/alignment_target_context")
        self._release_evidence_context_topic = rospy.get_param(
            "~release_evidence_context_topic",
            "/uav_vision/release_evidence_context")
        self._require_alignment_context = bool(
            rospy.get_param("~require_alignment_context", False))
        self._alignment_context_max_age = float(
            rospy.get_param("~alignment_context_max_age", 0.5))
        self._alignment_context_watchdog_rate = float(
            rospy.get_param("~alignment_context_watchdog_rate", 20.0))
        self._class_profile, self._allowed_semantic_classes = \
            resolve_class_profile(rospy.get_param("~class_profile", "full"))
        commands = rospy.get_param(
            "~allowed_alignment_commands", [COMMAND_ALIGN])
        if not isinstance(commands, list):
            commands = [commands]
        self._allowed_alignment_commands = frozenset(int(value) for value in commands)
        if not self._allowed_alignment_commands:
            raise ValueError("allowed_alignment_commands must not be empty")
        if self._alignment_context_max_age < 0.0:
            raise ValueError("alignment_context_max_age must be >= 0")
        if (self._require_alignment_context and
                self._alignment_context_watchdog_rate <= 0.0):
            raise ValueError(
                "alignment_context_watchdog_rate must be > 0 in strict mode")

        # 参数
        self._target_cx = rospy.get_param("~target_center_x", 640.0)
        self._target_cy = rospy.get_param("~target_center_y", 480.0)
        self._max_offset_px = rospy.get_param("~max_offset_px", 30.0)
        self._stable_frames = rospy.get_param("~stable_frames", 3)
        self._min_confidence = rospy.get_param("~min_confidence", 0.6)
        self._target_max_age = float(rospy.get_param("~target_max_age", 0.5))
        self._camera_model = PinholeCameraModel()
        self._camera_ready = False
        self._state_lock = threading.RLock()
        self._frame_fence = ObservationFence()
        self._exact_projection = bool(rospy.get_param("~exact_drop_projection", False))
        self._mapped_frames = deque(maxlen=64)
        self._calibrations = deque(maxlen=64)
        self._projection_jacobians = {}
        self._projection_principal_points = {}
        self._pending_targets = None
        if self._exact_projection:
            if not self._require_alignment_context or not self._use_camera_info:
                raise ValueError("exact_drop_projection requires alignment context and CameraInfo")
            self._map_frame = rospy.get_param("~map_frame", "camera_init")
            self._body_frame = rospy.get_param("~body_frame", "vision_body")
            self._ground_z = float(rospy.get_param("~ground_z", 0.0))
            # Zero inherits the existing pixel tolerance through exposure geometry.
            self._max_error_m = float(rospy.get_param("~max_alignment_error_m", 0.0))
            self._camera_info_max_skew = float(rospy.get_param("~camera_info_max_skew_sec", 0.10))
            self._camera_info_mode = rospy.get_param("~camera_info_mode", "fixed")
            self._allow_static_camera_info = bool(rospy.get_param("~allow_static_camera_info", True))
            self._body_max_age = float(rospy.get_param("~body_pose_max_age_sec", 0.10))
            self._tf_timeout = float(rospy.get_param("~tf_timeout", 0.05))
            self._rectify_input_pixels = bool(rospy.get_param("~rectify_input_pixels", True))
            self._slot_mode = rospy.get_param("~slot_compensation_mode", "physical_body_position")
            # One source of measured extrinsics: the controller's parameter tree.
            self._slot_parameter_ns = rospy.get_param("~slot_parameter_namespace", "/drop_system")
            self._slot_positions_body = rospy.get_param("~slot_positions_body", [])
            if (not self._map_frame or not self._body_frame or
                    self._slot_mode not in ("zero", "physical_body_position", "legacy_body_target_shift") or
                    self._camera_info_mode not in ("fixed", "per_frame") or
                    not math.isfinite(self._ground_z) or
                    not math.isfinite(self._max_error_m) or self._max_error_m < 0 or
                    any(not math.isfinite(v) or v <= 0 for v in
                        (self._camera_info_max_skew, self._body_max_age, self._tf_timeout))):
                raise ValueError("invalid exact drop projection configuration")
            self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
            rospy.Subscriber(rospy.get_param("~mapped_detections_topic", "/uav_vision/detections_mapped"),
                             TargetDetectionArray, self._on_mapped_frame, queue_size=1)

        self._consecutive_ok = 0
        self._align_mode = self._default_mode
        self._selected_target = None
        self._alignment_context = None
        self._alignment_context_frozen_key = None
        self._active_geometry_key = None
        self._active_geometry_last_seen = None
        self._last_context_watchdog_reason = None

        self._offset_pub = rospy.Publisher("/uav_vision/drop_offset",
                                           DropOffset, queue_size=1)
        self._ready_pub = rospy.Publisher("/uav_vision/drop_ready",
                                          DropReady, queue_size=1)
        self._evidence_pub = rospy.Publisher("/uav_vision/release_evidence",
                                             ReleaseEvidence, queue_size=1)
        self._evidence_context_pub = rospy.Publisher(
            self._release_evidence_context_topic,
            ReleaseEvidenceContext, queue_size=1)
        rospy.Subscriber(
            "/uav_vision/targets", TargetCandidateArray, self._on_targets)
        rospy.Subscriber(
            self._align_mode_topic, String, self._on_align_mode)
        rospy.Subscriber(
            "/uav_vision/selected_target", TargetCandidate,
            self._on_selected_target)
        rospy.Subscriber(
            self._alignment_context_topic, AlignmentTargetContext,
            self._on_alignment_context, queue_size=1)
        if self._use_camera_info:
            rospy.Subscriber(self._camera_info_topic, CameraInfo,
                             self._on_camera_info, queue_size=1)
        self._alignment_context_watchdog = None
        if self._require_alignment_context:
            self._alignment_context_watchdog = rospy.Timer(
                rospy.Duration(1.0 / self._alignment_context_watchdog_rate),
                self._on_alignment_context_watchdog)

        rospy.loginfo(
            "[DropAligner] ready target=(%.0f,%.0f) max_offset=%.0fpx "
            "stable=%d mode=%s profile=%s require_context=%s",
            self._target_cx, self._target_cy, self._max_offset_px,
            self._stable_frames, self._align_mode, self._class_profile,
            self._require_alignment_context)

    def _on_camera_info(self, msg):
        with self._state_lock:
            if getattr(self, "_exact_projection", False):
                # Keep independent models: a newer callback must not mutate a
                # calibration already paired with an earlier exposure.
                info = copy.deepcopy(msg)
                if info.header.stamp.to_sec() == 0 and self._calibrations:
                    old = self._calibrations[-1][0]
                    signature = lambda m: (m.header.frame_id, m.width, m.height,
                                           tuple(m.K), tuple(m.P), tuple(m.R), tuple(m.D),
                                           m.distortion_model, m.binning_x, m.binning_y,
                                           m.roi.x_offset, m.roi.y_offset, m.roi.width, m.roi.height, m.roi.do_rectify)
                    # A changed un-stamped calibration becomes effective at
                    # receipt, never retroactively for earlier images.
                    info.header.stamp = old.header.stamp if signature(old) == signature(info) else rospy.Time.now()
                if not calibrated_camera_info(info, info.header.frame_id):
                    self._calibrations.append((info, None))
                    return
                model = PinholeCameraModel()
                model.fromCameraInfo(info)
                self._calibrations.append((info, model))
            self._camera_model.fromCameraInfo(msg)
            self._target_cx = float(self._camera_model.cx())
            self._target_cy = float(self._camera_model.cy())
            self._camera_ready = True

    def _on_mapped_frame(self, msg):
        with self._state_lock:
            self._mapped_frames.append(copy.deepcopy(msg))
            pending = getattr(self, "_pending_targets", None)
            if pending is not None:
                self._pending_targets = None
                self._on_targets_locked(pending)

    def _project_current_target(self, target):
        frame = next((f for f in reversed(self._mapped_frames)
                      if f.header.stamp == target.last_seen), None)
        if frame is None:
            raise ValueError("source_observation_missing")
        source_frame = match_source_frame(target, frame)
        stamp = target.last_seen.to_sec()
        eligible = [(info, model) for info, model in reversed(self._calibrations)
                    if info.header.frame_id == source_frame and
                    ((0 < info.header.stamp.to_sec() <= stamp and
                      (self._camera_info_mode == "fixed" or
                       stamp-info.header.stamp.to_sec() <= self._camera_info_max_skew)) or
                     (info.header.stamp.to_sec() == 0 and self._allow_static_camera_info and
                      self._camera_info_mode == "fixed"))]
        if not eligible:
            raise ValueError("camera_info_unavailable_at_exposure")
        info, model = max(eligible, key=lambda item: item[0].header.stamp.to_sec())
        if model is None or not calibrated_camera_info(info, source_frame):
            raise ValueError("camera_info_invalid")
        pixel = (float(target.center_px.x), float(target.center_px.y))
        if not all(math.isfinite(v) for v in pixel) or not (
                0 <= pixel[0] < info.width and 0 <= pixel[1] < info.height):
            raise ValueError("pixel_invalid")
        transform = self._tf_buffer.lookup_transform(
            self._map_frame, source_frame, target.last_seen, rospy.Duration(self._tf_timeout))
        # Exact lookup only. A dynamic timestamp mismatch is never accepted.
        if (transform.header.stamp.to_sec() > 0 and
                transform.header.stamp != target.last_seen):
            raise ValueError("tf_not_at_exposure")
        if transform.header.frame_id != self._map_frame or transform.child_frame_id != source_frame:
            raise ValueError("tf_frame_mismatch")
        t, q = transform.transform.translation, transform.transform.rotation
        def project(raw_pixel):
            rectified = model.rectifyPoint(raw_pixel) if (
                self._rectify_input_pixels and any(abs(v) > 1e-12 for v in info.D)) else raw_pixel
            return intersect_ground(model.projectPixelTo3dRay(rectified),
                                    (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), self._ground_z)
        point = project(pixel)
        left, right = project((pixel[0]-1, pixel[1])), project((pixel[0]+1, pixel[1]))
        up, down = project((pixel[0], pixel[1]-1)), project((pixel[0], pixel[1]+1))
        self._projection_jacobians[(target.id, target.last_seen.to_nsec())] = (
            (right[0]-left[0])/2, (down[0]-up[0])/2,
            (right[1]-left[1])/2, (down[1]-up[1])/2)
        self._projection_principal_points[(target.id, target.last_seen.to_nsec())] = (model.cx(), model.cy())
        current = copy.deepcopy(target)
        current.header.frame_id = source_frame
        current.header.stamp = target.last_seen
        current.map_valid = True
        current.map_frame = self._map_frame
        current.map_point = Point(*point)
        current.transform_age_sec = 0.0
        return current

    def _metric_goal(self, target):
        # Latest *body* pose is used for present alignment and yaw compensation;
        # target projection above always uses the exposure camera transform.
        body = self._tf_buffer.lookup_transform(
            self._map_frame, self._body_frame, rospy.Time(0), rospy.Duration(self._tf_timeout))
        if (body.header.frame_id != self._map_frame or body.child_frame_id != self._body_frame or
                observation_age(body.header.stamp.to_sec(), rospy.Time.now().to_sec()) > self._body_max_age):
            raise ValueError("body_pose_stale_or_invalid")
        t, q = body.transform.translation, body.transform.rotation
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or abs(
                q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-3:
            raise ValueError("body_pose_invalid_quaternion")
        context = self._alignment_context
        offsets = [(0., 0.)]*3
        if self._slot_mode == "physical_body_position":
            offsets = self._slot_positions_body
        elif self._slot_mode != "zero":
            key = "dynamic_slot_offsets" if self._align_mode == "drop_cross" else "slot_offsets"
            offsets = rospy.get_param(self._slot_parameter_ns+"/"+key)
        if len(offsets) != 3 or any(len(value) != 2 for value in offsets):
            raise ValueError("slot_offsets_invalid")
        p = target.map_point
        goal, error = slot_goal((p.x, p.y, p.z), (t.x, t.y, t.z), (q.x, q.y, q.z, q.w),
                                int(context.payload_slot), offsets, self._slot_mode)
        limit = self._max_error_m or pixel_equivalent_limit(
            self._projection_jacobians[(target.id, target.last_seen.to_nsec())],
            (goal[0]-t.x, goal[1]-t.y), self._max_offset_px)
        return goal, error, limit

    def _sanitize_mode(self, mode):
        return mode if mode in VALID_ALIGN_MODES else "disabled"

    def _on_align_mode(self, msg):
        with self._state_lock:
            new_mode = self._sanitize_mode(msg.data.strip())
            if new_mode != self._align_mode:
                self._align_mode = new_mode
                self._clear_stability()
                rospy.loginfo("[DropAligner] align mode -> %s", self._align_mode)

    def _on_selected_target(self, msg):
        with self._state_lock:
            self._selected_target = msg

    def _on_alignment_context(self, msg):
        with self._state_lock:
            try:
                frozen_key = context_frozen_key(msg)
            except (AttributeError, TypeError, ValueError, OverflowError):
                frozen_key = ("malformed",)
            if (self._alignment_context is None or
                    frozen_key != self._alignment_context_frozen_key):
                # Optional mode is a byte-compatible diagnostic tap: receiving
                # or replacing context must not perturb the legacy streak.
                if self._require_alignment_context:
                    self._clear_stability()
                    self._last_context_watchdog_reason = None
                rospy.loginfo(
                    "[DropAligner] alignment context fence changed "
                    "mission=%s decision=%u target=%u attempt=%u slot=%u",
                    msg.mission_id, msg.decision_seq, msg.semantic_target_id,
                    msg.attempt, msg.payload_slot)
            self._alignment_context = msg
            self._alignment_context_frozen_key = frozen_key

    def _on_alignment_context_watchdog(self, _event):
        with self._state_lock:
            if self._align_mode == "disabled":
                self._clear_stability()
                self._last_context_watchdog_reason = None
                return
            pending = getattr(self, "_pending_targets", None)
            if pending is not None:
                latest = max((t.last_seen.to_sec() for t in pending.targets), default=0.)
                if observation_age(latest, rospy.Time.now().to_sec()) > self._target_max_age:
                    self._pending_targets = None
                    self._clear_stability()
                    self._publish_state(None, False, ["source_observation_timeout"])
                    self._last_context_watchdog_reason = "source_observation_timeout"
                    return
            valid, reason = self._base_context_status()
            if valid:
                if self._active_geometry_last_seen is not None:
                    geometry_age = observation_age(
                        self._active_geometry_last_seen.to_sec(), rospy.Time.now().to_sec())
                    if geometry_age > self._target_max_age:
                        reason = "alignment_context_geometry_stale"
                        self._clear_stability()
                        if reason != self._last_context_watchdog_reason:
                            self._publish_state(
                                None, False, [reason],
                                (False, reason, float("inf")))
                            self._last_context_watchdog_reason = reason
                        return
                self._last_context_watchdog_reason = None
                return
            had_stability = (
                self._consecutive_ok > 0 or self._active_geometry_key is not None)
            self._clear_stability()
            if reason != self._last_context_watchdog_reason or had_stability:
                self._publish_state(
                    None, False, [reason], (False, reason, float("inf")))
                self._last_context_watchdog_reason = reason

    def _clear_stability(self):
        self._consecutive_ok = 0
        self._active_geometry_key = None
        self._active_geometry_last_seen = None

    def _target_sort_key(self, target):
        return (target.geometry_confidence, target.class_confidence, target.observe_count)

    def _mode_reason(self):
        return {
            "disabled": "align disabled",
            "drop_circle": "no confirmed circle",
            "drop_cross": "no confirmed red_cross",
            "landing": "no confirmed landing_pad",
        }.get(self._align_mode, "invalid mode")

    def _base_context_status(self):
        return validate_alignment_context(
            self._alignment_context,
            rospy.Time.now(),
            self._class_profile,
            self._allowed_alignment_commands,
            self._align_mode,
            self._alignment_context_max_age,
            self._allowed_semantic_classes,
        )

    def _context_status_for_target(self, target):
        valid, reason = self._base_context_status()
        if not valid:
            return False, reason, float("inf")
        return associate_geometry(
            self._alignment_context, target, self._align_mode)

    def _choose_target(self, msg):
        if self._align_mode == "disabled":
            return None, "align disabled", None

        allowed_classes = MODE_CLASS_MAP.get(self._align_mode, set())
        if (not self._require_alignment_context and
                self._align_mode == "drop_cross" and
                self._selected_target is not None):
            for target in msg.targets:
                if (
                    target.id == self._selected_target.id
                    and target.class_name == "red_cross"
                    and target.state >= CONFIRMED_STATE
                    and target.center_refined
                    and self._observation_age(target) <= self._target_max_age
                ):
                    return target, None, None

        confirmed_candidates = [
            target for target in msg.targets
            if target.class_name in allowed_classes and
            target.state >= CONFIRMED_STATE and target.center_refined
        ]
        if not confirmed_candidates:
            return None, "no confirmed refined target", None

        # 地图记忆会有意保留到目标离开当前视野之后。因此投递对准必须先丢弃过期记录，再按
        # 质量排序；否则历史高质量圆环可能一直遮蔽当前位于飞机下方、质量较低的圆环。
        candidates = [
            target for target in confirmed_candidates
            if self._observation_age(target) <= self._target_max_age
        ]
        if not candidates:
            return None, "stale observation", None

        if getattr(self, "_exact_projection", False) and self._align_mode in ("drop_circle", "drop_cross"):
            self._projection_jacobians.clear()
            self._projection_principal_points.clear()
            projected, failure = [], "projection_unavailable"
            for target in candidates:
                try:
                    projected.append(self._project_current_target(target))
                except (ValueError, TypeError, cv2.error, tf2_ros.LookupException,
                        tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
                    failure = str(error) if isinstance(error, ValueError) else "tf_or_projection_unavailable"
            candidates = projected
            if not candidates:
                return None, failure, None

        if self._require_alignment_context:
            matches = []
            mismatches = []
            for target in candidates:
                valid, mismatch_reason, distance = \
                    self._context_status_for_target(target)
                if valid:
                    matches.append((distance, target))
                else:
                    mismatches.append((distance, target, mismatch_reason))
            if not matches:
                if not mismatches:
                    return (
                        None, "alignment_context_geometry_missing",
                        (False, "alignment_context_geometry_missing",
                         float("inf")))
                diagnostic = min(
                    mismatches,
                    key=lambda item: item[0]
                    if math.isfinite(item[0]) else float("inf"))
                return (
                    diagnostic[1], diagnostic[2],
                    (False, diagnostic[2], diagnostic[0]))
            # 同一语义靶附近出现多个圆环时先取地图距离最近者，再比较视觉质量。
            matches.sort(key=lambda item: (
                item[0],
                -item[1].geometry_confidence,
                -item[1].class_confidence,
                -item[1].observe_count,
            ))
            return (
                matches[0][1], None,
                (True, "alignment_context_valid", matches[0][0]))

        candidates.sort(key=self._target_sort_key, reverse=True)
        return candidates[0], None, None

    def _on_targets(self, msg):
        with self._state_lock:
            self._on_targets_locked(msg)

    def _on_targets_locked(self, msg):
        if self._align_mode == "disabled":
            self._clear_stability()
            self._publish_state(None, False, ["align_disabled"])
            return

        context_status = self._base_context_status()
        if self._require_alignment_context and not context_status[0]:
            self._clear_stability()
            self._publish_state(
                None, False, [context_status[1]],
                (context_status[0], context_status[1], float("inf")))
            return

        if not msg.targets:
            self._clear_stability()
            self._publish_state(None, False, ["no_targets"])
            return

        if getattr(self, "_exact_projection", False) and self._align_mode in ("drop_circle", "drop_cross"):
            # Independent ROS topics have no callback ordering guarantee. Hold
            # at most one pending candidate batch until its image metadata
            # arrives. Do not reset/renew the previous evidence while waiting.
            latest_ns = getattr(getattr(self, "_frame_fence", None), "latest", 0)
            observations = [t.last_seen.to_nsec() for t in msg.targets if
                            t.class_name in MODE_CLASS_MAP[self._align_mode] and
                            t.state >= CONFIRMED_STATE and t.center_refined and
                            self._observation_age(t) <= self._target_max_age and
                            t.last_seen.to_nsec() > latest_ns]
            available = {f.header.stamp.to_nsec() for f in self._mapped_frames}
            if any(stamp not in available for stamp in observations):
                pending = getattr(self, "_pending_targets", None)
                pending_ns = max((t.last_seen.to_nsec() for t in pending.targets), default=0) if pending else 0
                if max(observations) >= pending_ns:
                    self._pending_targets = copy.deepcopy(msg)
                return

        best, reason, chosen_context_status = self._choose_target(msg)
        if best is None:
            self._clear_stability()
            normalized_reason = (reason or self._mode_reason()).replace(" ", "_")
            context_failure = None
            if normalized_reason.startswith("alignment_context_"):
                context_failure = (False, normalized_reason, float("inf"))
            self._publish_state(
                None, False, [normalized_reason], context_failure)
            return

        context_target_status = chosen_context_status or \
            self._context_status_for_target(best)
        if (self._require_alignment_context and reason and
                reason.startswith("alignment_context_")):
            self._clear_stability()
            self._publish_state(
                best, False, [reason], context_target_status)
            return
        if self._require_alignment_context and not context_target_status[0]:
            self._clear_stability()
            self._publish_state(
                best, False, [context_target_status[1]], context_target_status)
            return

        age = self._observation_age(best)
        if age > self._target_max_age:
            self._clear_stability()
            self._publish_state(
                best, False, ["stale_observation"], context_target_status)
            return

        is_drop = self._align_mode in ("drop_circle", "drop_cross")
        if is_drop:
            if not hasattr(self, "_frame_fence"):
                self._frame_fence = ObservationFence()
            admission = self._frame_fence.accept(best.last_seen.to_nsec(), geometry_identity_key(best))
            if admission in ("duplicate_observation", "out_of_order_observation"):
                if admission == "duplicate_observation":
                    # Preserve the original evidence without renewing its
                    # timestamp or source freshness. Watchdog still expires it.
                    return
                # An older packet cannot revoke a newer accepted observation.
                # Neither its receipt nor this ignored callback extends the lease.
                return
            if admission == "new_identity":
                self._consecutive_ok = 0

        if self._require_alignment_context:
            current_geometry_key = geometry_identity_key(best)
            if (self._active_geometry_key is not None and
                    current_geometry_key != self._active_geometry_key):
                self._consecutive_ok = 0
            self._active_geometry_key = current_geometry_key
            self._active_geometry_last_seen = best.last_seen

        cx, cy = self._target_cx, self._target_cy
        if getattr(self, "_exact_projection", False) and is_drop:
            cx, cy = self._projection_principal_points[(best.id, best.last_seen.to_nsec())]
        dx = best.center_px.x - cx
        dy = best.center_px.y - cy
        dist = (dx * dx + dy * dy) ** 0.5

        # 置信度低于阈值的，不发有效偏移
        if best.class_confidence < self._min_confidence:
            self._clear_stability()
            self._publish_state(
                best, False, ["low_confidence"], context_target_status)
            return

        offset = DropOffset()
        offset.header = best.header
        if self._align_mode in ("drop_circle", "drop_cross"):
            # Pixel coordinates belong to last_seen, even when memory is republished.
            # Keep the candidate header and the legacy landing path unchanged.
            offset.header = copy.copy(best.header)
            offset.header.stamp = best.last_seen
        offset.dx_px = dx
        offset.dy_px = dy
        offset.radius_px = best.center_px.z
        offset.quality = best.geometry_confidence
        if getattr(self, "_exact_projection", False) and is_drop:
            try:
                goal, error, limit = self._metric_goal(best)
            except (ValueError, TypeError, KeyError, tf2_ros.LookupException,
                    tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self._clear_stability()
                self._publish_state(best, False, ["slot_or_body_geometry_invalid"], context_target_status)
                return
            offset.map_valid = True
            offset.map_point = Point(*goal)
            offset.map_frame = self._map_frame
            offset.alignment_error_m = error
            offset.alignment_tolerance_m = limit
            offset.target_id = best.id
            offset.target_first_seen = best.first_seen
            aligned = error <= limit
        else:
            aligned = dist <= self._max_offset_px
        self._offset_pub.publish(offset)

        if aligned:
            self._consecutive_ok += 1
        else:
            self._consecutive_ok = 0

        ready = self._consecutive_ok >= self._stable_frames
        reasons = []
        if not aligned:
            reasons.append("offset_exceeds_limit")
        elif not ready:
            reasons.append("insufficient_stable_frames")
        self._publish_state(best, aligned, reasons, context_target_status)

    @staticmethod
    def _observation_age(target):
        return observation_age(target.last_seen.to_sec(), rospy.Time.now().to_sec())

    def _publish_state(self, target, aligned, rejection_reasons,
                       context_status=None):
        if context_status is None:
            context_status = self._context_status_for_target(target)
        evidence = ReleaseEvidence()
        evidence.header.stamp = rospy.Time.now()
        evidence.align_mode = self._align_mode
        evidence.aligned = aligned
        evidence.stable_frames = self._consecutive_ok
        evidence.rejection_reasons = rejection_reasons
        if target is not None:
            evidence.header.frame_id = target.header.frame_id
            evidence.target_present = True
            evidence.target_id = target.id
            evidence.target_class = target.class_name
            evidence.target_confirmed = target.state >= CONFIRMED_STATE
            evidence.center_refined = target.center_refined
            evidence.geometry_verified = (
                target.center_refined and
                target.geometry_confidence >= self._min_confidence
            )
            evidence.observation_age_sec = self._observation_age(target)
            evidence.observation_fresh = evidence.observation_age_sec <= self._target_max_age
        evidence.evidence_valid = (
            evidence.target_present and evidence.target_confirmed and
            evidence.geometry_verified and evidence.center_refined and
            evidence.observation_fresh and aligned and
            self._consecutive_ok >= self._stable_frames and
            not rejection_reasons and
            (not self._require_alignment_context or context_status[0])
        )
        self._evidence_pub.publish(evidence)
        self._publish_evidence_context(evidence, target, context_status)
        reason = "evidence_valid" if evidence.evidence_valid else \
            (rejection_reasons[0] if rejection_reasons else "evidence_invalid")
        self._publish_ready(evidence.evidence_valid, reason)

    def _publish_evidence_context(self, evidence, target, context_status):
        wrapped = ReleaseEvidenceContext()
        wrapped.header = evidence.header
        wrapped.evidence = evidence
        wrapped.context_valid = bool(context_status[0])
        wrapped.context_reason = str(context_status[1])
        wrapped.association_distance_m = (
            float(context_status[2])
            if math.isfinite(float(context_status[2])) else -1.0)

        context = self._alignment_context
        if context is not None:
            wrapped.context_header = context.header
            wrapped.context_source = context.source
            wrapped.context_schema_version = context.schema_version
            wrapped.context_active = context.active
            wrapped.mission_id = context.mission_id
            wrapped.decision_seq = context.decision_seq
            wrapped.deadline = context.deadline
            wrapped.command = context.command
            wrapped.class_profile = context.class_profile
            wrapped.align_mode = context.align_mode
            wrapped.has_semantic_target = context.has_target
            wrapped.semantic_target_id = context.semantic_target_id
            wrapped.semantic_target_first_seen = \
                context.semantic_target_first_seen
            wrapped.target_observation_stamp = context.target_observation_stamp
            wrapped.semantic_target_class = context.semantic_target_class
            wrapped.attempt = context.attempt
            wrapped.payload_slot = context.payload_slot
            wrapped.semantic_target_pose = context.target_pose
            wrapped.max_association_distance_m = \
                context.max_association_distance_m

        if target is not None:
            wrapped.geometry_target_present = True
            wrapped.geometry_target_id = target.id
            wrapped.geometry_target_first_seen = target.first_seen
            wrapped.geometry_target_last_seen = target.last_seen
            wrapped.geometry_target_class = target.class_name
            wrapped.geometry_map_valid = target.map_valid
            wrapped.geometry_target_pose.header = target.header
            wrapped.geometry_target_pose.header.frame_id = target.map_frame
            wrapped.geometry_target_pose.pose.position = target.map_point
            wrapped.geometry_target_pose.pose.orientation.w = 1.0
        wrapped.semantic_geometry_match = bool(
            target is not None and context_status[0])
        self._evidence_context_pub.publish(wrapped)

    def _publish_ready(self, ready, reason):
        msg = DropReady()
        msg.header.stamp = rospy.Time.now()
        msg.ready = ready
        msg.reason = reason
        self._ready_pub.publish(msg)


def main():
    DropAligner()
    rospy.spin()


if __name__ == "__main__":
    main()
