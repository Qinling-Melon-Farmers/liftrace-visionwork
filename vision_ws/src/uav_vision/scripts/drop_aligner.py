#!/usr/bin/env python3
"""drop_aligner: 计算像素偏差，判定对准条件，发布 DropOffset + DropReady。"""
import math
import threading

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

from uav_vision.msg import (
    AlignmentTargetContext, DropOffset, DropReady, ReleaseEvidence,
    ReleaseEvidenceContext, TargetCandidate, TargetCandidateArray,
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
            self._camera_model.fromCameraInfo(msg)
            self._target_cx = float(self._camera_model.cx())
            self._target_cy = float(self._camera_model.cy())
            self._camera_ready = True

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
            valid, reason = self._base_context_status()
            if valid:
                if self._active_geometry_last_seen is not None:
                    observation_age = max(
                        0.0,
                        (rospy.Time.now() -
                         self._active_geometry_last_seen).to_sec())
                    if observation_age > self._target_max_age:
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

        if self._require_alignment_context:
            current_geometry_key = geometry_identity_key(best)
            if (self._active_geometry_key is not None and
                    current_geometry_key != self._active_geometry_key):
                self._consecutive_ok = 0
            self._active_geometry_key = current_geometry_key
            self._active_geometry_last_seen = best.last_seen

        age = self._observation_age(best)
        if age > self._target_max_age:
            self._clear_stability()
            self._publish_state(
                best, False, ["stale_observation"], context_target_status)
            return

        dx = best.center_px.x - self._target_cx
        dy = best.center_px.y - self._target_cy
        dist = (dx * dx + dy * dy) ** 0.5

        # 置信度低于阈值的，不发有效偏移
        if best.class_confidence < self._min_confidence:
            self._clear_stability()
            self._publish_state(
                best, False, ["low_confidence"], context_target_status)
            return

        offset = DropOffset()
        offset.header = best.header
        offset.dx_px = dx
        offset.dy_px = dy
        offset.radius_px = best.center_px.z
        offset.quality = best.geometry_confidence
        self._offset_pub.publish(offset)

        aligned = dist <= self._max_offset_px
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
        if target.last_seen.to_sec() <= 0.0:
            return float("inf")
        return max(0.0, (rospy.Time.now() - target.last_seen).to_sec())

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
