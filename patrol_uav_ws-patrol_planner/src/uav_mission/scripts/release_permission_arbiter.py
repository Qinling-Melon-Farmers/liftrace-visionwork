#!/usr/bin/env python3
"""Fail-closed release arbiter for the visual delivery mission.

The vision node publishes ReleaseEvidence.  This arbiter adds mission mode,
vehicle pose, payload sequencing and replay checks, then publishes a short
lived ReleasePermission.  It never calls an actuator.
"""
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int8, String

from uav_mission.msg import ReleasePermission, ReleaseResult
from uav_vision.msg import ReleaseEvidence, ReleaseEvidenceContext

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from release_commitment import (
    MODE_TARGET_CLASS, ReleaseCommitmentPolicy, strict_context_source,
)


class ReleasePermissionArbiter:
    def __init__(self):
        rospy.init_node("release_permission_arbiter")

        self._evidence_timeout = float(rospy.get_param("~evidence_timeout", 0.5))
        self._pose_timeout = float(rospy.get_param("~pose_timeout", 0.5))
        self._control_state_timeout = float(
            rospy.get_param("~control_state_timeout", 0.5))
        self._permission_lifetime = float(
            rospy.get_param("~permission_lifetime", 0.25))
        self._publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self._min_altitude = float(
            rospy.get_param("~min_release_altitude", -0.05))
        self._max_altitude = float(
            rospy.get_param("~max_release_altitude", 0.25))
        self._payload_slots = int(rospy.get_param("~payload_slots", 3))
        self._next_slot = int(rospy.get_param("~first_payload_slot", 1))
        self._required_control_state = int(
            rospy.get_param("~required_control_state", 2))
        self._commitment_timeout = float(
            rospy.get_param("~commitment_timeout", 30.0))
        self._commitment_max_drift = float(
            rospy.get_param("~commitment_max_drift", 0.20))
        self._require_evidence_context = bool(
            rospy.get_param("~require_evidence_context", False))
        self._class_profile = str(
            rospy.get_param("~class_profile", "r2026"))

        evidence_topic = rospy.get_param(
            "~evidence_topic", "/uav_vision/release_evidence")
        evidence_context_topic = rospy.get_param(
            "~evidence_context_topic",
            "/uav_vision/release_evidence_context")
        commitment_evidence_topic = rospy.get_param(
            "~commitment_evidence_topic",
            "/mission/release_commitment_evidence")
        align_mode_topic = rospy.get_param(
            "~align_mode_topic", "/uav_vision/align_mode")
        pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose")
        control_state_topic = rospy.get_param(
            "~control_state_topic", "/detect/point_class")
        permission_topic = rospy.get_param(
            "~permission_topic", "/mission/release_permission")
        permission_state_topic = rospy.get_param(
            "~permission_state_topic",
            "/mission/release_permission_active")
        result_topic = rospy.get_param(
            "~result_topic", "/mission/release_result")

        self._evidence = None
        self._evidence_context = None
        self._align_mode = "disabled"
        self._pose = None
        self._control_state = None
        self._control_state_stamp = rospy.Time(0)
        self._released_targets = set()
        self._completed_slots = set()
        self._commitment = None
        self._commitment_policy = ReleaseCommitmentPolicy(
            required_control_state=self._required_control_state,
            commitment_timeout=self._commitment_timeout,
            max_horizontal_drift=self._commitment_max_drift,
        )

        self._permission_pub = rospy.Publisher(
            permission_topic, ReleasePermission, queue_size=1)
        self._commitment_evidence_pub = rospy.Publisher(
            commitment_evidence_topic, ReleaseEvidence,
            queue_size=1, latch=True)
        self._permission_state_pub = rospy.Publisher(
            permission_state_topic, Bool, queue_size=1, latch=True)
        if self._require_evidence_context:
            rospy.Subscriber(
                evidence_context_topic, ReleaseEvidenceContext,
                self._on_evidence_context, queue_size=2)
        else:
            rospy.Subscriber(evidence_topic, ReleaseEvidence,
                             self._on_evidence, queue_size=2)
        rospy.Subscriber(align_mode_topic, String,
                         self._on_align_mode, queue_size=2)
        rospy.Subscriber(pose_topic, PoseStamped,
                         self._on_pose, queue_size=2)
        rospy.Subscriber(control_state_topic, Int8,
                         self._on_control_state, queue_size=2)
        rospy.Subscriber(result_topic, ReleaseResult,
                         self._on_result, queue_size=4)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / max(self._publish_rate, 1.0)),
            self._on_timer)
        self._permission_state_pub.publish(Bool(data=False))

        rospy.loginfo(
            "[ReleaseArbiter] ready slots=%d..%d evidence_timeout=%.2fs "
            "pose_timeout=%.2fs control_state=%d timeout=%.2fs "
            "altitude=[%.2f, %.2f]m commitment=%.1fs drift=%.2fm "
            "strict_context=%s",
            self._next_slot, self._payload_slots,
            self._evidence_timeout, self._pose_timeout,
            self._required_control_state, self._control_state_timeout,
            self._min_altitude, self._max_altitude,
            self._commitment_timeout, self._commitment_max_drift,
            self._require_evidence_context)

    def _on_evidence(self, msg):
        self._evidence = msg
        self._maybe_establish_commitment()

    def _on_evidence_context(self, msg):
        self._evidence_context = msg
        self._evidence = msg.evidence
        self._maybe_establish_commitment()

    def _on_align_mode(self, msg):
        self._align_mode = msg.data.strip()
        self._maybe_establish_commitment()

    def _on_pose(self, msg):
        self._pose = msg
        self._maybe_establish_commitment()

    def _on_control_state(self, msg):
        self._control_state = int(msg.data)
        self._control_state_stamp = rospy.Time.now()
        self._maybe_establish_commitment()

    def _on_result(self, msg):
        if not msg.success:
            return
        if msg.payload_slot in self._completed_slots:
            rospy.logwarn_throttle(
                1.0, "[ReleaseArbiter] duplicate success for slot %d ignored",
                msg.payload_slot)
            return
        if msg.payload_slot != self._next_slot:
            rospy.logwarn(
                "[ReleaseArbiter] out-of-order success slot=%d expected=%d ignored",
                msg.payload_slot, self._next_slot)
            return
        self._completed_slots.add(msg.payload_slot)
        self._released_targets.add((msg.align_mode, msg.target_id))
        if (self._commitment is not None and
                self._commitment.align_mode == msg.align_mode and
                self._commitment.target_id == msg.target_id):
            self._commitment = None
        self._next_slot += 1
        rospy.loginfo(
            "[ReleaseArbiter] release committed slot=%d target=%s/%d next_slot=%d",
            msg.payload_slot, msg.target_class, msg.target_id, self._next_slot)

    @staticmethod
    def _stamp_age(now, stamp):
        if stamp.to_sec() <= 0.0:
            return float("inf")
        return max(0.0, (now - stamp).to_sec())

    def _pose_xy(self):
        if self._pose is None:
            return None
        return (self._pose.pose.position.x, self._pose.pose.position.y)

    def _maybe_establish_commitment(self):
        if (self._commitment is not None or self._evidence is None or
                self._pose is None or self._control_state is None or
                self._align_mode not in MODE_TARGET_CLASS):
            return
        now = rospy.Time.now()
        evidence_valid, _, source = self._current_evidence_source(now)
        if not evidence_valid:
            return
        evidence_fresh = (
            self._stamp_age(now, self._evidence.header.stamp) <=
            self._evidence_timeout)
        pose_fresh = (
            self._stamp_age(now, self._pose.header.stamp) <=
            self._pose_timeout)
        control_state_fresh = (
            self._stamp_age(now, self._control_state_stamp) <=
            self._control_state_timeout)
        evidence = {
            "evidence_valid": True,
            "align_mode": self._evidence.align_mode,
            "target_id": source["target_id"],
            "target_class": source["target_class"],
            "geometry_target_class": source["geometry_target_class"],
            "stable_frames": int(self._evidence.stable_frames),
            "evidence_stamp_nsec": int(
                source["evidence_stamp"].to_nsec()),
        }
        commitment = self._commitment_policy.observe(
            now=now.to_sec(),
            evidence=evidence,
            control_state=self._control_state,
            pose=self._pose_xy(),
            next_slot=self._next_slot,
            released_targets=self._released_targets,
            evidence_fresh=evidence_fresh,
            pose_fresh=pose_fresh,
            control_state_fresh=control_state_fresh,
        )
        if commitment is None:
            return
        self._commitment = commitment
        self._commitment_evidence_pub.publish(self._evidence)
        rospy.loginfo(
            "[ReleaseArbiter] commitment locked slot=%d target=%s/%d "
            "mode=%s xy=(%.3f, %.3f)",
            commitment.payload_slot, commitment.target_class,
            commitment.target_id, commitment.align_mode,
            commitment.locked_x, commitment.locked_y)

    def _current_evidence_source(self, now):
        if self._evidence is None:
            return False, "no_release_evidence", None
        if self._require_evidence_context:
            return strict_context_source(
                self._evidence_context,
                now,
                self._evidence_timeout,
                self._class_profile,
                self._align_mode,
                self._next_slot,
            )
        evidence_age = self._stamp_age(now, self._evidence.header.stamp)
        if evidence_age > self._evidence_timeout:
            return False, "stale_release_evidence", None
        if self._evidence.align_mode != self._align_mode:
            return False, "align_mode_mismatch", None
        if not self._evidence.evidence_valid:
            return False, "visual_evidence_invalid", None
        expected_class = MODE_TARGET_CLASS[self._align_mode]
        if self._evidence.target_class != expected_class:
            return False, "evidence_target_class_mismatch", None
        return True, "permission_granted", {
            "target_id": int(self._evidence.target_id),
            "target_class": self._evidence.target_class,
            "geometry_target_class": self._evidence.target_class,
            "evidence_stamp": self._evidence.header.stamp,
            "stable_frames": int(self._evidence.stable_frames),
        }

    def _commitment_source(self):
        if self._commitment is None:
            return None
        stamp_nsec = self._commitment.evidence_stamp_nsec
        return {
            "target_id": self._commitment.target_id,
            "target_class": self._commitment.target_class,
            "geometry_target_class": MODE_TARGET_CLASS[
                self._commitment.align_mode],
            "evidence_stamp": rospy.Time(
                stamp_nsec // 1000000000,
                stamp_nsec % 1000000000),
        }

    def _evaluate(self, now):
        if self._next_slot > self._payload_slots:
            return False, "payload_exhausted", None
        if self._align_mode not in MODE_TARGET_CLASS:
            return False, "mission_not_in_drop_stage", None
        if self._control_state is None:
            return False, "no_control_state", None
        if self._stamp_age(now, self._control_state_stamp) > \
                self._control_state_timeout:
            return False, "stale_control_state", None
        if self._control_state != self._required_control_state:
            return False, "control_not_aligning", None
        if self._pose is None:
            return False, "no_vehicle_pose", None
        pose_age = self._stamp_age(now, self._pose.header.stamp)
        if pose_age > self._pose_timeout:
            return False, "stale_vehicle_pose", None

        current_valid, current_reason, source = \
            self._current_evidence_source(now)
        grant_reason = current_reason
        if current_valid and self._commitment is not None:
            current_target_key = (
                self._align_mode, source["target_id"])
            commitment_target_key = (
                self._commitment.align_mode,
                self._commitment.target_id)
            if current_target_key != commitment_target_key:
                self._commitment = None
                return False, "commitment_target_changed", source
        if not current_valid:
            commitment_valid, commitment_reason = \
                self._commitment_policy.evaluate(
                    commitment=self._commitment,
                    now=now.to_sec(),
                    align_mode=self._align_mode,
                    control_state=self._control_state,
                    pose=self._pose_xy(),
                    next_slot=self._next_slot,
                    released_targets=self._released_targets,
                    current_evidence_valid=False,
                )
            if not commitment_valid:
                if self._commitment is not None:
                    self._commitment = None
                    return False, commitment_reason, None
                return False, current_reason, None
            grant_reason = commitment_reason
            source = self._commitment_source()

        target_key = (self._align_mode, source["target_id"])
        if target_key in self._released_targets:
            return False, "target_already_released", source

        altitude = self._pose.pose.position.z
        if altitude < self._min_altitude or altitude > self._max_altitude:
            return False, "release_altitude_invalid", source
        return True, grant_reason, source

    def _on_timer(self, _event):
        now = rospy.Time.now()
        permitted, reason, source = self._evaluate(now)
        msg = ReleasePermission()
        msg.header.stamp = now
        msg.permitted = permitted
        msg.payload_slot = self._next_slot if self._next_slot <= 255 else 0
        msg.align_mode = self._align_mode
        msg.reason = reason
        if source is not None:
            msg.target_id = source["target_id"]
            msg.target_class = source["target_class"]
            msg.evidence_stamp = source["evidence_stamp"]
        msg.valid_until = now + rospy.Duration(self._permission_lifetime)
        self._permission_pub.publish(msg)
        self._permission_state_pub.publish(Bool(data=permitted))


def main():
    ReleasePermissionArbiter()
    rospy.spin()


if __name__ == "__main__":
    main()
