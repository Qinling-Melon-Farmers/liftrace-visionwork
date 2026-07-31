#!/usr/bin/env python3
"""Fail-closed release arbiter for the visual delivery mission.

The vision node publishes ReleaseEvidence.  This arbiter adds mission mode,
vehicle pose, payload sequencing and replay checks, then publishes a short
lived ReleasePermission.  It never calls an actuator.
"""
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String

from uav_mission.msg import ReleasePermission, ReleaseResult
from uav_vision.msg import ReleaseEvidence


MODE_TARGET_CLASS = {
    "drop_circle": "circle",
    "drop_cross": "red_cross",
}


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

        evidence_topic = rospy.get_param(
            "~evidence_topic", "/uav_vision/release_evidence")
        align_mode_topic = rospy.get_param(
            "~align_mode_topic", "/uav_vision/align_mode")
        pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose")
        control_state_topic = rospy.get_param(
            "~control_state_topic", "/detect/point_class")
        permission_topic = rospy.get_param(
            "~permission_topic", "/mission/release_permission")
        result_topic = rospy.get_param(
            "~result_topic", "/mission/release_result")

        self._evidence = None
        self._align_mode = "disabled"
        self._pose = None
        self._control_state = None
        self._control_state_stamp = rospy.Time(0)
        self._released_targets = set()
        self._completed_slots = set()

        self._permission_pub = rospy.Publisher(
            permission_topic, ReleasePermission, queue_size=1)
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

        rospy.loginfo(
            "[ReleaseArbiter] ready slots=%d..%d evidence_timeout=%.2fs "
            "pose_timeout=%.2fs control_state=%d timeout=%.2fs "
            "altitude=[%.2f, %.2f]m",
            self._next_slot, self._payload_slots,
            self._evidence_timeout, self._pose_timeout,
            self._required_control_state, self._control_state_timeout,
            self._min_altitude, self._max_altitude)

    def _on_evidence(self, msg):
        self._evidence = msg

    def _on_align_mode(self, msg):
        self._align_mode = msg.data.strip()

    def _on_pose(self, msg):
        self._pose = msg

    def _on_control_state(self, msg):
        self._control_state = int(msg.data)
        self._control_state_stamp = rospy.Time.now()

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
        self._next_slot += 1
        rospy.loginfo(
            "[ReleaseArbiter] release committed slot=%d target=%s/%d next_slot=%d",
            msg.payload_slot, msg.target_class, msg.target_id, self._next_slot)

    @staticmethod
    def _stamp_age(now, stamp):
        if stamp.to_sec() <= 0.0:
            return float("inf")
        return max(0.0, (now - stamp).to_sec())

    def _evaluate(self, now):
        if self._next_slot > self._payload_slots:
            return False, "payload_exhausted"
        if self._align_mode not in MODE_TARGET_CLASS:
            return False, "mission_not_in_drop_stage"
        if self._control_state is None:
            return False, "no_control_state"
        if self._stamp_age(now, self._control_state_stamp) > \
                self._control_state_timeout:
            return False, "stale_control_state"
        if self._control_state != self._required_control_state:
            return False, "control_not_aligning"
        if self._evidence is None:
            return False, "no_release_evidence"
        if self._pose is None:
            return False, "no_vehicle_pose"

        evidence_age = self._stamp_age(now, self._evidence.header.stamp)
        if evidence_age > self._evidence_timeout:
            return False, "stale_release_evidence"
        pose_age = self._stamp_age(now, self._pose.header.stamp)
        if pose_age > self._pose_timeout:
            return False, "stale_vehicle_pose"
        if self._evidence.align_mode != self._align_mode:
            return False, "align_mode_mismatch"
        if not self._evidence.evidence_valid:
            return False, "visual_evidence_invalid"

        expected_class = MODE_TARGET_CLASS[self._align_mode]
        if self._evidence.target_class != expected_class:
            return False, "evidence_target_class_mismatch"
        target_key = (self._align_mode, self._evidence.target_id)
        if target_key in self._released_targets:
            return False, "target_already_released"

        altitude = self._pose.pose.position.z
        if altitude < self._min_altitude or altitude > self._max_altitude:
            return False, "release_altitude_invalid"
        return True, "permission_granted"

    def _on_timer(self, _event):
        now = rospy.Time.now()
        permitted, reason = self._evaluate(now)
        msg = ReleasePermission()
        msg.header.stamp = now
        msg.permitted = permitted
        msg.payload_slot = self._next_slot if self._next_slot <= 255 else 0
        msg.align_mode = self._align_mode
        msg.reason = reason
        if self._evidence is not None:
            msg.target_id = self._evidence.target_id
            msg.target_class = self._evidence.target_class
            msg.evidence_stamp = self._evidence.header.stamp
        msg.valid_until = now + rospy.Duration(self._permission_lifetime)
        self._permission_pub.publish(msg)


def main():
    ReleasePermissionArbiter()
    rospy.spin()


if __name__ == "__main__":
    main()
