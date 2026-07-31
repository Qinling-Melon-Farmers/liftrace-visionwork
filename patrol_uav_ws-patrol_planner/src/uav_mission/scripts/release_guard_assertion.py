#!/usr/bin/env python3
"""Deterministic regression for the fail-closed release gateway."""
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String, UInt8

from patrol_control.srv import Servo
from uav_mission.msg import ReleasePermission, ReleaseResult
from uav_vision.msg import ReleaseEvidence


class ReleaseGuardAssertion:
    def __init__(self):
        rospy.init_node("release_guard_assertion")
        self._permission = None
        self._raw_calls = []
        self._results = []
        self._audit_status = "WAITING"
        self._mode_pub = rospy.Publisher(
            "/uav_vision/align_mode", String, queue_size=1)
        self._control_state_pub = rospy.Publisher(
            "/detect/point_class", Int8, queue_size=1)
        self._pose_pub = rospy.Publisher(
            "/mavros/local_position/pose", PoseStamped, queue_size=1)
        self._evidence_pub = rospy.Publisher(
            "/uav_vision/release_evidence", ReleaseEvidence, queue_size=1)
        rospy.Subscriber("/mission/release_permission", ReleasePermission,
                         self._on_permission, queue_size=4)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_result, queue_size=10)
        rospy.Subscriber("/uav_mission/mock_raw_servo_calls", UInt8,
                         self._on_raw_call, queue_size=10)
        rospy.Subscriber("/mission/visual_delivery_audit_status", String,
                         self._on_audit_status, queue_size=2)
        self._servo = rospy.ServiceProxy("/Servo", Servo)

    def _on_permission(self, msg):
        self._permission = msg

    def _on_result(self, msg):
        self._results.append(msg)

    def _on_raw_call(self, msg):
        self._raw_calls.append(int(msg.data))

    def _on_audit_status(self, msg):
        self._audit_status = msg.data

    @staticmethod
    def _evidence(target_id, mode="drop_circle", target_class="circle"):
        msg = ReleaseEvidence()
        msg.header.stamp = rospy.Time.now()
        msg.align_mode = mode
        msg.target_present = True
        msg.target_id = target_id
        msg.target_class = target_class
        msg.target_confirmed = True
        msg.geometry_verified = True
        msg.center_refined = True
        msg.observation_fresh = True
        msg.observation_age_sec = 0.0
        msg.aligned = True
        msg.stable_frames = 5
        msg.evidence_valid = True
        return msg

    @staticmethod
    def _pose(z):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def _publish(self, mode, target_id=None, z=0.10,
                 target_class="circle", control_state=2, duration=0.35):
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._mode_pub.publish(String(data=mode))
            self._control_state_pub.publish(Int8(data=control_state))
            self._pose_pub.publish(self._pose(z))
            if target_id is not None:
                self._evidence_pub.publish(
                    self._evidence(target_id, mode, target_class))
            rate.sleep()

    def _wait_permission(self, permitted, slot=None, reason=None, timeout=2.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            msg = self._permission
            if msg is not None and msg.permitted == permitted:
                if slot is not None and msg.payload_slot != slot:
                    rate.sleep()
                    continue
                if reason is not None and msg.reason != reason:
                    rate.sleep()
                    continue
                return msg
            rate.sleep()
        raise AssertionError(
            "permission timeout permitted=%s slot=%s reason=%s last=%s" %
            (permitted, slot, reason,
             None if self._permission is None else
             (self._permission.permitted, self._permission.payload_slot,
              self._permission.reason)))

    def _call(self, slot, expected):
        response = self._servo(slot)
        if bool(response.res) != expected:
            raise AssertionError(
                "Servo(%d) expected=%s actual=%s" %
                (slot, expected, response.res))

    def run(self):
        rospy.wait_for_service("/Servo", timeout=5.0)
        rospy.wait_for_service("/legacy/Servo_raw", timeout=5.0)
        rospy.sleep(0.3)

        # 1. No evidence: the legacy call must not reach raw Servo.
        self._publish("disabled", target_id=None)
        self._call(1, False)
        if self._raw_calls:
            raise AssertionError("raw call occurred without permission")

        # 2. Visual drop mode alone is insufficient: the legacy controller
        # must independently report Aligning (Dronemode value 2).
        self._publish("drop_circle", target_id=0, z=0.10, control_state=1)
        self._wait_permission(False, slot=1,
                              reason="control_not_aligning")
        self._call(1, False)
        if self._raw_calls:
            raise AssertionError("raw call occurred outside legacy Aligning")

        # 3. Valid vision at unsafe altitude remains denied.
        self._publish("drop_circle", target_id=0, z=1.0)
        self._wait_permission(False, slot=1,
                              reason="release_altitude_invalid")
        self._call(1, False)
        if self._raw_calls:
            raise AssertionError("raw call occurred at unsafe altitude")

        # 4. Valid slot 1 permission; out-of-order slot 2 is rejected.
        self._publish("drop_circle", target_id=0, z=0.10)
        self._wait_permission(True, slot=1)
        self._call(2, False)
        self._call(1, True)
        rospy.sleep(0.2)
        if self._raw_calls != [1]:
            raise AssertionError("unexpected raw calls after slot 1: %r" %
                                 self._raw_calls)

        # 5. Replaying slot 1 and releasing the same target into slot 2 fail.
        self._call(1, False)
        self._publish("drop_circle", target_id=0, z=0.10)
        self._wait_permission(False, slot=2,
                              reason="target_already_released")
        self._call(2, False)
        if self._raw_calls != [1]:
            raise AssertionError("duplicate target reached raw Servo")

        # 6. A new target can use the next sequential slot exactly once.
        self._publish("drop_circle", target_id=1, z=0.10)
        self._wait_permission(True, slot=2)
        self._call(2, True)
        rospy.sleep(0.2)
        if self._raw_calls != [1, 2]:
            raise AssertionError("unexpected raw calls after slot 2: %r" %
                                 self._raw_calls)

        # 7. Evidence expires even while pose and old control state stay fresh.
        self._publish("drop_circle", target_id=2, z=0.10, duration=0.10)
        stale_deadline = rospy.Time.now() + rospy.Duration(0.8)
        rate = rospy.Rate(30)
        while rospy.Time.now() < stale_deadline:
            self._mode_pub.publish(String(data="drop_circle"))
            self._control_state_pub.publish(Int8(data=2))
            self._pose_pub.publish(self._pose(0.10))
            rate.sleep()
        self._wait_permission(False, slot=3,
                              reason="stale_release_evidence")
        self._call(3, False)

        # 8. Fresh evidence permits final slot; payload is then exhausted.
        self._publish("drop_circle", target_id=2, z=0.10)
        self._wait_permission(True, slot=3)
        self._call(3, True)
        rospy.sleep(0.3)
        self._wait_permission(False, reason="payload_exhausted")
        self._call(3, False)

        if self._raw_calls != [1, 2, 3]:
            raise AssertionError("raw calls are not exactly [1,2,3]: %r" %
                                 self._raw_calls)
        successes = [result.payload_slot for result in self._results
                     if result.success]
        if successes != [1, 2, 3]:
            raise AssertionError("success audit mismatch: %r" % successes)
        audit_deadline = rospy.Time.now() + rospy.Duration(1.0)
        while self._audit_status == "WAITING" and \
                rospy.Time.now() < audit_deadline:
            rospy.sleep(0.02)
        if self._audit_status != "PASS":
            raise AssertionError(
                "visual delivery audit did not pass: %s" % self._audit_status)

        rospy.loginfo(
            "[ReleaseGuardAssertion] PASS raw_calls=%s results=%d",
            self._raw_calls, len(self._results))


def main():
    try:
        ReleaseGuardAssertion().run()
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logerr("[ReleaseGuardAssertion] FAIL: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
