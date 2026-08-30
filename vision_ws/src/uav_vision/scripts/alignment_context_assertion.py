#!/usr/bin/env python3
"""ROS assertion for strict frozen-context alignment and compatibility output."""

import threading
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String

from uav_vision.msg import (
    AlignmentTargetContext, ReleaseEvidence, ReleaseEvidenceContext,
    TargetCandidate, TargetCandidateArray,
)


CONFIRMED = 2


class AlignmentContextAssertion:
    def __init__(self):
        self._condition = threading.Condition()
        self._evidence = None
        self._wrapped = None
        self._wrapped_count = 0
        self._mode_pub = rospy.Publisher(
            "/uav_vision/align_mode", String, queue_size=1, latch=True)
        self._context_pub = rospy.Publisher(
            "/uav_vision/alignment_target_context",
            AlignmentTargetContext, queue_size=1, latch=True)
        self._targets_pub = rospy.Publisher(
            "/uav_vision/targets", TargetCandidateArray, queue_size=1)
        rospy.Subscriber(
            "/uav_vision/release_evidence", ReleaseEvidence,
            self._on_evidence, queue_size=10)
        rospy.Subscriber(
            "/uav_vision/release_evidence_context", ReleaseEvidenceContext,
            self._on_wrapped, queue_size=10)

    def _on_evidence(self, message):
        with self._condition:
            self._evidence = message
            self._condition.notify_all()

    def _on_wrapped(self, message):
        with self._condition:
            self._wrapped = message
            self._wrapped_count += 1
            self._condition.notify_all()

    def _wait_connections(self):
        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (self._mode_pub.get_num_connections() > 0 and
                    self._context_pub.get_num_connections() > 0 and
                    self._targets_pub.get_num_connections() > 0):
                return
            rospy.sleep(0.05)
        raise AssertionError("drop_aligner subscriptions not connected")

    @staticmethod
    def _context(decision_seq, mode, semantic_class, semantic_id,
                 first_seen, command=AlignmentTargetContext.ALIGN,
                 profile="r2026", deadline=None):
        now = rospy.Time.now()
        message = AlignmentTargetContext()
        message.header.stamp = now
        message.source = "alignment_context_assertion"
        message.schema_version = AlignmentTargetContext.SCHEMA_VERSION
        message.active = True
        message.mission_id = "context-mock"
        message.decision_seq = decision_seq
        message.deadline = deadline or (now + rospy.Duration(10.0))
        message.command = command
        message.class_profile = profile
        message.align_mode = mode
        message.has_target = True
        message.semantic_target_id = semantic_id
        message.semantic_target_first_seen = first_seen
        message.target_observation_stamp = first_seen + rospy.Duration(1.0)
        message.semantic_target_class = semantic_class
        message.attempt = 1
        message.payload_slot = 2
        message.target_pose.header.stamp = message.target_observation_stamp
        message.target_pose.header.frame_id = "camera_init"
        message.target_pose.pose.position.x = 1.0
        message.target_pose.pose.position.y = 2.0
        message.target_pose.pose.position.z = 0.0
        message.target_pose.pose.orientation.w = 1.0
        message.max_association_distance_m = 0.5
        return message

    @staticmethod
    def _target(target_id, class_name, first_seen, map_x=1.1):
        now = rospy.Time.now()
        target = TargetCandidate()
        target.header.stamp = now
        target.header.frame_id = "downward_camera_optical_frame"
        target.id = target_id
        target.class_name = class_name
        target.class_confidence = 0.95
        target.geometry_confidence = 0.95
        target.center_px.x = 320.0
        target.center_px.y = 240.0
        target.center_px.z = 60.0
        target.center_refined = True
        target.center_source = "alignment_context_assertion"
        target.association_valid = True
        target.map_valid = True
        target.map_frame = "camera_init"
        target.map_point.x = map_x
        target.map_point.y = 2.0
        target.map_point.z = 0.0
        target.map_quality = 0.95
        target.state = CONFIRMED
        target.observe_count = 10
        target.consecutive_observe_count = 10
        target.first_seen = first_seen
        target.last_seen = now
        return target

    def _publish_target(self, target):
        target.header.stamp = rospy.Time.now()
        target.last_seen = target.header.stamp
        message = TargetCandidateArray()
        message.header.stamp = target.header.stamp
        message.targets = [target]
        self._targets_pub.publish(message)

    def _drive(self, context, target, predicate, timeout=3.0,
               refresh_context=True):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if context is not None and refresh_context:
                context.header.stamp = rospy.Time.now()
                self._context_pub.publish(context)
            self._publish_target(target)
            with self._condition:
                wrapped = self._wrapped
            if wrapped is not None and predicate(wrapped):
                return wrapped
            rate.sleep()
        with self._condition:
            wrapped = self._wrapped
        raise AssertionError(
            "condition timed out; last context_reason={} evidence_reasons={}".format(
                wrapped.context_reason if wrapped else None,
                list(wrapped.evidence.rejection_reasons)
                if wrapped else None))

    def _single(self, context, target, predicate):
        context.header.stamp = rospy.Time.now()
        self._context_pub.publish(context)
        rospy.sleep(0.08)
        with self._condition:
            start_count = self._wrapped_count
        self._publish_target(target)
        deadline = time.monotonic() + 2.0
        with self._condition:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if (self._wrapped_count > start_count and
                        predicate(self._wrapped)):
                    return self._wrapped
                self._condition.wait(timeout=0.05)
        raise AssertionError("single-frame assertion timed out")

    def _wait_wrapped(self, start_count, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if (self._wrapped_count > start_count and
                        predicate(self._wrapped)):
                    return self._wrapped
                self._condition.wait(timeout=0.05)
        raise AssertionError("watchdog assertion timed out")

    def _drive_context_only(self, context, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            context.header.stamp = rospy.Time.now()
            self._context_pub.publish(context)
            with self._condition:
                wrapped = self._wrapped
            if wrapped is not None and predicate(wrapped):
                return wrapped
            rate.sleep()
        raise AssertionError("geometry watchdog assertion timed out")

    def run(self):
        self._wait_connections()
        self._mode_pub.publish(String(data="drop_circle"))
        rospy.sleep(0.1)
        first_seen = rospy.Time.now() - rospy.Duration(2.0)
        circle = self._target(42, "circle", first_seen)

        missing = self._drive(
            None, circle,
            lambda msg: msg.context_reason == "alignment_context_missing")
        assert not missing.evidence.evidence_valid
        assert missing.evidence.stable_frames == 0

        wrong_command = self._context(
            1, "drop_circle", "tent", 0, first_seen,
            command=AlignmentTargetContext.APPROACH)
        rejected = self._drive(
            wrong_command, circle,
            lambda msg: msg.context_reason ==
            "alignment_context_command_mismatch")
        assert not rejected.evidence.evidence_valid
        assert rejected.evidence.stable_frames == 0

        tank = self._context(
            2, "drop_circle", "tank", 0, first_seen)
        rejected = self._drive(
            tank, circle,
            lambda msg: msg.context_reason ==
            "alignment_context_profile_target_disallowed")
        assert not rejected.evidence.evidence_valid

        valid_circle = self._context(
            3, "drop_circle", "tent", 0, first_seen)
        accepted = self._drive(
            valid_circle, circle,
            lambda msg: (
                msg.context_valid and msg.evidence.evidence_valid and
                msg.semantic_target_id == 0 and
                msg.geometry_target_id == 42))
        assert accepted.semantic_geometry_match
        assert accepted.association_distance_m < 0.5
        assert accepted.context_header.stamp.to_sec() > 0.0
        assert accepted.context_source == "alignment_context_assertion"

        replacement = self._target(
            43, "circle", first_seen + rospy.Duration(0.1))
        changed_geometry = self._single(
            valid_circle, replacement,
            lambda msg: msg.geometry_target_id == 43)
        assert changed_geometry.context_valid
        assert changed_geometry.evidence.stable_frames == 1
        assert not changed_geometry.evidence.evidence_valid

        changed_decision = self._context(
            4, "drop_circle", "tent", 0, first_seen)
        changed_fence = self._single(
            changed_decision, replacement,
            lambda msg: msg.decision_seq == 4)
        assert changed_fence.context_valid
        assert changed_fence.evidence.stable_frames == 1
        assert not changed_fence.evidence.evidence_valid

        expired = self._context(
            5, "drop_circle", "tent", 0, first_seen,
            deadline=rospy.Time.now())
        rejected = self._single(
            expired, replacement,
            lambda msg: msg.context_reason ==
            "alignment_context_deadline_expired")
        assert rejected.evidence.stable_frames == 0
        assert not rejected.evidence.evidence_valid

        self._mode_pub.publish(String(data="drop_cross"))
        rospy.sleep(0.1)
        cross_first_seen = rospy.Time.now() - rospy.Duration(2.0)
        cross_context = self._context(
            6, "drop_cross", "red_cross", 0, cross_first_seen)
        cross = self._target(0, "red_cross", cross_first_seen)
        accepted = self._drive(
            cross_context, cross,
            lambda msg: (
                msg.context_valid and msg.evidence.evidence_valid and
                msg.semantic_target_id == 0 and
                msg.geometry_target_id == 0))
        assert accepted.semantic_geometry_match

        geometry_stale = self._drive_context_only(
            cross_context,
            lambda msg: msg.context_reason ==
            "alignment_context_geometry_stale")
        assert geometry_stale.evidence.stable_frames == 0
        assert not geometry_stale.evidence.evidence_valid

        accepted = self._drive(
            cross_context, cross,
            lambda msg: msg.context_valid and msg.evidence.evidence_valid)
        assert accepted.geometry_target_id == 0

        reused_id = self._target(
            0, "red_cross", cross_first_seen + rospy.Duration(0.1))
        rejected = self._drive(
            cross_context, reused_id,
            lambda msg: msg.context_reason ==
            "alignment_context_geometry_identity_mismatch")
        assert rejected.geometry_target_present
        assert rejected.geometry_target_id == 0
        assert rejected.evidence.stable_frames == 0
        assert not rejected.evidence.evidence_valid

        stale_context = self._context(
            7, "drop_cross", "red_cross", 0, cross_first_seen)
        fresh_once = self._single(
            stale_context, cross,
            lambda msg: msg.decision_seq == 7 and msg.context_valid)
        assert fresh_once.evidence.stable_frames == 1
        with self._condition:
            start_count = self._wrapped_count
        rejected = self._wait_wrapped(
            start_count,
            lambda msg: msg.context_reason == "alignment_context_stale")
        assert rejected.evidence.stable_frames == 0
        assert not rejected.evidence.evidence_valid

        rospy.loginfo(
            "[AlignmentContextAssertion] PASS strict context fencing, "
            "profile, lease and geometry identity")
        return 0


class AlignmentContextRostest(unittest.TestCase):
    def test_strict_alignment_context(self):
        try:
            result = AlignmentContextAssertion().run()
        except (AssertionError, RuntimeError) as error:
            self.fail(str(error))
        self.assertEqual(result, 0)


if __name__ == "__main__":
    rospy.init_node("alignment_context_assertion")
    rostest.rosrun(
        "uav_vision", "alignment_context", AlignmentContextRostest)
