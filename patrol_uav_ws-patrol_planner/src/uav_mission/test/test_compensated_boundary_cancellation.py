#!/usr/bin/env python3
"""Exercise production Bridge -> proxy -> executor -> mission core without ROS nodes."""
import ast
import copy
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_release_transaction_followups as fixtures
from uav_mission.mission_core import CandidateStatus, MissionPhase, SlotStatus
from uav_mission.release_transactions import NOT_STARTED, RAW_CALL_STARTED, action_identity

REASON = "compensated_target_outside_boundary"


class AlignmentContext:
    SCHEMA_VERSION = 1
    ALIGN = "ALIGN"

    def __init__(self):
        self.header = N()
        self.target_pose = N(header=N(), pose=N(position=N(), orientation=N()))


Bridge = fixtures.classes(
    "navigation_planner_bridge.py", ["NavigationPlannerBridge"],
    dict(fixtures.BR, AlignmentTargetContext=AlignmentContext,
         _ns_to_stamp=lambda ns: fixtures.Stamp(ns / fixtures.NS))
)["NavigationPlannerBridge"]


def bridge():
    obj = fixtures.bridge()
    obj.__class__ = Bridge
    obj._context_max_age = .5
    obj._association_distance = .8
    obj._executor_id = "test-bridge"
    obj._alignment_context_pub = fixtures.Publisher()
    del obj._publish_alignment_context  # Use actual production context publication.
    return obj


def feedback(obj):
    tx, now = obj._transaction, fixtures.Clock.value
    target = tx.decision.target
    return N(
        header=N(frame_id="camera_init", stamp=fixtures.Stamp(now)),
        observation_stamp=fixtures.Stamp(100.06), odom_stamp=fixtures.Stamp(now),
        mission_id=tx.decision.mission_id, decision_seq=tx.decision.decision_seq,
        attempt=target.attempt, payload_slot=target.payload_slot,
        semantic_target_id=target.target_id,
        semantic_target_first_seen=fixtures.Stamp(target.first_seen_ns / fixtures.NS),
        semantic_target_class=target.class_name, align_mode=tx.align_mode,
        valid=False, aligned=False, frozen=False,
        target_fc=N(x=1.12, y=0., z=1.18), reason=REASON)


class BoundaryCancellationTests(unittest.TestCase):
    def setUp(self):
        fixtures.Clock.value = 100.1

    def cancel_and_ack(self, obj, proxy):
        obj._on_drop_alignment_feedback(feedback(obj))
        self.assertEqual(obj._transaction.phase, "CANCEL_PENDING")
        self.assertEqual(obj._transaction.cancellation_reason, REASON)
        self.assertEqual(obj.core.slots[0].status, SlotStatus.RESERVED)
        self.assertIsNotNone(obj.core.active_action)
        ctx = obj._alignment_context_pub.messages[-1]
        self.assertFalse(ctx.active)
        self.assertEqual(ctx.command, AlignmentContext.ALIGN)
        self.assertEqual(ctx.target_pose.header.frame_id, "camera_init")
        proxy._on_alignment_context(ctx)
        self.assertEqual(len(proxy._result_pub.messages), 1)
        receipt = proxy._result_pub.messages[-1]
        self.assertEqual(receipt.execution_state, NOT_STARTED)
        self.assertTrue(receipt.terminal)
        self.assertEqual(action_identity(receipt),
                         action_identity(fixtures.permission()))
        obj._on_release_result(receipt)
        self.assertEqual(obj.errors, [])
        self.assertEqual(obj._transaction.phase, "TERMINAL")
        self.assertEqual(obj.core.phase, MissionPhase.SEARCH)
        self.assertIsNone(obj.core.active_action)
        self.assertEqual(obj.core.slots[0].status, SlotStatus.FREE)
        self.assertEqual(proxy._raw_client.calls, 0)
        self.assertFalse(proxy._locked_slots)
        event = obj.events[-1]
        self.assertEqual((event.status, event.stage, event.reason),
                         ("FAILED", "ALIGNMENT", REASON))
        self.assertTrue(event.retryable)
        self.assertFalse(event.payload_committed)
        return receipt

    def test_no_permission_cancel_receipt_frees_slot_and_next_target_can_start(self):
        obj, proxy = bridge(), fixtures.proxy()
        obj._transaction.phase = "ALIGN_COMMAND_SENT"
        obj._transaction.strict_evidence_stamp_ns = 0
        proxy._permission = None
        original = obj.core.active_action
        receipt = self.cancel_and_ack(obj, proxy)
        entry = obj.core.queue.entries[original.candidate_key]
        self.assertEqual(entry.status, CandidateStatus.COOLDOWN)
        self.assertAlmostEqual(entry.cooldown_until,
                               fixtures.Clock.value + obj.core.config.retry_cooldown)
        self.assertLess(fixtures.Clock.value, original.deadline_at)
        # Ordinary scheduling can reserve another candidate immediately.
        candidate = fixtures.candidate(target_id=8, class_name="panzer", now=100.1)
        action = obj.core.choose_confirmed(candidate, 100.1, (1., 0.))
        self.assertIsNotNone(action)
        self.assertEqual((action.candidate_key.target_id, action.payload_slot), (8, 1))
        # A replay of the old proxy receipt cannot free the new reservation.
        obj._on_release_result(receipt)
        self.assertEqual(obj.core.slots[0].status, SlotStatus.RESERVED)

    def test_denied_permission_also_cancels_before_raw_execution(self):
        obj, proxy = bridge(), fixtures.proxy()
        proxy._permission.permitted = False
        self.cancel_and_ack(obj, proxy)

    def test_old_latched_image_with_fresh_report_still_cancels(self):
        fixtures.Clock.value = 102.
        obj, proxy = bridge(), fixtures.proxy()
        self.assertGreater(fixtures.Clock.value - feedback(obj).observation_stamp.to_sec(), .5)
        self.cancel_and_ack(obj, proxy)

    def test_cross_mode_uses_same_production_cancellation_path(self):
        obj, proxy = bridge(), fixtures.proxy()
        obj._transaction.align_mode = "drop_cross"
        obj._on_drop_alignment_feedback(feedback(obj))
        proxy._on_alignment_context(obj._alignment_context_pub.messages[-1])
        obj._on_release_result(proxy._result_pub.messages[-1])
        self.assertEqual(obj._transaction.phase, "TERMINAL")
        self.assertEqual(obj.core.slots[0].status, SlotStatus.FREE)
        self.assertEqual(obj.errors, [])

    def assert_ignored(self, mutate_message=None, mutate_bridge=None):
        obj = bridge()
        msg = feedback(obj)
        if mutate_message:
            mutate_message(msg)
        if mutate_bridge:
            mutate_bridge(obj)
        old_phase = obj._transaction.phase if obj._transaction else None
        obj._on_drop_alignment_feedback(msg)
        self.assertEqual(obj._alignment_context_pub.messages, [])
        self.assertEqual(obj.events, [])
        self.assertEqual(obj.errors, [])
        self.assertEqual(obj.core.slots[0].status, SlotStatus.RESERVED)
        if obj._transaction:
            self.assertEqual(obj._transaction.phase, old_phase)
            self.assertEqual(obj._transaction.cancellation_reason, "")

    def test_foreign_identity_frame_mode_and_flags_are_ignored(self):
        for field, value in (
                ("mission_id", "old"), ("decision_seq", 2), ("attempt", 2),
                ("payload_slot", 2), ("semantic_target_id", 8),
                ("semantic_target_first_seen", fixtures.Stamp(98.)),
                ("semantic_target_class", "panzer"), ("align_mode", "drop_cross"),
                ("reason", "not_ready"), ("valid", True),
                ("aligned", True), ("frozen", True)):
            with self.subTest(field=field):
                self.assert_ignored(lambda msg: setattr(msg, field, value))
        self.assert_ignored(lambda msg: setattr(msg.header, "frame_id", "navigation"))
        self.assert_ignored(mutate_bridge=lambda obj: setattr(
            obj._transaction, "target_pose",
            fixtures.BR["SemanticTargetPose"]("navigation", 1., 0., 0., 100000000000)))

    def test_report_odom_and_attempt_timestamp_constraints(self):
        for field, values in (
                ("header", (0., 99.99, 100.100001)),
                ("observation_stamp", (0., 99.99, 100.100001)),
                ("odom_stamp", (0., 99.99, 100.100001))):
            for stamp in values:
                with self.subTest(field=field, stamp=stamp):
                    if field == "header":
                        self.assert_ignored(lambda m: setattr(m.header, "stamp", fixtures.Stamp(stamp)))
                    else:
                        self.assert_ignored(lambda m: setattr(m, field, fixtures.Stamp(stamp)))
        # Stale report or current odometry, despite an otherwise valid image.
        fixtures.Clock.value = 102.
        self.assert_ignored(lambda m: setattr(m.header, "stamp", fixtures.Stamp(101.4)))
        self.assert_ignored(lambda m: setattr(m, "odom_stamp", fixtures.Stamp(100.06)))
        # Future odometry relative to the report cannot describe that evaluation.
        self.assert_ignored(lambda m: setattr(m.header, "stamp", fixtures.Stamp(101.9)))

    def test_nonfinite_coordinate_and_malformed_feedback_are_ignored(self):
        for field, value in (("x", float("nan")), ("y", float("inf")),
                             ("z", float("nan"))):
            with self.subTest(field=field, value=value):
                self.assert_ignored(lambda m: setattr(m.target_fc, field, value))
        self.assert_ignored(lambda m: delattr(m, "observation_stamp"))

    def test_compensated_fc_beyond_semantic_association_distance_still_cancels(self):
        obj, proxy = bridge(), fixtures.proxy()
        msg = feedback(obj)
        msg.target_fc.x = obj._transaction.target_pose.x + .75 + .12
        self.assertGreater(msg.target_fc.x - obj._transaction.target_pose.x,
                           obj._association_distance)
        obj._on_drop_alignment_feedback(msg)
        self.assertEqual(obj._transaction.phase, "CANCEL_PENDING")
        proxy._on_alignment_context(obj._alignment_context_pub.messages[-1])
        obj._on_release_result(proxy._result_pub.messages[-1])
        self.assertEqual(obj._transaction.phase, "TERMINAL")
        self.assertEqual(obj.core.slots[0].status, SlotStatus.FREE)
        self.assertEqual(obj.events[-1].reason, REASON)
        self.assertEqual(obj.errors, [])

    def test_disabled_wrong_stage_expired_and_raw_started_are_ignored(self):
        self.assert_ignored(mutate_bridge=lambda obj: setattr(obj, "_output_enabled", False))
        self.assert_ignored(mutate_bridge=lambda obj: setattr(obj, "_transaction", None))
        self.assert_ignored(mutate_bridge=lambda obj: setattr(obj._transaction, "raw_call_observed", True))
        self.assert_ignored(mutate_bridge=lambda obj: setattr(obj._transaction, "target_pose", None))
        for phase in ("APPROACHING", "CAPTURE", "TERMINAL", "EXPIRED", "RELEASE_UNCERTAIN"):
            with self.subTest(phase=phase):
                self.assert_ignored(mutate_bridge=lambda obj: setattr(obj._transaction, "phase", phase))
        obj = bridge()
        fixtures.Clock.value = obj._transaction.decision.deadline_ns / fixtures.NS
        obj._on_drop_alignment_feedback(feedback(obj))
        self.assertEqual(obj._alignment_context_pub.messages, [])

    def test_duplicate_reports_and_wrong_attempt_receipt_do_not_free_slot(self):
        obj, proxy = bridge(), fixtures.proxy()
        msg = feedback(obj)
        obj._on_drop_alignment_feedback(msg)
        obj._on_drop_alignment_feedback(msg)
        self.assertEqual(len(obj._alignment_context_pub.messages), 1)
        ctx = obj._alignment_context_pub.messages[-1]
        proxy._on_alignment_context(ctx)
        proxy._on_alignment_context(ctx)
        self.assertEqual(len(proxy._result_pub.messages), 1)
        receipt = proxy._result_pub.messages[-1]
        wrong = copy.deepcopy(receipt)
        wrong.attempt += 1
        obj._on_release_result(wrong)
        self.assertEqual(obj._transaction.phase, "CANCEL_PENDING")
        self.assertEqual(obj.core.slots[0].status, SlotStatus.RESERVED)
        obj._on_release_result(receipt)
        self.assertEqual(obj.core.slots[0].status, SlotStatus.FREE)

    def test_unobserved_raw_race_cannot_manufacture_no_start_or_unlock_slot(self):
        obj, proxy = bridge(), fixtures.proxy(fixtures.Raw(success=False))
        proxy._on_servo_request(N(req=1))
        self.assertEqual(proxy._raw_client.calls, 1)
        obj._on_drop_alignment_feedback(feedback(obj))
        proxy._on_alignment_context(obj._alignment_context_pub.messages[-1])
        self.assertNotIn(NOT_STARTED, [m.execution_state for m in proxy._result_pub.messages])
        for msg in proxy._result_pub.messages:
            obj._on_release_result(msg)
        self.assertTrue(obj._transaction.raw_call_observed)
        self.assertEqual(obj.core.slots[0].status, SlotStatus.QUARANTINED)
        self.assertEqual(proxy._locked_slots, {1})
        late_false_proof = fixtures.result(fixtures.permission(), NOT_STARTED, stamp=100.1)
        obj._on_release_result(late_false_proof)
        self.assertEqual(obj.core.slots[0].status, SlotStatus.QUARANTINED)
        self.assertEqual(obj.errors, [])

    def test_production_subscriber_uses_parameter_then_controller_topic_then_default(self):
        tree = ast.parse((fixtures.P / "scripts/navigation_planner_bridge.py").read_text())
        init = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "__init__"
                    and any(isinstance(a, ast.Assign) and any(
                        isinstance(t, ast.Attribute) and t.attr == "_drop_alignment_feedback_sub"
                        for t in a.targets) for a in n.body))
        assignment = next(a for a in init.body if isinstance(a, ast.Assign) and any(
            isinstance(t, ast.Attribute) and t.attr == "_drop_alignment_feedback_sub"
            for t in a.targets))
        for params, topic in (
                ({}, "/uav_vision/drop_alignment_feedback"),
                ({"/drop_system/alignment_feedback_topic": "/control/feedback"}, "/control/feedback"),
                ({"/drop_system/alignment_feedback_topic": "/control/feedback",
                  "~target/alignment_feedback_topic": "/bridge/feedback"}, "/bridge/feedback")):
            obj = bridge()
            calls = []
            ros = N(get_param=lambda key, default: params.get(key, default),
                    Subscriber=lambda *args, **kwargs: calls.append((args, kwargs)))
            exec(compile(ast.Module(body=[assignment], type_ignores=[]), "subscriber", "exec"),
                 dict(self=obj, rospy=ros, DropAlignmentFeedback=N))
            args, kwargs = calls[0]
            self.assertEqual(args[0], topic)
            self.assertEqual(args[2], obj._on_drop_alignment_feedback)
            self.assertEqual(kwargs["queue_size"], 4)


if __name__ == "__main__":
    unittest.main()
