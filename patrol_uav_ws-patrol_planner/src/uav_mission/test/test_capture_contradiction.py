#!/usr/bin/env python3
"""Production CAPTURE callbacks + executor + mission runtime, without ROS nodes."""
from dataclasses import replace
from types import SimpleNamespace as N
import unittest

import test_release_transaction_followups as f
from uav_mission.coverage_route import CoverageRoute
from uav_mission.mission_core import CandidateStatus, MissionConfig, MissionCore, ResultEvent, SlotStatus
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.planner_execution import MotionDecision, MotionGoal, PlannerMotionConfig, PlannerMotionExecutor, TargetIdentity
from uav_mission.search_types import Waypoint

NS = f.NS


def make_bridge():
    f.Clock.value = 100.0
    obj = f.bridge()
    core = MissionCore(f.competition_profile(), MissionConfig(
        early_return_enabled=False, target_action_timeout=60., retry_cooldown=20., max_attempts=2))
    runtime = MissionRuntime(core, CoverageRoute((Waypoint(5., 0., 1.2), Waypoint(5., 4., 1.2)), 'capture-test', 1))
    runtime.start('mission-1', 100., (0., 0.))
    # The first delivery is already committed, as in the flight being fixed.
    core.slots[0].status = SlotStatus.COMMITTED
    core.queue.delivered_classes.add('bridge')
    source = f.candidate(target_id=3, class_name='panzer', now=100.1)
    runtime.ingest((source,), 100.1)
    f.Clock.value = 100.1
    action = runtime.tick(100.1, (0., 0.)).action
    assert action.command == 'APPROACH' and action.payload_slot == 2
    obj.core, obj.runtime, obj.source = core, runtime, source
    obj._executor = PlannerMotionExecutor(PlannerMotionConfig(executor_id='test-bridge'))
    identity = TargetIdentity(source.target_id, source.first_seen_ns, source.last_seen_ns,
                              source.class_name, action.attempt, action.payload_slot)
    decision = MotionDecision('mission-1', action.decision_seq, round(action.issued_at * NS),
        round(action.deadline_at * NS), 'APPROACH', 'r2026',
        MotionGoal(action.goal.frame_id, action.goal.x, action.goal.y, action.goal.z), identity)
    obj.events, obj.reductions, obj.commands, obj.errors, obj.context_events = [], [], [], [], []
    obj._contradiction_min_frames, obj._contradiction_min_span_ns = 3, 150_000_000
    obj._contradiction_radius, obj._contradiction_confidence = .35, .90
    obj._capture_max_age_ns = NS
    obj._transaction = f.BR['TargetTransaction'](decision=decision)
    obj._publish_mission_command = lambda d, command_name, **kw: obj.commands.append((command_name, d.target.class_name))
    def apply(out):
        obj.events.extend(out.events)
        for event in out.events:
            fields = {name: getattr(event, name) for name in ResultEvent.__dataclass_fields__}
            obj.reductions.append(runtime.apply_result(ResultEvent(**fields), f.Clock.value, (1., 0.)))
    obj._apply_outcome = apply
    obj._apply_outcome(obj._executor.submit_decision(decision, round(100.1 * NS)))
    # Fixture starts at the observed approach_arrival_confirmed handoff. Planner
    # motion itself is covered by test_planner_execution, not reimplemented here.
    obj._executor._active.handed_off = True
    return obj


def candidate(obj, t, label='pillbox', **fields):
    c = N(id=obj.source.target_id, first_seen=f.Stamp(obj.source.first_seen_ns / NS),
          last_seen=f.Stamp(t), map_point=N(x=1., y=0., z=0.), class_name=label,
          class_confidence=.96, map_valid=True, association_valid=True,
          reject_reason='', state=2, map_frame='camera_init')
    for name, value in fields.items():
        setattr(c, name, value)
    return c


def observe(obj, t, c=None, ingest=True):
    f.Clock.value = t
    if c is not None and ingest:
        source = replace(obj.source, class_name=c.class_name, last_seen_ns=c.last_seen.to_nsec(),
                         class_confidence=c.class_confidence, map_valid=c.map_valid)
        obj.runtime.ingest((source,), t)
    obj._on_targets(N(targets=[] if c is None else [c]))


def arrive(obj, t=100.2):
    f.Clock.value = t
    obj._begin_target_capture(round(t * NS))


class CaptureContradictionTests(unittest.TestCase):
    def assert_safe_exit(self, obj):
        self.assertEqual(obj.errors, [])
        self.assertEqual(obj._transaction.phase, 'TERMINAL')
        self.assertEqual(obj.commands, [])
        self.assertEqual(obj.context_events, [])
        self.assertEqual(obj.core.slots[0].status, SlotStatus.COMMITTED)
        self.assertEqual(obj.core.slots[1].status, SlotStatus.FREE)
        self.assertEqual(obj.core.committed_slots, 1)
        self.assertEqual(obj.core.active_action.command, 'RESUME')
        self.assertEqual(obj.runtime.route.current_index, 0)
        entry = obj.core.queue.entries[obj.source.key]
        self.assertEqual(entry.status, CandidateStatus.COOLDOWN)
        self.assertIsNone(entry.reserved_snapshot)
        self.assertEqual(entry.snapshot.class_name, 'pillbox')
        event = obj.events[-1]
        self.assertEqual((event.status, event.stage, event.terminal, event.retryable), ('FAILED', 'CAPTURE', True, True))
        self.assertEqual(event.target_class, 'panzer')
        self.assertFalse(event.payload_committed)
        self.assertEqual(event.reason, 'capture_semantic_contradiction:pillbox')

    def test_class_changes_before_arrival_then_capture_exits_without_align(self):
        obj = make_bridge()
        for t in (100.2, 100.3, 100.4):
            observe(obj, t, candidate(obj, t))
        self.assertEqual(obj._transaction.phase, 'APPROACHING')
        self.assertEqual(obj.core.slots[1].status, SlotStatus.RESERVED)
        arrive(obj, 100.45)
        for t in (100.5, 100.6):
            observe(obj, t, candidate(obj, t))
        self.assert_safe_exit(obj)

    def test_class_changes_during_capture_then_old_callbacks_cannot_revive(self):
        obj = make_bridge(); arrive(obj)
        for t in (100.3, 100.4, 100.5):
            observe(obj, t, candidate(obj, t))
        self.assert_safe_exit(obj)
        count = len(obj.events)
        observe(obj, 100.6, candidate(obj, 100.6, 'panzer'))
        self.assertEqual(len(obj.events), count)
        self.assertEqual(obj.commands, [])
        self.assertEqual(obj.core.active_action.command, 'RESUME')

    def test_short_dropout_single_flip_then_original_class_can_align(self):
        obj = make_bridge(); arrive(obj)
        observe(obj, 100.3, candidate(obj, 100.3))
        observe(obj, 100.4)
        observe(obj, 100.5, candidate(obj, 100.5, 'panzer'))
        self.assertEqual(obj.errors, [])
        self.assertEqual(obj._transaction.phase, 'ALIGN_COMMAND_SENT')
        self.assertEqual(obj.commands, [('ALIGN', 'panzer')])
        self.assertEqual(obj.context_events, [True])
        self.assertEqual(obj.core.slots[1].status, SlotStatus.RESERVED)
        self.assertEqual(obj._transaction.contradiction_window.count, 0)

    def test_repeated_stale_invalid_neighbor_and_foreign_identity_do_not_cancel(self):
        for mode in ('repeat', 'stale', 'future', 'low_confidence', 'map_invalid',
                     'unconfirmed', 'foreign_frame', 'foreign_id', 'foreign_birth',
                     'neighbor', 'nonfinite', 'dropout'):
            with self.subTest(mode=mode):
                obj = make_bridge(); arrive(obj)
                for t in (100.3, 100.4, 100.5, 100.6):
                    c = candidate(obj, t)
                    if mode == 'repeat': c.last_seen = f.Stamp(100.3)
                    if mode == 'stale': c.last_seen = f.Stamp(98.)
                    if mode == 'future': c.last_seen = f.Stamp(t + 1.)
                    if mode == 'low_confidence': c.class_confidence = .8
                    if mode == 'map_invalid': c.map_valid = False
                    if mode == 'unconfirmed': c.state = 1
                    if mode == 'foreign_frame': c.map_frame = 'other'
                    if mode == 'foreign_id': c.id = 99
                    if mode == 'foreign_birth': c.first_seen = f.Stamp(98.)
                    if mode == 'neighbor': c.map_point.x = 2.
                    if mode == 'nonfinite': c.map_point.z = float('nan')
                    observe(obj, t, None if mode == 'dropout' else c, ingest=False)
                self.assertEqual(obj.errors, [])
                self.assertEqual(obj._transaction.phase, 'CAPTURE')
                self.assertEqual(obj.core.slots[1].status, SlotStatus.RESERVED)
                self.assertFalse(any(e.terminal for e in obj.events))

    def test_release_facts_or_alignment_context_prevent_direct_slot_release(self):
        for field, value in [('raw_call_observed', True), ('release_invocation_observed', True),
                             ('strict_evidence_stamp_ns', 1), ('release_execution_id', 1),
                             ('release_ack_ns', 1), ('target_pose', N())]:
            with self.subTest(field=field):
                obj = make_bridge(); arrive(obj)
                setattr(obj._transaction, field, value)
                for t in (100.3, 100.4, 100.5):
                    observe(obj, t, candidate(obj, t))
                self.assertEqual(obj._transaction.phase, 'CAPTURE')
                self.assertEqual(obj.core.slots[1].status, SlotStatus.RESERVED)
                self.assertFalse(any(e.terminal for e in obj.events))
                self.assertEqual(obj.context_events, [])

    def test_deadline_orders_release_slot_resume_and_exhaust_same_identity(self):
        for timer_first in (False, True):
            with self.subTest(timer_first=timer_first):
                obj = make_bridge(); arrive(obj)
                deadline = obj._transaction.decision.deadline_ns / NS
                f.Clock.value = deadline - 18.429
                self.assertEqual(obj.runtime.tick(f.Clock.value, (1., 0.)).reason, 'decision_pending')
                self.assertEqual(obj.core.slots[1].status, SlotStatus.RESERVED)
                f.Clock.value = deadline + .01
                if timer_first:
                    obj.runtime.tick(f.Clock.value, (1., 0.))
                obj._expire_handoff_if_due(round(f.Clock.value * NS))
                self.assertEqual(obj.errors, [])
                self.assertEqual(obj.core.slots[1].status, SlotStatus.FREE)
                self.assertEqual(obj.core.active_action.command, 'RESUME')
                self.assertEqual(obj.core.queue.entries[obj.source.key].status, CandidateStatus.EXHAUSTED)
                self.assertEqual(obj.events[-1].reason, 'target_capture_deadline_reached')
                self.assertEqual(obj.events[-1].status, 'TIMED_OUT')
                source = replace(obj.source, class_name='pillbox', last_seen_ns=round((f.Clock.value - .01) * NS))
                verdict = obj.runtime.ingest((source,), f.Clock.value)
                self.assertFalse(verdict.candidate_validations[0].accepted)
                self.assertEqual(verdict.candidate_validations[0].reason, 'candidate_terminal')
                self.assertEqual(obj.commands, [])

    def test_early_failure_cooldown_retries_corrected_class_once_with_new_identity(self):
        obj = make_bridge(); arrive(obj)
        for t in (100.3, 100.4, 100.5):
            observe(obj, t, candidate(obj, t))
        self.assert_safe_exit(obj)
        old_decision = obj._transaction.decision
        entry = obj.core.queue.entries[obj.source.key]
        self.assertAlmostEqual(entry.cooldown_until, 120.5)
        source = replace(obj.source, class_name='pillbox', last_seen_ns=round(120.50 * NS))
        f.Clock.value = 120.51
        obj.runtime.ingest((source,), f.Clock.value)
        # Complete the resumed route leg: ordinary low-priority candidates need
        # not interrupt it, but can be selected at the next scheduling point.
        resume = obj.core.active_action
        e = f.result_for(resume, obj.events[-1].event_seq + 1, status='SUCCEEDED',
                         stage='PLANNER', terminal=True, executor_id='test-bridge',
                         event_time=f.Clock.value, reason='waypoint_reached')
        next_out = obj.runtime.apply_result(e, f.Clock.value, (5., 0.))
        self.assertTrue(next_out.accepted, next_out.reason)
        self.assertEqual(next_out.action.command, 'SEARCH')
        self.assertEqual(entry.status, CandidateStatus.PENDING)
        # Pillbox is below the high-weight interrupt set. Finish coverage before
        # fallback selects the corrected class; cooldown is not an immediate retry.
        e = f.result_for(next_out.action, e.event_seq + 1, status='SUCCEEDED',
                         stage='PLANNER', terminal=True, executor_id='test-bridge',
                         event_time=120.52, reason='waypoint_reached')
        next_out = obj.runtime.apply_result(e, 120.52, (5., 4.))
        self.assertTrue(next_out.accepted, next_out.reason)
        action = next_out.action
        self.assertEqual((action.command, action.target_class, action.attempt, action.payload_slot),
                         ('APPROACH', 'pillbox', 2, 2))
        self.assertGreater(action.decision_seq, old_decision.decision_seq)
        self.assertEqual(old_decision.target.class_name, 'panzer')
        self.assertEqual(obj.core.slots[0].status, SlotStatus.COMMITTED)
        # A second failure is bounded; it cannot create an endless capture loop.
        e2 = f.result_for(action, e.event_seq + 1, status='FAILED', stage='CAPTURE',
                          terminal=True, retryable=True, executor_id='test-bridge',
                          event_time=120.53, reason='capture_semantic_contradiction:panzer')
        ended = obj.runtime.apply_result(e2, 120.53, (1., 0.))
        self.assertTrue(ended.accepted, ended.reason)
        self.assertEqual(entry.status, CandidateStatus.EXHAUSTED)
        self.assertEqual(obj.core.slots[1].status, SlotStatus.FREE)
        self.assertEqual(obj.core.active_action.command, 'RETURN_HOME')


if __name__ == '__main__':
    unittest.main()