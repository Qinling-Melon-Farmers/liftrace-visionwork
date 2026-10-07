"""Production proxy -> Bridge -> Core retry regressions; no ROS/hardware."""
import copy
from dataclasses import replace
from types import SimpleNamespace as N
import threading
import unittest

import test_release_transaction_followups as transport
import test_high_view_full as full
from test_mission_core import candidate, result_for
from test_mission_runtime import candidate as runtime_candidate, result_for as runtime_result
from uav_mission.mission_core import CandidateStatus, SlotStatus

TEMPORAL = ('permission_clock_ahead', 'permission_expired', 'permission_stale')


class TemporalRetryTests(unittest.TestCase):
    def setUp(self):
        transport.Clock.value = 100.1

    def reject(self, reason='permission_clock_ahead', **changes):
        o = transport.bridge()
        message = transport.result(transport.permission(), transport.NOT_STARTED)
        message.reason = reason
        for key, value in changes.items():setattr(message, key, value)
        o._on_release_result(message)
        self.assertEqual(o.errors, [])
        return o, message

    def fresh(self, stamp=100.2):
        return replace(candidate(7, now=stamp), first_seen_ns=99_000_000_000)

    def test_each_explicit_temporal_no_call_bypasses_cooldown_without_resetting_attempt(self):
        for reason in TEMPORAL:
            with self.subTest(reason=reason):
                o, message = self.reject(reason)
                entry = o.core.queue.entries[self.fresh().key]
                self.assertEqual(entry.status, CandidateStatus.PENDING)
                self.assertEqual(entry.attempts, 1)
                self.assertEqual(entry.last_result, 'release_preflight_rejected:' + reason)
                event = o.events[-1]
                self.assertEqual(event.evidence_source, 'guarded_servo_proxy:1:NOT_STARTED')
                self.assertEqual(event.reason, entry.last_result)
                self.assertTrue(event.terminal and event.retryable)
                self.assertFalse(event.payload_committed)
                action = o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.))
                self.assertEqual(action.attempt, 2)
                self.assertEqual(action.candidate_key, self.fresh().key)
                self.assertEqual(action.payload_slot, 1)
                self.assertLessEqual(action.deadline_at, 700.)
                self.assertEqual(o.core.started_at, 100.)

    def test_actual_proxy_fact_and_reason_survive_bridge_into_core(self):
        p = transport.proxy()
        p._permission_refresh_wait = 0.
        p._permission.header.stamp = transport.Stamp(100.101)
        request = copy.deepcopy(p._permission);request.request_id = 9
        response = p._on_servo_action_request(request)
        self.assertEqual(response.reason, 'permission_clock_ahead')
        self.assertEqual(response.execution_state, transport.NOT_STARTED)
        self.assertEqual(p._raw_client.calls, 0)
        o = transport.bridge();o._on_release_result(p._result_pub.messages[-1])
        self.assertEqual(o.errors, [])
        self.assertEqual(o.core.queue.entries[self.fresh().key].status, CandidateStatus.PENDING)
        self.assertEqual(o.events[-1].reason, 'release_preflight_rejected:permission_clock_ahead')

    def test_actual_fresh_second_attempt_calls_raw_once_and_commits_once(self):
        p = transport.proxy();p._permission_refresh_wait = 0.
        p._permission.header.stamp = transport.Stamp(100.101)
        old = copy.deepcopy(p._permission);old.request_id = 20
        p._on_servo_action_request(old)
        o = transport.bridge();o._on_release_result(p._result_pub.messages[-1])
        action = o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.))
        self.assertIsNotNone(action)
        p._on_alignment_context(transport.context(old))
        target = o._transaction.decision.target
        decision = replace(o._transaction.decision, decision_seq=action.decision_seq,
            issued_at_ns=100_200_000_000, deadline_ns=round(action.deadline_at * transport.NS),
            target=replace(target, attempt=2, observation_ns=self.fresh().last_seen_ns))
        self.assertTrue(o._executor.submit_decision(decision, 100_200_000_000).accepted)
        o._executor._active.handed_off = True
        o._transaction = transport.BR['TargetTransaction'](decision=decision, phase='ALIGNMENT',
            target_pose=transport.BR['SemanticTargetPose']('camera_init', 1., 0., 0., 100_150_000_000),
            align_mode='drop_circle', strict_evidence_stamp_ns=100_150_000_000)
        new = transport.permission(seq=action.decision_seq, attempt=2)
        new.header.stamp = transport.Stamp(100.2);new.evidence_stamp = transport.Stamp(100.15)
        transport.Clock.value = 100.2;p._on_permission(new);new.request_id = 21
        start = len(p._result_pub.messages)
        self.assertTrue(p._on_servo_action_request(new).res)
        for message in p._result_pub.messages[start:]:o._on_release_result(message)
        o._on_release_result(p._result_pub.messages[-1])
        self.assertEqual(o.errors, [])
        self.assertEqual(o.core.committed_slots, 1)
        self.assertEqual(len([e for e in o.events if e.payload_committed]), 1)
        self.assertFalse(p._on_servo_action_request(new).res)
        self.assertEqual(p._raw_client.calls, 1)

    def test_foreign_or_old_revocation_cannot_revoke_new_fenced_attempt(self):
        p = transport.proxy();new = transport.permission(seq=2, attempt=2)
        p._on_permission(new)
        p._on_alignment_context(transport.context(transport.permission(seq=1)))
        foreign = transport.permission(seq=2, attempt=2, target_id=8)
        p._on_alignment_context(transport.context(foreign))
        new.request_id = 22
        self.assertTrue(p._on_servo_action_request(new).res)
        self.assertEqual(p._raw_client.calls, 1)

    def test_geometry_revoke_service_unavailable_and_unknown_reason_keep_twenty_seconds(self):
        for reason in ('release_altitude_invalid', 'permission_denied', 'alignment_context_revoked',
                       'permission_changed_before_call', 'raw_service_not_available', '', 'unknown'):
            with self.subTest(reason=reason):
                o, _ = self.reject(reason)
                entry = o.core.queue.entries[self.fresh().key]
                self.assertEqual(entry.status, CandidateStatus.COOLDOWN)
                self.assertAlmostEqual(entry.cooldown_until, 120.1)
                self.assertIsNone(o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.)))

    def test_cancellation_reason_overrides_temporal_message(self):
        o = transport.bridge();o._transaction.cancellation_reason = 'visual_identity_changed'
        message = transport.result(transport.permission(), transport.NOT_STARTED)
        message.reason = 'permission_clock_ahead';o._on_release_result(message)
        self.assertEqual(o.events[-1].reason, 'visual_identity_changed')
        self.assertEqual(o.core.queue.entries[self.fresh().key].status, CandidateStatus.COOLDOWN)

    def test_foreign_identity_or_nonterminal_no_start_cannot_enable_retry(self):
        for changes in (dict(mission_id='foreign'), dict(decision_seq=2), dict(attempt=2),
                        dict(payload_slot=2), dict(target_id=8), dict(target_first_seen=transport.Stamp(98.)),
                        dict(target_class='panzer'), dict(align_mode='drop_cross'), dict(terminal=False)):
            with self.subTest(changes=changes):
                o, _ = self.reject(**changes)
                self.assertEqual(o.core.queue.entries[self.fresh().key].status, CandidateStatus.EXECUTING)
                self.assertEqual(o.core.slots[0].status, SlotStatus.RESERVED)
                self.assertIsNone(o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.)))

    def test_core_requires_complete_no_call_source_and_terminal_proof(self):
        for source in ('guarded_servo_proxy:0:NOT_STARTED', 'guarded_servo_proxy:x:NOT_STARTED',
                       'guarded_servo_proxy:1:extra:NOT_STARTED', 'other:1:NOT_STARTED',
                       'guarded_servo_proxy:1'):
            with self.subTest(source=source):
                core = transport.core();action = core.choose_confirmed(candidate(7), 100., (0., 0.))
                event = replace(result_for(action, 1, status='FAILED', stage='ALIGNMENT', terminal=True,
                    retryable=True, reason='release_preflight_rejected:permission_clock_ahead'), evidence_source=source)
                self.assertTrue(core.apply_result(event, 100.1)[0])
                self.assertEqual(core.queue.entries[action.candidate_key].status, CandidateStatus.COOLDOWN)

    def test_duplicate_terminal_and_revocation_do_not_free_a_new_attempt(self):
        o, message = self.reject()
        count = len(o.events)
        o._on_release_result(message)
        revoked = copy.deepcopy(message);revoked.execution_id = 2;revoked.reason = 'alignment_context_revoked'
        o._on_release_result(revoked)
        self.assertEqual(len(o.events), count)
        old = o.events[-1]
        action = o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.))
        event = transport.ResultEvent(**{k:getattr(old, k) for k in transport.ResultEvent.__dataclass_fields__})
        self.assertFalse(o.core.apply_result(event, 100.2)[0])
        self.assertFalse(o.core.apply_result(replace(event, event_seq=event.event_seq+1), 100.2)[0])
        self.assertIs(o.core.active_action, action)
        self.assertEqual(o.core.slots[0].status, SlotStatus.RESERVED)
        self.assertEqual(o.core.queue.entries[action.candidate_key].attempts, 2)

    def test_second_temporal_failure_exhausts_attempts(self):
        o, _ = self.reject();a = o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.))
        event = replace(result_for(a, o.events[-1].event_seq+1, status='FAILED', stage='ALIGNMENT',
            terminal=True, retryable=True, reason='release_preflight_rejected:permission_clock_ahead',
            executor_id=o.core.executor_id, event_time=100.21), evidence_source='guarded_servo_proxy:2:NOT_STARTED')
        self.assertTrue(o.core.apply_result(event, 100.21)[0])
        self.assertEqual(o.core.queue.entries[a.candidate_key].status, CandidateStatus.EXHAUSTED)
        self.assertIsNone(o.core.choose_confirmed(self.fresh(100.3), 100.3, (0., 0.)))

    def test_retry_requires_fresh_visual_tf_and_noninvalidated_candidate(self):
        for mode in ('stale', 'tf', 'identity', 'invalidated'):
            with self.subTest(mode=mode):
                o, _ = self.reject();c = self.fresh();now = 100.2
                if mode == 'stale':now = 101.
                if mode == 'tf':c = replace(c, transform_age_sec=.501)
                if mode == 'identity':c = replace(c, association_valid=False)
                if mode == 'invalidated':o.core.queue.invalidate(c.key, 'class_disproved')
                self.assertIsNone(o.core.choose_confirmed(c, now, (0., 0.)))
                self.assertEqual(o.core.slots[0].status, SlotStatus.FREE)

    def test_total_deadline_and_nonretryable_no_start_are_not_bypassed(self):
        o, _ = self.reject()
        self.assertIsNone(o.core.choose_confirmed(self.fresh(700.), 700., (0., 0.)))
        core = transport.core();a = core.choose_confirmed(candidate(7), 100., (0., 0.))
        event = replace(result_for(a, 1, status='FAILED', stage='ALIGNMENT', terminal=True,
            reason='release_preflight_rejected:permission_clock_ahead'), evidence_source='guarded_servo_proxy:1:NOT_STARTED')
        self.assertTrue(core.apply_result(event, 100.1)[0])
        self.assertEqual(core.queue.entries[a.candidate_key].status, CandidateStatus.EXHAUSTED)

    def test_raw_started_then_temporal_no_start_cannot_unlock_or_retry(self):
        o = transport.bridge()
        started = transport.result(transport.permission(), transport.RAW_CALL_STARTED, terminal=False)
        o._on_release_result(started)
        denial = transport.result(transport.permission(), transport.NOT_STARTED, execution_id=2)
        denial.reason = 'permission_clock_ahead';o._on_release_result(denial)
        self.assertEqual(o.core.queue.entries[self.fresh().key].status, CandidateStatus.EXECUTING)
        self.assertTrue(o._transaction.raw_call_observed)
        self.assertIsNone(o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.)))

    def test_completed_then_temporal_no_start_keeps_committed_slot(self):
        o = transport.bridge();o._on_release_result(transport.result(transport.permission()))
        denial = transport.result(transport.permission(), transport.NOT_STARTED, execution_id=2)
        denial.reason = 'permission_stale';o._on_release_result(denial)
        self.assertEqual(o.core.committed_slots, 1)
        self.assertEqual(o.core.slots[0].status, SlotStatus.COMMITTED)
        self.assertEqual(len([e for e in o.events if e.payload_committed]), 1)

    def test_unknown_execution_stays_quarantined_even_with_temporal_reason(self):
        o = transport.bridge()
        message = transport.result(transport.permission(), transport.EXECUTION_UNKNOWN)
        message.reason = 'permission_clock_ahead';o._on_release_result(message)
        self.assertEqual(o.core.slots[0].status, SlotStatus.QUARANTINED)
        self.assertIsNone(o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.)))

    def test_core_raw_fact_latches_even_if_later_no_start_claim_is_temporal(self):
        core = transport.core();a = core.choose_confirmed(candidate(7), 100., (0., 0.))
        core.apply_result(result_for(a, 1, status='STARTED', stage='RELEASE'), 100.1)
        denial = replace(result_for(a, 2, status='FAILED', stage='ALIGNMENT', terminal=True, retryable=True,
            reason='release_preflight_rejected:permission_clock_ahead'), evidence_source='guarded_servo_proxy:2:NOT_STARTED')
        self.assertTrue(core.apply_result(denial, 100.2)[0])
        self.assertEqual(core.slots[0].status, SlotStatus.QUARANTINED)

    def test_late_old_raw_or_success_cannot_change_new_decision(self):
        o, _ = self.reject();new = o.core.choose_confirmed(self.fresh(), 100.2, (0., 0.))
        count = len(o.events)
        for fact, terminal in ((transport.RAW_CALL_STARTED, False), (transport.COMPLETED, True)):
            message = transport.result(transport.permission(), fact, terminal=terminal, execution_id=3)
            o._on_release_result(message)
        self.assertEqual(len(o.events), count)
        self.assertIs(o.core.active_action, new)
        self.assertEqual(o.core.queue.entries[new.candidate_key].attempts, 2)

    def test_proxy_late_old_execution_blocks_second_raw_under_new_attempt(self):
        # Adversarial ordering: temporal denial, then an old call physically
        # starts before revocation arrives. Slot fencing must still prevent
        # a new decision/attempt from calling the actuator twice.
        for raw_success in (True, False):
            with self.subTest(raw_success=raw_success):
                p = transport.proxy(transport.Raw(success=raw_success));p._permission_refresh_wait = 0.
                p._permission.header.stamp = transport.Stamp(100.101)
                req = copy.deepcopy(p._permission);req.request_id = 10
                self.assertEqual(p._on_servo_action_request(req).execution_state, transport.NOT_STARTED)
                transport.Clock.value = 100.101
                p._on_servo_action_request(req)
                self.assertEqual(p._raw_client.calls, 1)
                p._on_alignment_context(transport.context(req))
                new = transport.permission(seq=2, attempt=2)
                new.header.stamp = transport.Stamp(100.102);transport.Clock.value = 100.102
                p._on_permission(new);new.request_id = 11
                response = p._on_servo_action_request(new)
                self.assertFalse(response.res)
                self.assertEqual(response.execution_state, transport.EXECUTION_UNKNOWN)
                self.assertEqual(p._raw_client.calls, 1)
                self.assertIn(1, p._locked_slots)
                transport.Clock.value = 100.1

    def test_inflight_late_old_raw_blocks_new_attempt_before_ack(self):
        entered = threading.Event();release = threading.Event()
        class SlowRaw(transport.Raw):
            def __call__(self, slot):
                self.calls += 1;entered.set()
                if not release.wait(1.):raise RuntimeError('test ACK timeout')
                return N(res=True)
        p = transport.proxy(SlowRaw());p._permission_refresh_wait = 0.
        p._permission.header.stamp = transport.Stamp(100.101)
        old = copy.deepcopy(p._permission);old.request_id = 30
        self.assertEqual(p._on_servo_action_request(old).execution_state, transport.NOT_STARTED)
        transport.Clock.value = 100.101
        answers = []
        thread = threading.Thread(target=lambda: answers.append(p._on_servo_action_request(old)))
        thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            new = transport.permission(seq=2, attempt=2);new.request_id = 31
            new.header.stamp = transport.Stamp(100.102);transport.Clock.value = 100.102
            p._on_permission(new)
            denied = p._on_servo_action_request(new)
            self.assertFalse(denied.res)
            self.assertEqual(denied.execution_state, transport.EXECUTION_UNKNOWN)
            self.assertEqual(p._raw_client.calls, 1)
        finally:
            release.set();thread.join(1.)
        self.assertFalse(thread.is_alive())
        self.assertTrue(answers[0].res)
        self.assertIn(1, p._completed_slots)


class RuntimeRetryTests(unittest.TestCase):
    setUp = full.FullTests.setUp
    finish = full.FullTests.finish
    top3 = full.FullTests.top3
    map = full.FullTests.map
    to_capture = full.FullTests.to_capture

    def test_full_production_reacquire_retries_without_twenty_second_cooldown(self):
        self.to_capture();hint = self.r.selected
        # Same shape as old seed38: two classes delivered, one slot and the
        # selected real identity remain. These are prior-flight fixture facts.
        self.r.core.queue.delivered_classes = self.r.required - {hint.class_name}
        for slot in self.r.core.slots[:2]:slot.status = SlotStatus.COMMITTED
        self.r.update_pose((*hint.xy, 1.18), 114., 'camera_init')
        c = replace(runtime_candidate(target_id=hint.key.target_id, class_name=hint.class_name,
            now=114., x=hint.xy[0], y=hint.xy[1]), first_seen_ns=hint.key.first_seen_ns)
        self.r.ingest([c], 114.)
        first = self.r.tick(114.1, hint.xy).action
        self.assertEqual(first.command, 'APPROACH')
        self.seq += 1
        event = replace(runtime_result(first, self.seq, status='FAILED', stage='ALIGNMENT', terminal=True,
            retryable=True, reason='release_preflight_rejected:permission_clock_ahead'),
            event_stamp_ns=114_200_000_000, evidence_source='guarded_servo_proxy:1:NOT_STARTED')
        out = self.r.apply_result(event, 114.2, hint.xy)
        self.assertEqual(self.r.stage, 'REVISIT')
        self.assertEqual(self.r.selected.key, hint.key)
        self.assertEqual(out.action.command, 'SEARCH')
        self.finish(115.)
        self.r.update_pose((*hint.xy, 1.18), 115.1, 'camera_init')
        fresh = replace(c, last_seen_ns=115_080_000_000)
        self.r.ingest([fresh], 115.1)
        out = self.r.tick(115.15, hint.xy)
        self.assertIsNotNone(out.action)
        self.assertEqual(out.action.command, 'APPROACH')
        self.assertEqual(out.action.candidate_key, first.candidate_key)
        self.assertEqual(out.action.attempt, 2)
        self.assertLess(out.action.issued_at, 134.2)
        self.assertEqual(self.r.stage, 'DELIVERY')
        self.assertFalse(any(e['stage'] in ('LOW_COVERAGE_HANDOFF', 'UNCONFIRMED_LOCATION_RETIRED') for e in self.r.events))
        self.assertEqual(self.r.core.started_at, 100.)
        self.assertLessEqual(out.action.deadline_at, 700.)


if __name__ == '__main__':unittest.main()
