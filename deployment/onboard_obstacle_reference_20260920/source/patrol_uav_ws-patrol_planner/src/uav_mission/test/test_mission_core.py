#!/usr/bin/env python3

import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
import tempfile

from uav_mission.mission_core import (
    CandidateSnapshot,
    CandidateStatus,
    GoalSnapshot,
    MissionConfig,
    MissionCore,
    MissionPhase,
    ResultEvent,
    SlotStatus,
    validate_candidate,
)
from uav_mission.profile_policy import CompetitionProfile, load_profile


NSEC = 1_000_000_000


def competition_profile():
    return CompetitionProfile(
        name="r2026",
        weights={
            "tent": 1.0,
            "pillbox": 1.5,
            "bridge": 2.0,
            "panzer": 2.5,
            "red_cross": 10.0,
        },
        interrupt_top_k=3,
        required_deliveries=3,
    )


def candidate(target_id=1, class_name="bridge", now=100.0, x=1.0,
              y=0.0, map_quality=0.8, class_confidence=0.9):
    first_seen = int((now - 1.0) * NSEC)
    last_seen = int((now - 0.1) * NSEC)
    return CandidateSnapshot(
        target_id=target_id,
        class_name=class_name,
        class_confidence=class_confidence,
        geometry_confidence=0.85,
        map_quality=map_quality,
        x=x,
        y=y,
        z=0.0,
        map_frame="camera_init",
        state=2,
        consecutive_observe_count=3,
        map_valid=True,
        association_valid=True,
        reject_reason="",
        transform_age_sec=0.1,
        first_seen_ns=first_seen,
        last_seen_ns=last_seen,
    )


def result_for(action, event_seq, status="PROGRESS", stage="PLANNER",
               terminal=False, retryable=False, payload_committed=False,
               reason="progress", executor_id="executor-a",
               event_time=None):
    key = action.candidate_key
    stamp = action.issued_at + 0.01 if event_time is None else event_time
    return ResultEvent(
        mission_id="mission-1",
        executor_id=executor_id,
        event_seq=event_seq,
        event_stamp_ns=int(stamp * NSEC),
        decision_seq=action.decision_seq,
        command=action.command,
        has_target=action.has_target,
        target_id=(key.target_id if key else 0),
        target_first_seen_ns=(key.first_seen_ns if key else 0),
        target_class=action.target_class,
        attempt=action.attempt,
        payload_slot=action.payload_slot,
        status=status,
        stage=stage,
        terminal=terminal,
        retryable=retryable,
        payload_committed=payload_committed,
        reason=reason,
        evidence_source="deterministic_test",
    )


def release_ack(action, event_seq):
    return result_for(
        action,
        event_seq,
        status="PROGRESS",
        stage="RELEASE",
        terminal=False,
        retryable=False,
        payload_committed=True,
        reason="release_ack_success",
    )


class MissionCoreTest(unittest.TestCase):
    def make_core(self, config=None):
        core = MissionCore(competition_profile(), config)
        core.start("mission-1", 100.0)
        return core

    def dispatch(self, core, item, now=100.0, route_complete=False):
        accepted = core.ingest([item], now)
        self.assertTrue(accepted[0].accepted, accepted[0].reason)
        action = core.choose(now, (0.0, 0.0), route_complete)
        self.assertIsNotNone(action)
        self.assertEqual(action.command, "APPROACH")
        return action

    def finish_delivery(self, core, action, event_seq):
        accepted, reason, _ = core.apply_result(
            release_ack(action, event_seq), action.issued_at + 0.1)
        self.assertTrue(accepted, reason)
        accepted, reason, next_action = core.apply_result(
            result_for(
                action,
                event_seq + 1,
                status="SUCCEEDED",
                stage="RECOVERY",
                terminal=True,
                reason="recovery_complete",
            ),
            action.issued_at + 0.2,
        )
        self.assertTrue(accepted, reason)
        return next_action

    def test_profile_interrupt_order_excludes_tank(self):
        profile = competition_profile()
        self.assertEqual(
            profile.interrupt_classes,
            ("red_cross", "panzer", "bridge"),
        )
        self.assertFalse(profile.allows("tank"))
        with self.assertRaises(TypeError):
            profile.weights["tank"] = 5.0
        altered = CompetitionProfile(
            name="r2026",
            weights=dict(profile.weights, tank=5.0),
            interrupt_top_k=3,
            required_deliveries=3,
        )
        with self.assertRaisesRegex(ValueError, "frozen profile"):
            MissionCore(altered)

    def test_profile_loader_rejects_fractional_integer_fields(self):
        payload = """profiles:
  broken:
    classes: {tent: 1.0}
    interrupt_top_k: 1.5
    required_deliveries: 3
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.yaml"
            path.write_text(payload, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "interrupt_top_k"):
                load_profile(path, "broken")

    def test_candidate_contract_rejects_unsafe_variants(self):
        profile = competition_profile()
        config = MissionConfig()
        base = candidate()
        cases = (
            (replace(base, class_name="tank"), "profile_excluded"),
            (replace(base, state=3), "state_not_confirmed"),
            (replace(base, state=4), "state_not_confirmed"),
            (replace(base, consecutive_observe_count=2), "streak_too_short"),
            (replace(base, map_frame="map"), "map_frame_mismatch"),
            (replace(base, association_valid=False), "association_invalid"),
            (replace(base, last_seen_ns=99 * NSEC), "candidate_stale"),
            (replace(base, transform_age_sec=0.6), "transform_stale"),
            (replace(base, map_quality=1.1), "quality_out_of_range"),
            (replace(base, first_seen_ns=101 * NSEC),
             "source_stamp_order_invalid"),
        )
        for unsafe, expected in cases:
            with self.subTest(expected=expected):
                actual = validate_candidate(unsafe, 100.0, profile, config)
                self.assertFalse(actual.accepted)
                self.assertEqual(actual.reason, expected)

        legal_zero_id = validate_candidate(
            replace(base, target_id=0), 100.0, profile, config)
        self.assertTrue(legal_zero_id.accepted, legal_zero_id.reason)

    def test_terminal_candidate_update_invalidates_queue_entry(self):
        pending = self.make_core()
        item = candidate()
        pending.ingest([item], 100.0)
        expired = replace(item, state=4, last_seen_ns=100 * NSEC)
        validation = pending.ingest([expired], 100.1)[0]
        self.assertFalse(validation.accepted)
        self.assertEqual(
            pending.queue.entries[item.key].status,
            CandidateStatus.SKIPPED,
        )

        executing = self.make_core()
        action = self.dispatch(executing, item)
        rejected = replace(
            item,
            state=3,
            reject_reason="vision_rejected",
            last_seen_ns=100 * NSEC,
        )
        executing.ingest([rejected], 100.1)
        entry = executing.queue.entries[item.key]
        self.assertTrue(entry.retry_forbidden)
        accepted, reason, _ = executing.apply_result(
            result_for(
                action,
                1,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                retryable=True,
                reason="planner_failed",
            ),
            100.2,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(entry.status, CandidateStatus.EXHAUSTED)

    def test_low_weight_waits_then_coverage_fallback_prefers_weight(self):
        core = self.make_core()
        outcomes = core.ingest([
            candidate(1, "tent"),
            candidate(2, "pillbox", x=2.0),
        ], 100.0)
        self.assertTrue(all(item.accepted for item in outcomes))
        self.assertIsNone(core.choose(100.0, (0.0, 0.0)))
        action = core.choose(100.0, (0.0, 0.0), route_complete=True)
        self.assertEqual(action.target_class, "pillbox")
        self.assertEqual(action.reason, "coverage_complete_fallback")

    def test_high_weight_interrupt_and_no_execution_preemption(self):
        core = self.make_core()
        action = self.dispatch(core, candidate(1, "bridge"))
        self.assertEqual(action.reason, "high_weight_search_interrupt")
        core.ingest([candidate(2, "red_cross", now=100.1)], 100.1)
        self.assertIsNone(core.choose(100.1, (0.0, 0.0)))
        self.assertEqual(core.active_action, action)

    def test_search_and_resume_share_global_decision_sequence(self):
        core = self.make_core()
        goal = GoalSnapshot("camera_init", 1.0, 2.0, 2.2)
        search = core.dispatch_search_motion(
            "SEARCH", goal, "coverage_waypoint", 100.0)
        self.assertEqual(search.decision_seq, 1)
        self.assertTrue(search.has_goal)
        self.assertFalse(search.has_target)
        self.assertEqual(search.profile_name, "r2026")
        accepted, reason, next_action = core.apply_result(
            result_for(
                search,
                1,
                status="SUCCEEDED",
                stage="PLANNER",
                terminal=True,
                reason="waypoint_reached",
            ),
            101.0,
        )
        self.assertTrue(accepted, reason)
        self.assertIsNone(next_action)
        resume = core.dispatch_search_motion(
            "RESUME", goal, "resume_interrupted_waypoint", 101.0)
        self.assertEqual(resume.decision_seq, 2)
        accepted, reason, _ = core.apply_result(
            result_for(
                resume,
                2,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                retryable=True,
                reason="planner_failed",
            ),
            102.0,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "search_motion_failed")
        self.assertEqual(core.phase, MissionPhase.SEARCH)

    def test_target_action_freezes_dispatch_snapshot(self):
        core = self.make_core()
        original = candidate(1, "bridge")
        action = self.dispatch(core, original)
        updated = replace(
            original,
            class_name="red_cross",
            class_confidence=0.99,
            last_seen_ns=100 * NSEC,
        )
        validation = core.ingest([updated], 100.1)[0]
        self.assertTrue(validation.accepted, validation.reason)
        self.assertEqual(action.target_snapshot.class_name, "bridge")
        self.assertEqual(
            core.queue.entries[action.candidate_key].snapshot.class_name,
            "red_cross",
        )
        self.assertTrue(core.apply_result(release_ack(action, 1), 100.2)[0])
        self.assertIn("bridge", core.queue.delivered_classes)
        self.assertNotIn("red_cross", core.queue.delivered_classes)

    def test_best_candidate_per_class_competes(self):
        core = self.make_core()
        core.ingest([
            candidate(1, "bridge", x=0.2, map_quality=0.6),
            candidate(2, "bridge", x=3.0, map_quality=0.9),
            candidate(3, "panzer", x=5.0, map_quality=0.5),
        ], 100.0)
        ranked = core.queue.ranked(100.0, (0.0, 0.0))
        self.assertEqual([entry.snapshot.target_id for entry in ranked], [3, 2])

    def test_infeasible_top_rank_does_not_block_next_feasible_target(self):
        core = self.make_core()
        core.ingest([
            candidate(1, "red_cross", x=1000.0),
            candidate(2, "panzer", x=1.0),
        ], 100.0)
        action = core.choose(100.0, (0.0, 0.0))
        self.assertEqual(action.target_class, "panzer")

        fallback = self.make_core()
        fallback.ingest([
            candidate(1, "pillbox", x=1000.0),
            candidate(2, "tent", x=1.0),
        ], 100.0)
        action = fallback.choose(
            100.0, (0.0, 0.0), route_complete=True)
        self.assertEqual(action.target_class, "tent")

    def test_retry_cooldown_then_second_attempt(self):
        core = self.make_core()
        action = self.dispatch(core, candidate(1, "bridge"))
        accepted, reason, next_action = core.apply_result(
            result_for(
                action,
                1,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                retryable=True,
                reason="planner_failed",
            ),
            101.0,
        )
        self.assertTrue(accepted, reason)
        self.assertIsNone(next_action)
        self.assertIsNone(core.active_action)
        entry = core.queue.entries[action.candidate_key]
        self.assertEqual(entry.status, CandidateStatus.COOLDOWN)
        self.assertEqual(core.slots[0].status, SlotStatus.FREE)
        self.assertIsNone(core.choose(120.9, (0.0, 0.0)))
        retry = core.choose(121.0, (0.0, 0.0))
        self.assertEqual(retry.command, "APPROACH")
        self.assertEqual(retry.attempt, 2)

    def test_dynamic_budget_dispatches_in_guard_window(self):
        core = self.make_core()
        core.ingest([candidate(1, "tent", now=525.0, x=0.0)], 525.0)
        self.assertTrue(core.should_stop_search(525.0, (0.0, 0.0)))
        action = core.choose(525.0, (0.0, 0.0))
        self.assertEqual(action.command, "APPROACH")
        self.assertEqual(action.reason, "time_budget_fallback")

    def test_infeasible_candidate_returns_without_reservation(self):
        core = self.make_core()
        item = candidate(1, "tent", now=551.0, x=0.0)
        core.ingest([item], 551.0)
        action = core.choose(551.0, (0.0, 0.0))
        self.assertEqual(action.command, "RETURN_HOME")
        self.assertEqual(action.reason, "time_budget_no_feasible_candidate")
        self.assertEqual(core.queue.entries[item.key].status,
                         CandidateStatus.PENDING)

    def test_hard_return_at_510_seconds(self):
        core = self.make_core()
        action = core.choose(610.0, (0.0, 0.0))
        self.assertEqual(action.command, "RETURN_HOME")
        self.assertEqual(action.reason, "forced_return_deadline")
        rejected = core.ingest([candidate(now=610.0)], 610.0)
        self.assertFalse(rejected[0].accepted)
        self.assertEqual(rejected[0].reason,
                         "mission_not_accepting_candidates")

    def test_invalid_commit_does_not_consume_event_sequence(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        invalid = replace(release_ack(action, 1), terminal=True)
        accepted, reason, _ = core.apply_result(invalid, 101.0)
        self.assertFalse(accepted)
        self.assertEqual(reason, "invalid_payload_commit_event")
        accepted, reason, _ = core.apply_result(release_ack(action, 1), 101.0)
        self.assertTrue(accepted, reason)
        self.assertEqual(core.committed_slots, 1)

    def test_uncommitted_target_timeout_quarantines_slot_and_returns(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        handled, reason, _ = core.expire_active(action.deadline_at - 0.1)
        self.assertFalse(handled)
        self.assertEqual(reason, "decision_not_expired")
        handled, reason, next_action = core.expire_active(
            action.deadline_at + 0.1)
        self.assertTrue(handled)
        self.assertEqual(reason, "target_action_timed_out_uncertain")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertEqual(core.slots[0].status, SlotStatus.QUARANTINED)
        self.assertEqual(core.queue.entries[action.candidate_key].status,
                         CandidateStatus.EXHAUSTED)
        self.assertIn(action.decision_seq, core.quarantined_actions)

    def test_deadline_is_exclusive(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        handled, reason, next_action = core.expire_active(action.deadline_at)
        self.assertTrue(handled)
        self.assertEqual(reason, "target_action_timed_out_uncertain")
        self.assertEqual(next_action.command, "RETURN_HOME")

    def test_late_release_ack_is_callback_order_independent(self):
        ack_first = self.make_core()
        action = self.dispatch(ack_first, candidate())
        late_ack = result_for(
            action,
            1,
            status="PROGRESS",
            stage="RELEASE",
            payload_committed=True,
            reason="release_ack_success",
            event_time=action.deadline_at + 0.01,
        )
        accepted, reason, next_action = ack_first.apply_result(
            late_ack, action.deadline_at + 0.02)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "late_payload_committed")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertTrue(ack_first.mission_failed)
        self.assertEqual(ack_first.slots[0].status, SlotStatus.COMMITTED)

        timeout_first = self.make_core()
        timeout_action = self.dispatch(timeout_first, candidate())
        timeout_first.expire_active(timeout_action.deadline_at)
        accepted, reason, next_action = timeout_first.apply_result(
            result_for(
                timeout_action,
                1,
                status="PROGRESS",
                stage="RELEASE",
                payload_committed=True,
                reason="release_ack_success",
                event_time=timeout_action.deadline_at + 0.01,
            ),
            timeout_action.deadline_at + 0.02,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "late_payload_committed")
        self.assertIsNone(next_action)
        self.assertTrue(timeout_first.mission_failed)
        self.assertEqual(timeout_first.slots[0].status, SlotStatus.COMMITTED)
        self.assertEqual(
            ack_first.active_action.command,
            timeout_first.active_action.command,
        )

    def test_on_time_stamp_reported_after_deadline_still_expires(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        delayed_ack = release_ack(action, 1)
        self.assertLess(delayed_ack.event_time, action.deadline_at)
        accepted, reason, next_action = core.apply_result(
            delayed_ack, action.deadline_at)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "late_payload_committed")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertTrue(core.mission_failed)
        self.assertEqual(core.slots[0].status, SlotStatus.COMMITTED)

    def test_late_return_success_cannot_override_safety_timeout(self):
        result_first = self.make_core()
        return_action = result_first.choose(610.0, (0.0, 0.0))
        delayed_success = result_for(
            return_action,
            1,
            status="SUCCEEDED",
            stage="PLANNER",
            terminal=True,
            reason="home_reached",
        )
        accepted, reason, abort = result_first.apply_result(
            delayed_success, return_action.deadline_at)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "safety_motion_timed_out")
        self.assertEqual(abort.command, "ABORT")

        timer_first = self.make_core()
        timer_action = timer_first.choose(610.0, (0.0, 0.0))
        handled, reason, abort = timer_first.expire_active(
            timer_action.deadline_at)
        self.assertTrue(handled, reason)
        self.assertEqual(abort.command, "ABORT")
        accepted, reason, _ = timer_first.apply_result(
            result_for(
                timer_action,
                1,
                status="SUCCEEDED",
                stage="PLANNER",
                terminal=True,
                reason="home_reached",
            ),
            timer_action.deadline_at,
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "result_precedes_decision")
        self.assertEqual(
            result_first.active_action.command,
            timer_first.active_action.command,
        )

    def test_late_recovery_success_cannot_override_safety_timeout(self):
        result_first = self.make_core()
        action = self.dispatch(result_first, candidate())
        self.assertTrue(result_first.apply_result(
            release_ack(action, 1), action.issued_at + 0.1)[0])
        delayed_recovery = result_for(
            action,
            2,
            status="SUCCEEDED",
            stage="RECOVERY",
            terminal=True,
            reason="recovery_complete",
            event_time=action.deadline_at - 0.01,
        )
        accepted, reason, return_action = result_first.apply_result(
            delayed_recovery, action.deadline_at)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "committed_recovery_timed_out")
        self.assertEqual(return_action.command, "RETURN_HOME")
        self.assertTrue(result_first.mission_failed)

        timer_first = self.make_core()
        timer_action = self.dispatch(timer_first, candidate())
        self.assertTrue(timer_first.apply_result(
            release_ack(timer_action, 1), timer_action.issued_at + 0.1)[0])
        timer_first.expire_active(timer_action.deadline_at)
        accepted, reason, _ = timer_first.apply_result(
            result_for(
                timer_action,
                2,
                status="SUCCEEDED",
                stage="RECOVERY",
                terminal=True,
                reason="recovery_complete",
                event_time=timer_action.deadline_at - 0.01,
            ),
            timer_action.deadline_at,
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "decision_mismatch")
        self.assertTrue(timer_first.mission_failed)
        self.assertEqual(
            result_first.active_action.command,
            timer_first.active_action.command,
        )

    def test_late_landing_success_cannot_override_safety_timeout(self):
        def dispatch_land(core):
            return_action = core.choose(610.0, (0.0, 0.0))
            accepted, reason, land_action = core.apply_result(
                result_for(
                    return_action,
                    1,
                    status="SUCCEEDED",
                    stage="PLANNER",
                    terminal=True,
                    reason="home_reached",
                ),
                return_action.issued_at + 0.1,
            )
            self.assertTrue(accepted, reason)
            self.assertEqual(land_action.command, "LAND")
            return land_action

        result_first = self.make_core()
        land_action = dispatch_land(result_first)
        accepted, reason, abort = result_first.apply_result(
            result_for(
                land_action,
                2,
                status="SUCCEEDED",
                stage="LANDING",
                terminal=True,
                reason="landed",
                event_time=land_action.deadline_at - 0.01,
            ),
            land_action.deadline_at,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "safety_motion_timed_out")
        self.assertEqual(abort.command, "ABORT")

        timer_first = self.make_core()
        timer_land = dispatch_land(timer_first)
        timer_first.expire_active(timer_land.deadline_at)
        accepted, reason, _ = timer_first.apply_result(
            result_for(
                timer_land,
                2,
                status="SUCCEEDED",
                stage="LANDING",
                terminal=True,
                reason="landed",
                event_time=timer_land.deadline_at - 0.01,
            ),
            timer_land.deadline_at,
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "decision_mismatch")
        self.assertEqual(
            result_first.active_action.command,
            timer_first.active_action.command,
        )

    def test_expired_success_without_ack_can_be_corrected_same_sequence(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        invalid = result_for(
            action,
            1,
            status="SUCCEEDED",
            stage="RECOVERY",
            terminal=True,
            reason="done_without_release_ack",
            event_time=action.deadline_at + 0.01,
        )
        accepted, reason, _ = core.apply_result(
            invalid, action.deadline_at + 0.02)
        self.assertFalse(accepted)
        self.assertEqual(reason, "success_without_payload_commit")
        corrected = result_for(
            action,
            1,
            status="PROGRESS",
            stage="RELEASE",
            payload_committed=True,
            reason="release_ack_success",
            event_time=action.deadline_at + 0.01,
        )
        accepted, reason, next_action = core.apply_result(
            corrected, action.deadline_at + 0.02)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "late_payload_committed")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertEqual(core.slots[0].status, SlotStatus.COMMITTED)

    def test_late_target_failure_cannot_release_slot_for_retry(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        accepted, reason, next_action = core.apply_result(
            result_for(
                action,
                1,
                status="TIMED_OUT",
                stage="PLANNER",
                terminal=True,
                retryable=True,
                reason="executor_timeout",
                event_time=action.deadline_at + 0.01,
            ),
            action.deadline_at + 0.02,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "target_action_timed_out_uncertain")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertEqual(core.slots[0].status, SlotStatus.QUARANTINED)
        self.assertEqual(
            core.queue.entries[action.candidate_key].status,
            CandidateStatus.EXHAUSTED,
        )

    def test_release_stage_failure_quarantines_then_late_ack_commits(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        accepted, reason, next_action = core.apply_result(
            result_for(
                action,
                1,
                status="TIMED_OUT",
                stage="RELEASE",
                terminal=True,
                retryable=True,
                reason="release_result_missing",
            ),
            action.issued_at + 0.1,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "candidate_release_state_uncertain")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertEqual(core.slots[0].status, SlotStatus.QUARANTINED)
        self.assertEqual(
            core.queue.entries[action.candidate_key].status,
            CandidateStatus.EXHAUSTED,
        )
        self.assertIn(action.decision_seq, core.quarantined_actions)

        accepted, reason, next_action = core.apply_result(
            release_ack(action, 2), action.issued_at + 0.2)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "late_payload_committed")
        self.assertIsNone(next_action)
        self.assertEqual(core.slots[0].status, SlotStatus.COMMITTED)
        self.assertEqual(core.committed_slots, 1)

    def test_release_stage_latch_cannot_be_cleared_by_stage_rollback(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        accepted, reason, _ = core.apply_result(
            result_for(
                action,
                1,
                status="PROGRESS",
                stage="RELEASE",
                reason="release_started_without_ack",
            ),
            action.issued_at + 0.1,
        )
        self.assertTrue(accepted, reason)
        accepted, reason, next_action = core.apply_result(
            result_for(
                action,
                2,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                retryable=True,
                reason="stage_rollback_failure",
            ),
            action.issued_at + 0.2,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "candidate_release_state_uncertain")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertEqual(core.slots[0].status, SlotStatus.QUARANTINED)

    def test_unknown_target_stage_is_rejected_without_sequence_consumption(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        invalid = result_for(action, 1, stage="LANDING")
        accepted, reason, _ = core.apply_result(
            invalid, action.issued_at + 0.1)
        self.assertFalse(accepted)
        self.assertEqual(reason, "target_stage_invalid")
        accepted, reason, _ = core.apply_result(
            result_for(action, 1, stage="PLANNER"),
            action.issued_at + 0.2,
        )
        self.assertTrue(accepted, reason)

    def test_late_release_ack_reconciles_quarantined_slot(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        core.expire_active(action.deadline_at + 0.1)
        accepted, reason, next_action = core.apply_result(
            release_ack(action, 1), action.deadline_at + 1.0)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "late_payload_committed")
        self.assertIsNone(next_action)
        self.assertEqual(core.committed_slots, 1)
        self.assertEqual(core.slots[0].status, SlotStatus.COMMITTED)
        self.assertNotIn(action.decision_seq, core.quarantined_actions)

    def test_quarantined_success_without_ack_stays_quarantined(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        core.expire_active(action.deadline_at + 0.1)
        invalid = result_for(
            action,
            1,
            status="SUCCEEDED",
            stage="RECOVERY",
            terminal=True,
            reason="done_without_ack",
        )
        accepted, reason, _ = core.apply_result(
            invalid, action.deadline_at + 1.0)
        self.assertFalse(accepted)
        self.assertEqual(reason, "success_without_payload_commit")
        self.assertEqual(core.slots[0].status, SlotStatus.QUARANTINED)
        self.assertIn(action.decision_seq, core.quarantined_actions)

    def test_quarantined_negative_terminal_keeps_release_tombstone(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        core.expire_active(action.deadline_at)
        accepted, reason, _ = core.apply_result(
            result_for(
                action,
                1,
                status="FAILED",
                stage="RECOVERY",
                terminal=True,
                reason="executor_failed_after_timeout",
            ),
            action.deadline_at + 0.1,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "quarantined_terminal_recorded")
        self.assertIn(action.decision_seq, core.quarantined_actions)

    def test_committed_recovery_timeout_keeps_slot_and_returns(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        self.assertTrue(core.apply_result(release_ack(action, 1), 100.1)[0])
        handled, reason, next_action = core.expire_active(
            action.deadline_at + 0.1)
        self.assertTrue(handled)
        self.assertEqual(reason, "committed_recovery_timed_out")
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertEqual(core.committed_slots, 1)

    def test_committed_recovery_cannot_be_retryable(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        self.assertTrue(core.apply_result(release_ack(action, 1), 100.1)[0])
        retryable = result_for(
            action,
            2,
            status="FAILED",
            stage="RECOVERY",
            terminal=True,
            retryable=True,
            reason="recovery_failed",
        )
        accepted, reason, _ = core.apply_result(retryable, 100.2)
        self.assertFalse(accepted)
        self.assertEqual(reason, "committed_result_must_not_retry")
        accepted, reason, next_action = core.apply_result(
            replace(retryable, retryable=False), 100.2)
        self.assertTrue(accepted, reason)
        self.assertEqual(next_action.command, "RETURN_HOME")

    def test_result_before_decision_time_does_not_consume_sequence(self):
        core = self.make_core()
        action = self.dispatch(core, candidate(now=101.0), now=101.0)
        progress = replace(
            result_for(action, 1), event_stamp_ns=int(100.899 * NSEC))
        accepted, reason, _ = core.apply_result(progress, 101.1)
        self.assertFalse(accepted)
        self.assertEqual(reason, "result_precedes_decision")
        corrected = replace(
            progress, event_stamp_ns=int(101.01 * NSEC))
        self.assertTrue(core.apply_result(corrected, 101.1)[0])

    def test_result_source_clock_jitter_uses_existing_tolerance(self):
        core = self.make_core()
        action = self.dispatch(core, candidate(now=101.0), now=101.0)
        slightly_early = replace(
            result_for(action, 1), event_stamp_ns=int(100.95 * NSEC))
        accepted, reason, _ = core.apply_result(slightly_early, 101.1)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "progress_recorded")

    def test_duplicate_and_out_of_order_results_are_idempotent(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        progress = result_for(action, 2)
        self.assertTrue(core.apply_result(progress, 100.1)[0])
        accepted, reason, _ = core.apply_result(progress, 100.2)
        self.assertFalse(accepted)
        self.assertEqual(reason, "event_duplicate_or_out_of_order")
        accepted, reason, _ = core.apply_result(
            replace(progress, event_seq=1), 100.3)
        self.assertFalse(accepted)
        self.assertEqual(reason, "event_duplicate_or_out_of_order")

    def test_nonterminal_result_cannot_claim_retryable(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        invalid = result_for(action, 1, retryable=True)
        accepted, reason, _ = core.apply_result(invalid, 100.1)
        self.assertFalse(accepted)
        self.assertEqual(reason, "nonterminal_result_must_not_retry")
        self.assertTrue(
            core.apply_result(replace(invalid, retryable=False), 100.1)[0])

    def test_success_without_commit_can_be_corrected_same_sequence(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        invalid = result_for(
            action, 1, status="SUCCEEDED", stage="RECOVERY",
            terminal=True, reason="done")
        accepted, reason, _ = core.apply_result(invalid, 101.0)
        self.assertFalse(accepted)
        self.assertEqual(reason, "success_without_payload_commit")
        corrected = replace(
            invalid,
            status="FAILED",
            stage="PLANNER",
            retryable=False,
            reason="planner_failed",
        )
        self.assertTrue(core.apply_result(corrected, 101.0)[0])

    def test_release_commit_survives_recovery_failure(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        self.assertTrue(core.apply_result(release_ack(action, 1), 101.0)[0])
        wrong_stage = result_for(
            action, 2, status="FAILED", stage="ALIGNMENT", terminal=True,
            reason="recovery_failed")
        accepted, reason, _ = core.apply_result(wrong_stage, 102.0)
        self.assertFalse(accepted)
        self.assertEqual(reason, "committed_terminal_stage_invalid")
        failed = replace(wrong_stage, stage="RECOVERY")
        accepted, reason, next_action = core.apply_result(failed, 102.0)
        self.assertTrue(accepted, reason)
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertTrue(core.mission_failed)
        self.assertEqual(core.committed_slots, 1)
        self.assertEqual(core.queue.entries[action.candidate_key].status,
                         CandidateStatus.SUCCEEDED)

    def test_three_commits_immediately_return(self):
        core = self.make_core()
        event_seq = 1
        for index, class_name in enumerate(
                ("bridge", "panzer", "red_cross"), start=1):
            action = self.dispatch(
                core,
                candidate(index, class_name, now=100.0 + index),
                now=100.0 + index,
            )
            next_action = self.finish_delivery(core, action, event_seq)
            event_seq += 2
        self.assertEqual(core.committed_slots, 3)
        self.assertEqual(next_action.command, "RETURN_HOME")
        self.assertEqual(next_action.reason, "required_deliveries_complete")
        self.assertEqual(core.phase, MissionPhase.RETURN_HOME)

    def test_three_commits_follow_configured_route_before_land(self):
        route = (
            GoalSnapshot("camera_init", -2.0, 6.0, 1.0),
            GoalSnapshot("camera_init", 0.0, 8.0, 1.2),
            GoalSnapshot("camera_init", 3.0, 8.0, 0.75),
        )
        core = self.make_core(MissionConfig(
            post_delivery_route=route,
            post_delivery_route_revision="three-door-test-r1",
            landing_xy=(3.0, 8.0),
        ))
        event_seq = 1
        for index, class_name in enumerate(
                ("bridge", "panzer", "red_cross"), start=1):
            action = self.dispatch(
                core,
                candidate(index, class_name, now=100.0 + index),
                now=100.0 + index,
            )
            next_action = self.finish_delivery(core, action, event_seq)
            event_seq += 2

        self.assertEqual(core.phase, MissionPhase.POST_DELIVERY_ROUTE)
        for route_index, expected in enumerate(route):
            with self.subTest(route_index=route_index):
                self.assertEqual(next_action.command, "RETURN_HOME")
                self.assertEqual(next_action.goal, expected)
                self.assertIn(
                    "post_delivery_route:%d/%d" %
                    (route_index + 1, len(route)),
                    next_action.reason,
                )
                accepted, reason, following = core.apply_result(
                    result_for(
                        next_action,
                        event_seq,
                        status="SUCCEEDED",
                        stage="PLANNER",
                        terminal=True,
                        reason="route_point_reached",
                    ),
                    next_action.issued_at + 0.2,
                )
                self.assertTrue(accepted, reason)
                event_seq += 1
                next_action = following

        self.assertEqual(next_action.command, "LAND")
        self.assertEqual(next_action.reason, "post_delivery_route_complete")
        self.assertEqual(
            next_action.deadline_at,
            core.started_at + core.config.mission_timeout,
        )
        self.assertEqual(core.post_delivery_route_index, len(route))
        self.assertEqual(core.phase, MissionPhase.LAND)

    def test_post_delivery_route_failure_aborts_without_skipping_to_land(self):
        route = (
            GoalSnapshot("camera_init", -2.0, 6.0, 1.0),
            GoalSnapshot("camera_init", 3.0, 8.0, 0.75),
        )
        core = self.make_core(MissionConfig(
            post_delivery_route=route,
            post_delivery_route_revision="route-failure-test-r1",
            landing_xy=(3.0, 8.0),
        ))
        event_seq = 1
        for index, class_name in enumerate(
                ("bridge", "panzer", "red_cross"), start=1):
            action = self.dispatch(
                core,
                candidate(index, class_name, now=100.0 + index),
                now=100.0 + index,
            )
            route_action = self.finish_delivery(core, action, event_seq)
            event_seq += 2
        accepted, reason, abort = core.apply_result(
            result_for(
                route_action,
                event_seq,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                reason="door_unreachable",
            ),
            route_action.issued_at + 0.2,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "return_home_failed")
        self.assertEqual(abort.command, "ABORT")
        self.assertEqual(core.phase, MissionPhase.ABORTED)

    def test_forced_return_bypasses_optional_post_delivery_route(self):
        core = self.make_core(MissionConfig(
            post_delivery_route=(
                GoalSnapshot("camera_init", 3.0, 8.0, 0.75),),
            post_delivery_route_revision="optional-route-test-r1",
            landing_xy=(3.0, 8.0),
        ))
        action = core.choose(610.0, (1.0, 1.0))
        self.assertEqual(action.command, "RETURN_HOME")
        self.assertEqual(action.goal.x, 0.0)
        self.assertEqual(action.goal.y, 0.0)
        self.assertEqual(core.phase, MissionPhase.RETURN_HOME)

    def test_executor_change_and_target_identity_mismatch_fail_closed(self):
        core = self.make_core()
        action = self.dispatch(core, candidate())
        first = result_for(action, 1)
        self.assertTrue(core.apply_result(first, 100.1)[0])
        changed = result_for(action, 2, executor_id="executor-b")
        accepted, reason, _ = core.apply_result(changed, 100.2)
        self.assertFalse(accepted)
        self.assertEqual(reason, "executor_changed")
        mismatch = replace(result_for(action, 2),
                           target_first_seen_ns=1)
        accepted, reason, _ = core.apply_result(mismatch, 100.2)
        self.assertFalse(accepted)
        self.assertEqual(reason, "target_mismatch")
        command_mismatch = replace(result_for(action, 2), command="ALIGN")
        accepted, reason, _ = core.apply_result(command_mismatch, 100.2)
        self.assertFalse(accepted)
        self.assertEqual(reason, "command_mismatch")
        missing_flag = replace(result_for(action, 2), has_target=False)
        accepted, reason, _ = core.apply_result(missing_flag, 100.2)
        self.assertFalse(accepted)
        self.assertEqual(reason, "result_target_flag_mismatch")

    def test_return_land_lifecycle(self):
        core = self.make_core()
        action = core.choose(610.0, (0.0, 0.0))
        self.assertEqual(action.command, "RETURN_HOME")
        accepted, reason, land = core.apply_result(
            result_for(
                action,
                1,
                status="SUCCEEDED",
                stage="PLANNER",
                terminal=True,
                reason="home_reached",
            ),
            611.0,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(land.command, "LAND")
        accepted, reason, next_action = core.apply_result(
            result_for(
                land,
                2,
                status="SUCCEEDED",
                stage="LANDING",
                terminal=True,
                reason="landed",
            ),
            612.0,
        )
        self.assertTrue(accepted, reason)
        self.assertIsNone(next_action)
        self.assertEqual(core.phase, MissionPhase.COMPLETE)

    def test_return_failure_aborts_without_landing(self):
        core = self.make_core()
        action = core.choose(610.0, (0.0, 0.0))
        accepted, reason, abort = core.apply_result(
            result_for(
                action,
                1,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                reason="return_planner_failed",
            ),
            611.0,
        )
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "return_home_failed")
        self.assertEqual(abort.command, "ABORT")
        self.assertEqual(core.phase, MissionPhase.ABORTED)

    def test_safety_action_after_mission_limit_has_valid_lease(self):
        core = self.make_core()
        action = core.choose(700.0, (0.0, 0.0))
        self.assertEqual(action.command, "RETURN_HOME")
        self.assertGreater(action.deadline_at, action.issued_at)
        self.assertTrue(core.mission_failed)

    def test_competition_hard_limits_fail_closed(self):
        invalid_configs = (
            {"mission_timeout": 601.0},
            {"forced_return_at": 511.0, "return_land_reserve": 89.0},
            {"max_target_z": 4.1},
            {"approach_altitude": 4.1},
            {"return_altitude": 4.1},
            {"mission_frame": " "},
            {"mission_timeout": True},
            {"min_streak": True},
            {"max_attempts": 2.0},
            {"decision_guard": "15"},
            {"home_xy": "12"},
            {"landing_xy": "12"},
            {"post_delivery_route_revision": " "},
            {"post_delivery_route": [[0.0, 0.0, 1.0]]},
            {"post_delivery_route": (
                GoalSnapshot("map", 0.0, 0.0, 1.0),)},
            {"post_delivery_route": (
                GoalSnapshot("camera_init", 1.0, 1.0, 1.0),),
             "landing_xy": (0.0, 0.0),
             "landing_anchor_tolerance": 0.1},
        )
        for values in invalid_configs:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    MissionConfig(**values)
        for altitude in (-0.1, 4.01):
            with self.subTest(altitude=altitude):
                with self.assertRaises(ValueError):
                    GoalSnapshot("camera_init", 0.0, 0.0, altitude)
        config = MissionConfig()
        with self.assertRaises(FrozenInstanceError):
            config.forced_return_at = 511.0
        self.assertEqual(MissionConfig(home_xy=[1, 2]).home_xy, (1.0, 2.0))

    def test_explicit_abort_preserves_only_committed_slots(self):
        uncommitted = self.make_core()
        action = self.dispatch(uncommitted, candidate())
        abort = uncommitted.abort("executor_changed", 100.1)
        self.assertEqual(abort.command, "ABORT")
        self.assertEqual(
            uncommitted.slots[0].status, SlotStatus.QUARANTINED)
        self.assertEqual(
            uncommitted.queue.entries[action.candidate_key].status,
            CandidateStatus.EXHAUSTED,
        )

        committed = self.make_core()
        action = self.dispatch(committed, candidate())
        self.assertTrue(
            committed.apply_result(release_ack(action, 1), 100.1)[0])
        committed.abort("pose_stale", 100.2)
        self.assertEqual(committed.committed_slots, 1)
        self.assertEqual(committed.slots[0].status, SlotStatus.COMMITTED)

    def test_search_dispatch_is_cut_off_at_forced_return(self):
        core = self.make_core()
        goal = GoalSnapshot("camera_init", 1.0, 2.0, 2.2)
        action = core.dispatch_search_motion(
            "SEARCH", goal, "coverage_waypoint", 609.5)
        self.assertEqual(action.deadline_at, 610.0)
        handled, reason, return_action = core.expire_active(610.1)
        self.assertTrue(handled)
        self.assertEqual(reason, "forced_return_deadline")
        self.assertEqual(return_action.command, "RETURN_HOME")
        late = self.make_core()
        with self.assertRaisesRegex(RuntimeError, "forced return"):
            late.dispatch_search_motion(
                "RESUME", goal, "resume_waypoint", 610.1)


if __name__ == "__main__":
    unittest.main()
