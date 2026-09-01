#!/usr/bin/env python3

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import unittest

from uav_mission.coverage_route import CoverageRoute
from uav_mission.mission_core import (
    CandidateSnapshot,
    GoalSnapshot,
    MissionConfig,
    MissionCore,
    MissionPhase,
    ResultEvent,
    SlotStatus,
)
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.profile_policy import CompetitionProfile
from uav_mission.search_types import Waypoint


NSEC = 1_000_000_000


def profile():
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


def candidate(target_id=1, class_name="bridge", now=100.0,
              x=1.0, y=0.0):
    return CandidateSnapshot(
        target_id=target_id,
        class_name=class_name,
        class_confidence=0.9,
        geometry_confidence=0.85,
        map_quality=0.8,
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
        first_seen_ns=int((now - 1.0) * NSEC),
        last_seen_ns=int((now - 0.05) * NSEC),
    )


def result_for(action, event_seq, status="PROGRESS", stage="PLANNER",
               terminal=False, retryable=False, payload_committed=False,
               reason="progress"):
    key = action.candidate_key
    return ResultEvent(
        mission_id="mission-runtime",
        executor_id="executor-a",
        event_seq=event_seq,
        event_stamp_ns=int((action.issued_at + 0.01) * NSEC),
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
        evidence_source="runtime_test",
    )


def release_ack(action, event_seq):
    return result_for(
        action,
        event_seq,
        status="PROGRESS",
        stage="RELEASE",
        payload_committed=True,
        reason="release_ack_success",
    )


class MissionRuntimeTest(unittest.TestCase):
    def make_runtime(self, route_size=2, max_failures=2):
        points = tuple(
            Waypoint(float(index), 0.0, 2.2)
            for index in range(route_size)
        )
        return MissionRuntime(
            MissionCore(profile()),
            CoverageRoute(points, "test-route-r1", max_failures),
        )

    def start(self, runtime, now=100.0):
        outcome = runtime.start("mission-runtime", now, (0.0, 0.0))
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual(outcome.action.command, "SEARCH")
        return outcome.action

    def finish_delivery(self, runtime, action, event_seq, now):
        committed = runtime.apply_result(
            release_ack(action, event_seq), now, (action.goal.x, action.goal.y))
        self.assertTrue(committed.accepted, committed.reason)
        self.assertEqual(committed.reason, "payload_committed")
        recovered = runtime.apply_result(
            result_for(
                action,
                event_seq + 1,
                status="SUCCEEDED",
                stage="RECOVERY",
                terminal=True,
                reason="recovery_complete",
            ),
            now + 0.01,
            (action.goal.x, action.goal.y),
        )
        self.assertTrue(recovered.accepted, recovered.reason)
        return recovered

    def test_start_binds_first_search_and_snapshot(self):
        runtime = self.make_runtime()
        action = self.start(runtime)
        snapshot = runtime.snapshot()
        self.assertEqual(snapshot.active_decision_seq, action.decision_seq)
        self.assertEqual(
            snapshot.route_active_decision_seq, action.decision_seq)
        self.assertEqual(snapshot.route_index, 0)
        self.assertEqual(snapshot.active_command, "SEARCH")

    def test_snapshot_exposes_post_delivery_route_cursor(self):
        config = MissionConfig(
            post_delivery_route=(
                GoalSnapshot("camera_init", -2.0, 6.0, 1.0),
                GoalSnapshot("camera_init", 3.0, 8.0, 0.75),
            ),
            post_delivery_route_revision="three-door-runtime-r1",
            landing_xy=(3.0, 8.0),
        )
        runtime = MissionRuntime(
            MissionCore(profile(), config),
            CoverageRoute((Waypoint(0.0, 0.0, 2.2),),
                          "test-route-r1", 2),
        )
        self.start(runtime)
        snapshot = runtime.snapshot()
        self.assertEqual(
            snapshot.post_delivery_route_revision,
            "three-door-runtime-r1",
        )
        self.assertEqual(snapshot.post_delivery_route_index, 0)
        self.assertEqual(snapshot.post_delivery_route_size, 2)
        self.assertFalse(snapshot.post_delivery_route_complete)

    def test_high_weight_interrupt_resumes_same_waypoint(self):
        runtime = self.make_runtime()
        search = self.start(runtime)
        runtime.ingest([candidate()], 100.0)
        interrupted = runtime.tick(100.1, (0.0, 0.0))
        self.assertEqual(interrupted.action.command, "APPROACH")
        self.assertEqual(interrupted.route_outcome.reason, "route_interrupted")
        self.assertEqual(runtime.route.current_index, 0)
        self.assertEqual(
            runtime.route.last_interrupted.decision_seq, search.decision_seq)

        resumed = self.finish_delivery(
            runtime, interrupted.action, 1, 100.2)
        self.assertEqual(resumed.action.command, "RESUME")
        self.assertEqual(resumed.action.goal.x, search.goal.x)
        self.assertEqual(runtime.route.current_index, 0)

    def test_low_weight_waits_until_route_completion(self):
        runtime = self.make_runtime(route_size=1)
        search = self.start(runtime)
        runtime.ingest([candidate(class_name="tent")], 100.0)
        waiting = runtime.tick(100.1, (0.0, 0.0))
        self.assertEqual(waiting.reason, "search_continues")
        self.assertIsNone(waiting.action)

        selected = runtime.apply_result(
            result_for(
                search,
                1,
                status="SUCCEEDED",
                stage="PLANNER",
                terminal=True,
                reason="waypoint_reached",
            ),
            100.2,
            (0.0, 0.0),
        )
        self.assertEqual(selected.action.command, "APPROACH")
        self.assertEqual(selected.action.target_class, "tent")
        self.assertTrue(runtime.route.is_complete)

    def test_search_success_advances_once_and_dispatches_next(self):
        runtime = self.make_runtime(route_size=2)
        search = self.start(runtime)
        outcome = runtime.apply_result(
            result_for(
                search,
                1,
                status="SUCCEEDED",
                stage="PLANNER",
                terminal=True,
                reason="waypoint_reached",
            ),
            100.1,
            (0.0, 0.0),
        )
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertTrue(outcome.route_outcome.advanced)
        self.assertEqual(runtime.route.current_index, 1)
        self.assertEqual(outcome.action.command, "SEARCH")
        self.assertEqual(outcome.action.goal.x, 1.0)

    def test_bounded_search_failures_retry_then_skip(self):
        runtime = self.make_runtime(route_size=2, max_failures=2)
        first = self.start(runtime)
        retry = runtime.apply_result(
            result_for(
                first,
                1,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                retryable=True,
                reason="planner_failed",
            ),
            100.1,
            (0.0, 0.0),
        )
        self.assertEqual(retry.route_outcome.reason, "route_waypoint_retry")
        self.assertEqual(retry.action.command, "RESUME")
        self.assertEqual(runtime.route.current_index, 0)

        skipped = runtime.apply_result(
            result_for(
                retry.action,
                2,
                status="FAILED",
                stage="PLANNER",
                terminal=True,
                retryable=True,
                reason="planner_failed_again",
            ),
            100.2,
            (0.0, 0.0),
        )
        self.assertEqual(skipped.route_outcome.reason,
                         "route_waypoint_skipped")
        self.assertEqual(runtime.route.skipped_indices, [0])
        self.assertEqual(runtime.route.current_index, 1)
        self.assertEqual(skipped.action.command, "SEARCH")

    def test_hard_return_retires_active_search_at_510_seconds(self):
        runtime = self.make_runtime()
        self.start(runtime)
        outcome = runtime.tick(610.0, (0.0, 0.0))
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual(outcome.action.command, "RETURN_HOME")
        self.assertEqual(outcome.reason, "forced_return_deadline")
        self.assertIsNone(runtime.route.active)
        self.assertEqual(runtime.route.current_index, 0)
        self.assertEqual(runtime.route.skipped_indices, [])
        self.assertEqual(outcome.route_outcome.reason, "route_interrupted")

    def test_hard_return_does_not_turn_prior_timeout_into_skip(self):
        runtime = self.make_runtime(route_size=2, max_failures=2)
        first = self.start(runtime)
        retry = runtime.tick(first.deadline_at, (0.0, 0.0))
        self.assertEqual(retry.route_outcome.reason, "route_waypoint_retry")
        self.assertEqual(retry.action.command, "RESUME")

        outcome = runtime.tick(610.0, (0.0, 0.0))
        self.assertEqual(outcome.action.command, "RETURN_HOME")
        self.assertEqual(outcome.route_outcome.reason, "route_interrupted")
        self.assertEqual(runtime.route.current_index, 0)
        self.assertEqual(runtime.route.skipped_indices, [])

    def test_late_search_result_hard_return_does_not_skip_waypoint(self):
        runtime = self.make_runtime(route_size=2, max_failures=2)
        first = self.start(runtime)
        retry = runtime.tick(first.deadline_at, (0.0, 0.0))
        self.assertEqual(retry.route_outcome.reason, "route_waypoint_retry")

        outcome = runtime.apply_result(
            result_for(retry.action, 1),
            610.0,
            (0.0, 0.0),
        )
        self.assertEqual(outcome.reason, "forced_return_deadline")
        self.assertEqual(outcome.action.command, "RETURN_HOME")
        self.assertEqual(outcome.route_outcome.reason, "route_interrupted")
        self.assertEqual(runtime.route.current_index, 0)
        self.assertEqual(runtime.route.skipped_indices, [])

    def test_clock_rollback_aborts_once_at_last_valid_time(self):
        runtime = self.make_runtime(route_size=2)
        first = self.start(runtime)
        advanced = runtime.apply_result(
            result_for(
                first,
                1,
                status="SUCCEEDED",
                stage="PLANNER",
                terminal=True,
                reason="waypoint_reached",
            ),
            159.0,
            (0.0, 0.0),
        )
        self.assertEqual(advanced.action.command, "SEARCH")
        self.assertEqual(advanced.action.issued_at, 159.0)

        aborted = runtime.tick(101.0, (0.0, 0.0))
        self.assertFalse(aborted.accepted)
        self.assertEqual(aborted.reason, "runtime_clock_rollback")
        self.assertEqual(aborted.action.command, "ABORT")
        self.assertEqual(aborted.action.issued_at, 159.0)
        self.assertEqual(runtime.core.phase, MissionPhase.ABORTED)
        self.assertIsNone(runtime.route.active)
        decision_seq = aborted.action.decision_seq

        repeated = runtime.tick(101.0, (0.0, 0.0))
        self.assertFalse(repeated.accepted)
        self.assertEqual(repeated.reason, "runtime_clock_rollback")
        self.assertIsNone(repeated.action)
        self.assertEqual(runtime.core.decision_seq, decision_seq)

    def test_executor_change_atomically_aborts_once(self):
        runtime = self.make_runtime()
        search = self.start(runtime)
        accepted = runtime.apply_result(
            result_for(search, 1), 100.1, (0.0, 0.0))
        self.assertTrue(accepted.accepted, accepted.reason)

        changed = replace(
            result_for(search, 2), executor_id="executor-b")
        aborted = runtime.apply_result(
            changed, 100.2, (0.0, 0.0))
        self.assertFalse(aborted.accepted)
        self.assertEqual(aborted.reason, "executor_changed")
        self.assertEqual(aborted.action.command, "ABORT")
        self.assertIsNone(runtime.route.active)
        decision_seq = aborted.action.decision_seq

        repeated = runtime.apply_result(
            replace(changed, event_seq=3), 100.3, (0.0, 0.0))
        self.assertFalse(repeated.accepted)
        self.assertEqual(repeated.reason, "executor_changed")
        self.assertIsNone(repeated.action)
        self.assertEqual(runtime.core.decision_seq, decision_seq)

    def test_target_timeout_quarantines_and_late_ack_reconciles(self):
        runtime = self.make_runtime()
        self.start(runtime)
        runtime.ingest([candidate()], 100.0)
        target = runtime.tick(100.1, (0.0, 0.0)).action
        timed_out = runtime.tick(target.deadline_at, (1.0, 0.0))
        self.assertEqual(
            timed_out.reason, "target_action_timed_out_uncertain")
        self.assertEqual(timed_out.action.command, "RETURN_HOME")
        self.assertEqual(runtime.core.slots[0].status, SlotStatus.QUARANTINED)

        reconciled = runtime.apply_result(
            release_ack(target, 1),
            target.deadline_at + 0.1,
            (1.0, 0.0),
        )
        self.assertTrue(reconciled.accepted, reconciled.reason)
        self.assertEqual(reconciled.reason, "late_payload_committed")
        self.assertEqual(runtime.core.slots[0].status, SlotStatus.COMMITTED)
        self.assertEqual(
            runtime.core.active_action.decision_seq,
            timed_out.action.decision_seq,
        )

    def test_abort_retires_route_and_is_idempotent(self):
        runtime = self.make_runtime()
        self.start(runtime)
        aborted = runtime.abort("test_abort", 100.1)
        self.assertTrue(aborted.accepted, aborted.reason)
        self.assertEqual(aborted.action.command, "ABORT")
        self.assertIsNone(runtime.route.active)
        decision_seq = aborted.action.decision_seq

        repeated = runtime.abort("test_abort_again", 100.2)
        self.assertTrue(repeated.accepted, repeated.reason)
        self.assertEqual(repeated.reason, "mission_already_aborted")
        self.assertIsNone(repeated.action)
        self.assertEqual(runtime.core.decision_seq, decision_seq)

    def test_three_commits_immediately_return(self):
        runtime = self.make_runtime(route_size=2)
        self.start(runtime)
        event_seq = 1
        now = 100.0
        for target_id, class_name in (
                (1, "red_cross"), (2, "panzer"), (3, "bridge")):
            runtime.ingest([
                candidate(target_id, class_name, now=now)
            ], now)
            selected = runtime.tick(now + 0.05, (0.0, 0.0))
            self.assertEqual(selected.action.command, "APPROACH")
            finished = self.finish_delivery(
                runtime, selected.action, event_seq, now + 0.1)
            event_seq += 2
            now += 0.3
        self.assertEqual(finished.action.command, "RETURN_HOME")
        self.assertEqual(runtime.core.committed_slots, 3)
        self.assertTrue(all(
            slot.status == SlotStatus.COMMITTED
            for slot in runtime.core.slots
        ))

    def test_wrong_and_retired_results_never_advance_route(self):
        runtime = self.make_runtime()
        search = self.start(runtime)
        wrong = replace(result_for(
            search,
            1,
            status="SUCCEEDED",
            stage="PLANNER",
            terminal=True,
        ), decision_seq=999)
        rejected = runtime.apply_result(wrong, 100.1, (0.0, 0.0))
        self.assertFalse(rejected.accepted)
        self.assertEqual(runtime.route.current_index, 0)
        self.assertEqual(
            runtime.route.active.decision_seq, search.decision_seq)

        runtime.ingest([candidate()], 100.1)
        target = runtime.tick(100.2, (0.0, 0.0)).action
        retired = runtime.apply_result(
            result_for(
                search,
                1,
                status="SUCCEEDED",
                stage="PLANNER",
                terminal=True,
            ),
            100.3,
            (0.0, 0.0),
        )
        self.assertFalse(retired.accepted)
        self.assertEqual(runtime.route.current_index, 0)
        self.assertEqual(runtime.core.active_action.decision_seq,
                         target.decision_seq)

    def test_concurrent_ticks_select_one_interrupt_atomically(self):
        runtime = self.make_runtime()
        self.start(runtime)
        runtime.ingest([candidate()], 100.0)
        with ThreadPoolExecutor(max_workers=8) as pool:
            outcomes = tuple(pool.map(
                lambda _: runtime.tick(100.1, (0.0, 0.0)),
                range(16),
            ))
        approaches = [
            item.action for item in outcomes
            if item.action is not None and item.action.command == "APPROACH"
        ]
        self.assertEqual(len(approaches), 1)
        self.assertIsNone(runtime.route.active)
        self.assertEqual(runtime.core.active_action.command, "APPROACH")


if __name__ == "__main__":
    unittest.main()
