#!/usr/bin/env python3

from dataclasses import replace
import unittest

from uav_mission.planner_execution import (
    ABORT_SAFE,
    CANCEL_PLANNER_GOAL,
    LAND_EXTERNAL,
    MotionDecision,
    MotionGoal,
    OdomSample,
    PUBLISH_PLANNER_GOAL,
    PlannerMotionConfig,
    PlannerMotionExecutor,
    PlannerStatusEvent,
    SequencedMotionGoal,
    START_TARGET_TRANSACTION,
    TargetIdentity,
)


NSEC = 1_000_000_000
BASE = 1_000 * NSEC


def goal(x=1.0, frame="camera_init"):
    return MotionGoal(frame, x, 2.0, 2.2)


def target(target_id=7, class_name="bridge"):
    return TargetIdentity(
        target_id=target_id,
        first_seen_ns=900 * NSEC,
        observation_ns=999 * NSEC,
        class_name=class_name,
        attempt=1,
        payload_slot=2,
    )


def decision(seq=1, command="SEARCH", issued_ns=BASE,
             deadline_ns=BASE + 5 * NSEC, motion_goal=None,
             target_identity=None):
    if motion_goal is None and command in (
            "SEARCH", "RESUME", "APPROACH", "RETURN_HOME"):
        motion_goal = goal()
    if target_identity is None and command == "APPROACH":
        target_identity = target()
    return MotionDecision(
        mission_id="mission-exec",
        decision_seq=seq,
        issued_at_ns=issued_ns,
        deadline_ns=deadline_ns,
        command=command,
        class_profile="r2026",
        goal=motion_goal,
        target=target_identity,
    )


def planner(event_seq, goal_seq, status, stamp_ns, attempt=None,
            reason="test", requested=None, effective=None, distance=1.0):
    if attempt is None:
        attempt = 0 if status in ("ACCEPTED", "CANCELLED") else 1
    requested = requested or goal()
    effective = effective or requested
    return PlannerStatusEvent(
        event_seq=event_seq,
        goal_seq=goal_seq,
        status=status,
        stamp_ns=stamp_ns,
        requested_goal=SequencedMotionGoal(goal_seq, requested),
        effective_goal=SequencedMotionGoal(goal_seq, effective),
        distance_to_goal=distance,
        planning_attempt=attempt,
        reason=reason,
    )


def odom(stamp_ns, x=1.0, frame="camera_init", speed=0.0):
    return OdomSample(
        stamp_ns=stamp_ns,
        frame_id=frame,
        x=x,
        y=2.0,
        z=2.2,
        vx=speed,
        vy=0.0,
        vz=0.0,
    )


class PlannerMotionExecutorTest(unittest.TestCase):
    def make_executor(self, **overrides):
        values = dict(
            executor_id="executor-test",
            arrival_distance_m=0.25,
            arrival_speed_mps=0.15,
            arrival_dwell_ns=100_000_000,
            odom_max_age_ns=200_000_000,
        )
        values.update(overrides)
        return PlannerMotionExecutor(PlannerMotionConfig(**values))

    def submit_search(self, executor):
        outcome = executor.submit_decision(decision(), BASE)
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual([item.kind for item in outcome.intents],
                         [PUBLISH_PLANNER_GOAL])
        self.assertEqual(outcome.intents[0].decision_seq, 1)
        self.assertEqual(len(outcome.events), 1)
        self.assertEqual(outcome.events[0].status, "ACCEPTED")
        self.assertEqual(outcome.events[0].stage, "DISPATCH")
        self.assertFalse(outcome.events[0].terminal)
        self.assertEqual(outcome.events[0].event_seq, 1)
        return outcome

    def make_ready(self, executor, goal_seq=1, event_seq=1,
                   start_ns=BASE + 10_000_000):
        accepted = executor.apply_planner_status(
            planner(event_seq, goal_seq, "ACCEPTED", start_ns), start_ns)
        self.assertTrue(accepted.accepted, accepted.reason)
        planning = executor.apply_planner_status(
            planner(event_seq + 1, goal_seq, "PLANNING",
                    start_ns + 10_000_000),
            start_ns + 10_000_000,
        )
        self.assertTrue(planning.accepted, planning.reason)
        ready = executor.apply_planner_status(
            planner(event_seq + 2, goal_seq, "TRAJECTORY_READY",
                    start_ns + 20_000_000),
            start_ns + 20_000_000,
        )
        self.assertTrue(ready.accepted, ready.reason)
        finished = executor.apply_planner_status(
            planner(event_seq + 3, goal_seq, "TRAJECTORY_FINISHED",
                    start_ns + 30_000_000),
            start_ns + 30_000_000,
        )
        self.assertTrue(finished.accepted, finished.reason)
        return start_ns + 30_000_000

    def arrive(self, executor, finish_ns):
        first_stamp = finish_ns + 10_000_000
        waiting = executor.apply_odom(
            odom(first_stamp), first_stamp)
        self.assertEqual(waiting.reason, "arrival_dwell_pending")
        last_stamp = first_stamp + 100_000_000
        return executor.apply_odom(odom(last_stamp), last_stamp)

    def test_cold_start_accepts_current_valid_decision(self):
        executor = self.make_executor()
        accepted = executor.submit_decision(
            decision(seq=42, command="APPROACH"), BASE)
        self.assertTrue(accepted.accepted, accepted.reason)
        self.assertEqual(accepted.intents[0].decision_seq, 42)

    def test_decision_is_continuous_idempotent_and_conflict_detecting(self):
        executor = self.make_executor()
        original = decision()
        executor.submit_decision(original, BASE)
        replay = executor.submit_decision(original, BASE)
        self.assertTrue(replay.accepted)
        self.assertEqual(replay.reason, "decision_idempotent")
        self.assertEqual(replay.intents, ())

        conflict = executor.submit_decision(
            replace(original, goal=goal(x=2.0)), BASE + 1)
        self.assertFalse(conflict.accepted)
        self.assertEqual(conflict.reason, "decision_sequence_conflict")
        self.assertEqual(conflict.events[0].decision_seq, 1)
        self.assertEqual(conflict.events[0].event_seq, 2)
        self.assertEqual(conflict.events[0].status, "FAILED")
        self.assertEqual(conflict.intents[0].kind, ABORT_SAFE)

    def test_decision_gap_is_allowed(self):
        executor = self.make_executor()
        self.submit_search(executor)
        outcome = executor.submit_decision(
            decision(seq=3, command="APPROACH", issued_ns=BASE + 1),
            BASE + 1,
        )
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual(outcome.intents[0].decision_seq, 3)

    def test_new_manager_decision_replaces_active_motion(self):
        executor = self.make_executor()
        self.submit_search(executor)
        approach = decision(
            seq=2,
            command="APPROACH",
            issued_ns=BASE + 1,
            deadline_ns=BASE + 5 * NSEC,
        )
        replacement = executor.submit_decision(approach, BASE + 1)
        self.assertTrue(replacement.accepted, replacement.reason)
        self.assertEqual(replacement.intents[0].kind, PUBLISH_PLANNER_GOAL)

        old_cancel = executor.apply_planner_status(
            planner(1, 1, "CANCELLED", BASE + 2), BASE + 2)
        self.assertTrue(old_cancel.accepted, old_cancel.reason)
        self.assertEqual(old_cancel.reason, "replacement_cancel_confirmed")

        other = self.make_executor()
        self.submit_search(other)
        replacement = other.submit_decision(
            decision(
                seq=2,
                command="RESUME",
                issued_ns=BASE + 1,
                deadline_ns=BASE + 5 * NSEC,
            ),
            BASE + 1,
        )
        self.assertTrue(replacement.accepted, replacement.reason)
        self.assertEqual(replacement.intents[0].kind, PUBLISH_PLANNER_GOAL)

    def test_return_home_may_replace_active_resume(self):
        executor = self.make_executor()
        first = decision(command="SEARCH", deadline_ns=BASE + 1)
        executor.submit_decision(first, BASE)
        executor.tick(BASE + 1)
        resume = decision(
            seq=2,
            command="RESUME",
            issued_ns=BASE + 2,
            deadline_ns=BASE + 5 * NSEC,
        )
        executor.submit_decision(resume, BASE + 2)
        returning = executor.submit_decision(
            decision(
                seq=3,
                command="RETURN_HOME",
                issued_ns=BASE + 3,
                deadline_ns=BASE + 5 * NSEC,
            ),
            BASE + 3,
        )
        self.assertTrue(returning.accepted, returning.reason)
        self.assertEqual(returning.intents[0].kind, PUBLISH_PLANNER_GOAL)

    def test_deadline_is_exclusive(self):
        executor = self.make_executor()
        search = decision(deadline_ns=BASE + 100)
        executor.submit_decision(search, BASE)
        before = executor.tick(search.deadline_ns - 1)
        self.assertEqual(before.reason, "executor_pending")
        expired = executor.tick(search.deadline_ns)
        self.assertEqual(expired.reason, "decision_timed_out")
        self.assertEqual(expired.events[0].status, "TIMED_OUT")
        self.assertEqual(expired.events[0].event_stamp_ns,
                         search.deadline_ns)
        self.assertEqual(expired.intents[0].kind, CANCEL_PLANNER_GOAL)

    def test_status_at_deadline_cannot_complete_decision(self):
        executor = self.make_executor()
        search = decision(deadline_ns=BASE + 100)
        executor.submit_decision(search, BASE)
        outcome = executor.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE + 100), BASE + 100)
        self.assertEqual(outcome.reason, "decision_timed_out")
        self.assertEqual(outcome.events[0].status, "TIMED_OUT")
        self.assertEqual(outcome.snapshot.last_planner_event_seq, 0)

    def test_requires_accepted_ready_finished_before_odom_dwell(self):
        executor = self.make_executor()
        self.submit_search(executor)
        pre_finish = executor.apply_odom(
            odom(BASE + 5_000_000), BASE + 5_000_000)
        self.assertEqual(pre_finish.reason, "waiting_for_trajectory_finished")

        finish_ns = self.make_ready(executor)
        arrived = self.arrive(executor, finish_ns)
        self.assertEqual(arrived.reason, "motion_succeeded")
        event = arrived.events[0]
        self.assertEqual(event.status, "SUCCEEDED")
        self.assertTrue(event.terminal)
        self.assertEqual(event.event_seq, 2)

    def test_same_frame_fresh_distance_speed_and_continuous_dwell(self):
        executor = self.make_executor()
        self.submit_search(executor)
        finish_ns = self.make_ready(executor)

        mismatch_ns = finish_ns + 10_000_000
        mismatch = executor.apply_odom(
            odom(mismatch_ns, frame="map"), mismatch_ns)
        self.assertEqual(mismatch.reason, "odom_goal_frame_mismatch")
        far_ns = mismatch_ns + 10_000_000
        self.assertEqual(
            executor.apply_odom(odom(far_ns, x=1.5), far_ns).reason,
            "arrival_threshold_not_met",
        )
        fast_ns = far_ns + 10_000_000
        self.assertEqual(
            executor.apply_odom(odom(fast_ns, speed=0.3), fast_ns).reason,
            "arrival_threshold_not_met",
        )
        stale_stamp = fast_ns + 10_000_000
        stale = executor.apply_odom(
            odom(stale_stamp), stale_stamp + 200_000_001)
        self.assertEqual(stale.reason, "odom_stale")

        first_ns = stale_stamp + 300_000_000
        first = executor.apply_odom(odom(first_ns), first_ns)
        self.assertEqual(first.reason, "arrival_dwell_pending")
        executor.tick(first_ns + 200_000_001)
        second_ns = first_ns + 300_000_000
        second = executor.apply_odom(odom(second_ns), second_ns)
        self.assertEqual(second.reason, "arrival_dwell_pending")
        arrived = executor.apply_odom(
            odom(second_ns + 100_000_000), second_ns + 100_000_000)
        self.assertEqual(arrived.reason, "motion_succeeded")

    def test_approach_arrival_only_progresses_and_hands_off(self):
        executor = self.make_executor()
        self.submit_search(executor)
        approach_decision = decision(
            seq=2,
            command="APPROACH",
            issued_ns=BASE + 1,
            deadline_ns=BASE + 5 * NSEC,
        )
        executor.submit_decision(approach_decision, BASE + 1)
        cancelled = executor.apply_planner_status(
            planner(1, 1, "CANCELLED", BASE + 2), BASE + 2)
        self.assertEqual(cancelled.reason, "replacement_cancel_confirmed")
        finish_ns = self.make_ready(
            executor, goal_seq=2, event_seq=2,
            start_ns=BASE + 10_000_000)
        arrived = self.arrive(executor, finish_ns)

        self.assertEqual(arrived.intents[0].kind,
                         START_TARGET_TRANSACTION)
        self.assertEqual(len(arrived.events), 1)
        event = arrived.events[0]
        self.assertEqual(event.status, "PROGRESS")
        self.assertFalse(event.terminal)
        self.assertNotEqual(event.status, "SUCCEEDED")
        self.assertEqual(event.target_id, approach_decision.target.target_id)
        self.assertEqual(event.target_first_seen_ns,
                         approach_decision.target.first_seen_ns)
        self.assertEqual(event.event_seq, 3)
        self.assertTrue(arrived.snapshot.active_handed_off)

    def test_land_and_abort_only_create_external_intents(self):
        for command, expected_kind in (
                ("LAND", LAND_EXTERNAL), ("ABORT", ABORT_SAFE)):
            executor = self.make_executor()
            first = decision(deadline_ns=BASE + 1)
            executor.submit_decision(first, BASE)
            executor.tick(BASE + 1)
            external = decision(
                seq=2,
                command=command,
                issued_ns=BASE + 2,
                deadline_ns=BASE + NSEC,
                motion_goal=None,
            )
            outcome = executor.submit_decision(external, BASE + 2)
            self.assertTrue(outcome.accepted, outcome.reason)
            self.assertEqual([item.kind for item in outcome.intents],
                             [expected_kind])
            self.assertEqual(outcome.events, ())
            self.assertIsNone(outcome.intents[0].goal)

    def test_planner_event_is_idempotent_but_conflict_fails_closed(self):
        executor = self.make_executor()
        self.submit_search(executor)
        accepted = planner(1, 1, "ACCEPTED", BASE + 1)
        executor.apply_planner_status(accepted, BASE + 1)
        replay = executor.apply_planner_status(accepted, BASE + 1)
        self.assertTrue(replay.accepted)
        self.assertEqual(replay.reason, "planner_event_idempotent")
        conflict = executor.apply_planner_status(
            replace(accepted, status="PLANNING"), BASE + 2)
        self.assertFalse(conflict.accepted)
        self.assertEqual(conflict.reason, "planner_event_sequence_conflict")
        self.assertEqual(conflict.intents[0].kind, ABORT_SAFE)

    def test_stale_planner_event_is_ignored(self):
        executor = self.make_executor()
        self.submit_search(executor)
        executor.apply_planner_status(
            planner(2, 1, "ACCEPTED", BASE + 1), BASE + 1)
        rolled_back = executor.apply_planner_status(
            planner(1, 1, "PLANNING", BASE + 2), BASE + 2)
        self.assertFalse(rolled_back.accepted)
        self.assertEqual(rolled_back.reason, "stale_planner_event_ignored")
        self.assertFalse(rolled_back.snapshot.faulted)

    def test_unknown_planner_goal_is_ignored(self):
        executor = self.make_executor()
        self.submit_search(executor)
        unknown = executor.apply_planner_status(
            planner(1, 99, "ACCEPTED", BASE + 1), BASE + 1)
        self.assertFalse(unknown.accepted)
        self.assertEqual(unknown.reason, "foreign_planner_goal_ignored")
        self.assertFalse(unknown.snapshot.faulted)

    def test_finished_without_accepted_and_ready_fails_closed(self):
        executor = self.make_executor()
        self.submit_search(executor)
        finished = executor.apply_planner_status(
            planner(1, 1, "TRAJECTORY_FINISHED", BASE + 1), BASE + 1)
        self.assertFalse(finished.accepted)
        self.assertEqual(finished.reason, "planner_status_before_accepted")

        other = self.make_executor()
        self.submit_search(other)
        other.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE + 1), BASE + 1)
        other.apply_planner_status(
            planner(2, 1, "PLANNING", BASE + 2), BASE + 2)
        without_ready = other.apply_planner_status(
            planner(3, 1, "TRAJECTORY_FINISHED", BASE + 3), BASE + 3)
        self.assertFalse(without_ready.accepted)
        self.assertEqual(
            without_ready.reason, "trajectory_finished_without_ready")

    def test_replanning_requires_a_new_ready_event(self):
        executor = self.make_executor()
        self.submit_search(executor)
        executor.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE + 1), BASE + 1)
        executor.apply_planner_status(
            planner(2, 1, "PLANNING", BASE + 2), BASE + 2)
        executor.apply_planner_status(
            planner(3, 1, "TRAJECTORY_READY", BASE + 3), BASE + 3)
        executor.apply_planner_status(
            planner(4, 1, "REPLANNING", BASE + 4, attempt=2), BASE + 4)
        finished = executor.apply_planner_status(
            planner(5, 1, "TRAJECTORY_FINISHED", BASE + 5, attempt=2),
            BASE + 5,
        )
        self.assertFalse(finished.accepted)
        self.assertEqual(finished.reason,
                         "trajectory_finished_without_ready")

    def test_executor_result_sequence_is_global_and_identity_is_frozen(self):
        executor = self.make_executor()
        self.submit_search(executor)
        finish_ns = self.make_ready(executor)
        first = self.arrive(executor, finish_ns).events[0]
        self.assertEqual(first.event_seq, 2)
        self.assertEqual(first.mission_id, "mission-exec")
        self.assertEqual(first.executor_id, "executor-test")
        self.assertEqual(first.decision_seq, 1)

        second_decision = decision(
            seq=2,
            command="RESUME",
            issued_ns=finish_ns + 200_000_000,
            deadline_ns=finish_ns + 300_000_000,
        )
        executor.submit_decision(second_decision,
                                 second_decision.issued_at_ns)
        second = executor.tick(second_decision.deadline_ns).events[0]
        self.assertEqual(second.event_seq, 4)
        self.assertEqual(second.decision_seq, 2)
        self.assertEqual(second.command, "RESUME")

    def test_motion_dispatch_ack_freezes_identity_and_replay_is_silent(self):
        executor = self.make_executor()
        self.submit_search(executor)
        approach = decision(
            seq=2,
            command="APPROACH",
            issued_ns=BASE + 1,
            deadline_ns=BASE + 5 * NSEC,
            target_identity=target(target_id=42, class_name="red_cross"),
        )
        dispatched = executor.submit_decision(approach, BASE + 1)
        self.assertEqual(len(dispatched.events), 1)
        event = dispatched.events[0]
        self.assertEqual(event.status, "ACCEPTED")
        self.assertEqual(event.stage, "DISPATCH")
        self.assertFalse(event.terminal)
        self.assertEqual(event.event_seq, 2)
        self.assertEqual(event.decision_seq, 2)
        self.assertEqual(event.target_id, 42)
        self.assertEqual(event.target_class, "red_cross")
        self.assertEqual(event.executor_id, "executor-test")

        replay = executor.submit_decision(approach, BASE + 1)
        self.assertTrue(replay.accepted)
        self.assertEqual(replay.reason, "decision_idempotent")
        self.assertEqual(replay.intents, ())
        self.assertEqual(replay.events, ())
        self.assertEqual(replay.snapshot.executor_event_seq, 2)

    def test_zero_target_id_is_valid_and_has_target_is_explicit(self):
        executor = self.make_executor()
        self.submit_search(executor)
        zero_identity = target(target_id=0)
        approach = decision(
            seq=2,
            command="APPROACH",
            issued_ns=BASE + 1,
            deadline_ns=BASE + 5 * NSEC,
            target_identity=zero_identity,
        )
        dispatched = executor.submit_decision(approach, BASE + 1)
        self.assertTrue(dispatched.accepted, dispatched.reason)
        event = dispatched.events[0]
        self.assertTrue(event.has_target)
        self.assertEqual(event.target_id, 0)
        self.assertEqual(event.target_first_seen_ns,
                         zero_identity.first_seen_ns)
        self.assertEqual(dispatched.intents[0].target, zero_identity)

        with self.assertRaises(ValueError):
            target(target_id=-1)

    def test_replacement_requires_old_cancel_before_new_accept(self):
        wrong = self.make_executor()
        self.submit_search(wrong)
        replacement = decision(
            seq=2,
            command="APPROACH",
            issued_ns=BASE + 1,
            deadline_ns=BASE + 5 * NSEC,
        )
        wrong.submit_decision(replacement, BASE + 1)
        accepted_too_early = wrong.apply_planner_status(
            planner(1, 2, "ACCEPTED", BASE + 2), BASE + 2)
        self.assertFalse(accepted_too_early.accepted)
        self.assertEqual(
            accepted_too_early.reason,
            "replacement_accepted_before_cancel",
        )
        self.assertEqual(accepted_too_early.intents[0].kind, ABORT_SAFE)

        correct = self.make_executor()
        self.submit_search(correct)
        correct.submit_decision(replacement, BASE + 1)
        cancelled = correct.apply_planner_status(
            planner(1, 1, "CANCELLED", BASE + 2), BASE + 2)
        self.assertTrue(cancelled.accepted, cancelled.reason)
        self.assertEqual(cancelled.reason, "replacement_cancel_confirmed")
        self.assertEqual(cancelled.snapshot.awaiting_cancel_goal_seq, 0)
        accepted = correct.apply_planner_status(
            planner(2, 2, "ACCEPTED", BASE + 3), BASE + 3)
        self.assertTrue(accepted.accepted, accepted.reason)

    def test_finished_search_replacement_does_not_wait_for_cancel(self):
        executor = self.make_executor()
        self.submit_search(executor)
        finish_ns = self.make_ready(executor)
        replacement = decision(
            seq=2,
            command="APPROACH",
            issued_ns=finish_ns + 1,
            deadline_ns=BASE + 5 * NSEC,
        )
        dispatched = executor.submit_decision(replacement, finish_ns + 1)
        self.assertTrue(dispatched.accepted, dispatched.reason)
        self.assertEqual(dispatched.snapshot.awaiting_cancel_goal_seq, 0)
        accepted = executor.apply_planner_status(
            planner(5, 2, "ACCEPTED", finish_ns + 2), finish_ns + 2)
        self.assertTrue(accepted.accepted, accepted.reason)

    def test_planner_goal_payload_is_correlated_and_bounded(self):
        raw = goal()
        base_event = planner(1, 1, "ACCEPTED", BASE + 1)
        cases = (
            (
                replace(
                    base_event,
                    requested_goal=SequencedMotionGoal(2, raw),
                ),
                "planner_goal_sequence_mismatch",
            ),
            (
                replace(
                    base_event,
                    effective_goal=SequencedMotionGoal(2, raw),
                ),
                "planner_goal_sequence_mismatch",
            ),
            (
                replace(
                    base_event,
                    requested_goal=SequencedMotionGoal(1, goal(x=1.1)),
                ),
                "planner_requested_goal_mismatch",
            ),
            (
                replace(
                    base_event,
                    effective_goal=SequencedMotionGoal(
                        1, goal(frame="map")),
                ),
                "planner_effective_goal_frame_mismatch",
            ),
            (
                replace(
                    base_event,
                    effective_goal=SequencedMotionGoal(
                        1, MotionGoal("camera_init", 1.0, 2.0, 4.01)),
                ),
                "planner_effective_goal_height_invalid",
            ),
            (
                replace(
                    base_event,
                    effective_goal=SequencedMotionGoal(
                        1, goal(x=2.100001)),
                ),
                "planner_effective_goal_offset_exceeded",
            ),
        )
        for event, expected_reason in cases:
            with self.subTest(expected_reason=expected_reason):
                executor = self.make_executor()
                self.submit_search(executor)
                outcome = executor.apply_planner_status(
                    event, event.stamp_ns)
                self.assertFalse(outcome.accepted)
                self.assertEqual(outcome.reason, expected_reason)
                self.assertEqual(outcome.intents[0].kind, ABORT_SAFE)

    def test_arrival_uses_latest_effective_goal(self):
        executor = self.make_executor()
        self.submit_search(executor)
        adjusted = goal(x=1.5)
        executor.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE + 10_000_000),
            BASE + 10_000_000,
        )
        executor.apply_planner_status(
            planner(2, 1, "PLANNING", BASE + 20_000_000),
            BASE + 20_000_000,
        )
        executor.apply_planner_status(
            planner(
                3, 1, "TRAJECTORY_READY", BASE + 30_000_000,
                effective=adjusted,
            ),
            BASE + 30_000_000,
        )
        finished = executor.apply_planner_status(
            planner(
                4, 1, "TRAJECTORY_FINISHED", BASE + 40_000_000,
                effective=adjusted,
            ),
            BASE + 40_000_000,
        )
        self.assertTrue(finished.accepted, finished.reason)

        raw_position = executor.apply_odom(
            odom(BASE + 50_000_000, x=1.0), BASE + 50_000_000)
        self.assertEqual(raw_position.reason, "arrival_threshold_not_met")
        first = executor.apply_odom(
            odom(BASE + 60_000_000, x=1.5), BASE + 60_000_000)
        self.assertEqual(first.reason, "arrival_dwell_pending")
        arrived = executor.apply_odom(
            odom(BASE + 160_000_000, x=1.5), BASE + 160_000_000)
        self.assertEqual(arrived.reason, "motion_succeeded")

    def test_submit_enforces_motion_contract_not_profile_policy(self):
        first_decision_cases = (
            (
                replace(decision(), goal=goal(frame="map")),
                "decision_goal_frame_mismatch",
            ),
            (
                replace(
                    decision(),
                    goal=MotionGoal("camera_init", 1.0, 2.0, 4.01),
                ),
                "decision_goal_height_invalid",
            ),
        )
        for invalid, expected_reason in first_decision_cases:
            with self.subTest(expected_reason=expected_reason):
                executor = self.make_executor()
                outcome = executor.submit_decision(invalid, BASE)
                self.assertFalse(outcome.accepted)
                self.assertEqual(outcome.reason, expected_reason)

        executor = self.make_executor()
        accepted = executor.submit_decision(
            replace(decision(seq=17), class_profile="future"), BASE)
        self.assertTrue(accepted.accepted, accepted.reason)

    def test_new_sequence_atomically_retires_expired_old_decision(self):
        executor = self.make_executor()
        first = decision(deadline_ns=BASE + 100)
        self.assertTrue(executor.submit_decision(first, BASE).accepted)
        next_decision = decision(
            seq=2,
            command="RESUME",
            issued_ns=first.deadline_ns,
            deadline_ns=BASE + NSEC,
        )
        outcome = executor.submit_decision(
            next_decision, first.deadline_ns)
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual(outcome.reason, "planner_goal_intent")
        self.assertEqual(outcome.events[0].status, "ACCEPTED")
        self.assertEqual(outcome.events[0].decision_seq, 2)
        self.assertEqual(outcome.events[0].event_seq, 2)
        self.assertEqual(outcome.intents[0].kind, PUBLISH_PLANNER_GOAL)

    def test_planner_acceptance_timeout_is_exclusive(self):
        executor = self.make_executor()
        self.submit_search(executor)
        timeout_at = BASE + executor.config.planner_accept_timeout_ns
        before = executor.tick(timeout_at - 1)
        self.assertEqual(before.reason, "executor_pending")
        timed_out = executor.tick(timeout_at)
        self.assertEqual(timed_out.reason, "planner_accept_timed_out")
        self.assertEqual(timed_out.events[0].status, "TIMED_OUT")
        self.assertEqual(timed_out.events[0].stage, "PLANNER")
        self.assertEqual(timed_out.events[0].event_seq, 2)
        self.assertEqual(timed_out.intents[0].kind, CANCEL_PLANNER_GOAL)

    def test_planner_status_honors_dispatch_and_future_tolerance(self):
        before_dispatch = self.make_executor()
        delayed = replace(decision(), issued_at_ns=BASE - 100)
        before_dispatch.submit_decision(delayed, BASE)
        early = before_dispatch.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE - 1), BASE)
        self.assertFalse(early.accepted)
        self.assertEqual(early.reason, "planner_event_precedes_dispatch")

        too_future = self.make_executor()
        self.submit_search(too_future)
        future_stamp = (
            BASE + too_future.config.source_future_tolerance_ns + 1)
        future = too_future.apply_planner_status(
            planner(1, 1, "ACCEPTED", future_stamp), BASE)
        self.assertFalse(future.accepted)
        self.assertEqual(future.reason, "planner_event_from_future")

        tolerated = self.make_executor()
        self.submit_search(tolerated)
        tolerated_stamp = (
            BASE + tolerated.config.source_future_tolerance_ns)
        accepted = tolerated.apply_planner_status(
            planner(1, 1, "ACCEPTED", tolerated_stamp), BASE)
        self.assertTrue(accepted.accepted, accepted.reason)

    def test_future_odom_does_not_pollute_last_valid_source(self):
        executor = self.make_executor()
        self.submit_search(executor)
        finish_ns = self.make_ready(executor)
        receipt_ns = finish_ns + 100_000_000
        invalid_future_stamp = (
            receipt_ns + executor.config.source_future_tolerance_ns + 1)
        future = executor.apply_odom(
            odom(invalid_future_stamp), receipt_ns)
        self.assertFalse(future.accepted)
        self.assertEqual(future.reason, "odom_from_future")

        valid_stamp = finish_ns + 10_000_000
        valid = executor.apply_odom(
            odom(valid_stamp), receipt_ns + 1)
        self.assertTrue(valid.accepted, valid.reason)
        self.assertEqual(valid.reason, "arrival_dwell_pending")
        arrived_stamp = valid_stamp + 100_000_000
        arrived = executor.apply_odom(
            odom(arrived_stamp), receipt_ns + 10_000_000)
        self.assertEqual(arrived.reason, "motion_succeeded")

    def test_planning_attempt_follows_producer_sequence(self):
        executor = self.make_executor()
        self.submit_search(executor)
        executor.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE + 1), BASE + 1)
        executor.apply_planner_status(
            planner(2, 1, "PLANNING", BASE + 2, attempt=1), BASE + 2)
        executor.apply_planner_status(
            planner(3, 1, "FAILED_ATTEMPT", BASE + 3, attempt=1),
            BASE + 3,
        )
        retry = executor.apply_planner_status(
            planner(4, 1, "PLANNING", BASE + 4, attempt=2), BASE + 4)
        self.assertTrue(retry.accepted, retry.reason)
        ready = executor.apply_planner_status(
            planner(5, 1, "TRAJECTORY_READY", BASE + 5, attempt=2),
            BASE + 5,
        )
        self.assertTrue(ready.accepted, ready.reason)

        rollback = self.make_executor()
        self.submit_search(rollback)
        rollback.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE + 1), BASE + 1)
        rollback.apply_planner_status(
            planner(2, 1, "PLANNING", BASE + 2, attempt=1), BASE + 2)
        invalid = rollback.apply_planner_status(
            planner(3, 1, "TRAJECTORY_READY", BASE + 3, attempt=0),
            BASE + 3,
        )
        self.assertFalse(invalid.accepted)
        self.assertEqual(invalid.reason, "planner_attempt_inconsistent")

        limited = self.make_executor(max_planning_attempts=1)
        self.submit_search(limited)
        limited.apply_planner_status(
            planner(1, 1, "ACCEPTED", BASE + 1), BASE + 1)
        limited.apply_planner_status(
            planner(2, 1, "PLANNING", BASE + 2, attempt=1), BASE + 2)
        limited.apply_planner_status(
            planner(3, 1, "FAILED_ATTEMPT", BASE + 3, attempt=1),
            BASE + 3,
        )
        exceeded = limited.apply_planner_status(
            planner(4, 1, "PLANNING", BASE + 4, attempt=2), BASE + 4)
        self.assertFalse(exceeded.accepted)
        self.assertEqual(exceeded.reason, "planner_attempt_limit_exceeded")

    def test_hold_is_safe_external_intent_and_align_is_rejected(self):
        executor = self.make_executor()
        self.submit_search(executor)
        hold = decision(
            seq=2,
            command="HOLD",
            issued_ns=BASE + 1,
            deadline_ns=BASE + NSEC,
            motion_goal=None,
        )
        held = executor.submit_decision(hold, BASE + 1)
        self.assertTrue(held.accepted, held.reason)
        self.assertEqual([item.kind for item in held.intents], [ABORT_SAFE])
        self.assertEqual(held.events, ())

        align_executor = self.make_executor()
        align = decision(
            command="ALIGN",
            motion_goal=None,
            target_identity=None,
        )
        rejected = align_executor.submit_decision(align, BASE)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason,
                         "align_not_owned_by_motion_executor")
        self.assertEqual(rejected.intents[0].kind, ABORT_SAFE)
        self.assertEqual(rejected.events, ())

    def test_planner_distance_accepts_nan_but_rejects_infinity(self):
        diagnostic_nan = planner(
            1, 1, "ACCEPTED", BASE + 1, distance=float("nan"))
        self.assertTrue(diagnostic_nan.distance_to_goal !=
                        diagnostic_nan.distance_to_goal)
        with self.assertRaises(ValueError):
            planner(1, 1, "ACCEPTED", BASE + 1,
                    distance=float("inf"))

    def test_executor_clock_rollback_fails_closed_once(self):
        executor = self.make_executor()
        self.submit_search(executor)
        executor.tick(BASE + 100)
        rollback = executor.tick(BASE + 99)
        self.assertFalse(rollback.accepted)
        self.assertEqual(rollback.reason, "executor_clock_rollback")
        self.assertEqual(rollback.intents[0].kind, ABORT_SAFE)
        event_seq = rollback.events[0].event_seq
        repeated = executor.tick(BASE + 98)
        self.assertFalse(repeated.accepted)
        self.assertEqual(repeated.intents, ())
        self.assertEqual(repeated.events, ())
        self.assertEqual(repeated.snapshot.executor_event_seq, event_seq)


if __name__ == "__main__":
    unittest.main()
