#!/usr/bin/env python3
from dataclasses import replace
import unittest
from uav_mission.planner_execution import (MotionDecision, MotionGoal,
    OdomSample, PlannerMotionConfig, PlannerMotionExecutor,
    PlannerStatusEvent, SequencedMotionGoal, TargetIdentity)

NSEC = 1_000_000_000
BASE = 1_000 * NSEC

def goal(x=1.0, frame="camera_init", z=2.2):
    return MotionGoal(frame, x, 2.0, z)

def target():
    return TargetIdentity(0, 900*NSEC, 999*NSEC, "bridge", 1, 2)

def decision(seq=8, command="SEARCH", issued=BASE, deadline=None):
    motion = command in (
        "SEARCH", "RESUME", "APPROACH", "RETURN_HOME", "ABORT")
    return MotionDecision("mission", seq, issued, deadline or issued+5*NSEC,
        command, "r2026", goal() if motion else None,
        target() if command == "APPROACH" else None)

def status(event_seq, goal_seq, name, stamp, attempt=None):
    attempt = (0 if name in ("ACCEPTED", "CANCELLED") else 1) \
        if attempt is None else attempt
    item = SequencedMotionGoal(goal_seq, goal())
    return PlannerStatusEvent(event_seq, goal_seq, name, stamp, item, item,
                              0.0, attempt, "test")

def odom(stamp, speed=0.0):
    return OdomSample(stamp, "camera_init", 1.0, 2.0, 2.2,
                      speed, 0.0, 0.0)

class PlannerMotionExecutorTest(unittest.TestCase):
    def make(self):
        return PlannerMotionExecutor(PlannerMotionConfig(
            executor_id="executor-test", arrival_distance_m=.25,
            arrival_speed_mps=.15, arrival_dwell_ns=100_000_000,
            odom_max_age_ns=200_000_000))

    def dispatch(self, executor, item=None, now=BASE):
        outcome = executor.submit_decision(item or decision(), now)
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertIsNotNone(outcome.planner_goal)
        self.assertEqual(outcome.events[0].status, "ACCEPTED")
        return outcome

    def finish(self, executor, seq=8, first_event=1):
        now = BASE + 10_000_000
        for offset, name in enumerate(("ACCEPTED", "PLANNING",
                "TRAJECTORY_READY", "TRAJECTORY_FINISHED")):
            outcome = executor.apply_planner_status(status(
                first_event+offset, seq, name, now+offset*10_000_000),
                now+offset*10_000_000)
            self.assertTrue(outcome.accepted, outcome.reason)
        return now + 30_000_000

    def test_late_start_and_zero_target_id(self):
        out = self.dispatch(self.make(), decision(42, "APPROACH"))
        self.assertEqual(out.planner_goal.decision_seq, 42)
        self.assertEqual(out.events[0].target_id, 0)

    def test_replay_conflict_and_gap(self):
        executor = self.make(); original = decision()
        self.dispatch(executor, original)
        self.assertIsNone(executor.submit_decision(original, BASE+1).planner_goal)
        conflict = executor.submit_decision(
            replace(original, goal=goal(2.0)), BASE+2)
        self.assertFalse(conflict.accepted)
        self.assertEqual(conflict.events[0].status, "FAILED")
        self.assertFalse(conflict.snapshot.faulted)
        executor = self.make(); self.dispatch(executor)
        gap = executor.submit_decision(decision(20, "APPROACH", BASE+1), BASE+1)
        self.assertTrue(gap.accepted)
        self.assertEqual(executor.submit_decision(decision(7), BASE+2).reason,
                         "stale_decision_ignored")

    def test_replacement_cancel_fence(self):
        executor = self.make(); self.dispatch(executor)
        self.dispatch(executor, decision(20, "APPROACH", BASE+1), BASE+1)
        early = executor.apply_planner_status(
            status(1, 20, "ACCEPTED", BASE+2), BASE+2)
        self.assertFalse(early.accepted)
        executor = self.make(); self.dispatch(executor)
        self.dispatch(executor, decision(20, "APPROACH", BASE+1), BASE+1)
        cancel = executor.apply_planner_status(
            status(1, 8, "CANCELLED", BASE+2), BASE+2)
        self.assertEqual(cancel.reason, "replacement_cancel_confirmed")
        accepted = executor.apply_planner_status(
            status(2, 20, "ACCEPTED", BASE+3), BASE+3)
        self.assertTrue(accepted.accepted, accepted.reason)

    def test_unknown_and_stale_telemetry_ignored(self):
        executor = self.make(); self.dispatch(executor)
        unknown = executor.apply_planner_status(
            status(5, 99, "ACCEPTED", BASE+1), BASE+1)
        self.assertEqual(unknown.reason, "foreign_planner_goal_ignored")
        stale = executor.apply_planner_status(
            status(4, 99, "ACCEPTED", BASE+2), BASE+2)
        self.assertEqual(stale.reason, "foreign_planner_goal_ignored")
        self.assertFalse(stale.snapshot.faulted)

    def test_finished_requires_odom_dwell(self):
        executor = self.make(); self.dispatch(executor)
        finished = self.finish(executor)
        moving = executor.apply_odom(odom(finished+1, .2), finished+1)
        self.assertEqual(moving.reason, "arrival_threshold_not_met")
        first = executor.apply_odom(odom(finished+10), finished+10)
        self.assertEqual(first.reason, "arrival_dwell_pending")
        arrived = executor.apply_odom(odom(finished+100_000_010),
                                      finished+100_000_010)
        self.assertEqual(arrived.events[0].status, "SUCCEEDED")

    def test_approach_handoff_is_nonterminal(self):
        executor = self.make(); self.dispatch(executor, decision(8, "APPROACH"))
        finished = self.finish(executor)
        executor.apply_odom(odom(finished+10), finished+10)
        out = executor.apply_odom(odom(finished+100_000_010),
                                  finished+100_000_010)
        self.assertEqual(out.handoff, "TARGET_TRANSACTION")
        self.assertEqual(out.events[0].status, "PROGRESS")
        self.assertFalse(out.events[0].terminal)

    def test_target_handoff_reuses_planner_result_sequence(self):
        executor = self.make(); self.dispatch(executor, decision(8, "APPROACH"))
        finished = self.finish(executor)
        executor.apply_odom(odom(finished+10), finished+10)
        handoff_time = finished + 100_000_010
        handoff = executor.apply_odom(odom(handoff_time), handoff_time)

        capture = executor.report_target_stage(
            8, handoff_time+1, "STARTED", "CAPTURE",
            reason="semantic_target_recaptured")
        alignment = executor.report_target_stage(
            8, handoff_time+2, "STARTED", "ALIGNMENT",
            reason="strict_alignment_context_valid")
        release = executor.report_target_stage(
            8, handoff_time+3, "PROGRESS", "RELEASE",
            payload_committed=True, reason="release_ack_success",
            evidence_source="guarded_servo_proxy:4")
        recovery = executor.report_target_stage(
            8, handoff_time+4, "SUCCEEDED", "RECOVERY", terminal=True,
            reason="recovery_confirmed")

        events = [handoff.events[0], capture.events[0], alignment.events[0],
                  release.events[0], recovery.events[0]]
        self.assertEqual(
            [event.event_seq for event in events],
            list(range(events[0].event_seq, events[0].event_seq+5)))
        self.assertTrue(release.events[0].payload_committed)
        self.assertFalse(recovery.events[0].payload_committed)
        self.assertTrue(recovery.events[0].terminal)
        self.assertFalse(executor.report_target_stage(
            8, handoff_time+5, "PROGRESS", "ALIGNMENT",
            reason="late_progress").accepted)

    def test_target_success_requires_release_commit(self):
        executor = self.make(); self.dispatch(executor, decision(8, "APPROACH"))
        finished = self.finish(executor)
        executor.apply_odom(odom(finished+10), finished+10)
        handoff_time = finished + 100_000_010
        executor.apply_odom(odom(handoff_time), handoff_time)
        outcome = executor.report_target_stage(
            8, handoff_time+1, "SUCCEEDED", "RECOVERY", terminal=True,
            reason="recovery_confirmed")
        self.assertFalse(outcome.accepted)
        self.assertEqual(outcome.reason, "target_success_without_commit")

    def test_release_timeout_allows_one_exact_late_commit(self):
        executor = self.make(); self.dispatch(executor, decision(8, "APPROACH"))
        finished = self.finish(executor)
        executor.apply_odom(odom(finished+10), finished+10)
        handoff_time = finished + 100_000_010
        executor.apply_odom(odom(handoff_time), handoff_time)
        timed_out = executor.report_target_stage(
            8, BASE+5*NSEC, "TIMED_OUT", "RELEASE", terminal=True,
            reason="release_result_deadline_reached")
        self.assertTrue(timed_out.accepted, timed_out.reason)
        self.assertTrue(timed_out.events[0].terminal)
        self.assertFalse(timed_out.events[0].retryable)

        committed = executor.report_target_stage(
            8, BASE+5*NSEC+1, "PROGRESS", "RELEASE",
            payload_committed=True, reason="release_ack_success",
            evidence_source="guarded_servo_proxy:1")
        self.assertTrue(committed.accepted, committed.reason)
        self.assertTrue(committed.events[0].payload_committed)
        self.assertEqual(committed.events[0].event_seq,
                         timed_out.events[0].event_seq+1)
        duplicate = executor.report_target_stage(
            8, BASE+5*NSEC+2, "PROGRESS", "RELEASE",
            payload_committed=True, reason="release_ack_success",
            evidence_source="guarded_servo_proxy:1")
        self.assertFalse(duplicate.accepted)

    def test_other_target_timeout_does_not_allow_late_commit(self):
        executor = self.make(); self.dispatch(executor, decision(8, "APPROACH"))
        finished = self.finish(executor)
        executor.apply_odom(odom(finished+10), finished+10)
        handoff_time = finished + 100_000_010
        executor.apply_odom(odom(handoff_time), handoff_time)
        timed_out = executor.report_target_stage(
            8, BASE+5*NSEC, "TIMED_OUT", "CAPTURE", terminal=True,
            retryable=True, reason="target_capture_deadline_reached")
        self.assertTrue(timed_out.accepted, timed_out.reason)
        late = executor.report_target_stage(
            8, BASE+5*NSEC+1, "PROGRESS", "RELEASE",
            payload_committed=True, reason="release_ack_success",
            evidence_source="guarded_servo_proxy:1")
        self.assertFalse(late.accepted)

    def test_late_release_commit_survives_one_replacement(self):
        executor = self.make(); self.dispatch(executor, decision(8, "APPROACH"))
        finished = self.finish(executor)
        executor.apply_odom(odom(finished+10), finished+10)
        handoff_time = finished + 100_000_010
        executor.apply_odom(odom(handoff_time), handoff_time)
        timed_out = executor.report_target_stage(
            8, BASE+5*NSEC, "TIMED_OUT", "RELEASE", terminal=True,
            reason="release_result_deadline_reached")
        self.assertTrue(timed_out.accepted, timed_out.reason)

        replacement_time = BASE + 5*NSEC + 1
        replacement = decision(
            20, "SEARCH", replacement_time, replacement_time+5*NSEC)
        self.dispatch(executor, replacement, replacement_time)
        committed = executor.report_target_stage(
            8, replacement_time+1, "PROGRESS", "RELEASE",
            payload_committed=True, reason="release_ack_success",
            evidence_source="guarded_servo_proxy:2")
        self.assertTrue(committed.accepted, committed.reason)
        self.assertTrue(committed.events[0].payload_committed)
        self.assertEqual(committed.events[0].decision_seq, 8)
        self.assertFalse(executor.report_target_stage(
            8, replacement_time+2, "PROGRESS", "RELEASE",
            payload_committed=True, reason="release_ack_success",
            evidence_source="guarded_servo_proxy:2").accepted)

    def test_landing_handoff_reports_observed_terminal(self):
        executor = self.make()
        handoff = executor.submit_decision(decision(9, "LAND"), BASE)
        self.assertEqual(handoff.handoff, "LAND")
        started = executor.report_landing(
            9, BASE+1, "STARTED", False, "landing_command_forwarded")
        landed = executor.report_landing(
            9, BASE+2, "SUCCEEDED", True, "landing_settle_confirmed")
        self.assertEqual(started.events[0].stage, "LANDING")
        self.assertEqual(landed.events[0].event_seq,
                         started.events[0].event_seq+1)
        self.assertTrue(landed.events[0].terminal)

    def test_deadlines_emit_typed_timeout(self):
        executor = self.make(); self.dispatch(executor, decision(deadline=BASE+100))
        out = executor.tick(BASE+100)
        self.assertEqual(out.events[0].status, "TIMED_OUT")
        self.assertEqual(out.handoff, "CANCEL_REQUIRED")
        executor = self.make(); self.dispatch(executor)
        self.assertEqual(executor.tick(BASE+2*NSEC).reason,
                         "planner_accept_timed_out")

    def test_land_is_handoff_and_hold_is_rejected(self):
        out = self.make().submit_decision(decision(command="LAND"), BASE)
        self.assertEqual(out.handoff, "LAND")
        self.assertIsNone(out.planner_goal)
        self.assertEqual(out.events, ())
        rejected = self.make().submit_decision(
            decision(command="HOLD"), BASE)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "hold_not_supported")

    def test_abort_is_a_typed_hold_motion(self):
        executor = self.make()
        item = decision(command="ABORT")
        dispatched = self.dispatch(executor, item)
        self.assertEqual(dispatched.planner_goal.command, "ABORT")
        finished = self.finish(executor)
        executor.apply_odom(odom(finished+10), finished+10)
        stopped = executor.apply_odom(
            odom(finished+100_000_010), finished+100_000_010)
        self.assertEqual(stopped.events[0].command, "ABORT")
        self.assertEqual(stopped.events[0].status, "SUCCEEDED")
        self.assertTrue(stopped.events[0].terminal)

    def test_motion_contract(self):
        for item in (replace(decision(), goal=goal(frame="map")),
                     replace(decision(), goal=goal(z=4.1))):
            self.assertFalse(self.make().submit_decision(item, BASE).accepted)
        out = self.make().submit_decision(
            decision(issued=BASE, deadline=BASE+1), BASE+1)
        self.assertFalse(out.accepted)

    def test_configurable_frame_orientation_and_planner_progress_events(self):
        executor = PlannerMotionExecutor(PlannerMotionConfig(mission_frame="map"))
        item = replace(decision(), goal=MotionGoal(
            "map", 1.0, 2.0, 2.2, 0.0, 0.0, 1.0, 1.0))
        dispatched = executor.submit_decision(item, BASE)
        self.assertTrue(dispatched.accepted, dispatched.reason)
        self.assertAlmostEqual(dispatched.planner_goal.goal.qz, 2 ** -0.5)
        planner_goal = dispatched.planner_goal.goal
        sequenced = SequencedMotionGoal(8, planner_goal)
        accepted = executor.apply_planner_status(PlannerStatusEvent(
            1, 8, "ACCEPTED", BASE+1, sequenced, sequenced, 0.0, 0, ""),
            BASE+1)
        self.assertEqual((accepted.events[0].status, accepted.events[0].stage),
                         ("STARTED", "PLANNER"))
        planning = executor.apply_planner_status(PlannerStatusEvent(
            2, 8, "PLANNING", BASE+2, sequenced, sequenced, 0.0, 1, ""),
            BASE+2)
        self.assertEqual(planning.events, ())
        failed = executor.apply_planner_status(PlannerStatusEvent(
            3, 8, "FAILED_ATTEMPT", BASE+3, sequenced, sequenced, 0.0, 1, ""),
            BASE+3)
        self.assertEqual(failed.events[0].status, "PROGRESS")
        ready = executor.apply_planner_status(PlannerStatusEvent(
            4, 8, "TRAJECTORY_READY", BASE+4, sequenced, sequenced, 0.0, 1, ""),
            BASE+4)
        self.assertEqual(ready.events[0].status, "PROGRESS")
        with self.assertRaises(ValueError):
            MotionGoal("map", 1.0, 2.0, 2.2, 0.0, 0.0, 0.0, 0.0)

    def test_replanning_does_not_consume_failed_attempt_budget(self):
        executor = PlannerMotionExecutor(PlannerMotionConfig(
            executor_id="executor-test", max_planning_attempts=2))
        self.dispatch(executor)
        sequenced = SequencedMotionGoal(8, goal())
        accepted = executor.apply_planner_status(PlannerStatusEvent(
            1, 8, "ACCEPTED", BASE+1, sequenced, sequenced, 0.0, 0, ""),
            BASE+1)
        self.assertTrue(accepted.accepted, accepted.reason)
        for attempt in range(1, 26):
            outcome = executor.apply_planner_status(PlannerStatusEvent(
                attempt+1, 8, "REPLANNING", BASE+attempt+1,
                sequenced, sequenced, 1.0, attempt, ""),
                BASE+attempt+1)
            self.assertTrue(outcome.accepted, outcome.reason)
        self.assertFalse(outcome.snapshot.faulted)

    def test_only_explicit_failed_attempts_consume_budget(self):
        executor = PlannerMotionExecutor(PlannerMotionConfig(
            executor_id="executor-test", max_planning_attempts=2))
        self.dispatch(executor)
        sequenced = SequencedMotionGoal(8, goal())
        executor.apply_planner_status(PlannerStatusEvent(
            1, 8, "ACCEPTED", BASE+1, sequenced, sequenced, 0.0, 0, ""),
            BASE+1)
        event_seq = 2
        for attempt in range(1, 4):
            planning = executor.apply_planner_status(PlannerStatusEvent(
                event_seq, 8, "REPLANNING", BASE+event_seq,
                sequenced, sequenced, 1.0, attempt, ""),
                BASE+event_seq)
            self.assertTrue(planning.accepted, planning.reason)
            event_seq += 1
            failed = executor.apply_planner_status(PlannerStatusEvent(
                event_seq, 8, "FAILED_ATTEMPT", BASE+event_seq,
                sequenced, sequenced, 1.0, attempt, ""),
                BASE+event_seq)
            event_seq += 1
            if attempt <= 2:
                self.assertTrue(failed.accepted, failed.reason)
            else:
                self.assertFalse(failed.accepted)
                self.assertEqual(
                    failed.reason, "planner_attempt_limit_exceeded")

if __name__ == "__main__": unittest.main()
