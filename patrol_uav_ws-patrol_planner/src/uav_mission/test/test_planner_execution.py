#!/usr/bin/env python3
from dataclasses import replace
import math
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

def status(event_seq, goal_seq, name, stamp, attempt=None, motion_goal=None):
    attempt = (0 if name in ("ACCEPTED", "CANCELLED") else 1) \
        if attempt is None else attempt
    item = SequencedMotionGoal(goal_seq, motion_goal or goal())
    return PlannerStatusEvent(event_seq, goal_seq, name, stamp, item, item,
                              0.0, attempt, "test")

def odom(stamp, speed=0.0):
    return OdomSample(stamp, "camera_init", 1.0, 2.0, 2.2,
                      speed, 0.0, 0.0)

class PlannerMotionExecutorTest(unittest.TestCase):
    def test_return_heading_must_settle_before_advancing(self):
        executor = PlannerMotionExecutor(PlannerMotionConfig(
            arrival_distance_m=.25, arrival_dwell_ns=100_000_000,
            odom_max_age_ns=200_000_000, return_yaw_tolerance_rad=.1))
        heading = math.pi / 2
        g = replace(goal(), qz=math.sin(heading/2), qw=math.cos(heading/2))
        self.dispatch(executor, replace(decision(command="RETURN_HOME"), goal=g))
        now = BASE + 10_000_000
        for seq, name in enumerate(("ACCEPTED", "TRAJECTORY_READY"), 1):
            event = status(seq, 8, name, now+seq, motion_goal=g)
            self.assertTrue(executor.apply_planner_status(event, now+seq).accepted)
        now += 10_000_000
        out = executor.apply_odom(odom(now), now)
        self.assertEqual(out.reason, "arrival_yaw_not_met")
        now += 100_000_000
        out = executor.apply_odom(replace(odom(now), yaw=heading+2*math.pi), now)
        self.assertEqual(out.reason, "arrival_dwell_pending")
        now += 50_000_000
        out = executor.apply_odom(replace(odom(now), yaw=heading+.2), now)
        self.assertEqual(out.reason, "arrival_yaw_not_met")
        now += 50_000_000
        out = executor.apply_odom(replace(odom(now), yaw=heading), now)
        self.assertEqual(out.reason, "arrival_dwell_pending")
        now += 150_000_000
        out = executor.apply_odom(replace(odom(now), yaw=heading), now)
        self.assertTrue(any(e.status == "SUCCEEDED" for e in out.events))

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

    def test_retired_goal_geometry_cannot_fail_replacement(self):
        for name in ("REPLANNING", "TRAJECTORY_READY", "CANCELLED"):
            with self.subTest(status=name):
                executor = self.make()
                self.dispatch(executor)
                self.dispatch(executor, decision(20, issued=BASE+1), BASE+1)
                late = replace(status(1, 8, name, BASE+2),
                    effective_goal=SequencedMotionGoal(8, goal(4.0)))
                outcome = executor.apply_planner_status(late, BASE+2)
                self.assertFalse(outcome.events)
                self.assertFalse(outcome.snapshot.active_terminal)
                accepted = executor.apply_planner_status(
                    status(2, 20, "ACCEPTED", BASE+3), BASE+3)
                self.assertTrue(accepted.accepted, accepted.reason)

    def test_return_arrival_uses_original_portal(self):
        executor = self.make()
        self.dispatch(executor, decision(command="RETURN_HOME"))
        now = BASE + 10_000_000
        for seq, name in enumerate(("ACCEPTED", "TRAJECTORY_READY"), 1):
            event = replace(status(seq, 8, name, now+seq),
                effective_goal=SequencedMotionGoal(8, goal(1.3)))
            self.assertTrue(executor.apply_planner_status(event, now+seq).accepted)
        for stamp in (now+10_000_000, now+200_000_000):
            outcome = executor.apply_odom(replace(odom(stamp), x=1.3), stamp)
            self.assertEqual(outcome.reason, "arrival_threshold_not_met")

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

    def test_replacement_acceptance_survives_dropped_cancel_telemetry(self):
        executor = self.make(); self.dispatch(executor)
        accepted_old = executor.apply_planner_status(
            status(1, 8, "ACCEPTED", BASE+1), BASE+1)
        self.assertTrue(accepted_old.accepted, accepted_old.reason)
        self.dispatch(executor, decision(20, "APPROACH", BASE+1), BASE+1)
        accepted_new = executor.apply_planner_status(
            status(2, 20, "ACCEPTED", BASE+2), BASE+2)
        self.assertTrue(accepted_new.accepted, accepted_new.reason)
        self.assertEqual(accepted_new.reason, "planner_goal_accepted")

        executor = self.make(); self.dispatch(executor)
        accepted_old = executor.apply_planner_status(
            status(1, 8, "ACCEPTED", BASE+1), BASE+1)
        self.assertTrue(accepted_old.accepted, accepted_old.reason)
        self.dispatch(executor, decision(20, "APPROACH", BASE+1), BASE+1)
        cancel = executor.apply_planner_status(
            status(2, 8, "CANCELLED", BASE+2), BASE+2)
        self.assertEqual(cancel.reason, "replacement_cancel_confirmed")
        accepted = executor.apply_planner_status(
            status(3, 20, "ACCEPTED", BASE+3), BASE+3)
        self.assertTrue(accepted.accepted, accepted.reason)

    def test_timeout_replacement_resolves_and_ignores_late_generation(self):
        executor = self.make()
        first = decision(deadline=BASE+20*NSEC)
        self.dispatch(executor, first)
        self.assertEqual(executor.resolve_goal_seq_by_stamp(BASE), 8)

        timed_out = executor.tick(BASE+5*NSEC)
        self.assertEqual(timed_out.reason, "planner_accept_timed_out")
        replacement_time = BASE+5*NSEC+1
        replacement = replace(
            decision(20, "SEARCH", replacement_time,
                     replacement_time+20*NSEC),
            goal=goal(2.0),
        )
        self.dispatch(executor, replacement, replacement_time)
        # The first goal was never accepted by the planner, so no CANCELLED
        # fact can be required.  Its retained stamp is dropped and any late
        # transport telemetry is ignored by the ROS adapter.
        self.assertEqual(executor.resolve_goal_seq_by_stamp(BASE), 0)
        self.assertEqual(
            executor.resolve_goal_seq_by_stamp(replacement_time), 20)
        self.assertEqual(
            executor.resolve_goal_seq_by_stamp(replacement_time+1), 0)
        accepted = executor.apply_planner_status(
            status(3, 20, "ACCEPTED", replacement_time+3,
                   motion_goal=goal(2.0)),
            replacement_time+3)
        self.assertTrue(accepted.accepted, accepted.reason)
        self.assertEqual(accepted.reason, "planner_goal_accepted")

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

    def test_near_goal_dwell_finishes_after_prior_valid_trajectory(self):
        executor = self.make(); self.dispatch(executor)
        events = (
            (1, "ACCEPTED", 0),
            (2, "PLANNING", 1),
            (3, "TRAJECTORY_READY", 1),
            (4, "REPLANNING", 2),
            (5, "FAILED_ATTEMPT", 2),
        )
        for event_seq, name, attempt in events:
            stamp = BASE + event_seq * 10_000_000
            outcome = executor.apply_planner_status(
                status(event_seq, 8, name, stamp, attempt=attempt), stamp)
            self.assertTrue(outcome.accepted, outcome.reason)

        first_stamp = BASE + 60_000_000
        first = executor.apply_odom(odom(first_stamp), first_stamp)
        self.assertEqual(first.reason, "arrival_dwell_pending")
        arrived_stamp = first_stamp + 100_000_000
        arrived = executor.apply_odom(odom(arrived_stamp), arrived_stamp)
        self.assertEqual(arrived.reason, "motion_succeeded")
        self.assertEqual(arrived.events[0].status, "SUCCEEDED")

    def test_failed_attempt_storm_remains_bounded_by_action_deadline(self):
        executor = self.make()
        item = decision(deadline=BASE+2*NSEC)
        self.dispatch(executor, item)
        accepted = executor.apply_planner_status(
            status(1, 8, "ACCEPTED", BASE+1, attempt=0), BASE+1)
        self.assertTrue(accepted.accepted, accepted.reason)
        event_seq = 2
        for attempt in range(1, 51):
            stamp = BASE + event_seq
            planning = executor.apply_planner_status(status(
                event_seq, 8, "REPLANNING", stamp, attempt=attempt), stamp)
            self.assertTrue(planning.accepted, planning.reason)
            event_seq += 1
            stamp = BASE + event_seq
            failed = executor.apply_planner_status(status(
                event_seq, 8, "FAILED_ATTEMPT", stamp, attempt=attempt),
                stamp)
            self.assertTrue(failed.accepted, failed.reason)
            event_seq += 1
        timed_out = executor.tick(BASE+2*NSEC)
        self.assertEqual(timed_out.reason, "decision_timed_out")
        self.assertEqual(timed_out.events[0].status, "TIMED_OUT")

    def test_approach_has_a_bounded_visual_handoff_tolerance(self):
        executor = self.make()
        self.dispatch(executor, decision(8, "APPROACH"))
        finished = self.finish(executor)
        first = OdomSample(
            finished+10, "camera_init", 1.34, 2.0, 2.2,
            0.0, 0.0, 0.0)
        dwell = executor.apply_odom(first, finished+10)
        self.assertEqual(dwell.reason, "arrival_dwell_pending")
        second = replace(first, stamp_ns=finished+100_000_010)
        arrived = executor.apply_odom(second, second.stamp_ns)
        self.assertEqual(arrived.handoff, "TARGET_TRANSACTION")

        search = self.make()
        self.dispatch(search)
        search_finished = self.finish(search)
        outside = replace(first, stamp_ns=search_finished+10)
        rejected = search.apply_odom(outside, outside.stamp_ns)
        self.assertEqual(rejected.reason, "arrival_threshold_not_met")

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
        executor = self.make()
        self.dispatch(executor, decision(deadline=BASE+10*NSEC))
        self.assertEqual(executor.tick(BASE+2*NSEC).reason,
                         "executor_pending")
        self.assertEqual(executor.tick(BASE+5*NSEC).reason,
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

    def test_decision_source_clock_jitter_is_bounded(self):
        within_tolerance = decision(issued=BASE+1_000_000)
        accepted = self.make().submit_decision(within_tolerance, BASE)
        self.assertTrue(accepted.accepted, accepted.reason)
        self.assertEqual(accepted.planner_goal.decision_seq,
                         within_tolerance.decision_seq)

        beyond_tolerance = decision(issued=BASE+100_000_001)
        rejected = self.make().submit_decision(beyond_tolerance, BASE)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "decision_from_future")

    def test_planner_event_source_clock_jitter_is_bounded(self):
        dispatch_ns = BASE + 200_000_000
        item = decision(deadline=BASE + 5*NSEC)

        executor = self.make()
        self.dispatch(executor, item, dispatch_ns)
        within_tolerance = executor.apply_planner_status(
            status(1, 8, "ACCEPTED", dispatch_ns-100_000_000),
            dispatch_ns)
        self.assertTrue(within_tolerance.accepted, within_tolerance.reason)
        self.assertEqual(within_tolerance.reason, "planner_goal_accepted")

        executor = self.make()
        self.dispatch(executor, item, dispatch_ns)
        beyond_tolerance = executor.apply_planner_status(
            status(1, 8, "ACCEPTED", dispatch_ns-100_000_001),
            dispatch_ns)
        self.assertFalse(beyond_tolerance.accepted)
        self.assertEqual(beyond_tolerance.reason,
                         "planner_event_precedes_dispatch")

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

    def test_missing_attempt_telemetry_does_not_abort_current_goal(self):
        executor = self.make()
        self.dispatch(executor)
        sequenced = SequencedMotionGoal(8, goal())
        accepted = executor.apply_planner_status(PlannerStatusEvent(
            1, 8, "ACCEPTED", BASE+1, sequenced, sequenced, 0.0, 0, ""),
            BASE+1)
        self.assertTrue(accepted.accepted, accepted.reason)
        jumped = executor.apply_planner_status(PlannerStatusEvent(
            2, 8, "REPLANNING", BASE+2,
            sequenced, sequenced, 1.0, 25, ""), BASE+2)
        self.assertTrue(jumped.accepted, jumped.reason)
        ready = executor.apply_planner_status(PlannerStatusEvent(
            3, 8, "TRAJECTORY_READY", BASE+3,
            sequenced, sequenced, 0.2, 25, ""), BASE+3)
        self.assertTrue(ready.accepted, ready.reason)

    def test_finish_is_authoritative_without_ready_telemetry(self):
        executor = self.make()
        self.dispatch(executor)
        sequenced = SequencedMotionGoal(8, goal())
        accepted = executor.apply_planner_status(PlannerStatusEvent(
            1, 8, "ACCEPTED", BASE+1,
            sequenced, sequenced, 0.0, 0, ""), BASE+1)
        self.assertTrue(accepted.accepted, accepted.reason)
        planning = executor.apply_planner_status(PlannerStatusEvent(
            2, 8, "PLANNING", BASE+2,
            sequenced, sequenced, 0.0, 1, ""), BASE+2)
        self.assertTrue(planning.accepted, planning.reason)
        finished = executor.apply_planner_status(PlannerStatusEvent(
            3, 8, "TRAJECTORY_FINISHED", BASE+3,
            sequenced, sequenced, 0.0, 1, ""), BASE+3)
        self.assertTrue(finished.accepted, finished.reason)
        late = executor.apply_planner_status(PlannerStatusEvent(
            4, 8, "REPLANNING", BASE+4,
            sequenced, sequenced, 0.0, 2, ""), BASE+4)
        self.assertFalse(late.accepted)
        self.assertEqual(late.reason,
                         "planner_status_after_finished_ignored")
        self.assertFalse(late.snapshot.faulted)

if __name__ == "__main__": unittest.main()
