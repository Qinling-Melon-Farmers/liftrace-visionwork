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
    motion = command in ("SEARCH", "RESUME", "APPROACH", "RETURN_HOME")
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

    def test_deadlines_emit_typed_timeout(self):
        executor = self.make(); self.dispatch(executor, decision(deadline=BASE+100))
        out = executor.tick(BASE+100)
        self.assertEqual(out.events[0].status, "TIMED_OUT")
        self.assertEqual(out.handoff, "CANCEL_REQUIRED")
        executor = self.make(); self.dispatch(executor)
        self.assertEqual(executor.tick(BASE+2*NSEC).reason,
                         "planner_accept_timed_out")

    def test_external_commands_are_handoff_only(self):
        for command in ("LAND", "HOLD", "ABORT"):
            out = self.make().submit_decision(decision(command=command), BASE)
            self.assertEqual(out.handoff, command)
            self.assertIsNone(out.planner_goal)
            self.assertEqual(out.events, ())

    def test_motion_contract(self):
        for item in (replace(decision(), goal=goal(frame="map")),
                     replace(decision(), goal=goal(z=4.1))):
            self.assertFalse(self.make().submit_decision(item, BASE).accepted)
        out = self.make().submit_decision(
            decision(issued=BASE, deadline=BASE+1), BASE+1)
        self.assertFalse(out.accepted)

if __name__ == "__main__": unittest.main()
