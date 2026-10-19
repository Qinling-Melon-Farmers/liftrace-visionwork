#!/usr/bin/env python3

import unittest

from uav_mission.mission_core import (
    GoalSnapshot,
    MissionCore,
    ResultEvent,
)
from uav_mission.planner_execution import (
    MotionDecision,
    MotionGoal,
    PlannerMotionConfig,
    PlannerMotionExecutor,
)
from uav_mission.profile_policy import CompetitionProfile


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


def result_from_execution(event):
    return ResultEvent(
        mission_id=event.mission_id,
        executor_id=event.executor_id,
        event_seq=event.event_seq,
        event_stamp_ns=event.event_stamp_ns,
        decision_seq=event.decision_seq,
        command=event.command,
        has_target=event.has_target,
        target_id=event.target_id,
        target_first_seen_ns=event.target_first_seen_ns,
        target_class=event.target_class,
        attempt=event.attempt,
        payload_slot=event.payload_slot,
        status=event.status,
        stage=event.stage,
        terminal=event.terminal,
        retryable=event.retryable,
        payload_committed=event.payload_committed,
        reason=event.reason,
        evidence_source=event.evidence_source,
    )


class PlannerMissionContractTest(unittest.TestCase):
    def test_dispatch_acceptance_and_accept_timeout_reduce_in_mission_core(self):
        start = 100.0
        core = MissionCore(competition_profile())
        core.start("mission-contract", start)
        action = core.dispatch_search_motion(
            "SEARCH",
            GoalSnapshot("camera_init", 1.0, 2.0, 2.2),
            "contract_test",
            start,
        )
        executor = PlannerMotionExecutor(PlannerMotionConfig(
            executor_id="planner-contract-executor",
            planner_accept_timeout_ns=5 * NSEC,
        ))
        decision = MotionDecision(
            mission_id=core.mission_id,
            decision_seq=action.decision_seq,
            issued_at_ns=int(action.issued_at * NSEC),
            deadline_ns=int(action.deadline_at * NSEC),
            command=action.command,
            class_profile=action.profile_name,
            goal=MotionGoal(
                action.goal.frame_id,
                action.goal.x,
                action.goal.y,
                action.goal.z,
            ),
        )

        dispatch = executor.submit_decision(decision, int(start * NSEC))
        self.assertTrue(dispatch.accepted, dispatch.reason)
        self.assertEqual(len(dispatch.events), 1)
        accepted_event = dispatch.events[0]
        self.assertEqual(
            (accepted_event.status, accepted_event.stage,
             accepted_event.terminal),
            ("ACCEPTED", "DISPATCH", False),
        )
        accepted, reason, next_action = core.apply_result(
            result_from_execution(accepted_event), start)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "progress_recorded")
        self.assertIsNone(next_action)

        timeout_time = start + 5.0
        timeout = executor.tick(int(timeout_time * NSEC))
        self.assertEqual(len(timeout.events), 1)
        timeout_event = timeout.events[0]
        self.assertEqual(
            (timeout_event.status, timeout_event.stage,
             timeout_event.terminal),
            ("TIMED_OUT", "PLANNER", True),
        )
        accepted, reason, next_action = core.apply_result(
            result_from_execution(timeout_event), timeout_time)
        self.assertTrue(accepted, reason)
        self.assertEqual(reason, "search_motion_failed")
        self.assertIsNone(next_action)
        self.assertIsNone(core.active_action)


if __name__ == "__main__":
    unittest.main()
