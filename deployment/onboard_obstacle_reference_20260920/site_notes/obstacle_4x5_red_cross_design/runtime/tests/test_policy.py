import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from single_drop_core import SingleDropCore, StrictRoute, OnceRelease, bounded_z, landing_ready
from uav_mission.mission_core import CandidateSnapshot, MissionConfig, MissionPhase, ResultEvent
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.search_types import Waypoint


def candidate(now=100.05, target_id=1, class_name="red_cross"):
    return CandidateSnapshot(target_id, class_name, .9, .9, .9, .5, 2.7, -.25,
                             "camera_init", 2, 5, True, True, "", .01,
                             int(99e9), int(now * 1e9))


class MissionTest(unittest.TestCase):
    def setUp(self):
        config = MissionConfig(approach_altitude=.25, return_altitude=.25,
                               home_xy=(1.4, 4.4), landing_xy=(1.4, 4.4),
                               early_return_enabled=False, max_attempts=1)
        self.runtime = MissionRuntime(SingleDropCore(config), StrictRoute(
            (Waypoint(0, .6, .25), Waypoint(-1.4, 1, .25), Waypoint(1.4, 4.4, .25)), "test", 2))
        self.now = 100.0
        self.sequence = 0
        self.action = self.runtime.start("test", self.now, (0, 0)).action

    def result(self, status="SUCCEEDED", stage="PLANNER", terminal=True, committed=False):
        self.sequence += 1
        self.now += .01
        action = self.runtime.core.active_action
        key = action.candidate_key
        event = ResultEvent(
            mission_id="test", executor_id="bridge", event_seq=self.sequence,
            event_stamp_ns=int(self.now * 1e9), decision_seq=action.decision_seq,
            command=action.command, has_target=action.has_target,
            target_id=key.target_id if key else 0,
            target_first_seen_ns=key.first_seen_ns if key else 0,
            target_class=action.target_class, attempt=action.attempt,
            payload_slot=action.payload_slot, status=status, stage=stage,
            terminal=terminal, retryable=False, payload_committed=committed,
            reason="release_ack_success" if committed else "test_event", evidence_source="test")
        return self.runtime.apply_result(event, self.now, (action.goal.x, action.goal.y) if action.goal else (1.4, 4.4))

    def interrupt(self):
        self.now += .1
        self.runtime.ingest([candidate(self.now)], self.now)
        self.now += .01
        outcome = self.runtime.tick(self.now, (0, .6))
        self.assertEqual(outcome.action.command, "APPROACH")
        self.assertEqual(outcome.action.payload_slot, 1)
        self.assertEqual(outcome.action.profile_name, "r2026")
        return outcome.action

    def test_no_target_completes_route_then_final_anchor_land(self):
        for index in range(3):
            outcome = self.result()
            self.assertTrue(outcome.accepted, outcome.reason)
            self.assertEqual(self.runtime.route.current_index, index + 1)
        self.assertEqual(outcome.action.command, "RETURN_HOME")
        self.assertEqual((outcome.action.goal.x, outcome.action.goal.y), (1.4, 4.4))
        self.assertEqual(self.result().action.command, "LAND")
        self.assertEqual(self.runtime.core.committed_slots, 0)

    def test_success_resumes_interrupted_cursor_without_second_delivery(self):
        self.result()
        self.interrupt()
        self.assertTrue(self.result("PROGRESS", "RELEASE", False, True).accepted)
        outcome = self.result("SUCCEEDED", "RECOVERY")
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual(outcome.action.command, "RESUME")
        self.assertEqual(self.runtime.route.current_index, 1)
        self.assertEqual(outcome.action.goal.x, -1.4)
        self.assertEqual(self.runtime.core.committed_slots, 1)
        self.now += .1
        self.runtime.ingest([candidate(self.now, 2)], self.now)
        self.assertIsNone(self.runtime.tick(self.now + .01, (0, 1)).action)

    def test_failed_alignment_resumes_without_retry(self):
        self.interrupt()
        outcome = self.result("FAILED", "ALIGNMENT")
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual(outcome.action.command, "RESUME")
        self.assertEqual(self.runtime.route.current_index, 0)
        self.assertEqual(self.runtime.core.delivery_outcome, "failed_before_release")

    def test_uncertain_release_timeout_resumes_without_freeing_slot(self):
        action = self.interrupt()
        self.now = action.deadline_at + .01
        outcome = self.runtime.tick(self.now, (.5, 2.7))
        self.assertEqual(outcome.action.command, "RESUME")
        self.assertEqual(self.runtime.core.slots[0].status.value, "QUARANTINED")
        self.assertTrue(self.runtime.core.attempt_started)

    def test_failed_recovery_resumes_route(self):
        self.interrupt()
        self.result("PROGRESS", "RELEASE", False, True)
        outcome = self.result("FAILED", "RECOVERY")
        self.assertTrue(outcome.accepted, outcome.reason)
        self.assertEqual(outcome.action.command, "RESUME")
        self.assertEqual(self.runtime.core.delivery_outcome, "committed_recovery_failed")

    def test_other_class_and_outside_target_cannot_interrupt(self):
        self.now += .1
        self.runtime.ingest([candidate(self.now, class_name="bridge"), replace(candidate(self.now), x=1.9)], self.now)
        self.assertIsNone(self.runtime.tick(self.now + .01, (0, 0)).action)

    def test_repeated_motion_failure_aborts_without_skipping(self):
        self.result("FAILED")
        outcome = self.result("FAILED")
        self.assertEqual(self.runtime.core.phase, MissionPhase.ABORTED)
        self.assertEqual(self.runtime.route.current_index, 0)
        self.assertEqual(outcome.action.command, "ABORT")


class GuardTest(unittest.TestCase):
    def test_once_survives_new_instance_and_rejects_other_slots(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "right.receipt")
            self.assertFalse(OnceRelease(path).consume(2))
            self.assertTrue(OnceRelease(path).consume(1))
            self.assertFalse(OnceRelease(path).consume(1))

    def test_every_setpoint_height_is_capped_and_nan_rejected(self):
        self.assertEqual(bounded_z(1.2), .25)
        self.assertEqual(bounded_z(.1), .1)
        for value in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                bounded_z(value)

    def test_landing_requires_fresh_offboard_settled_feedback(self):
        state = SimpleNamespace(connected=True, armed=True, mode="OFFBOARD")
        odom = SimpleNamespace(
            header=SimpleNamespace(frame_id="camera_init", stamp=SimpleNamespace(to_sec=lambda: 100.0)),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.4, y=4.4, z=.25))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0, y=0, z=0))))
        self.assertTrue(landing_ready(100.1, state, 100, odom, (1.4, 4.4)))
        self.assertFalse(landing_ready(101, state, 100, odom, (1.4, 4.4)))
        state.mode = "POSCTL"
        self.assertFalse(landing_ready(100.1, state, 100, odom, (1.4, 4.4)))
        state.mode = "OFFBOARD"
        odom.pose.pose.position.z = .65
        self.assertFalse(landing_ready(100.1, state, 100, odom, (1.4, 4.4)))


if __name__ == "__main__":
    unittest.main()
