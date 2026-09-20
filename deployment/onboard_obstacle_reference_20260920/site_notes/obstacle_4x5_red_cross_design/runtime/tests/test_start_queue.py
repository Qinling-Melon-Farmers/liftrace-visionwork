import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from test_mission_manager import TestMissionManager


class StartQueueTest(unittest.TestCase):
    def setUp(self):
        self.manager = TestMissionManager.__new__(TestMissionManager)
        self.manager._lock = threading.RLock()
        self.manager.started_once = False
        self.manager.start_pending = False
        self.manager.control_ready = False
        self.manager.state = SimpleNamespace(connected=True, armed=True, mode="MANUAL")
        self.manager.state_at = 100.0
        self.manager.odom = SimpleNamespace(
            header=SimpleNamespace(frame_id="camera_init", stamp=SimpleNamespace(to_sec=lambda: 100.0)),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0, z=.25))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0, y=0, z=0))))
        self.manager._publish_status = Mock()
        self.parent_start = patch("test_mission_manager.NavigationMissionManager._on_start",
                                  return_value=SimpleNamespace(success=True, message="mission-id")).start()
        self.addCleanup(patch.stopall)
        patch("test_mission_manager.rospy.Time.now", return_value=SimpleNamespace(to_sec=lambda: 100.1)).start()
        self.clock = patch("test_mission_manager.time.monotonic", return_value=10.0).start()

    def queue(self):
        response = self.manager._on_start(None)
        self.assertTrue(response.success)
        self.parent_start.assert_not_called()

    def test_manual_request_waits_then_stable_takeoff_starts_once(self):
        self.queue()
        self.manager.pending_start_tick()
        self.parent_start.assert_not_called()
        self.manager.state.mode = "OFFBOARD"
        self.manager.control_ready = True
        self.manager.pending_start_tick()
        self.parent_start.assert_not_called()
        self.clock.return_value = 11.1
        self.manager.pending_start_tick()
        self.parent_start.assert_called_once()
        self.assertTrue(self.manager.started_once)
        self.assertFalse(self.manager.start_pending)
        self.assertFalse(self.manager._on_start(None).success)

    def test_duplicate_request_does_not_extend_timeout(self):
        self.queue()
        self.clock.return_value = 50.0
        self.queue()
        self.assertEqual(self.manager.start_deadline, 130.0)
        self.clock.return_value = 130.0
        self.manager.pending_start_tick()
        self.assertFalse(self.manager.start_pending)
        self.parent_start.assert_not_called()

    def test_disarm_cancels_pending_request(self):
        self.queue()
        self.manager.pending_start_tick()
        self.manager.state.armed = False
        self.manager.pending_start_tick()
        self.assertFalse(self.manager.start_pending)
        self.parent_start.assert_not_called()

    def test_leaving_offboard_cancels_pending_request(self):
        self.queue()
        self.manager.state.mode = "OFFBOARD"
        self.manager.pending_start_tick()
        self.manager.state.mode = "POSCTL"
        self.manager.pending_start_tick()
        self.assertFalse(self.manager.start_pending)
        self.parent_start.assert_not_called()

    def test_abort_cancels_before_runtime_exists(self):
        self.queue()
        self.assertTrue(self.manager._on_abort(None).success)
        self.assertFalse(self.manager.start_pending)
        self.parent_start.assert_not_called()

    def test_invalid_height_or_stale_pose_prevents_start(self):
        self.queue()
        self.manager.state.mode = "OFFBOARD"
        self.manager.control_ready = True
        self.manager.odom.pose.pose.position.z = -.06
        self.manager.pending_start_tick()
        self.clock.return_value = 20.0
        self.manager.pending_start_tick()
        self.parent_start.assert_not_called()
        self.manager.odom.pose.pose.position.z = .25
        self.manager.odom.header.stamp.to_sec = lambda: 90.0
        self.manager.pending_start_tick()
        self.parent_start.assert_not_called()

    def test_control_ready_required(self):
        self.queue()
        self.manager.state.mode = "OFFBOARD"
        self.manager.pending_start_tick()
        self.clock.return_value = 20.0
        self.manager.pending_start_tick()
        self.parent_start.assert_not_called()

    def test_parent_rejection_is_not_retried(self):
        self.queue()
        self.manager.state.mode = "OFFBOARD"
        self.manager.control_ready = True
        self.manager.pending_start_tick()
        self.parent_start.return_value = SimpleNamespace(success=False, message="map_not_ready")
        self.clock.return_value = 11.1
        self.manager.pending_start_tick()
        self.assertFalse(self.manager.start_pending)
        self.assertFalse(self.manager.started_once)
        self.assertEqual(self.manager._last_reason, "queued_start_rejected:map_not_ready")


if __name__ == "__main__":
    unittest.main()
