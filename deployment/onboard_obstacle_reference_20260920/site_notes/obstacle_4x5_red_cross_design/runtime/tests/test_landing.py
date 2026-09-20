import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from test_mission_manager import TestMissionManager
from uav_mission.mission_core import MissionPhase
from mavros_msgs.msg import ExtendedState


class LandingTest(unittest.TestCase):
    def setUp(self):
        self.manager = TestMissionManager.__new__(TestMissionManager)
        self.manager._lock = threading.RLock()
        self.manager.state = SimpleNamespace(connected=True, armed=True, mode="OFFBOARD")
        self.manager.state_at = 100.0
        self.manager.odom = SimpleNamespace(
            header=SimpleNamespace(frame_id="camera_init", stamp=SimpleNamespace(to_sec=lambda: 100.0)),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.4, y=4.4, z=.25))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0, y=0, z=0))))
        self.manager.endpoint = (1.4, 4.4)
        self.manager.extended = None
        self.manager.extended_at = 0.0
        self.manager.land_since = 98.0
        self.manager.land_requested_at = None
        self.manager.last_land_request = 0.0
        self.manager.land_attempts = 0
        self.manager.land_mode_seen = False
        self.manager.set_mode = Mock(return_value=SimpleNamespace(mode_sent=True))
        self.manager._runtime = SimpleNamespace(
            route=SimpleNamespace(is_complete=True),
            core=SimpleNamespace(phase=MissionPhase.LAND, active_action=SimpleNamespace(deadline_at=200)))

    def tick(self):
        with patch("test_mission_manager.rospy.Time.now", return_value=SimpleNamespace(to_sec=lambda: 100.1)):
            self.manager.land_tick()

    def test_request_is_not_landing_success_and_not_repeated(self):
        self.tick()
        self.assertEqual(self.manager.land_requested_at, 100.1)
        self.assertEqual(self.manager._runtime.core.phase, MissionPhase.LAND)
        self.tick()
        self.assertEqual(self.manager.set_mode.call_count, 1)

    def test_landed_disarmed_requires_post_request_fresh_facts(self):
        self.manager.land_requested_at = 99.0
        self.manager.land_mode_seen = True
        self.manager.state.armed = False
        self.manager.extended = SimpleNamespace(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        self.manager.extended_at = 98.0
        self.tick()
        self.assertEqual(self.manager._runtime.core.phase, MissionPhase.LAND)
        self.manager.extended_at = 100.0
        self.tick()
        self.assertEqual(self.manager._runtime.core.phase, MissionPhase.COMPLETE)

    def test_manual_takeover_never_requests_mode(self):
        self.manager.state.mode = "POSCTL"
        self.tick()
        self.manager.set_mode.assert_not_called()

    def test_unfinished_route_never_requests_mode(self):
        self.manager._runtime.route.is_complete = False
        self.tick()
        self.manager.set_mode.assert_not_called()

    def test_unstable_endpoint_never_requests_mode(self):
        self.manager.odom.pose.pose.position.x = 1.0
        self.tick()
        self.manager.set_mode.assert_not_called()


if __name__ == "__main__":
    unittest.main()
