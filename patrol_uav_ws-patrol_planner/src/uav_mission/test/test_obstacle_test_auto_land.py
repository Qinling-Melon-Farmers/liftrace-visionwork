import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

spec = importlib.util.spec_from_file_location('land', str(
    Path(__file__).resolve().parents[1]/'scripts/obstacle_test_auto_land.py'))
land = importlib.util.module_from_spec(spec)
spec.loader.exec_module(land)


class LandTests(unittest.TestCase):
    def setUp(self):
        n = self.n = land.AutoLand.__new__(land.AutoLand)
        n.lock = threading.RLock()
        n.frame, n.xy, n.z, n.revision = 'camera_init', [0, .6], .28, 'test-r2'
        n.dwell, n.xy_tol, n.z_tol, n.speed = 1., .18, .15, .12
        n.status = dict(start_mode='post_delivery', mission_id='one', phase='LAND',
                        active_command='LAND', post_delivery_route_revision='test-r2',
                        post_delivery_route_complete=True, mission_failed=False)
        n.status_at = n.state_at = 10.
        n.state = State(connected=True, armed=True, mode='OFFBOARD')
        n.odom = Odometry()
        n.odom.header.frame_id = 'camera_init'
        n.odom.pose.pose.position.y = .6
        n.odom.pose.pose.position.z = .28
        n.since, n.attempts, n.last_attempt, n.handed_over = None, 0, 0., False
        n.mode = Mock(return_value=Mock(mode_sent=True))

    def tick(self, t):
        self.n.odom.header.stamp = rospy.Time.from_sec(t)
        self.n.state_at = self.n.odom.header.stamp.to_sec()
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(t)), \
                patch.object(rospy, 'logwarn'):
            self.n.tick(None)

    def test_requests_once_after_dwell(self):
        self.tick(10); self.tick(10.5)
        self.n.mode.assert_not_called()
        self.tick(11.1); self.tick(11.2)
        self.n.mode.assert_called_once_with(base_mode=0, custom_mode='AUTO.LAND')

    def test_no_early_landing_or_failed_route(self):
        for key, value in [('phase', 'POST_DELIVERY'), ('post_delivery_route_complete', False),
                           ('mission_failed', True), ('post_delivery_route_revision', 'other')]:
            original = self.n.status[key]; self.n.status[key] = value
            self.tick(10); self.tick(11.1)
            self.n.mode.assert_not_called(); self.n.status[key] = original

    def test_manual_mode_or_disarmed(self):
        self.n.state.mode = 'STABILIZED'; self.tick(10); self.tick(11.1)
        self.n.state.mode = 'OFFBOARD'; self.n.state.armed = False
        self.tick(12); self.tick(13.1); self.n.mode.assert_not_called()

    def test_position_speed_and_frame(self):
        self.n.odom.pose.pose.position.x = .5; self.tick(10); self.tick(11.1)
        self.n.odom.pose.pose.position.x = 0
        self.n.odom.twist.twist.linear.x = .3; self.tick(12); self.tick(13.1)
        self.n.odom.twist.twist.linear.x = 0
        self.n.odom.header.frame_id = 'map'; self.tick(14); self.tick(15.1)
        self.n.mode.assert_not_called()

    def test_expired_trigger(self):
        self.tick(26); self.tick(27.1); self.n.mode.assert_not_called()

    def test_rejected_mode_has_bounded_retries(self):
        self.n.mode.return_value.mode_sent = False
        for t in [10, 11.1, 13.2, 15.3, 17.4]: self.tick(t)
        self.assertEqual(self.n.mode.call_count, 3)


if __name__ == '__main__':
    unittest.main()
