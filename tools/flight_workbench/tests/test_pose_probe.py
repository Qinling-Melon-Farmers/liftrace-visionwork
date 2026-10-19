#!/usr/bin/env python3
"""Real ROS message/wire regressions for read-only pose extraction.

No ROS initialization, SSH, publishers, services or hardware operations.
Use rl_drone with the installed ROS generated messages:
  PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages \
    python tools/flight_workbench/tests/test_pose_probe.py
WB_FIELD_PROBE_SOURCE optionally selects a fetched historical probe source.
"""
import importlib.util
from contextlib import redirect_stderr
import io
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import genpy

TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))
import wb_status


def load_probe(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


probe = load_probe(TOOL_DIR / 'board_probe.py', 'pose_probe_under_test')

# Values from the operator's rostopic echo, 2026-10-07. Preserve FC local Z.
POSITION = (.0009825327433645725, -.0016001919284462929, -.04578689485788345)
QUATERNION = (-.001758973368638668, -.008329988421206943,
              .017054427221993594, .9998183721154955)
SECS, NSECS = 1791360765, 663627904


def real_message(odometry=False, frame='map', quaternion=QUATERNION):
    msg = Odometry() if odometry else PoseStamped()
    msg.header.seq = 11777
    msg.header.frame_id = frame
    msg.header.stamp = genpy.Time(SECS, NSECS)
    pose = msg.pose.pose if odometry else msg.pose
    pose.position.x, pose.position.y, pose.position.z = POSITION
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    if odometry:
        msg.child_frame_id = 'body'
        msg.pose.covariance = [0.] * 36
    # Use actual genpy serialization/deserialization, rather than SimpleNamespace.
    buffer = io.BytesIO()
    msg.serialize(buffer)
    return type(msg)().deserialize(buffer.getvalue())


class FakeRospy:
    class AnyMsg:
        pass

    def __init__(self):
        self.callbacks = {}

    def Subscriber(self, topic, kind, callback, **kwargs):
        assert kwargs.get('queue_size') == 1
        self.callbacks[topic] = callback
        return object()


class RealPoseTest(unittest.TestCase):
    def extract(self, msg, odometry=False):
        stamp = msg.header.stamp.to_sec()
        return probe.pose_extractor(lambda: stamp + .025, odometry)(msg)

    def assert_pose(self, row, frame):
        self.assertIsInstance(row, dict)
        self.assertEqual(row['frame'], frame)
        self.assertEqual(row['stamp'], genpy.Time(SECS, NSECS).to_sec())
        self.assertAlmostEqual(row['source_age'], .025, places=5)
        for field, expected in zip(('x', 'y', 'z'), POSITION):
            self.assertAlmostEqual(row[field], expected, places=12)
        for field in ('roll_deg', 'pitch_deg', 'yaw_deg'):
            self.assertTrue(math.isfinite(row[field]))
        # Independent expected orientation values, rather than repeating extractor math.
        self.assertAlmostEqual(row['roll_deg'], -.21787, delta=.001)
        self.assertAlmostEqual(row['pitch_deg'], -.95098, delta=.001)
        self.assertAlmostEqual(row['yaw_deg'], 1.95638, delta=.001)
        json.dumps(row, allow_nan=False)

    def test_actual_pose_stamped_wire_message(self):
        self.assert_pose(self.extract(real_message()), 'map')

    def test_actual_odometry_nested_pose_wire_message(self):
        self.assert_pose(self.extract(real_message(True, 'camera_init'), True), 'camera_init')

    def test_all_four_observation_paths_decode_actual_wire_messages(self):
        for key, odometry, frame in (('fc_pose', False, 'map'), ('lio_pose', True, 'camera_init'),
                                     ('ev_pose', False, 'map'), ('setpoint', False, 'map')):
            with self.subTest(key=key):
                self.assert_pose(self.extract(real_message(odometry, frame), odometry), frame)

    def test_quaternion_scaling_does_not_change_attitude(self):
        original = self.extract(real_message())
        scaled = self.extract(real_message(quaternion=tuple(7*v for v in QUATERNION)))
        for field in ('roll_deg', 'pitch_deg', 'yaw_deg'):
            self.assertAlmostEqual(original[field], scaled[field], places=10)

    def test_zero_and_nonfinite_quaternions_are_not_fabricated_poses(self):
        for quaternion in ((0., 0., 0., 0.), (float('nan'), 0., 0., 1.),
                           (0., 0., float('inf'), 1.)):
            with self.subTest(quaternion=quaternion):
                self.assertIsNone(self.extract(real_message(quaternion=quaternion)))

    def test_nonfinite_position_and_stamp_are_rejected(self):
        msg = real_message()
        msg.pose.position.z = float('inf')
        self.assertIsNone(self.extract(msg))
        msg = real_message()
        with patch.object(type(msg.header.stamp), 'to_sec', return_value=float('nan')):
            self.assertIsNone(self.extract(msg))

    def test_missing_clock_stays_invalid(self):
        self.assertIsNone(probe.pose_extractor(lambda: None)(real_message()))


class CallbackDiagnosticTest(unittest.TestCase):
    def setUp(self):
        self.topic = '/mavros/local_position/pose'
        self.rospy = FakeRospy()
        self.tracker = probe.Tracker([self.topic])
        self.payloads = {}

    def subscribe(self, extractor):
        extractor.msg_type = PoseStamped
        subscriptions = probe.subscribe(self.rospy, [self.topic], self.tracker,
                                         self.payloads, {self.topic: extractor})
        return subscriptions

    def diagnostic(self, subscriptions):
        value = self.payloads.get(self.topic)
        return probe.observation_diagnostic(self.topic, self.tracker.snapshot()[self.topic],
            {self.topic: ['/mavros']}, subscriptions[self.topic], value)

    def test_real_message_callback_receives_valid_pose(self):
        msg = real_message()
        subscriptions = self.subscribe(probe.pose_extractor(lambda: msg.header.stamp.to_sec()+.01))
        self.rospy.callbacks[self.topic](msg)
        self.assertEqual(self.diagnostic(subscriptions)['status'], 'receiving')
        self.assertEqual(self.payloads[self.topic]['frame'], 'map')

    def test_exception_reason_survives_callback_log_and_json_stays_clean(self):
        def broken(msg):
            raise TypeError('pose decoder regression: expected geometry_msgs/PoseStamped')
        subscriptions = self.subscribe(broken)
        log = io.StringIO()
        with redirect_stderr(log):
            self.rospy.callbacks[self.topic](real_message())
        diagnostic = self.diagnostic(subscriptions)
        self.assertEqual(diagnostic['status'], 'parse_error')
        self.assertEqual(diagnostic['count'], 1)
        self.assertIsNone(self.payloads[self.topic])
        self.assertIn('PROBE_PARSE_ERROR topic='+self.topic, log.getvalue())
        self.assertIn('type=PoseStamped reason=TypeError: pose decoder regression', log.getvalue())
        self.assertIsNone(wb_status.parse_probe_line(log.getvalue()))
        row = wb_status.parse_probe_line(json.dumps({'master': True, 'observe': {'fc_pose': None},
            'observe_status': {'fc_pose': diagnostic}}, allow_nan=False))
        self.assertEqual(row['observe_status']['fc_pose']['status'], 'parse_error')

    def test_recovered_callback_resets_error_throttle(self):
        broken = [True]
        msg = real_message()
        valid = probe.pose_extractor(lambda: msg.header.stamp.to_sec()+.01)
        def extractor(value):
            if broken[0]:
                raise ValueError('temporary invalid pose')
            return valid(value)
        subscriptions = self.subscribe(extractor)
        log = io.StringIO()
        with redirect_stderr(log):
            self.rospy.callbacks[self.topic](msg)
        self.assertIn('ValueError: temporary invalid pose', log.getvalue())
        broken[0] = False
        self.rospy.callbacks[self.topic](msg)
        diagnostic = self.diagnostic(subscriptions)
        self.assertEqual(diagnostic['status'], 'receiving')
        self.assertEqual(diagnostic['count'], 2)
        broken[0] = True
        with redirect_stderr(log):
            self.rospy.callbacks[self.topic](msg)
        self.assertEqual(log.getvalue().count('PROBE_PARSE_ERROR'), 2)

    def test_same_error_is_logged_once_then_changed_reason_is_logged(self):
        reason = ['decoder first error']
        def broken(msg):
            raise TypeError(reason[0])
        self.subscribe(broken)
        log = io.StringIO()
        with redirect_stderr(log):
            for _ in range(30):
                self.rospy.callbacks[self.topic](real_message())
            reason[0] = 'decoder changed error'
            self.rospy.callbacks[self.topic](real_message())
        self.assertEqual(log.getvalue().count('PROBE_PARSE_ERROR'), 2)
        self.assertIn('decoder first error', log.getvalue())
        self.assertIn('decoder changed error', log.getvalue())

    def test_wrong_runtime_message_type_has_specific_failure_log(self):
        subscriptions = self.subscribe(probe.pose_extractor(lambda: 100.))
        log = io.StringIO()
        with redirect_stderr(log):
            self.rospy.callbacks[self.topic](FakeRospy.AnyMsg())
        self.assertEqual(self.diagnostic(subscriptions)['status'], 'parse_error')
        self.assertIn('type=AnyMsg reason=AttributeError', log.getvalue())
        self.assertIsNone(self.payloads[self.topic])


class FieldSourceReproductionTest(unittest.TestCase):
    def test_fetched_field_source_decodes_real_messages_on_python38_plus(self):
        path = Path(os.environ.get('WB_FIELD_PROBE_SOURCE', '/tmp/board_probe_field_20261007.py'))
        if not path.is_file():
            self.skipTest('Fetched field source not available')
        field = load_probe(path, 'historical_field_pose_probe')
        for odometry in (False, True):
            with self.subTest(odometry=odometry):
                row = field.pose_extractor(lambda: genpy.Time(SECS, NSECS).to_sec()+.01, odometry)(
                    real_message(odometry))
                self.assertIsInstance(row, dict)
                self.assertAlmostEqual(row['z'], POSITION[2], places=12)
                self.assertAlmostEqual(row['source_age'], .01, places=5)


class RosInitGuardEvidenceTest(unittest.TestCase):
    def test_real_noetic_repeat_init_can_return_with_ros_time_uninitialized(self):
        import rospy
        import rospy.client
        # Exercise only the already-initialized early-return guard in installed
        # Noetic. Mock start_node as an additional guarantee of no ROS startup.
        init_args = ('pose_guard_evidence', sys.argv, False, None, False, True)
        with patch.object(rospy.client, '_init_node_args', init_args), \
                patch.object(rospy.rostime, '_rostime_initialized', False), \
                patch.object(rospy.impl.init, 'start_node') as start_node:
            rospy.init_node('pose_guard_evidence', disable_signals=True)
            start_node.assert_not_called()
            with self.assertRaises(rospy.exceptions.ROSInitException):
                rospy.Time.now()


if __name__ == '__main__':
    unittest.main()
