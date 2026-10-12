"""Mode 4 raw setpoint TF conversion, rejection and shared output arbitration."""
import copy
import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import PositionTarget

spec = importlib.util.spec_from_file_location('touchdown_adapter', str(
    Path(__file__).resolve().parents[1] / 'scripts/navigation_frame_adapter.py'))
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class TouchdownSetpointTests(unittest.TestCase):
    def setUp(self):
        self.node = adapter.FrameAdapter.__new__(adapter.FrameAdapter)
        self.node.max_age, self.node.future, self.node.wait = .3, .05, .08
        self.node.buffer = Mock()
        self.node.command_lock = threading.Lock()
        self.node.last_command_stamp = rospy.Time(0)
        self.tf = TransformStamped()
        self.tf.header.stamp = rospy.Time(10)
        self.tf.transform.translation.x = 3.
        self.tf.transform.translation.y = -2.
        self.tf.transform.translation.z = -.22
        self.tf.transform.rotation.z = np.sqrt(.5)
        self.tf.transform.rotation.w = np.sqrt(.5)
        self.node.buffer.lookup_transform.return_value = self.tf
        self.pub = Mock()
        self.msg = PositionTarget()
        self.msg.header.stamp = rospy.Time(10)
        self.msg.header.frame_id = 'camera_init'
        self.msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.msg.type_mask = 2524
        self.msg.position.x, self.msg.position.y, self.msg.position.z = 1., 2., .45
        self.msg.velocity.z = -.30
        self.msg.yaw = .2

    def call(self, msg=None, lane='landing_touchdown'):
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(10)), patch.object(rospy, 'logwarn_throttle'):
            self.node.callback(msg or self.msg, ('camera_init', 'map', self.pub, lane))

    def test_fixed_xy_negative_vz_and_yaw_transform_preserves_source_stamp(self):
        original = copy.deepcopy(self.msg)
        self.call()
        out = self.pub.publish.call_args.args[0]
        self.assertEqual(out.type_mask, 2524)
        self.assertEqual(out.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        np.testing.assert_allclose([out.position.x, out.position.y, out.position.z], [1., -1., .23])
        self.assertAlmostEqual(out.yaw, .2 + np.pi / 2)
        self.assertEqual(out.velocity.z, -.30)
        self.assertEqual(out.velocity.x, 0.)
        self.assertEqual(out.velocity.y, 0.)
        self.assertFalse(out.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                                         PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_YAW))
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_PZ)
        self.assertEqual(out.header.stamp, self.msg.header.stamp)
        self.assertEqual(out.header.frame_id, 'map')
        self.assertEqual(self.msg, original)
        self.node.buffer.lookup_transform.assert_called_once_with('map', 'camera_init', self.msg.header.stamp, rospy.Duration(.08))

    def test_current_z_is_transformed_as_anchor_and_never_enabled_for_position_control(self):
        for height in (.9, .3, -.02):
            self.msg.position.z = height
            self.msg.header.stamp = rospy.Time(10 + (1.0 - height) * .01)
            self.call()
            out = self.pub.publish.call_args.args[0]
            self.assertAlmostEqual(out.position.z, height - .22)
            self.assertTrue(out.type_mask & PositionTarget.IGNORE_PZ)
            self.assertEqual(out.velocity.z, -.30)

    def test_bad_frames_masks_and_nonfinite_enabled_fields_reject_without_advancing_watermark(self):
        changes = [('coordinate_frame', 8), ('type_mask', 2531), ('type_mask', 0)]
        for key, value in changes:
            msg = copy.deepcopy(self.msg); setattr(msg, key, value); self.call(msg)
        for field, key in (('position', 'x'), ('position', 'y'), ('position', 'z'), ('velocity', 'z')):
            msg = copy.deepcopy(self.msg); setattr(getattr(msg, field), key, float('nan')); self.call(msg)
        msg = copy.deepcopy(self.msg); msg.yaw = float('inf'); self.call(msg)
        for speed in (0., .30, float('-inf')):
            msg = copy.deepcopy(self.msg); msg.velocity.z = speed; self.call(msg)
        self.pub.publish.assert_not_called()
        self.assertEqual(self.node.last_command_stamp, rospy.Time(0))

    def test_source_and_dynamic_tf_freshness_are_strict_no_latest_fallback(self):
        for frame, stamp in (('wrong', 10.), ('camera_init', 0.), ('camera_init', 9.6), ('camera_init', 10.1)):
            msg = copy.deepcopy(self.msg); msg.header.frame_id = frame; msg.header.stamp = rospy.Time.from_sec(stamp)
            self.call(msg)
        self.node.buffer.lookup_transform.assert_not_called()
        self.tf.header.stamp = rospy.Time(9)
        self.call()
        self.pub.publish.assert_not_called()
        self.assertEqual(self.node.last_command_stamp, rospy.Time(0))
        self.tf.header.stamp = rospy.Time(0)  # Static TF is accepted.
        self.call()
        self.pub.publish.assert_called_once()

    def test_missing_invalid_and_tilted_tf_rejected(self):
        self.node.buffer.lookup_transform.side_effect = tf2_ros.LookupException('missing')
        self.call()
        self.node.buffer.lookup_transform.side_effect = None
        for x, w in ((.05, 1.), (1., 0.), (float('nan'), 1.)):
            self.tf.transform.rotation.x = x
            self.tf.transform.rotation.z = 0.
            self.tf.transform.rotation.w = w
            self.call()
        self.pub.publish.assert_not_called()
        self.assertEqual(self.node.last_command_stamp, rospy.Time(0))

    def test_touchdown_and_normal_abort_hold_share_existing_watermark(self):
        self.call()
        pose = PoseStamped(); pose.header = copy.deepcopy(self.msg.header)
        pose.pose.orientation.w = 1.
        pose.header.stamp = rospy.Time.from_sec(9.99); self.call(pose, 'setpoint')
        self.assertEqual(self.pub.publish.call_count, 1)
        pose.header.stamp = rospy.Time.from_sec(10.02); self.call(pose, 'setpoint')
        self.assertEqual(self.pub.publish.call_count, 2)
        self.assertIsInstance(self.pub.publish.call_args.args[0], PoseStamped)
        self.msg.header.stamp = rospy.Time.from_sec(10.01); self.call()
        self.assertEqual(self.pub.publish.call_count, 2)
        self.assertEqual(self.node.last_command_stamp, pose.header.stamp)
        self.msg.header.stamp = pose.header.stamp; self.call()  # Same-stamp raw must not override position hold.
        self.assertEqual(self.pub.publish.call_count, 2)

    def test_custom_input_topic_and_feedback_only_mode(self):
        for enabled in (False, True):
            def param(name, default):
                return {'~enable_setpoints':enabled, '~landing_touchdown_setpoint_topic':'/custom/touchdown'}.get(name, default)
            with patch.object(rospy, 'get_param', side_effect=param), patch.object(rospy, 'resolve_name', side_effect=lambda s:s), \
                    patch.object(rospy, 'Publisher') as publisher, patch.object(rospy, 'Subscriber') as subscriber, \
                    patch.object(tf2_ros, 'Buffer'), patch.object(tf2_ros, 'TransformListener'):
                adapter.FrameAdapter()
                topics = [call.args[0] for call in subscriber.call_args_list]
                self.assertEqual('/custom/touchdown' in topics, enabled)
                if enabled:
                    raw = next(call for call in subscriber.call_args_list if call.args[0] == '/custom/touchdown')
                    self.assertIs(raw.args[1], PositionTarget)
                else:
                    self.assertEqual(publisher.call_count, 2)


if __name__ == '__main__':
    unittest.main()
