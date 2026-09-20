import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

spec = importlib.util.spec_from_file_location('adapter', str(
    Path(__file__).resolve().parents[1] / 'scripts/navigation_frame_adapter.py'))
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class FrameTests(unittest.TestCase):
    def setUp(self):
        self.node = adapter.FrameAdapter.__new__(adapter.FrameAdapter)
        self.node.max_age, self.node.future, self.node.wait = .3, .05, .08
        self.node.buffer = Mock()
        self.tf = TransformStamped()
        self.tf.header.stamp = rospy.Time(10)
        self.tf.transform.translation.x = 3
        self.tf.transform.translation.y = -2
        self.tf.transform.rotation.z = np.sqrt(.5)
        self.tf.transform.rotation.w = np.sqrt(.5)
        self.node.buffer.lookup_transform.return_value = self.tf
        self.pub = Mock()
        self.msg = PoseStamped()
        self.msg.header.stamp = rospy.Time(10)
        self.msg.header.frame_id = 'map'
        self.msg.pose.position.x = 1
        self.msg.pose.orientation.w = 1

    def call(self, msg=None):
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(10)), \
                patch.object(rospy, 'logwarn_throttle'):
            self.node.callback(msg or self.msg, ('map', 'camera_init', self.pub))

    def test_rotation_translation_and_stamp(self):
        self.call()
        out = self.pub.publish.call_args[0][0]
        np.testing.assert_allclose([out.pose.position.x, out.pose.position.y], [3, -1])
        self.assertEqual(out.header.stamp, self.msg.header.stamp)
        self.assertEqual(out.header.frame_id, 'camera_init')
        self.assertEqual(self.msg.header.frame_id, 'map')
        self.assertAlmostEqual(out.pose.orientation.z, np.sqrt(.5))

    def test_inverse_setpoint(self):
        forward, _ = adapter.convert_pose(self.msg.pose, self.tf.transform)
        inverse = TransformStamped().transform
        inverse.rotation.z = -np.sqrt(.5)
        inverse.rotation.w = np.sqrt(.5)
        inverse.translation.x, inverse.translation.y = 2, 3
        back, _ = adapter.convert_pose(forward, inverse)
        np.testing.assert_allclose([back.position.x, back.position.y], [1, 0], atol=1e-12)
        self.assertAlmostEqual(back.orientation.w, 1)

    def test_body_twist_and_covariance(self):
        msg = Odometry()
        msg.header = self.msg.header
        msg.child_frame_id = 'base_link'
        msg.pose.pose = self.msg.pose
        msg.pose.covariance = np.diag([1, 4, 9, 16, 25, 36]).ravel().tolist()
        msg.twist.twist.linear.x = 2
        self.call(msg)
        out = self.pub.publish.call_args[0][0]
        self.assertEqual(out.twist, msg.twist)
        self.assertEqual(out.child_frame_id, 'base_link')
        np.testing.assert_allclose(np.diag(np.array(out.pose.covariance).reshape(6, 6)),
                                   [4, 1, 9, 25, 16, 36])

    def test_reject_wrong_frame(self):
        self.msg.header.frame_id = 'wrong'; self.call()
        self.pub.publish.assert_not_called()

    def test_reject_stale_future_zero(self):
        for stamp in [0, 9, 11]:
            self.msg.header.stamp = rospy.Time(stamp); self.call()
        self.pub.publish.assert_not_called()

    def test_reject_missing_tf(self):
        self.node.buffer.lookup_transform.side_effect = tf2_ros.LookupException('missing')
        self.call(); self.pub.publish.assert_not_called()

    def test_reject_invalid_orientation(self):
        self.msg.pose.orientation.w = 0; self.call()
        self.pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
