"""ROS-free checks of production raw-setpoint masks and command ordering.

Extract only the production functions being tested; no ROS node or C++ build.
Full TF/pose integration remains in test_navigation_frame_adapter.py.
"""
import ast
import copy
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np


class PositionTarget:
    FRAME_LOCAL_NED = 1
    IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    FORCE, IGNORE_YAW, IGNORE_YAW_RATE = 512, 1024, 2048

    def __init__(self):
        self.position = NS(x=0., y=0., z=0.)
        self.velocity = NS(x=0., y=0., z=0.)
        self.acceleration_or_force = NS(x=0., y=0., z=0.)


source = Path(__file__).resolve().parents[1] / 'scripts/navigation_frame_adapter.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
             and node.name == 'vertical_target']
adapter = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'FrameAdapter')
callback = next(node for node in adapter.body if isinstance(node, ast.FunctionDef) and node.name == 'callback')
init = next(node for node in adapter.body if isinstance(node, ast.FunctionDef) and node.name == '__init__')
namespace = dict(copy=copy, np=np, PositionTarget=PositionTarget, math=math,
                 threading=threading, PoseStamped=object, Odometry=type('Odom', (), {}),
                 euler_from_quaternion=lambda q: (0., 0., 2*math.atan2(q[2], q[3])))
exec(compile(ast.Module(body=functions+[callback, init], type_ignores=[]), str(source), 'exec'), namespace)


class VerticalSetpointTests(unittest.TestCase):
    def test_mask_controls_zero_xy_velocity_z_and_yaw(self):
        pose = NS(position=NS(x=9., y=-8., z=.13),
                  orientation=NS(x=0., y=0., z=math.sin(.3), w=math.cos(.3)))
        header = NS(frame_id='camera_init', stamp=100.)
        out = namespace['vertical_target'](header, pose, np.eye(3))
        self.assertEqual(out.type_mask, 2531)
        self.assertEqual(out.coordinate_frame, 1)
        self.assertEqual(out.position.z, .13)
        self.assertAlmostEqual(out.yaw, .6)
        self.assertEqual((out.velocity.x, out.velocity.y), (0., 0.))
        self.assertFalse(out.type_mask & (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY))
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_AFX)
        self.assertEqual((out.position.x, out.position.y), (0., 0.))
        out.header.frame_id = 'map'
        self.assertEqual(header.frame_id, 'camera_init')

    def test_yaw_only_rotation_allowed_but_tilt_or_upside_down_rejected(self):
        pose = NS(position=NS(z=.35), orientation=NS(x=0., y=0., z=0., w=1.))
        yaw_only = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        namespace['vertical_target'](NS(), pose, yaw_only)
        for matrix in (np.diag([1., -1., -1.]),
                       np.array([[1., 0., 0.], [0., .99995, -.01], [0., .01, .99995]])):
            with self.assertRaisesRegex(ValueError, 'gravity-aligned'):
                namespace['vertical_target'](NS(), pose, matrix)

    def test_both_command_lanes_share_timestamp_watermark_including_abort(self):
        node = NS(command_lock=threading.Lock(), last_command_stamp=0.,
                  convert_and_publish=Mock(return_value=True))
        def call(stamp, lane):
            namespace['callback'](node, NS(header=NS(stamp=stamp)), ('mission','map',None,lane))
        call(10., 'setpoint')
        call(11., 'landing_vertical')
        call(10.5, 'setpoint')  # queued XYZ from before freeze
        call(11., 'landing_vertical')  # duplicate
        self.assertEqual(node.convert_and_publish.call_count, 2)
        call(12., 'setpoint')  # abort restores full position hold
        call(11.5, 'landing_vertical')
        self.assertEqual(node.convert_and_publish.call_count, 3)
        self.assertEqual(node.last_command_stamp, 12.)

    def test_rejected_conversion_does_not_poison_ordering_or_feedback(self):
        node = NS(command_lock=threading.Lock(), last_command_stamp=10.,
                  convert_and_publish=Mock(return_value=False))
        namespace['callback'](node, NS(header=NS(stamp=11.)), ('mission','map',None,'landing_vertical'))
        self.assertEqual(node.last_command_stamp, 10.)
        namespace['callback'](node, NS(header=NS(stamp=9.)), ('map','mission',None,'pose'))
        self.assertEqual(node.convert_and_publish.call_count, 2)

    def test_feedback_only_adapter_has_no_command_publishers_or_subscribers(self):
        ros = Mock()
        ros.get_param.side_effect = lambda name, default: default
        ros.Duration.side_effect = lambda value: value
        ros.Time.side_effect = lambda value: value
        ros.resolve_name.side_effect = lambda name: name
        namespace.update(rospy=ros, tf2_ros=Mock())
        node = NS(callback=Mock())
        namespace['__init__'](node)
        self.assertEqual([c.args[0] for c in ros.Publisher.call_args_list],
                         ['/navigation/local_pose', '/navigation/local_odom'])
        self.assertEqual(ros.Subscriber.call_count, 2)


if __name__ == '__main__':
    unittest.main()
