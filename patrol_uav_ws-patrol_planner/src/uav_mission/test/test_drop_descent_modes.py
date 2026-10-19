"""ROS-free tests of the production delivery mask, configuration and lane ordering."""
import ast
import copy
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from uav_mission.drop_descent_config import drop_descent_parameters


class PositionTarget:
    FRAME_LOCAL_NED = 1
    IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    FORCE, IGNORE_YAW, IGNORE_YAW_RATE = 512, 1024, 2048

    def __init__(self):
        self.position = NS(x=0., y=0., z=0.)
        self.velocity = NS(x=99., y=99., z=0.)


source = PACKAGE/'scripts/navigation_frame_adapter.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
target = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'drop_vertical_target')
velocity_helpers = [n for n in tree.body if
                    (isinstance(n, ast.FunctionDef) and n.name in ('drop_velocity_pose', 'drop_velocity_target')) or
                    (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
                     t.id == 'DROP_VELOCITY_MASK' for t in n.targets))]
adapter = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FrameAdapter')
callback = next(n for n in adapter.body if isinstance(n, ast.FunctionDef) and n.name == 'callback')
namespace = dict(copy=copy, math=math, np=np, PositionTarget=PositionTarget,
                 Pose=lambda: NS(position=NS(x=0., y=0., z=0.), orientation=NS()),
                 quaternion_from_euler=lambda r, p, y: (0., 0., math.sin(y/2), math.cos(y/2)),
                 euler_from_quaternion=lambda q: (0., 0., 2*math.atan2(q[2], q[3])))
exec(compile(ast.Module(body=[target, callback]+velocity_helpers, type_ignores=[]), str(source), 'exec'), namespace)


class DropDescentModes(unittest.TestCase):
    def test_default_and_four_mode_values(self):
        self.assertEqual(drop_descent_parameters({})['mode'], 1)
        for mode in (1, 2, 3, 4):
            cfg = drop_descent_parameters(dict(drop_descent_mode=mode, drop_descent_speed_mps=.25))
            self.assertEqual(cfg['mode'], mode)
            self.assertEqual(cfg['speed_mps'], .25)
            self.assertEqual(cfg['zero_velocity_setpoint_topic'], '/navigation/drop_setpoint_vertical')
            self.assertEqual(cfg['velocity_setpoint_topic'], '/navigation/drop_setpoint_velocity')
            self.assertEqual(cfg['velocity_speed_mps'], .60)

    def test_invalid_modes_and_rates_are_rejected(self):
        for mode in (True, 0, 5, '2', 2.0):
            with self.assertRaises(ValueError): drop_descent_parameters(dict(drop_descent_mode=mode))
        for speed in (True, 0, -.1, 1.01, float('nan'), float('inf'), '0.2'):
            with self.assertRaises(ValueError): drop_descent_parameters(dict(drop_descent_speed_mps=speed))

    def test_mode_four_parameters_are_separate_from_old_z_ramp(self):
        cfg = drop_descent_parameters(dict(drop_descent_mode=4, drop_descent_speed_mps=.2,
                                          drop_descent_mode4_speed_mps=.6,
                                          drop_descent_braking_mps2=.8))
        self.assertEqual((cfg['speed_mps'], cfg['velocity_speed_mps'], cfg['braking_mps2']), (.2, .6, .8))
        for key, upper in [('drop_descent_mode4_speed_mps', 1.),
                           ('drop_descent_acceleration_mps2', 2.),
                           ('drop_descent_braking_mps2', 2.),
                           ('drop_descent_hold_tolerance_m', .03)]:
            for value in (True, 0, -.1, upper+.01, float('nan'), float('inf'), '.6'):
                with self.assertRaises(ValueError): drop_descent_parameters({key: value})

    def test_mode_three_ignores_xy_position_and_controls_zero_xy_velocity(self):
        pose = NS(position=NS(x=9., y=-8., z=.40),
                  orientation=NS(x=0., y=0., z=math.sin(.3), w=math.cos(.3)))
        header = NS(frame_id='camera_init', stamp=100.)
        out = namespace['drop_vertical_target'](header, pose, np.eye(3))
        self.assertEqual(out.type_mask, 2531)
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_PX)
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_PY)
        self.assertFalse(out.type_mask & PositionTarget.IGNORE_PZ)
        self.assertFalse(out.type_mask & PositionTarget.IGNORE_VX)
        self.assertFalse(out.type_mask & PositionTarget.IGNORE_VY)
        self.assertEqual((out.velocity.x, out.velocity.y), (0., 0.))
        self.assertEqual(out.position.z, .40)
        self.assertAlmostEqual(out.yaw, .6)
        out.header.frame_id = 'map'
        self.assertEqual(header.frame_id, 'camera_init')

    def test_mode_three_requires_gravity_aligned_frame_conversion(self):
        pose = NS(position=NS(z=.40), orientation=NS(x=0., y=0., z=0., w=1.))
        yaw = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        namespace['drop_vertical_target'](NS(), pose, yaw)
        with self.assertRaises(ValueError):
            namespace['drop_vertical_target'](NS(), pose, np.diag([1., -1., -1.]))

    def velocity_message(self, vz=-.6):
        msg = PositionTarget()
        msg.header = NS(frame_id='camera_init', stamp=10.)
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = namespace['DROP_VELOCITY_MASK']
        msg.position = NS(x=1., y=2., z=.8)
        msg.velocity.z = vz
        msg.yaw = .2
        return msg

    def test_mode_four_fixed_xy_and_vz_mask_without_z_position_or_xy_velocity(self):
        msg = self.velocity_message()
        pose = namespace['drop_velocity_pose'](msg)
        out = namespace['drop_velocity_target'](msg, pose, np.eye(3))
        self.assertEqual(out.type_mask, 2524)
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_PZ)
        self.assertFalse(out.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                                        PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_YAW))
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_VX)
        self.assertTrue(out.type_mask & PositionTarget.IGNORE_VY)
        self.assertEqual((out.position.x, out.position.y, out.velocity.z), (1., 2., -.6))
        self.assertAlmostEqual(out.yaw, .2)
        self.assertEqual(out.header.stamp, msg.header.stamp)
        out.position.x = 9.; out.header.frame_id = 'map'
        self.assertEqual((msg.position.x, msg.header.frame_id), (1., 'camera_init'))

    def test_mode_four_transformed_xy_yaw_and_zero_vz_brake_keep_enu_sign(self):
        msg = self.velocity_message(0.)
        pose = namespace['drop_velocity_pose'](msg)
        pose.position = NS(x=3., y=-2., z=.58)
        pose.orientation.z, pose.orientation.w = math.sin(.6), math.cos(.6)
        yaw_rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        for vz in (0., -.03, -.6):
            msg.velocity.z = vz
            out = namespace['drop_velocity_target'](msg, pose, yaw_rotation)
            self.assertEqual((out.position.x, out.position.y, out.position.z), (3., -2., .58))
            self.assertEqual(out.velocity.z, vz)
            self.assertAlmostEqual(out.yaw, 1.2)
        with self.assertRaises(ValueError):
            namespace['drop_velocity_target'](msg, pose, np.diag([1., -1., -1.]))

    def test_mode_four_rejects_wrong_mask_upward_velocity_and_nonfinite_fields(self):
        for mask in (0, 2531):
            msg = self.velocity_message(); msg.type_mask = mask
            with self.assertRaises(ValueError): namespace['drop_velocity_pose'](msg)
        for vz in (.01, -1.01, float('nan'), float('-inf')):
            with self.assertRaises(ValueError): namespace['drop_velocity_pose'](self.velocity_message(vz))
        for key in ('x', 'y', 'z'):
            msg = self.velocity_message(); setattr(msg.position, key, float('nan'))
            with self.assertRaises(ValueError): namespace['drop_velocity_pose'](msg)
        msg = self.velocity_message(); msg.coordinate_frame = 8
        with self.assertRaises(ValueError): namespace['drop_velocity_pose'](msg)

    def test_late_position_drop_or_h_messages_cannot_replace_newer_command(self):
        node = NS(command_lock=threading.Lock(), last_command_stamp=0.,
                  convert_and_publish=Mock(return_value=True))
        for stamp, lane in [(10., 'setpoint'), (11., 'drop_vertical'), (10.5, 'setpoint'),
                            (10.7, 'landing_vertical'), (12., 'drop_velocity'),
                            (11.5, 'drop_vertical'), (13., 'setpoint'), (12.5, 'drop_velocity')]:
            namespace['callback'](node, NS(header=NS(stamp=stamp)), ('mission', 'map', None, lane))
        self.assertEqual(node.convert_and_publish.call_count, 4)
        self.assertEqual(node.last_command_stamp, 13.)

    def test_rejected_drop_conversion_does_not_advance_command_watermark(self):
        node = NS(command_lock=threading.Lock(), last_command_stamp=10.,
                  convert_and_publish=Mock(return_value=False))
        namespace['callback'](node, NS(header=NS(stamp=11.)), ('mission', 'map', None, 'drop_vertical'))
        self.assertEqual(node.last_command_stamp, 10.)


if __name__ == '__main__':
    unittest.main()
