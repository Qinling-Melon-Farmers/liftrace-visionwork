#!/usr/bin/env python3
"""合并航段可逐门取证；共享开关不放宽方向、范围、顺序及碰撞检查。"""
import ast
import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/navigation_vcl06_assertion.py'
SPEC = importlib.util.spec_from_file_location('shared_corridor_gate', str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

# Oct5 冻结路线：动作3真实连续穿过两扇门，门和航点坐标均保持原样。
ROUTE = ((6.9, 4.25, .68), (8.75, 4.25, .68),
         (8.75, -2.3, .68), (8.75, -4.2, .68))
ORDER = ('corridor_entry', 'Wall_20', 'Wall_22')
DOORS = (
    dict(name='corridor_entry', axis='x', coordinate=7.975, direction='positive',
         route_indices=[2], lateral_min=3.5, lateral_max=5.0, z_min=0., z_max=1.2),
    dict(name='Wall_20', axis='y', coordinate=1.6, direction='negative',
         route_indices=[3], lateral_min=8.0, lateral_max=8.8, z_min=0., z_max=1.2),
    dict(name='Wall_22', axis='y', coordinate=-1.6, direction='negative',
         route_indices=[3], lateral_min=8.7, lateral_max=9.5, z_min=0., z_max=1.2),
)

def reducer(**overrides):
    parameters = dict(post_delivery_route=ROUTE, post_delivery_route_revision='route-speed-snake3',
                      post_delivery_doors=copy.deepcopy(DOORS), expected_door_order=ORDER,
                      field_bounds=dict(min_x=-.5, max_x=9.5, min_y=-5., max_y=5.),
                      ground_z=-.22, allow_shared_door_route_indices=True)
    parameters.update(overrides)
    return MODULE.Vcl06GateReducer(**parameters)


def production_node_reducer(parameters):
    """执行ROS壳中原有的真实构造表达式；不创建节点、订阅或ROS master。"""
    tree = ast.parse(SCRIPT.read_text())
    node = next(item for item in tree.body
                if isinstance(item, ast.ClassDef) and item.name == 'NavigationVcl06AssertionNode')
    init = next(item for item in node.body
                if isinstance(item, ast.FunctionDef) and item.name == '__init__')
    call = next(item for item in ast.walk(init)
                if isinstance(item, ast.Call) and isinstance(item.func, ast.Name)
                and item.func.id == 'Vcl06GateReducer')
    scope = dict(MODULE.__dict__)
    scope.update(rospy=SimpleNamespace(get_param=lambda key, default=None: parameters.get(key, default)),
                 self=SimpleNamespace(_expected_goal_publisher='/navigation/planner_bridge'),
                 mission_frame='camera_init')
    return eval(compile(ast.Expression(call), str(SCRIPT), 'eval'), scope)


class SharedCorridorGateTest(unittest.TestCase):
    def entry(self, r):
        r.active_post_delivery_route_index = 2
        r.observe_pose(7.5, 4.25, .68, 'camera_init')
        r.observe_pose(8.75, 4.25, .68, 'camera_init')
        r.active_post_delivery_route_index = 3

    def through(self, r, door_index):
        self.entry(r)
        if door_index >= 1:
            r.observe_pose(8.75, 1.8, .68, 'camera_init')
            r.observe_pose(8.75, 1.4, .68, 'camera_init')
        if door_index >= 2:
            r.observe_pose(8.75, -1.4, .68, 'camera_init')
            r.observe_pose(8.75, -1.8, .68, 'camera_init')

    def test_shared_indices_rejected_by_default_and_explicit_false(self):
        with self.assertRaisesRegex(ValueError, 'route_indices overlap'):
            MODULE._normalize_doors(DOORS, len(ROUTE), ORDER)
        with self.assertRaisesRegex(ValueError, 'route_indices overlap'):
            reducer(allow_shared_door_route_indices=False)

    def test_ros_parameter_is_explicit_opt_in_in_real_constructor_call(self):
        parameters = {'~mission/post_delivery_route': ROUTE,
                      '~mission/post_delivery_route_revision': 'route-speed-snake3',
                      '~post_delivery_gate/doors': DOORS,
                      '~post_delivery_gate/expected_door_order': ORDER}
        for value in (None, False):
            if value is not None:
                parameters['~post_delivery_gate/allow_shared_route_indices'] = value
            with self.assertRaisesRegex(ValueError, 'route_indices overlap'):
                production_node_reducer(parameters)
        parameters['~post_delivery_gate/allow_shared_route_indices'] = True
        r = production_node_reducer(parameters)
        self.assertEqual(tuple(d.route_indices for d in r.post_delivery_doors), ((2,), (3,), (3,)))
        self.assertEqual(r.post_delivery_route, ROUTE)

    def test_single_action_has_two_separate_forward_crossing_observations(self):
        r = reducer()
        self.through(r, 2)
        self.assertEqual(r.errors, [])
        self.assertEqual(tuple(d['name'] for d in r.door_crossings), ORDER)
        self.assertEqual([d['route_index'] for d in r.door_crossings], [2, 3, 3])
        for door in r.door_crossings:
            self.assertAlmostEqual(door['z'], .90)

    def test_reverse_crossing_rejected_for_each_shared_door(self):
        for door_index, y in ((1, 1.8), (2, -1.4)):
            with self.subTest(door=ORDER[door_index]):
                r = reducer()
                self.through(r, door_index)
                r.observe_pose(8.75, y, .68, 'camera_init')
                self.assertIn('door_direction_invalid:' + ORDER[door_index], r.errors)

    def test_shared_door_lateral_bounds_still_enforced(self):
        for index, x, start_y, end_y in ((1, 9., 1.8, 1.4), (2, 8.4, -1.4, -1.8)):
            with self.subTest(door=ORDER[index]):
                r = reducer()
                self.through(r, index - 1)
                r.observe_pose(x, start_y, .68, 'camera_init')
                r.observe_pose(x, end_y, .68, 'camera_init')
                self.assertIn('door_lateral_out_of_bounds:' + ORDER[index], r.errors)

    def test_shared_door_height_bounds_still_enforced(self):
        for index, start_y, end_y in ((1, 1.8, 1.4), (2, -1.4, -1.8)):
            with self.subTest(door=ORDER[index]):
                r = reducer()
                self.through(r, index - 1)
                r.observe_pose(8.75, start_y, 1., 'camera_init')
                r.observe_pose(8.75, end_y, 1., 'camera_init')
                self.assertIn('door_height_out_of_bounds:' + ORDER[index], r.errors)

    def test_missing_first_door_cannot_be_replaced_by_second_door(self):
        r = reducer()
        r.active_post_delivery_route_index = 3
        r.observe_pose(8.75, -1.4, .68, 'camera_init')
        r.observe_pose(8.75, -1.8, .68, 'camera_init')
        self.assertIn('door_crossing_out_of_order:Wall_22', r.errors)
        self.assertEqual(r.door_crossings, [])

    def test_no_crossing_evidence_on_wrong_action_index(self):
        r = reducer()
        r.active_post_delivery_route_index = 4
        r.observe_pose(8.75, 1.8, .68, 'camera_init')
        r.observe_pose(8.75, -1.8, .68, 'camera_init')
        self.assertEqual(r.door_crossings, [])

    def test_shared_switch_does_not_accept_missing_or_reordered_doors(self):
        for doors in (DOORS[:-1], tuple(reversed(DOORS))):
            with self.subTest(doors=doors):
                with self.assertRaisesRegex(ValueError, 'door order'):
                    reducer(post_delivery_doors=doors)

    def test_shared_switch_keeps_collision_failure(self):
        r = reducer()
        r.observe_status('contact', {'status': 'READY', 'ready': True, 'actual_collision_count': 1})
        self.assertIn('actual_collision', r.report()['errors'])


if __name__ == '__main__':
    unittest.main()