#!/usr/bin/env python3
"""Stage transitions and height evaluation must preserve the full mission."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest
import yaml
from test_staggered_corridor_gate import MODULE

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((ROOT/'config/vcl06_full_low_corridor_runtime.yaml').read_text())

class FullLowCorridorTest(unittest.TestCase):
    def test_original_search_and_delivery_policy_preserved(self):
        original = yaml.safe_load((ROOT/'config/vcl06_horizontal_field_runtime.yaml').read_text())
        self.assertEqual(CONFIG['search'], original['search'])
        changed = {'post_delivery_route', 'post_delivery_route_revision',
                   'post_delivery_parameter_stages'}
        for name, value in original['mission'].items():
            if name not in changed:
                self.assertEqual(CONFIG['mission'][name], value, name)

    def test_parameter_stage_runs_before_next_goal_and_not_during_search(self):
        tree = ast.parse((ROOT/'scripts/navigation_mission_manager.py').read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                      and n.name == '_publish_action')
        stage = next(n for n in method.body if isinstance(n, ast.If)
                     and isinstance(n.test, ast.BoolOp))
        code = compile(ast.Module(body=[copy.deepcopy(stage)], type_ignores=[]), 'stage', 'exec')
        writes = {}
        ros = SimpleNamespace(get_param=lambda *_: CONFIG['mission']['post_delivery_parameter_stages'],
                              set_param=lambda k,v: writes.update({k:v}), loginfo=lambda *_: None)
        for command, completed in [('SEARCH',0), ('RETURN_HOME',0),
                                   ('RETURN_HOME',1), ('RETURN_HOME',2)]:
            context = {'rospy':ros, 'self':SimpleNamespace(_runtime=SimpleNamespace(
                core=SimpleNamespace(post_delivery_route_index=completed))),
                'action':SimpleNamespace(command=command, reason='post_delivery_route:1/9:test')}
            exec(code, context)
            ceiling='/fast_planner_node/sdf_map/virtual_ceil_height'
            if command == 'SEARCH':
                self.assertEqual(writes,{})
            elif completed < 2:
                self.assertNotIn(ceiling,writes)
            else:
                self.assertEqual(writes[ceiling],.55)
        route=CONFIG['mission']['post_delivery_route']
        self.assertEqual(route[0][:2],route[1][:2])
        self.assertEqual(route[1][2],.40)

    def test_low_ceiling_only_in_corridor_including_between_doors(self):
        region=CONFIG['post_delivery_gate']['low_height_region']
        reducer=MODULE.Vcl06GateReducer(low_height_region=region)
        reducer.observe_pose(0, 4, 1.6, 'camera_init')
        self.assertEqual(reducer.height_violations,0)
        reducer.observe_pose(1, 7.8, .69, 'camera_init')
        self.assertEqual(reducer.height_violations,0)
        reducer.observe_pose(1, 7.8, .71, 'camera_init')
        self.assertEqual(reducer.height_violations,1)

if __name__ == '__main__':
    unittest.main()
