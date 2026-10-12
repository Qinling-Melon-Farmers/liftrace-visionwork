"""Offline board entry policy and actual dispatch parameter regression."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace as N
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT / 'deployment/board_trials_4x4'
MISSION = ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path[:0] = [str(BASE / 'common/uav_board_trials/scripts'),
               str(MISSION / 'src'), str(ROOT / 'vision_ws/src/uav_high_view/src')]
from trial_config import generate, TRIAL_FOLDERS, apply_site_profile
from uav_mission.competition_config import generate as generate_competition
from uav_mission.execution_speed import FollowingSpeed
from uav_mission.corridor_speed import CorridorSpeed, CorridorSpeedConfig
from uav_mission.motion_optimization import MotionOptimization
from uav_mission.navigation_following_config import navigation_following_profile

EXPECTED = dict(cruise_lead_m=1., precision_lead_m=.6, corridor_lead_m=.6,
                boundary_lead_m=.6, corridor_after_waypoints=1, low_coverage_lead_m=.6)


class NavigationFollowingConfig(unittest.TestCase):
    def assert_generated(self, generate_fn, settings, rig):
        original = copy.deepcopy(settings)
        with tempfile.TemporaryDirectory() as out:
            generate_fn(ROOT, out, settings, (.01, -.01, -.05), rig)
            r, c, o = [yaml.safe_load((Path(out) / (name + '.yaml')).read_text())
                       for name in ('runtime', 'control', 'overrides')]
        self.assertEqual(settings, original, 'historical input must not be rewritten')
        self.assertEqual(r['following_speed_profile'], EXPECTED)
        self.assertEqual(c['px4_max_distance'], .6)
        if generate_fn is generate:
            self.assertEqual(o['/px4_max_distance'], .6)
        self.assertEqual(o['/traj_server/traj_server/target_dist'], .6)
        if generate_fn is generate:
            for ns, v, a in (('manager', 'max_vel', 'max_acc'), ('search', 'max_vel', 'max_acc'),
                             ('optimization', 'max_vel', 'max_acc'), ('bspline', 'limit_vel', 'limit_acc')):
                self.assertEqual(o['/fast_planner_node/' + ns + '/' + v], settings['cruise_speed'])
                self.assertEqual(o['/fast_planner_node/' + ns + '/' + a], settings['cruise_acceleration'])
        if 'corridor_speed_schedule' in r:
            schedule = r['corridor_speed_schedule']
            self.assertEqual((schedule['open_lead_m'], schedule['door_lead_m']), (.6, .6))
        return r, c, o

    def test_all_nine_trial_paths_include_boundary_floor(self):
        rig = yaml.safe_load((BASE / 'common/uav_board_trials/config/known_rig.yaml').read_text())
        for trial, folder in TRIAL_FOLDERS.items():
            with self.subTest(trial=trial):
                s = yaml.safe_load((BASE / folder / 'settings.yaml').read_text())
                if trial in ('corridor_landing', 'full_mission'):
                    apply_site_profile(s, yaml.safe_load((BASE / '08_full_mission/site_20261007_221730.yaml').read_text()))
                self.assert_generated(generate, s, rig)

    def test_formal_and_archived_inputs_generate_current_policy(self):
        rig = yaml.safe_load((MISSION / 'config/competition/known_rig.yaml').read_text())
        for name in ('field_20261007_validated.yaml', 'field.example.yaml',
                     'candidates/snake_motion.yaml', 'candidates/rectangle_motion.yaml',
                     'candidates/snake3_motion.yaml', 'candidates/recovery_validated_field_local.yaml'):
            with self.subTest(profile=name):
                s = yaml.safe_load((ROOT / 'deployment/competition' / name).read_text())
                if not s['site_confirmed']:
                    s.update(site_confirmed=True, corridor_waypoints=[
                        dict(x=6.7, y=4., agl=1.4), dict(x=6.7, y=4., agl=.9),
                        dict(x=8.3, y=4., agl=.9), dict(x=8.3, y=-4., agl=.9)],
                        landing_xy=[8.5, -4.2])
                self.assert_generated(generate_competition, s, rig)

    def test_invalid_input_is_not_silently_repaired(self):
        for values in (dict(cruise_lead_m=float('nan')), dict(corridor_after_waypoints=0),
                       dict(precision_lead_m=.1, corridor_lead_m=.4)):
            with self.assertRaises(ValueError):
                navigation_following_profile(values)

    def test_explicit_old_site_leads_do_not_override_generation_policy(self):
        s = yaml.safe_load((BASE / '08_full_mission/settings.yaml').read_text())
        site = yaml.safe_load((BASE / '08_full_mission/site_20261007_221730.yaml').read_text())
        apply_site_profile(s, site)
        s['following_speed_profile'] = dict(cruise_lead_m=1., precision_lead_m=.4,
                                           boundary_lead_m=.2, corridor_lead_m=.4)
        original = copy.deepcopy(s)
        rig = yaml.safe_load((BASE / 'common/uav_board_trials/config/known_rig.yaml').read_text())
        r, c, o = self.assert_generated(generate, s, rig)
        self.assertEqual(s, original)
        self.assertEqual(r['high_view_probe']['config']['survey_xy'],
                         [[x+.01, y-.01] for x, y in site['flight_area']['survey_xy']])
        for point in site['corridor_waypoints']:
            self.assertIn([point['x']+.01, point['y']-.01, point['agl']-.27],
                          r['mission']['post_delivery_route'])

    def test_real_dispatch_writes_matching_leads_for_navigation_phases(self):
        tree = ast.parse((MISSION / 'scripts/navigation_mission_manager.py').read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                      and n.name == '_apply_following_speed')
        params = {'~following_speed_profile': EXPECTED,
                  '~corridor_speed_schedule': dict(axis=1, wall_coordinates=[-1.6, 1.6],
                      open_lead_m=.6, door_lead_m=.6)}
        ros = N(get_param=lambda k, default=None: params.get(k, default),
                set_param=lambda k, v: params.update({k: v}), loginfo=lambda *args: None,
                Time=N(now=lambda: N(to_sec=lambda: 100.)))
        namespace = dict(rospy=ros, FollowingSpeed=FollowingSpeed, MotionOptimization=MotionOptimization,
                         CorridorSpeed=CorridorSpeed, CorridorSpeedConfig=CorridorSpeedConfig)
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'dispatch', 'exec'), namespace)
        obj = N(_runtime=N(core=N(post_delivery_route_index=0, config=N(landing_xy=(8., -4.))),
                           boundary_policy=None), _current_xy=lambda: (8., 0.))
        for command, reason, completed, expected in (
                ('SEARCH', '', 0, 1.), ('RESUME', '', 0, 1.), ('APPROACH', '', 0, .6),
                ('SEARCH', 'high_view_full:LOW_COVERAGE', 0, .6),
                ('RESUME', 'high_view_full:LOW_COVERAGE', 0, .6),
                ('RETURN_HOME', 'emergency', 0, .6),
                ('RETURN_HOME', 'post_delivery_route:1', 0, 1.),
                ('RETURN_HOME', 'post_delivery_route:2', 1, .6),
                ('RETURN_HOME', 'post_delivery_route:3', 2, .6),
                ('LAND', '', 9, .6), ('HOLD', '', 9, .6), ('ABORT', '', 9, .6)):
            with self.subTest(command=command, completed=completed):
                obj._runtime.core.post_delivery_route_index = completed
                namespace['_apply_following_speed'](obj, N(command=command, reason=reason, goal=None), True)
                self.assertEqual(params['/px4_max_distance'], expected)
                self.assertEqual(params['/traj_server/traj_server/target_dist'], expected)
        speed = FollowingSpeed(**EXPECTED)
        for command in ('SEARCH', 'RESUME', 'APPROACH'):
            self.assertEqual(speed.select(command, '', 0, near_boundary=True)[1], .6)


if __name__ == '__main__':
    unittest.main()
