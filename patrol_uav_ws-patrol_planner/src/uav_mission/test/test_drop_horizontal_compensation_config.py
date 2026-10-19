"""Generate formal/trial configurations for both gates and all descent modes."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[4]
MISSION = ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path.insert(0, str(MISSION/'src'))
from uav_mission.drop_descent_config import drop_horizontal_compensation_mode


class HorizontalCompensationConfig(unittest.TestCase):
    def test_default_and_invalid_modes(self):
        self.assertEqual(drop_horizontal_compensation_mode({}), 2)
        for mode in (1, 2):
            self.assertEqual(drop_horizontal_compensation_mode(dict(drop_horizontal_compensation_mode=mode)), mode)
        for value in (True, False, 0, 3, -1, 1., '1', None):
            with self.assertRaises(ValueError):
                drop_horizontal_compensation_mode(dict(drop_horizontal_compensation_mode=value))

    def check_generator(self, generate, settings, rig):
        original = copy.deepcopy(settings)
        for descent in (1, 2, 3):
            controls = []
            for gate in (1, 2):
                selected = dict(settings, drop_horizontal_compensation_mode=gate, drop_descent_mode=descent)
                with tempfile.TemporaryDirectory() as out:
                    generate(ROOT, out, selected, (.02, -.01, .12), rig)
                    control = yaml.safe_load((Path(out)/'control.yaml').read_text())
                drop = control['drop_system']
                self.assertEqual(drop['descent']['mode'], descent)
                self.assertEqual(drop['pre_descent']['mode'], gate)
                self.assertEqual(drop['pre_descent']['stable_xy_tolerance_m'], .04)
                self.assertEqual(drop['pre_descent']['max_horizontal_speed_mps'], .10)
                self.assertEqual(drop['pre_descent']['stable_duration_sec'], 1.)
                self.assertEqual(drop['release_timeout'], dict(enabled=True, after_sec=20.))
                self.assertTrue(drop['release_height_only'])
                drop['pre_descent'].pop('mode')
                controls.append(control)
            # Gate switching must not alter H, routes, recovery, geometry or descent mode.
            self.assertEqual(controls[0], controls[1])
        self.assertEqual(settings, original)

    def test_formal_generation(self):
        from uav_mission.competition_config import generate
        s = yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        s['motion_optimization']['enabled'] = False
        s.update(site_confirmed=True, landing_xy=[8.5, -4.2], corridor_waypoints=[
            dict(x=6.7, y=4., agl=1.4), dict(x=6.7, y=4., agl=.9),
            dict(x=8.3, y=4., agl=.9), dict(x=8.3, y=-4., agl=.9)])
        rig = yaml.safe_load((MISSION/'config/competition/known_rig.yaml').read_text())
        self.check_generator(generate, s, rig)

    def test_trial_generation(self):
        base = ROOT/'deployment/board_trials_4x4'
        sys.path.insert(0, str(base/'common/uav_board_trials/scripts'))
        from trial_config import generate
        s = yaml.safe_load((base/'08_full_mission/settings.yaml').read_text())
        s['motion_optimization']['enabled'] = False
        s.update(landing_xy=[2.5, 0.], corridor_waypoints=[
            dict(x=.6, y=0., agl=.9), dict(x=1.5, y=.4, agl=.9)])
        rig = yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
        self.check_generator(generate, s, rig)


if __name__ == '__main__':
    unittest.main()
