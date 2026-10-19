"""Mode 2 alone allows a Z position trajectory slope up to 3 m/s."""
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from uav_mission.drop_descent_config import drop_descent_parameters
from uav_mission.landing_posctl_config import landing_descent_parameters


class ModeTwoSpeedLimit(unittest.TestCase):
    def test_drop_mode_two_accepts_inclusive_three_mps(self):
        for speed in (.20, 1., 1.01, 2., 3.):
            self.assertEqual(drop_descent_parameters(dict(drop_descent_mode=2,
                             drop_descent_speed_mps=speed))['speed_mps'], speed)

    def test_h_mode_two_accepts_inclusive_three_mps(self):
        for speed in (.20, 1., 1.01, 2., 3.):
            self.assertEqual(landing_descent_parameters(dict(landing_descent_mode=2,
                             landing_descent_speed_mps=speed))['speed_mps'], speed)

    def test_both_mode_two_reject_invalid_and_above_three(self):
        for function, prefix in ((drop_descent_parameters, 'drop'),
                                 (landing_descent_parameters, 'landing')):
            for value in (True, False, 0, -.1, 3.000001, float('nan'), float('inf'), '3'):
                with self.assertRaises(ValueError):
                    function({prefix+'_descent_mode': 2, prefix+'_descent_speed_mps': value})

    def test_other_modes_keep_one_mps_limit_and_default_speed(self):
        for function, prefix in ((drop_descent_parameters, 'drop'),
                                 (landing_descent_parameters, 'landing')):
            self.assertEqual(function({})['speed_mps'], .20)
            for mode in (1, 3, 4):
                selected = {prefix+'_descent_mode': mode, prefix+'_descent_speed_mps': 1.}
                if prefix == 'landing' and mode == 4:
                    selected['landing_handoff_mode'] = 'OFFBOARD'
                self.assertEqual(function(selected)['speed_mps'], 1.)
                with self.assertRaises(ValueError):
                    function(dict(selected, **{prefix+'_descent_speed_mps': 1.01}))

    def test_formal_and_trial_generation_accept_both_mode_two_rates_at_three(self):
        root = Path(__file__).resolve().parents[4]
        sys.path.insert(0, str(root/'vision_ws/src/uav_high_view/src'))
        from uav_mission.competition_config import generate as formal
        trials = root/'deployment/board_trials_4x4'
        sys.path.insert(0, str(trials/'common/uav_board_trials/scripts'))
        from trial_config import generate as trial
        for generate, settings_path, rig_path, landing, waypoints in (
                (formal, root/'deployment/competition/field.example.yaml',
                 root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml',
                 [8.5, -4.2], [dict(x=6.7, y=4., agl=1.4), dict(x=6.7, y=4., agl=.9),
                               dict(x=8.3, y=4., agl=.9), dict(x=8.3, y=-4., agl=.9)]),
                (trial, trials/'08_full_mission/settings.yaml',
                 trials/'common/uav_board_trials/config/known_rig.yaml',
                 [2.5, 0.], [dict(x=.6, y=0., agl=.9), dict(x=1.5, y=.4, agl=.9)])):
            settings = yaml.safe_load(settings_path.read_text())
            settings['motion_optimization']['enabled'] = False
            settings.update(site_confirmed=True, landing_xy=landing, corridor_waypoints=waypoints,
                            drop_descent_mode=2, drop_descent_speed_mps=3.,
                            landing_descent_mode=2, landing_descent_speed_mps=3.,
                            landing_handoff_mode='POSCTL')
            rig = yaml.safe_load(rig_path.read_text())
            with tempfile.TemporaryDirectory() as output:
                generate(root, output, settings, (.02, -.01, .12), rig)
                control = yaml.safe_load((Path(output)/'control.yaml').read_text())
            self.assertEqual(control['drop_system']['descent']['speed_mps'], 3.)
            self.assertEqual(control['external_landing']['descent']['speed_mps'], 3.)


if __name__ == '__main__':
    unittest.main()
