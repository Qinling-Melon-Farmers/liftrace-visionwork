"""Generate both real entry points: mode 4 cannot change H, height or permission."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[4]
MISSION = ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path.insert(0, str(MISSION/'src'))
sys.path.insert(0, str(ROOT/'vision_ws/src/uav_high_view/src'))


class VelocityDescentGeneration(unittest.TestCase):
    def check_generator(self, generate, settings, rig):
        original = copy.deepcopy(settings)
        for gate in (1, 2):
            reference = None
            for mode in (1, 2, 3, 4):
                selected = dict(settings, drop_descent_mode=mode, drop_horizontal_compensation_mode=gate)
                with tempfile.TemporaryDirectory() as out:
                    generate(ROOT, out, selected, (.02, -.01, .12), rig)
                    control = yaml.safe_load((Path(out)/'control.yaml').read_text())
                cfg = control['drop_system']['descent']
                self.assertEqual(cfg['mode'], mode)
                self.assertEqual(cfg['speed_mps'], settings.get('drop_descent_speed_mps', .20))
                self.assertEqual(cfg['velocity_speed_mps'], .60)
                self.assertEqual(cfg['acceleration_mps2'], .60)
                self.assertEqual(cfg['braking_mps2'], .60)
                self.assertEqual(cfg['hold_tolerance_m'], .015)
                self.assertEqual(cfg['velocity_setpoint_topic'], '/navigation/drop_setpoint_velocity')
                self.assertTrue(control['drop_system']['release_height_only'])
                cfg.pop('mode')
                if reference is None: reference = control
                else: self.assertEqual(control, reference)
        self.assertEqual(settings, original)

    def test_formal_entry_all_four_descent_modes_and_two_compensation_modes(self):
        from uav_mission.competition_config import generate
        settings = yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        settings['motion_optimization']['enabled'] = False
        settings.update(site_confirmed=True, landing_xy=[8.5, -4.2], corridor_waypoints=[
            dict(x=6.7, y=4., agl=1.4), dict(x=6.7, y=4., agl=.9),
            dict(x=8.3, y=4., agl=.9), dict(x=8.3, y=-4., agl=.9)])
        rig = yaml.safe_load((MISSION/'config/competition/known_rig.yaml').read_text())
        self.check_generator(generate, settings, rig)

    def test_trial_entry_all_four_descent_modes_and_two_compensation_modes(self):
        base = ROOT/'deployment/board_trials_4x4'
        sys.path.insert(0, str(base/'common/uav_board_trials/scripts'))
        from trial_config import generate
        settings = yaml.safe_load((base/'08_full_mission/settings.yaml').read_text())
        settings['motion_optimization']['enabled'] = False
        settings.update(landing_xy=[2.5, 0.], corridor_waypoints=[
            dict(x=.6, y=0., agl=.9), dict(x=1.5, y=.4, agl=.9)])
        rig = yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
        self.check_generator(generate, settings, rig)


if __name__ == '__main__':
    unittest.main()
