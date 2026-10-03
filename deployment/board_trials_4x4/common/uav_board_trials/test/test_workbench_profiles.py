"""Site automation must agree with generated control and preserve H landing."""
from pathlib import Path
import tempfile
import unittest
import yaml
from trial_config import apply_site_profile, generate, validate_settings

ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT/'deployment/board_trials_4x4'
SITE = ROOT/'deployment/site_20260928'


class Profiles(unittest.TestCase):
    def settings(self, folder, profile):
        settings = yaml.safe_load((BASE/folder/'settings.yaml').read_text())
        return apply_site_profile(settings, yaml.safe_load((SITE/profile).read_text()))

    def test_h_auto_hover_uses_actual_takeoff_height(self):
        settings = self.settings('03_h_landing', 'h_landing_test_area.yaml')
        self.assertTrue(settings['auto_start_after_arm'])
        self.assertNotIn('terminal_hover_agl', settings)
        rig = yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for z in (-.09, 0., .09):
            with tempfile.TemporaryDirectory() as tmp:
                ref = generate(ROOT, tmp, settings, (0.,0.,z), rig)
                control = yaml.safe_load((Path(tmp)/'control.yaml').read_text())
                self.assertEqual(ref['takeoff_z'], control['waypoints'][0]['z'])
                self.assertAlmostEqual(ref['takeoff_z']-ref['ground_z'], 1.)
                self.assertNotAlmostEqual(ref['takeoff_z'], ref['low_z'], delta=.15)
                self.assertTrue(control['switch']['auto_land'])
                self.assertEqual(control['drop_system']['enable_drop'], False)
                self.assertAlmostEqual(control['external_landing']['capture_height']-ref['ground_z'], 1.8)

    def test_corridor_full_require_measured_geometry(self):
        for folder, profile in [('04_corridor_landing','corridor_landing_test_area.yaml'),
                                ('08_full_mission','full_mission_test_area.yaml')]:
            settings = self.settings(folder, profile)
            self.assertTrue(settings['auto_start_after_arm'])
            self.assertNotIn('terminal_hover_agl', settings)
            self.assertEqual(settings['actuator_mode'], 'mock')
            with self.assertRaises(ValueError): validate_settings(settings)
            # Explicit test fixture only; never written to a deployable YAML.
            apply_site_profile(settings, dict(corridor_waypoints=[dict(x=.6,y=0.,agl=1.),
                dict(x=1.5,y=.4,agl=1.)], landing_xy=[2.5,0.]))
            validate_settings(settings)
            rig = yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
            with tempfile.TemporaryDirectory() as tmp:
                ref = generate(ROOT,tmp,settings,(0.,0.,0.),rig)
                control = yaml.safe_load((Path(tmp)/'control.yaml').read_text())
                runtime = yaml.safe_load((Path(tmp)/'runtime.yaml').read_text())
                self.assertEqual(ref['takeoff_z'],control['waypoints'][0]['z'])
                self.assertTrue(control['switch']['auto_land'])
                self.assertEqual(len(runtime['mission']['post_delivery_route']),4)

    def test_cannot_import_terminal_hover_into_h_profiles(self):
        settings = self.settings('03_h_landing','h_landing_test_area.yaml')
        with self.assertRaisesRegex(ValueError,'terminal_hover'):
            apply_site_profile(settings,dict(terminal_hover_agl=.3))

    def test_profile_does_not_change_mission_or_actuator(self):
        settings = self.settings('03_h_landing','h_landing_test_area.yaml')
        for key, value in [('mode','memory_only'),('actuator_mode','real'),('corridor_waypoints',[])]:
            with self.assertRaises(ValueError): apply_site_profile(settings,{key:value})

    def test_h_center_must_be_finite_and_inside_flight_area(self):
        settings = self.settings('03_h_landing','h_landing_test_area.yaml')
        for center in (None,[float('nan'),0.],[7.,0.]):
            settings['landing_xy']=center
            with self.assertRaises(ValueError): validate_settings(settings)


if __name__ == '__main__':
    unittest.main(verbosity=2)
