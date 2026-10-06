"""Site automation must agree with generated control and preserve H landing."""
from pathlib import Path
import tempfile
import unittest
import yaml
from trial_config import apply_site_profile, generate, validate_settings
from trial_bag import topics_for

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
                runtime = yaml.safe_load((Path(tmp)/'runtime.yaml').read_text())
                self.assertAlmostEqual(runtime['mission']['post_delivery_route'][-1][2]-ref['ground_z'], 1.2)
                self.assertTrue(control['switch']['auto_land'])
                self.assertEqual(control['drop_system']['enable_drop'], False)
                self.assertAlmostEqual(control['external_landing']['capture_height']-ref['ground_z'], 1.2)
                self.assertAlmostEqual(control['land_height']-ref['ground_z'], .40)
                self.assertAlmostEqual(control['external_landing']['auto_land_height']-ref['ground_z'], .55)
                self.assertEqual(control['external_landing']['handoff_mode'], 'POSCTL')
                overrides = yaml.safe_load((Path(tmp)/'overrides.yaml').read_text())
                self.assertIs(overrides['/landing_detector/landing_enable_h_stroke_fallback'], True)

    def test_h_and_complete_mission_share_landing_gates(self):
        rig = yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        gates = []
        for folder, profile in [('03_h_landing','h_landing_test_area.yaml'),
                                ('04_corridor_landing','corridor_landing_test_area.yaml'),
                                ('08_full_mission','full_mission_test_area.yaml')]:
            settings = self.settings(folder, profile)
            if folder != '03_h_landing':
                # Test geometry only; retain empty measured-site YAMLs on disk.
                apply_site_profile(settings, dict(corridor_waypoints=[dict(x=.6,y=0.,agl=1.),
                    dict(x=1.5,y=.4,agl=1.)], landing_xy=[2.5,0.]))
            with tempfile.TemporaryDirectory() as tmp:
                ref = generate(ROOT, tmp, settings, (0.,0.,-.05), rig)
                control = yaml.safe_load((Path(tmp)/'control.yaml').read_text())
                overrides = yaml.safe_load((Path(tmp)/'overrides.yaml').read_text())
                self.assertIs(overrides['/landing_detector/landing_enable_h_stroke_fallback'], True)
                self.assertTrue(control['switch']['auto_land'])
                self.assertEqual(control['switch']['flag_landing_detect'], 1)
                self.assertAlmostEqual(control['land_height']-ref['ground_z'], .40)
                landing = dict(control['external_landing'])
                self.assertEqual(landing.pop('handoff_mode'),
                                 'POSCTL')
                self.assertAlmostEqual(landing.pop('capture_height')-ref['ground_z'], settings['landing_capture_agl'])
                self.assertEqual(landing['detections_topic'], '/uav_vision/detections_mapped')
                self.assertAlmostEqual(landing['auto_land_height']-ref['ground_z'], .55)
                self.assertEqual(landing['alignment_tolerance'], .08)
                self.assertEqual(landing['stable_frames'], 10)
                self.assertEqual(landing['mark_max_age_sec'], .5)
                gates.append(landing)
        self.assertEqual(gates[0], gates[1])
        self.assertEqual(gates[0], gates[2])

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

    def test_handoff_mode_must_be_explicit_supported_h_mode(self):
        settings = self.settings('03_h_landing','h_landing_test_area.yaml')
        for mode in ('', 'OFFBOARD', 'MANUAL', None):
            settings['landing_handoff_mode'] = mode
            with self.assertRaisesRegex(ValueError,'landing_handoff_mode'):
                validate_settings(settings)
        apply_site_profile(settings, dict(landing_handoff_mode='AUTO.LAND'))
        validate_settings(settings)
        settings = yaml.safe_load((BASE/'01_visual_interrupt/settings.yaml').read_text())
        settings['landing_handoff_mode'] = 'POSCTL'
        with self.assertRaisesRegex(ValueError,'only applies'):
            validate_settings(settings)

    def test_custom_handoff_topic_matches_generated_control_and_bag(self):
        settings = self.settings('03_h_landing', 'h_landing_test_area.yaml')
        rig = yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for topic in ('/patrol_control/external_landing_handoff', '/site/h_landing_handoff'):
            apply_site_profile(settings, dict(landing_handoff_status_topic=topic))
            with tempfile.TemporaryDirectory() as tmp:
                generate(ROOT, tmp, settings, (0., 0., 0.), rig)
                control = yaml.safe_load((Path(tmp)/'control.yaml').read_text())
                self.assertEqual(control['external_landing']['handoff_status_topic'], topic)
                self.assertIn(topic, topics_for(settings))
                if topic != '/patrol_control/external_landing_handoff':
                    self.assertNotIn('/patrol_control/external_landing_handoff', topics_for(settings))
        for topic in ('', 'relative', '/bad topic', None):
            settings['landing_handoff_status_topic'] = topic
            with self.assertRaisesRegex(ValueError, 'landing_handoff_status_topic'):
                validate_settings(settings)

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
