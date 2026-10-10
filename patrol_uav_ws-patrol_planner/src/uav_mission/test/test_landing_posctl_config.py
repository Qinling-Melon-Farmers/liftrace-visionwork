"""离线验证 POSCTL 低位配置及 AUTO.LAND 原高度，不启动 ROS。"""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[4]
PKG = ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path[:0] = [str(PKG / 'src'), str(ROOT / 'vision_ws/src/uav_high_view/src'),
]
from uav_mission.landing_posctl_config import (DEFAULTS, validate_landing_posctl,
                                              landing_control_parameters)
from uav_mission.competition_config import generate as competition_generate


class LowHConfigTests(unittest.TestCase):
    def setUp(self):
        self.rig = yaml.safe_load((PKG / 'config/competition/known_rig.yaml').read_text())
        self.settings = yaml.safe_load((ROOT / 'deployment/competition/field.example.yaml').read_text())
        # H configuration fixtures isolate motion policy and need no wall-plane measurements.
        self.settings.setdefault('motion_optimization', {})['enabled'] = False
        # Isolate low local-Z H behavior from the independent drop-Z > 0.05 limit.
        self.settings['drop_agl'] = .45
        self.settings.update(site_confirmed=True,
            corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),
                                dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],
            landing_xy=[8.5,-4.2])

    def generated(self, function, settings, z=-0.04349584877490997):
        with tempfile.TemporaryDirectory(prefix='low_h_config_') as path:
            ref = function(ROOT, path, settings, (0., 0., z), self.rig)
            control = yaml.safe_load((Path(path) / 'control.yaml').read_text())
        return ref, control

    def check_profile(self, ref, control, mode):
        landing = control['external_landing']
        target, trigger = (.35, .37) if mode == 'POSCTL' else (.40, .55)
        self.assertAlmostEqual(control['land_height'] - ref['ground_z'], target)
        self.assertAlmostEqual(landing['auto_land_height'] - ref['ground_z'], trigger)
        self.assertEqual(landing['alignment_tolerance'], .08)
        self.assertEqual(landing['stable_frames'], 10)
        self.assertEqual(landing['handoff_mode'], mode)
        if mode == 'POSCTL':
            self.assertEqual(landing['posctl'], {k:v for k,v in DEFAULTS.items()
                                               if k not in ('target_agl','trigger_agl')})
        else:
            self.assertNotIn('posctl', landing)

    def test_competition_modes_translate_the_same_ground_reference(self):
        for mode in ('POSCTL', 'AUTO.LAND'):
            settings = copy.deepcopy(self.settings)
            settings['landing_handoff_mode'] = mode
            for z in (-.09, -.05, 0., .09):
                with self.subTest(mode=mode, z=z):
                    ref, control = self.generated(competition_generate, settings, z)
                    self.check_profile(ref, control, mode)

    def test_all_hardware_templates_explicitly_declare_the_same_candidate(self):
        files = [ROOT / 'deployment/competition' / name for name in
                  ('field.example.yaml', 'candidates/rectangle_motion.yaml',
                   'candidates/snake_motion.yaml', 'candidates/snake3_motion.yaml')]
        for file in files:
            with self.subTest(file=str(file)):
                settings = yaml.safe_load(file.read_text())
                self.assertEqual(settings['landing_handoff_mode'], 'POSCTL')
                expected = dict(DEFAULTS)
                self.assertEqual(settings['landing_posctl'], expected)

    def test_slot_semantics_and_compensation_preserve_both_measured_tables(self):
        rig_before = copy.deepcopy(self.rig)
        for function, settings in ((competition_generate, self.settings),):
            with self.subTest(generator=function.__module__):
                _, control = self.generated(function, settings)
                drop = control['drop_system']
                self.assertEqual(drop['slot_offset_semantics'], 'body_flu_lever_arm')
                self.assertIs(drop['compensated_alignment'], True)
                for key in ('slot_offsets', 'dynamic_slot_offsets'):
                    self.assertEqual(drop[key], rig_before[key])
                self.assertEqual(self.rig, rig_before)

    def test_speed_error_duration_and_height_overrides_reach_control(self):
        settings = copy.deepcopy(self.settings)
        settings['landing_posctl'].update(target_agl=.36, trigger_agl=.39,
            xy_tolerance_m=.04, height_tolerance_m=.015,
            capture_height_tolerance_m=.12,
            max_horizontal_speed_mps=.025, max_vertical_speed_mps=.04,
            stable_duration_sec=.75, max_odom_age_sec=.15,
            max_sample_gap_sec=.15, min_samples=4)
        ref, control = self.generated(competition_generate, settings)
        self.assertAlmostEqual(control['land_height']-ref['ground_z'], .36)
        self.assertAlmostEqual(control['external_landing']['auto_land_height']-ref['ground_z'], .39)
        self.assertEqual(control['external_landing']['posctl'],
                         {k:v for k,v in settings['landing_posctl'].items()
                          if k not in ('target_agl','trigger_agl')})

    def test_invalid_values_order_and_unknown_keys_fail_before_generation(self):
        for key in DEFAULTS:
            bad_values = (True, 2, 3.0) if key == 'min_samples' else (True, 0, -1, float('nan'), float('inf'), '0.1')
            for value in bad_values:
                settings = copy.deepcopy(self.settings)
                settings['landing_posctl'][key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    validate_landing_posctl(settings)
        for change in ({'trigger_agl':.34}, {'trigger_agl':1.8},
                       {'xy_tolerance_m':.081}, {'extra':1}):
            settings = copy.deepcopy(self.settings)
            settings['landing_posctl'].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_landing_posctl(settings)

    def test_stationary_clearance_and_existing_local_z_limit_are_enforced(self):
        settings = copy.deepcopy(self.settings)
        settings['landing_posctl']['target_agl'] = .23
        with self.assertRaisesRegex(ValueError, 'stationary FC'):
            landing_control_parameters(settings, -.2635, .22)
        # 原 FC 基准 -0.09m 保持：地面 -0.31m，目标本地 Z 0.04m 合法。
        ref, control = self.generated(competition_generate, self.settings, -.09)
        self.assertAlmostEqual(ref['ground_z'], -.31)
        self.assertAlmostEqual(control['land_height'], .04)
        for function, settings in ((competition_generate, self.settings),):
            with self.subTest(generator=function.__module__), self.assertRaisesRegex(ValueError, 'positive local-Z'):
                self.generated(function, settings, -.14)

    def test_missing_block_uses_candidate_only_for_posctl(self):
        settings = dict(landing_capture_agl=1.2, landing_handoff_mode='POSCTL')
        target, trigger, stable = landing_control_parameters(settings, -.22, .22)
        self.assertAlmostEqual(target, .13)
        self.assertAlmostEqual(trigger, .15)
        self.assertEqual(stable['min_samples'], 3)
        settings['landing_handoff_mode'] = 'AUTO.LAND'
        target, trigger, stable = landing_control_parameters(settings, -.22, .22)
        self.assertAlmostEqual(target, .18)
        self.assertAlmostEqual(trigger, .33)
        self.assertIsNone(stable)

    def test_h_trial_reference_fixture_remains_isolated(self):
        reference = yaml.safe_load((PKG / 'test/fixtures/h_trial_reference_20261008.yaml').read_text())
        self.assertEqual(reference['landing_capture_agl'], 1.2)
        self.assertNotIn('landing_posctl', reference)
        self.assertNotIn('landing_enable_h_stroke_fallback', reference)


if __name__ == '__main__':
    unittest.main()
