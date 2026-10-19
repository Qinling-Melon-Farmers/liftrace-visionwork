"""离线验证 POSCTL 低位配置及 AUTO.LAND 原高度，不启动 ROS。"""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[4]
PKG = ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission'
TRIAL = ROOT / 'deployment/board_trials_4x4/common/uav_board_trials'
sys.path[:0] = [str(PKG / 'src'), str(ROOT / 'vision_ws/src/uav_high_view/src'),
               str(TRIAL / 'scripts')]
from uav_mission.landing_posctl_config import (DEFAULTS, validate_landing_posctl,
                                              landing_control_parameters, landing_descent_parameters,
                                              landing_alignment_mode)
from uav_mission.competition_config import generate as competition_generate
from trial_config import generate as trial_generate, apply_site_profile, validate_settings


class LowHConfigTests(unittest.TestCase):
    def setUp(self):
        self.rig = yaml.safe_load((TRIAL / 'config/known_rig.yaml').read_text())
        self.settings = yaml.safe_load((ROOT / 'deployment/competition/field.example.yaml').read_text())
        # H configuration fixtures isolate motion policy and need no wall-plane measurements.
        self.settings.setdefault('motion_optimization', {})['enabled'] = False
        # Isolate H height bounds from the independent delivery local-Z >5cm gate.
        self.settings.update(drop_agl=.40, release_min_agl=.37, release_max_agl=.45)
        self.settings.update(site_confirmed=True,
            corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),
                                dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],
            landing_xy=[8.5,-4.2])

    def generated(self, function, settings, z=-0.04349584877490997):
        with tempfile.TemporaryDirectory(prefix='low_h_config_') as path:
            ref = function(ROOT, path, settings, (0., 0., z), self.rig)
            control = yaml.safe_load((Path(path) / 'control.yaml').read_text())
        return ref, control

    def check_profile(self, ref, control, mode, alignment='non_vh', posctl_trigger=.40):
        landing = control['external_landing']
        target, trigger = (.35, posctl_trigger) if mode == 'POSCTL' else (.40, .55)
        self.assertAlmostEqual(control['land_height'] - ref['ground_z'], target)
        self.assertAlmostEqual(landing['auto_land_height'] - ref['ground_z'], trigger)
        self.assertEqual(landing['alignment_tolerance'], .08)
        self.assertEqual(landing['stable_frames'], 10)
        self.assertEqual(landing['alignment_mode'], alignment)
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

    def test_three_h_trial_templates_keep_auto_land_isolated(self):
        for name in ('03_h_landing', '04_corridor_landing', '08_full_mission'):
            settings = yaml.safe_load((ROOT / 'deployment/board_trials_4x4' / name / 'settings.yaml').read_text())
            if name != '03_h_landing':
                settings.update(corridor_waypoints=[dict(x=1.,y=0.,agl=.9),
                                                     dict(x=2.,y=0.,agl=.9)],
                                landing_xy=[2.,0.])
            # Keep this H-only fixture independent of the board's lower delivery window.
            if name == '08_full_mission':
                settings.update(drop_agl=.40, release_min_agl=.37, release_max_agl=.45)
            for mode in ('POSCTL', 'AUTO.LAND'):
                with self.subTest(name=name, mode=mode):
                    settings['landing_handoff_mode'] = mode
                    # This regression isolates handoff profiles, not preset-H fallback.
                    settings['landing_coordinate_fallback'] = {'enabled': False}
                    settings['landing_alignment_mode'] = 'vh' if name in ('03_h_landing', '08_full_mission') and mode == 'POSCTL' else 'non_vh'
                    ref, control = self.generated(trial_generate, settings)
                    self.check_profile(ref, control, mode, settings['landing_alignment_mode'],
                                       .50 if name == '08_full_mission' else .40)

    def test_all_hardware_templates_explicitly_declare_the_same_candidate(self):
        files = [ROOT / 'deployment/board_trials_4x4' / name / 'settings.yaml'
                 for name in ('03_h_landing', '04_corridor_landing', '08_full_mission')]
        files += [ROOT / 'deployment/competition' / name for name in
                  ('field.example.yaml', 'field_20261007_validated.yaml',
                   'candidates/rectangle_motion.yaml', 'candidates/snake_motion.yaml')]
        for file in files:
            with self.subTest(file=str(file)):
                settings = yaml.safe_load(file.read_text())
                self.assertEqual(settings['landing_handoff_mode'], 'POSCTL')
                expected = dict(DEFAULTS)
                if file.parent.name == '08_full_mission':
                    expected['trigger_agl'] = .50  # Preserve current field handoff.
                if file.name == 'field_20261007_validated.yaml':
                    # This confirmed field file is the preserved historical snapshot.
                    expected.update(trigger_agl=.37, height_tolerance_m=.02,
                                    max_horizontal_speed_mps=.03,
                                    max_vertical_speed_mps=.05, stable_duration_sec=.5)
                self.assertEqual(settings['landing_posctl'], expected)

    def test_slot_semantics_and_compensation_preserve_both_measured_tables(self):
        trial = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/03_h_landing/settings.yaml').read_text())
        rig_before = copy.deepcopy(self.rig)
        for function, settings in ((trial_generate, trial),
                                   (competition_generate, self.settings)):
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
        trial = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/03_h_landing/settings.yaml').read_text())
        _, control = self.generated(trial_generate, trial, -.09)
        self.assertAlmostEqual(control['land_height'], .04)
        for function, settings in ((competition_generate, self.settings), (trial_generate, trial)):
            with self.subTest(generator=function.__module__), self.assertRaisesRegex(ValueError, 'local-Z'):
                self.generated(function, settings, -.14)

    def test_missing_block_uses_candidate_only_for_posctl(self):
        settings = dict(landing_capture_agl=1.2, landing_handoff_mode='POSCTL')
        target, trigger, stable = landing_control_parameters(settings, -.22, .22)
        self.assertAlmostEqual(target, .13)
        self.assertAlmostEqual(trigger, .18)
        self.assertEqual(stable['min_samples'], 3)
        settings['landing_handoff_mode'] = 'AUTO.LAND'
        target, trigger, stable = landing_control_parameters(settings, -.22, .22)
        self.assertAlmostEqual(target, .18)
        self.assertAlmostEqual(trigger, .33)
        self.assertIsNone(stable)

    def test_site_override_only_applies_to_h_modes(self):
        h = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/03_h_landing/settings.yaml').read_text())
        apply_site_profile(h, {'landing_posctl': {'xy_tolerance_m':.04}})
        validate_settings(h)
        self.assertEqual(validate_landing_posctl(h)['xy_tolerance_m'], .04)
        self.assertEqual(validate_landing_posctl(h)['target_agl'], .35)
        non_h = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/01_visual_interrupt/settings.yaml').read_text())
        with self.assertRaises(ValueError):
            apply_site_profile(non_h, {'landing_posctl': {}})
        non_h['landing_posctl'] = {}
        with self.assertRaisesRegex(ValueError, 'visual H landing'):
            validate_settings(non_h)

    def test_high_switch_reaches_both_generators_and_preserves_low_parameters(self):
        trial = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/03_h_landing/settings.yaml').read_text())
        trial['landing_alignment_mode'] = 'non_vh'  # Explicitly exercise the fallback policy.
        for function, settings in ((trial_generate, trial), (competition_generate, self.settings)):
            _, low = self.generated(function, settings)
            high_settings = copy.deepcopy(settings)
            high_settings['landing_alignment_mode'] = 'vh'
            _, high = self.generated(function, high_settings)
            self.assertEqual(high['external_landing']['alignment_mode'], 'vh')
            high['external_landing']['alignment_mode'] = 'non_vh'
            self.assertEqual(high, low)
        apply_site_profile(trial, {'landing_alignment_mode': 'vh'})
        validate_settings(trial)
        for mode in ('invalid', None, True):
            trial['landing_alignment_mode'] = mode
            with self.assertRaises(ValueError): validate_settings(trial)
        trial.update(landing_alignment_mode='vh', landing_handoff_mode='AUTO.LAND')
        with self.assertRaisesRegex(ValueError, 'requires POSCTL'): validate_settings(trial)
        non_h = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/01_visual_interrupt/settings.yaml').read_text())
        with self.assertRaises(ValueError): apply_site_profile(non_h, {'landing_alignment_mode': 'vh'})


    def test_descent_modes_are_independent_of_capture_and_delivery(self):
        trial = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/03_h_landing/settings.yaml').read_text())
        for function, settings in ((trial_generate, trial), (competition_generate, self.settings)):
            _, base = self.generated(function, settings)
            for alignment in ('non_vh', 'vh'):
                for mode in (1, 2, 3):
                    configured = copy.deepcopy(settings)
                    configured.update(landing_alignment_mode=alignment, landing_descent_mode=mode,
                                      landing_descent_speed_mps=.25)
                    _, actual = self.generated(function, configured)
                    self.assertEqual(actual['external_landing']['descent'],
                                     dict(mode=mode, speed_mps=.25, max_step_dt_sec=.20))
                    actual['external_landing']['alignment_mode'] = base['external_landing']['alignment_mode']
                    actual['external_landing']['descent'] = base['external_landing']['descent']
                    self.assertEqual(actual, base)
        apply_site_profile(trial, dict(landing_descent_mode=3, landing_descent_speed_mps=.3))
        validate_settings(trial)
        non_h = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/01_visual_interrupt/settings.yaml').read_text())
        with self.assertRaises(ValueError): apply_site_profile(non_h, dict(landing_descent_mode=3))
        non_h['landing_descent_mode'] = 3
        with self.assertRaises(ValueError): validate_settings(non_h)

    def test_descent_invalid_values_and_defaults(self):
        self.assertEqual(landing_descent_parameters({}), dict(mode=1, speed_mps=.20, max_step_dt_sec=.20))
        for mode in (True, 0, 4, '2', 2.0):
            with self.assertRaises(ValueError): landing_descent_parameters(dict(landing_descent_mode=mode))
        for speed in (True, 0, -.1, 1.01, float('nan'), float('inf'), '0.2'):
            with self.assertRaises(ValueError): landing_descent_parameters(dict(landing_descent_speed_mps=speed))


    def test_names_legacy_aliases_and_current_full_mission_selection(self):
        self.assertEqual(landing_alignment_mode({}), 'non_vh')
        for supplied, expected in (('low', 'non_vh'), ('high', 'vh'),
                                   ('non_vh', 'non_vh'), ('vh', 'vh')):
            settings = copy.deepcopy(self.settings)
            settings['landing_alignment_mode'] = supplied
            self.assertEqual(landing_alignment_mode(settings), expected)
            _, control = self.generated(competition_generate, settings)
            self.assertEqual(control['external_landing']['alignment_mode'], expected)
        settings = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/08_full_mission/settings.yaml').read_text())
        self.assertEqual(settings['landing_alignment_mode'], 'vh')
        self.assertEqual(settings['landing_descent_mode'], 2)
        settings.update(corridor_waypoints=[dict(x=1., y=0., agl=.9), dict(x=2., y=0., agl=.9)], landing_xy=[2., 0.])
        _, control = self.generated(trial_generate, settings)
        self.assertEqual(control['external_landing']['alignment_mode'], 'vh')
        self.assertEqual(control['external_landing']['descent']['mode'], 2)


if __name__ == '__main__':
    unittest.main()
