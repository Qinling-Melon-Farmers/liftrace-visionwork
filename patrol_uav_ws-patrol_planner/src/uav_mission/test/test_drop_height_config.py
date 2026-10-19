"""Descent target vs final release window vs broader task preauthorization."""
import copy
import sys
import tempfile
import unittest
from pathlib import Path

import yaml
from uav_mission.drop_height_config import release_heights
from uav_mission.competition_config import generate, validate

ROOT = Path(__file__).resolve().parents[4]


class DropHeightConfigTests(unittest.TestCase):
    def test_explicit_window_keeps_permission_tolerance_independent_of_target(self):
        for target in (.35, .375, .40, .425, .45):
            values = release_heights(dict(drop_agl=target, release_min_agl=.35, release_max_agl=.45))
            for actual, expected in zip(values, (.35, .45, .27, .47)):
                self.assertAlmostEqual(actual, expected)

    def test_legacy_and_no_drop_settings_keep_original_generated_ranges(self):
        for target in (.10, .35, .6):
            values = release_heights(dict(drop_agl=target))
            for actual, expected in zip(values, (target, target+.1, target-.08, target+.12)):
                self.assertAlmostEqual(actual, expected)

    def test_invalid_or_partial_window_is_rejected(self):
        valid = dict(drop_agl=.40, release_min_agl=.35, release_max_agl=.45)
        bad = [dict(drop_agl=.4, release_min_agl=.35),
               dict(drop_agl=.4, release_max_agl=.45)]
        for key in ('drop_agl', 'release_min_agl', 'release_max_agl'):
            bad.extend(dict(valid, **{key: v}) for v in (None, True, '0.4', float('nan'), float('inf')))
        bad.extend([dict(valid, drop_agl=.46), dict(valid, release_min_agl=.41),
                    dict(valid, release_max_agl=.39), dict(valid, release_min_agl=0)])
        for config in bad:
            with self.subTest(config=config), self.assertRaises(ValueError):
                release_heights(config)

    def test_historical_measured_profile_retains_original_descent_and_permission(self):
        settings = yaml.safe_load((ROOT/'deployment/competition/field_20261007_validated.yaml').read_text())
        original = copy.deepcopy(settings)
        self.assertEqual(settings['drop_agl'], .35)
        self.assertNotIn('release_min_agl', settings)
        self.assertNotIn('release_max_agl', settings)
        rig = yaml.safe_load((ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
        with tempfile.TemporaryDirectory() as path:
            ref = generate(ROOT, path, settings, (0, 0, -.05), rig)
            control = yaml.safe_load((Path(path)/'control.yaml').read_text())['drop_system']
            overrides = yaml.safe_load((Path(path)/'overrides.yaml').read_text())
        ground = ref['ground_z']
        for key, agl in [('release_setpoint_height', .35), ('release_min_height', .35), ('height_threshold', .45)]:
            self.assertAlmostEqual(control[key]-ground, agl)
        self.assertAlmostEqual(overrides['/release_permission_arbiter/min_release_altitude']-ground, .27)
        self.assertAlmostEqual(overrides['/release_permission_arbiter/max_release_altitude']-ground, .47)
        self.assertEqual(settings, original)

    def test_shifted_origin_preserves_valid_agl_release_floor(self):
        settings = yaml.safe_load((ROOT/'deployment/competition/field_20261007_validated.yaml').read_text())
        settings.update(drop_agl=.40, release_min_agl=.35, release_max_agl=.45)
        validate(settings, flight=True)
        rig = yaml.safe_load((ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
        # A shifted local origin must not invalidate a positive AGL floor.
        with tempfile.TemporaryDirectory() as path:
            ref = generate(ROOT, path, settings, (0, 0, -.09), rig)
            control = yaml.safe_load((Path(path)/'control.yaml').read_text())['drop_system']
        self.assertAlmostEqual(control['release_min_height'], .04)
        self.assertAlmostEqual(control['release_min_height']-ref['ground_z'], .35)



    def test_requested_lower_window_generates_in_competition(self):
        settings = yaml.safe_load((ROOT/'deployment/competition/field_20261007_validated.yaml').read_text())
        settings.update(drop_agl=.30, release_min_agl=.28, release_max_agl=.35)
        validate(settings, flight=True)
        rig = yaml.safe_load((ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
        with tempfile.TemporaryDirectory() as path:
            ref = generate(ROOT, path, settings, (0, 0, 0), rig)
            control = yaml.safe_load((Path(path)/'control.yaml').read_text())['drop_system']
            overrides = yaml.safe_load((Path(path)/'overrides.yaml').read_text())
        for key, agl in [('release_setpoint_height', .30), ('release_min_height', .28), ('height_threshold', .35)]:
            self.assertAlmostEqual(control[key]-ref['ground_z'], agl)
        self.assertAlmostEqual(overrides['/release_permission_arbiter/min_release_altitude']-ref['ground_z'], .20)
        self.assertAlmostEqual(overrides['/release_permission_arbiter/max_release_altitude']-ref['ground_z'], .37)
        # Positive AGL remains valid even if the local floor is below .05m.
        with tempfile.TemporaryDirectory() as path:
            ref = generate(ROOT, path, settings, (0, 0, -.02), rig)
            control = yaml.safe_load((Path(path)/'control.yaml').read_text())['drop_system']
        self.assertAlmostEqual(control['release_min_height'], .04)
        self.assertAlmostEqual(control['release_setpoint_height']-ref['ground_z'], .30)

    def test_five_delivery_defaults_generate_requested_window(self):
        previous_path = sys.path[:]
        self.addCleanup(lambda: setattr(sys, 'path', previous_path))
        sys.path.insert(0, str(ROOT/'deployment/board_trials_4x4/common/uav_board_trials/scripts'))
        from trial_config import apply_site_profile, generate as generate_trial
        base = ROOT/'deployment/board_trials_4x4'
        rig = yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for folder in ('01_visual_interrupt', '02_high_view_revisit', '05_low_multi', '06_high_priority', '08_full_mission'):
            with self.subTest(folder=folder):
                settings = yaml.safe_load((base/folder/'settings.yaml').read_text())
                expected = [.15,.10,.43] if folder == '08_full_mission' else [.30,.28,.35]
                self.assertEqual([settings[k] for k in ('drop_agl','release_min_agl','release_max_agl')], expected)
                if folder == '08_full_mission':
                    # Geometry is an offline fixture, never a deployed replacement.
                    apply_site_profile(settings, dict(corridor_waypoints=[dict(x=.6,y=0.,agl=1.),dict(x=1.5,y=.4,agl=1.)],landing_xy=[2.5,0.]))
                with tempfile.TemporaryDirectory() as path:
                    ref = generate_trial(ROOT, path, settings, (0,0,0), rig)
                    control = yaml.safe_load((Path(path)/'control.yaml').read_text())['drop_system']
                for key, agl in zip(('release_setpoint_height','release_min_height','height_threshold'), expected):
                    self.assertAlmostEqual(control[key]-ref['ground_z'], agl)


if __name__ == '__main__':
    unittest.main()
