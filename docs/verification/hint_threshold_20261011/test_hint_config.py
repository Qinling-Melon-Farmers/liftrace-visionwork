"""Production coarse-HINT wiring only; no ROS, inference, or actuator calls."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'deployment/board_trials_4x4'
sys.path[:0] = [str(BASE / 'common/uav_board_trials/scripts'),
               str(ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission/src'),
               str(ROOT / 'vision_ws/src/uav_high_view/src')]
from trial_config import generate as trial_generate


PROFILES = ['field.example.yaml', 'candidates/snake_motion.yaml',
            'candidates/rectangle_motion.yaml', 'candidates/snake3_motion.yaml',
            'candidates/recovery_validated_field_local.yaml']
PROJECTOR = '/target_map_projector/coarse_min_confidence'


def read(path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def generated(generator, settings, rig):
    with tempfile.TemporaryDirectory() as directory:
        generator(ROOT, directory, settings, (.02, -.01, .09), rig)
        return {n: read(Path(directory) / (n + '.yaml'))
                for n in ('runtime', 'control', 'overrides')}


def field_settings(name):
    settings = read(ROOT / 'deployment/competition' / name)
    # Only an offline geometry fixture; source files stay unconfirmed.
    if not settings['site_confirmed']:
        settings.update(site_confirmed=True,
            corridor_waypoints=[dict(x=6.7, y=4., agl=1.4),
                               dict(x=6.7, y=4., agl=.9),
                               dict(x=8.3, y=4., agl=.9),
                               dict(x=8.3, y=-4., agl=.9)],
            landing_xy=[8.5, -4.2])
    return settings


class HintThresholdConfigTests(unittest.TestCase):
    def assert_hint_pair(self, docs, threshold):
        self.assertEqual(docs['runtime']['high_view_full']['policy']['coarse_min_confidence'], threshold)
        self.assertEqual(docs['overrides'][PROJECTOR], threshold)
        for key in docs['overrides']:
            self.assertNotIn('std_class_confidence', key)
            self.assertNotIn('cross_class_confidence', key)

    def test_all_trial_modules_generate_matching_hint_gates(self):
        rig = read(BASE / 'common/uav_board_trials/config/known_rig.yaml')
        for path in sorted(BASE.glob('*/settings.yaml')):
            settings = read(path)
            if settings.get('trial_kind') in ('corridor_landing', 'full_mission'):
                settings.update(corridor_waypoints=[dict(x=.6, y=0., agl=1.),
                                dict(x=1.5, y=.4, agl=1.)], landing_xy=[2.5, 0.])
            with self.subTest(module=path.parent.name):
                self.assert_hint_pair(generated(trial_generate, settings, rig), .75)

    def test_current_competition_profiles_generate_matching_hint_gates(self):
        rig = read(ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml')
        for name in PROFILES:
            with self.subTest(profile=name):
                self.assert_hint_pair(generated(competition_generate, field_settings(name), rig), .75)

    def test_confidence_override_changes_only_coarse_hint_gates(self):
        rig = read(ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml')
        settings = field_settings('field.example.yaml')
        reference = None
        for threshold in (.60, .70, .75):
            settings['survey_policy']['coarse_min_confidence'] = threshold
            docs = generated(competition_generate, settings, rig)
            self.assert_hint_pair(docs, threshold)
            normalized = copy.deepcopy(docs)
            del normalized['runtime']['high_view_full']['policy']['coarse_min_confidence']
            del normalized['overrides'][PROJECTOR]
            if reference is None:
                reference = normalized
            else:
                self.assertEqual(normalized, reference)

    def test_low_confirmation_thresholds_are_preserved(self):
        memory = read(ROOT / 'vision_ws/src/uav_vision/config/target_memory.yaml')
        self.assertEqual(memory['std_class_confidence'], 0.6)
        self.assertEqual(memory['cross_class_confidence'], 0.8)
        self.assertEqual(memory['std_geometry_confidence'], 0.7)
        self.assertEqual(memory['cross_geometry_confidence'], 0.7)

    def test_historical_field_keeps_recorded_hint_threshold(self):
        historical = read(ROOT / 'deployment/competition/field_20261007_validated.yaml')
        self.assertEqual(historical['survey_policy']['coarse_min_confidence'], .60)


# This branch deliberately lacks these deployment entry points.
delattr(HintThresholdConfigTests, 'test_current_competition_profiles_generate_matching_hint_gates')
delattr(HintThresholdConfigTests, 'test_confidence_override_changes_only_coarse_hint_gates')
delattr(HintThresholdConfigTests, 'test_historical_field_keeps_recorded_hint_threshold')

if __name__ == '__main__':
    unittest.main()
