"""Exercise opt-in mode 4 through real generators and offline CLI paths."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[4]
PKG = ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission'
TRIAL = ROOT / 'deployment/board_trials_4x4/common/uav_board_trials'
sys.path[:0] = [str(PKG / 'src'), str(ROOT / 'vision_ws/src/uav_high_view/src'), str(TRIAL / 'scripts')]
HAS_FORMAL = (PKG / 'src/uav_mission/competition_config.py').is_file()
if HAS_FORMAL:
    from uav_mission.competition_config import generate as formal_generate, validate as formal_validate
else:
    formal_generate = formal_validate = None
from uav_mission.landing_posctl_config import landing_control_parameters, landing_descent_parameters
from uav_mission.landing_offboard_config import (DEFAULTS, DEFAULT_TOPIC, apply_landing_override,
                                               landing_offboard_parameters, validate_landing_offboard)
if TRIAL.is_dir():
    from trial_config import apply_site_profile, generate as trial_generate, validate_settings

PATCH = ROOT / 'deployment/competition/candidates/h_offboard_mode4.override.yaml'
FORMAL_FILES = ('field.example.yaml', 'candidates/rectangle_motion.yaml', 'candidates/snake_motion.yaml',
                'candidates/snake3_motion.yaml', 'candidates/recovery_validated_field_local.yaml')
TRIAL_FILES = ('03_h_landing', '04_corridor_landing', '08_full_mission')


class OffboardHelperContractTests(unittest.TestCase):
    """Also runs in the formal-only navigation branch, without board-trial fixtures."""
    def test_default_contract_and_profile_speed_are_independent(self):
        settings = dict(landing_descent_mode=4, landing_handoff_mode='OFFBOARD', landing_capture_agl=.9)
        parameters = landing_offboard_parameters(settings, -.13, .22)
        self.assertEqual(parameters, dict(offboard_ground_z=-.13, offboard_fc_clearance=.22,
            offboard_descent_speed=.30, offboard_touchdown_overtravel=.04,
            offboard_ground_confirm_duration=.3, offboard_state_max_age=2.5,
            offboard_disarm_retry=1., offboard_setpoint_topic=DEFAULT_TOPIC))
        self.assertEqual(landing_descent_parameters(settings)['speed_mps'], .20)
        staging, unused_handoff, stable = landing_control_parameters(settings, -.13, .22)
        self.assertAlmostEqual(staging, .17)
        self.assertAlmostEqual(unused_handoff, .24)
        self.assertIsNone(stable)
        self.assertAlmostEqual(parameters['offboard_ground_z'] + parameters['offboard_fc_clearance']
                               - parameters['offboard_touchdown_overtravel'], .05)

    def test_mode_pair_and_physical_range_without_trial_package(self):
        for settings in (dict(landing_descent_mode=4), dict(landing_handoff_mode='OFFBOARD'),
                         dict(landing_descent_mode=4, landing_handoff_mode='POSCTL')):
            with self.assertRaises(ValueError): landing_descent_parameters(settings)
        valid = dict(landing_descent_mode=4, landing_handoff_mode='OFFBOARD', landing_capture_agl=.9)
        for key in DEFAULTS:
            with self.assertRaises(ValueError):
                validate_landing_offboard(dict(valid, landing_offboard={key:float('nan')}))
        with self.assertRaises(ValueError):
            landing_offboard_parameters(dict(valid, landing_offboard={'touchdown_overtravel':.22}), -.13, .22)

    @unittest.skipUnless(HAS_FORMAL, 'H branch does not package formal competition entry')
    def test_formal_generator_contract_without_trial_package(self):
        settings = yaml.safe_load((ROOT / 'deployment/competition/field.example.yaml').read_text())
        settings.setdefault('motion_optimization', {})['enabled'] = False
        settings.update(site_confirmed=True,
            corridor_waypoints=[dict(x=6.7, y=4., agl=1.4), dict(x=6.7, y=4., agl=.9),
                                dict(x=8.3, y=4., agl=.9), dict(x=8.3, y=-4., agl=.9)],
            landing_xy=[8.5, -4.2])
        settings = apply_landing_override(settings, dict(landing_descent_mode=4,
            landing_handoff_mode='OFFBOARD', landing_alignment_mode='high'))
        rig = yaml.safe_load((PKG / 'config/competition/known_rig.yaml').read_text())
        with tempfile.TemporaryDirectory() as out:
            ref = formal_generate(ROOT, out, settings, (0., 0., .09), rig)
            control = yaml.safe_load((Path(out) / 'control.yaml').read_text())
        landing = control['external_landing']
        self.assertEqual(landing['offboard_ground_z'], ref['ground_z'])
        self.assertEqual(landing['offboard_fc_clearance'], rig['fc_ground_clearance'])
        self.assertEqual(landing['offboard_descent_speed'], .30)
        self.assertEqual(landing['offboard_setpoint_topic'], DEFAULT_TOPIC)
        self.assertNotIn('posctl', landing)


@unittest.skipUnless(TRIAL.is_dir(), 'trial modules not packaged on this branch')
class OffboardConfigTests(unittest.TestCase):
    def setUp(self):
        self.patch = yaml.safe_load(PATCH.read_text())
        self.rig = yaml.safe_load((TRIAL / 'config/known_rig.yaml').read_text())

    def settings(self, name, formal=False):
        path = ROOT / ('deployment/competition' if formal else 'deployment/board_trials_4x4') / name
        settings = yaml.safe_load((path if formal else path / 'settings.yaml').read_text())
        # Only fixture geometry/policy: keep production release/local-Z constraints intact.
        settings.setdefault('motion_optimization', {})['enabled'] = False
        if formal:
            if not settings['site_confirmed']:
                settings.update(site_confirmed=True,
                    corridor_waypoints=[dict(x=6.7, y=4., agl=1.4), dict(x=6.7, y=4., agl=.9),
                                        dict(x=8.3, y=4., agl=.9), dict(x=8.3, y=-4., agl=.9)],
                    landing_xy=[8.5, -4.2])
        elif name != '03_h_landing':
            settings.update(corridor_waypoints=[dict(x=1., y=0., agl=.9), dict(x=2., y=0., agl=.9)],
                            landing_xy=[2., 0.])
        return settings

    def generate(self, function, settings, z=.09):
        with tempfile.TemporaryDirectory(prefix='h_mode4_') as out:
            ref = function(ROOT, out, settings, (0., 0., z), self.rig)
            docs = {name: yaml.safe_load((Path(out) / (name + '.yaml')).read_text())
                    for name in ('control', 'runtime', 'overrides')}
        return ref, docs

    def test_every_template_stays_opt_in_and_real_generators_wire_contract(self):
        for formal, names in ((False, TRIAL_FILES), (True, FORMAL_FILES)):
            if formal and not HAS_FORMAL: continue
            for name in names:
                base = self.settings(name, formal)
                self.assertEqual(base.get('landing_descent_mode', 1), 1)
                self.assertEqual(base['landing_handoff_mode'], 'POSCTL')
                before = copy.deepcopy(base)
                for alignment in ('low', 'high'):
                    configured = apply_landing_override(base, dict(self.patch, landing_alignment_mode=alignment))
                    function = formal_generate if formal else trial_generate
                    for z in (0., .09):
                        with self.subTest(name=name, alignment=alignment, z=z):
                            ref, docs = self.generate(function, configured, z)
                            landing = docs['control']['external_landing']; ground = ref['ground_z']
                            self.assertEqual(landing['handoff_mode'], 'OFFBOARD')
                            self.assertEqual(landing['alignment_mode'], alignment)
                            self.assertEqual(landing['descent'], dict(mode=4, speed_mps=.20, max_step_dt_sec=.20))
                            self.assertNotIn('posctl', landing)
                            self.assertAlmostEqual(docs['control']['land_height'], ground + .30)
                            self.assertAlmostEqual(landing['auto_land_height'], ground + .37)
                            self.assertEqual(landing['offboard_ground_z'], ground)
                            self.assertEqual(landing['offboard_fc_clearance'], self.rig['fc_ground_clearance'])
                            for key, value in DEFAULTS.items():
                                self.assertEqual(landing['offboard_' + key], value)
                            self.assertAlmostEqual(landing['offboard_ground_z'] + landing['offboard_fc_clearance']
                                                   - landing['offboard_touchdown_overtravel'], z - .04)
                            self.assertEqual(landing['offboard_setpoint_topic'], DEFAULT_TOPIC)
                            self.assertEqual(docs['overrides']['/navigation_frame_adapter/landing_touchdown_setpoint_topic'], DEFAULT_TOPIC)
                            # Existing measured POSCTL fallback remains in the source profile.
                            self.assertEqual(configured['landing_posctl'], base['landing_posctl'])
                self.assertEqual(base, before)

    def test_mode4_iff_offboard_for_both_validators_and_helper(self):
        for formal, name in ((False, TRIAL_FILES[0]), (True, FORMAL_FILES[0])):
            if formal and not HAS_FORMAL: continue
            base = self.settings(name, formal)
            validator = formal_validate if formal else validate_settings
            for mode in (1, 2, 3, 4):
                for handoff in ('AUTO.LAND', 'POSCTL', 'OFFBOARD'):
                    settings = dict(base, landing_descent_mode=mode, landing_handoff_mode=handoff)
                    with self.subTest(formal=formal, mode=mode, handoff=handoff):
                        if (mode == 4) == (handoff == 'OFFBOARD'):
                            validator(settings)
                        else:
                            with self.assertRaisesRegex(ValueError, 'landing_handoff_mode'):
                                validator(settings)
        self.assertEqual(landing_descent_parameters({})['speed_mps'], .20)
        self.assertEqual(landing_descent_parameters(dict(landing_descent_mode=4, landing_handoff_mode='OFFBOARD'))['speed_mps'], .20)
        self.assertEqual(validate_landing_offboard({})['descent_speed'], .30)

    def test_finite_physical_settings_and_stationary_clearance(self):
        settings = apply_landing_override(self.settings(TRIAL_FILES[0]), self.patch)
        for key in DEFAULTS:
            for value in (True, 0, -.1, float('nan'), float('inf'), '0.3', None):
                bad = copy.deepcopy(settings); bad['landing_offboard'][key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    validate_settings(bad)
        for patch in ({'descent_speed':1.01}, {'unknown':.1}, {'setpoint_topic':'/bad topic'}, {'setpoint_topic':None}):
            bad = copy.deepcopy(settings); bad['landing_offboard'].update(patch)
            with self.assertRaises(ValueError): validate_settings(bad)
        for block in ([], None, 'bad'):
            with self.assertRaises(ValueError): validate_settings(dict(settings, landing_offboard=block))
        for clearance in (0, True, -.1, float('nan'), float('inf'), .04, .03, .30):
            with self.assertRaises(ValueError): landing_control_parameters(settings, -.13, clearance)
        for ground in (True, float('nan'), float('inf')):
            with self.assertRaises(ValueError): landing_control_parameters(settings, ground, .22)
        for capture in (.37, .3, float('nan'), True, None):
            with self.assertRaises(ValueError): landing_control_parameters(dict(settings, landing_capture_agl=capture), -.13, .22)

    def test_overrides_reach_controller_and_adapter_without_changing_profile_default(self):
        settings = dict(self.settings(TRIAL_FILES[0]), landing_descent_mode=4, landing_handoff_mode='OFFBOARD',
                        landing_offboard=dict(descent_speed=.35, touchdown_overtravel=.03,
                            ground_confirm_duration=.4, state_max_age=1.5, disarm_retry=.8,
                            setpoint_topic='/custom/touchdown'))
        _, docs = self.generate(trial_generate, settings)
        landing = docs['control']['external_landing']
        self.assertEqual(landing['descent']['speed_mps'], .20)
        self.assertEqual(landing['offboard_descent_speed'], .35)
        self.assertEqual(landing['offboard_touchdown_overtravel'], .03)
        self.assertEqual(landing['offboard_ground_confirm_duration'], .4)
        self.assertEqual(landing['offboard_state_max_age'], 1.5)
        self.assertEqual(landing['offboard_disarm_retry'], .8)
        self.assertEqual(docs['overrides']['/navigation_frame_adapter/landing_touchdown_setpoint_topic'], '/custom/touchdown')

    def test_non_h_rejects_candidate_and_override_cannot_change_flight_constraints(self):
        non_h = yaml.safe_load((ROOT / 'deployment/board_trials_4x4/01_visual_interrupt/settings.yaml').read_text())
        with self.assertRaises(ValueError): apply_site_profile(non_h, self.patch)
        non_h['landing_offboard'] = {}
        with self.assertRaisesRegex(ValueError, 'visual H landing'): validate_settings(non_h)
        for patch in ({'release_min_agl':.3}, {'landing_posctl':{}}, {}, []):
            with self.assertRaises(ValueError): apply_landing_override(self.settings(TRIAL_FILES[0]), patch)
        configured = apply_landing_override(self.settings(FORMAL_FILES[0], True) if HAS_FORMAL
                                            else self.settings(TRIAL_FILES[0]), self.patch)
        with self.assertRaisesRegex(ValueError, 'positive local-Z'):
            self.generate(formal_generate if HAS_FORMAL else trial_generate, configured, -.09 if HAS_FORMAL else -.14)  # Existing release minimum guard.

    def test_trial_cli_check_explicit_opt_in(self):
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path[:3]))
        script = TRIAL / 'scripts/run_trial.py'
        with tempfile.TemporaryDirectory() as tmp:
            command = [sys.executable, str(script), 'landing', 'preview', '--root', str(ROOT),
                       '--landing-config', str(PATCH), '--check-config']
            result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('CONFIG_VALID; no ROS nodes started', result.stdout)
            # The explicit switches also accept 4/high without a whole candidate file.
            command = [sys.executable, str(script), 'landing', 'preview', '--root', str(ROOT),
                       '--landing-descent-mode', '4', '--landing-handoff-mode', 'OFFBOARD',
                       '--landing-alignment-mode', 'high', '--check-config']
            result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for option in ('--landing-descent-mode', '--landing-handoff-mode'):
                invalid = command.copy(); index = invalid.index(option); del invalid[index:index + 2]
                result = subprocess.run(invalid, env=env, text=True, capture_output=True, timeout=30)
                self.assertNotEqual(result.returncode, 0)

    def test_both_light_bag_lists_record_default_and_custom_touchdown_input(self):
        from trial_bag import topics_for as trial_topics
        bag_implementations = [trial_topics]
        if HAS_FORMAL:
            from uav_mission.hardware_bag import topics_for as formal_topics
            bag_implementations.append(formal_topics)
        for topics_for in bag_implementations:
            for configured in ({}, {'landing_offboard':{'setpoint_topic':'/custom/touchdown'}}):
                settings = dict(configured, competition_recording=True, record_map_clouds=False)
                topics = topics_for(settings)
                expected = configured.get('landing_offboard', {}).get('setpoint_topic', DEFAULT_TOPIC)
                self.assertEqual(topics[topics.index('/navigation/landing_setpoint_vertical') + 1], expected)
                self.assertIn('/mavros/setpoint_raw/local', topics)
                self.assertEqual(topics.count(expected), 1)
                self.assertNotIn('/livox/lidar', topics)
                self.assertNotIn('/freedom/static_pointcloud', topics)

    @unittest.skipUnless(HAS_FORMAL, 'H branch does not package formal competition entry')
    def test_competition_cli_generates_only_with_valid_field_and_reference(self):
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path[:3]))
        with tempfile.TemporaryDirectory() as tmp:
            field = Path(tmp) / 'field.yaml'
            field.write_text(yaml.safe_dump(self.settings(FORMAL_FILES[0], True)))
            command = [sys.executable, str(PKG / 'scripts/competition_supervisor.py'), 'preview',
                       '--root', str(ROOT), '--site-config', str(field), '--landing-config', str(PATCH),
                       '--check-config', '--output-dir', str(Path(tmp) / 'out'), '--fc-reference', '0', '0', '.09']
            result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIs(json.loads(result.stdout)['ros_started'], False)
            landing = yaml.safe_load((Path(tmp) / 'out/control.yaml').read_text())['external_landing']
            self.assertEqual(landing['offboard_descent_speed'], .30)


if __name__ == '__main__':
    unittest.main()
