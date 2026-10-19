"""Offline checks: standalone H uses 08 policy, retaining its own mission/geometry."""
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
IMPORT_PATHS = [str(PKG/'src'), str(ROOT/'vision_ws/src/uav_high_view/src'), str(TRIAL/'scripts')]
sys.path[:0] = IMPORT_PATHS
from trial_config import load_trial_settings, apply_site_profile, generate


class HLandingSourceTests(unittest.TestCase):
    def test_generated_h_policy_matches_08_without_delivery(self):
        h = load_trial_settings(ROOT, '03_h_landing')
        full = load_trial_settings(ROOT, '08_full_mission')
        for key, value in full.items():
            if key.startswith('landing_') and key != 'landing_xy': self.assertEqual(h[key], value)
        self.assertEqual(h['mode'], 'landing')
        self.assertEqual(h['landing_xy'], [2., 0.])
        full.update(corridor_waypoints=[dict(x=1., y=0., agl=.9), dict(x=2., y=0., agl=.9)], landing_xy=[2., 0.])
        # Isolate H from unrelated delivery local-Z bounds and wall geometry.
        full.update(drop_agl=.40, release_min_agl=.37, release_max_agl=.45)
        rig = yaml.safe_load((TRIAL/'config/known_rig.yaml').read_text())
        controls = []
        for settings in (h, full):
            settings['motion_optimization'] = dict(enabled=False)
            with tempfile.TemporaryDirectory(prefix='h_policy_') as folder:
                generate(ROOT, folder, settings, (0., 0., .09), rig)
                controls.append(yaml.safe_load((Path(folder)/'control.yaml').read_text()))
        self.assertEqual(controls[0]['external_landing'], controls[1]['external_landing'])
        self.assertEqual(controls[0]['land_height'], controls[1]['land_height'])
        self.assertFalse(controls[0]['drop_system']['enable_drop'])

    def test_future_source_updates_and_optional_block_removal(self):
        h = dict(mode='landing', landing_xy=[2., 0.], drop_agl=.6, landing_offboard=dict(descent_speed=.9))
        full = copy.deepcopy(load_trial_settings(ROOT, '08_full_mission'))
        full.update(landing_descent_speed_mps=.42, landing_capture_agl=1.3)
        full.pop('landing_offboard', None)
        with tempfile.TemporaryDirectory(prefix='h_source_') as folder:
            base = Path(folder)/'deployment/board_trials_4x4'
            for name, value in (('03_h_landing', h), ('08_full_mission', full)):
                (base/name).mkdir(parents=True)
                (base/name/'settings.yaml').write_text(yaml.safe_dump(value))
            resolved = load_trial_settings(folder, '03_h_landing')
        self.assertEqual(resolved['landing_descent_speed_mps'], .42)
        self.assertEqual(resolved['landing_capture_agl'], 1.3)
        self.assertNotIn('landing_offboard', resolved)
        self.assertEqual(resolved['landing_xy'], [2., 0.])
        self.assertEqual(resolved['drop_agl'], .6)

    def test_other_modules_and_site_geometry_remain_independent(self):
        for name in ('04_corridor_landing', '08_full_mission'):
            self.assertEqual(load_trial_settings(ROOT, name), yaml.safe_load(
                (ROOT/'deployment/board_trials_4x4'/name/'settings.yaml').read_text()))
        h = load_trial_settings(ROOT, '03_h_landing')
        apply_site_profile(h, dict(landing_xy=[2.5, .1]))
        self.assertEqual(h['landing_xy'], [2.5, .1])
        self.assertEqual(h['landing_descent_speed_mps'], load_trial_settings(ROOT, '08_full_mission')['landing_descent_speed_mps'])

    def test_check_config_entry_and_explicit_override(self):
        env = dict(os.environ, PYTHONUTF8='1', PYTHONPATH=os.pathsep.join(IMPORT_PATHS))
        cmd = [sys.executable, str(TRIAL/'scripts/run_trial.py'), 'landing', 'preview',
               '--root', str(ROOT), '--check-config', '--site-config',
               str(ROOT/'deployment/site_20260928/h_landing_test_area.yaml')]
        full = load_trial_settings(ROOT, '08_full_mission')
        for extra, speed in (([], full['landing_descent_speed_mps']), (['--landing-descent-speed-mps', '.25'], .25)):
            run = subprocess.run(cmd+extra, env=env, capture_output=True, text=True, check=True, timeout=30)
            result = json.loads(run.stdout.strip().splitlines()[-1])['settings']
            self.assertEqual(result['landing_alignment_mode'], full['landing_alignment_mode'])
            self.assertEqual(result['landing_descent_mode'], full['landing_descent_mode'])
            self.assertEqual(result['landing_descent_speed_mps'], speed)
            self.assertIn('no ROS nodes started', run.stdout)


if __name__ == '__main__': unittest.main()
