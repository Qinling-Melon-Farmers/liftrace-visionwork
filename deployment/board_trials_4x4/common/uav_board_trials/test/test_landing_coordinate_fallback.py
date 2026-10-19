"""Offline coordinate fallback validation, inheritance and generated control contracts."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[5]
TRIAL = ROOT / 'deployment/board_trials_4x4/common/uav_board_trials'
PKG = ROOT / 'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path[:0] = [str(TRIAL/'scripts'), str(PKG/'src'), str(ROOT/'vision_ws/src/uav_high_view/src')]
from uav_mission.landing_coordinate_fallback_config import DEFAULTS, landing_coordinate_fallback_parameters as parameters
from uav_mission.landing_offboard_config import apply_landing_override
from trial_config import apply_site_profile, generate, load_trial_settings, validate_settings
import prepare_simulation

BASE=ROOT/'deployment/board_trials_4x4'

class CoordinateFallback(unittest.TestCase):
    def settings(self,folder='08_full_mission'):
        s=load_trial_settings(ROOT,folder)
        if folder!='03_h_landing':
            name='site_lawnmower_20261016.yaml' if folder=='10_low_coverage_full' else 'site_waypoint_map_20261016.yaml'
            apply_site_profile(s,yaml.safe_load((BASE/folder/name).read_text()))
        return s

    def generated(self,s):
        rig=yaml.safe_load((TRIAL/'config/known_rig.yaml').read_text())
        with tempfile.TemporaryDirectory() as out:
            generate(ROOT,out,s,(0.,0.,0.),rig)
            return {name:yaml.safe_load((Path(out)/(name+'.yaml')).read_text())
                    for name in ('control','runtime','overrides')}

    def test_default_disabled_and_no_shared_mutation(self):
        a=parameters({});self.assertEqual(a,DEFAULTS);self.assertFalse(a['enabled'])
        self.assertEqual(a['wait_sec'],15.0)  # 兼容库默认不跟随显式现场10秒设置。
        a['enabled']=True
        self.assertFalse(parameters({})['enabled'])
        self.assertFalse(parameters({'landing_handoff_mode':'AUTO.LAND'})['enabled'])

    def test_bad_block_types_unknown_fields_and_boolean(self):
        for block in (None,False,[],{'unknown':1},{'enabled':1},{'enabled':'true'}):
            with self.subTest(block=block),self.assertRaises(ValueError):
                parameters({'landing_coordinate_fallback':block})

    def test_all_numeric_fields_reject_nonfinite_nonpositive_and_bool(self):
        for key in set(DEFAULTS)-{'enabled'}:
            for value in (0,-.1,float('nan'),float('inf'),-float('inf'),True,'0.1',None):
                with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                    parameters({'landing_coordinate_fallback':{key:value}})

    def test_wait_upper_bound_and_arrival_descent_order(self):
        self.assertEqual(parameters({'landing_coordinate_fallback':{'wait_sec':60}})['wait_sec'],60.)
        for block in ({'wait_sec':60.0001},{'arrival_xy_tolerance_m':.079}):
            with self.assertRaises(ValueError):parameters({'landing_coordinate_fallback':block})
        self.assertEqual(parameters({'landing_coordinate_fallback':{'arrival_xy_tolerance_m':.08}})['arrival_xy_tolerance_m'],.08)

    def test_enabled_requires_external_posctl_and_finite_measured_xy(self):
        valid=dict(landing_handoff_mode='POSCTL',landing_xy=[7.51,-3.85],landing_coordinate_fallback={'enabled':True})
        self.assertTrue(parameters(valid,external_land=True)['enabled'])
        with self.assertRaises(ValueError):parameters(valid)
        for handoff in ('AUTO.LAND','OFFBOARD',None):
            with self.assertRaises(ValueError):parameters(dict(valid,landing_handoff_mode=handoff),external_land=True)
        for xy in (None,[],[1],[1,2,3],[True,0],[float('nan'),0],[0,float('inf')],['1',0]):
            with self.subTest(xy=xy),self.assertRaises(ValueError):
                parameters(dict(valid,landing_xy=xy),external_land=True)

    def test_03_04_08_10_inheritance_and_real_generation_preserves_h_policy(self):
        for folder in ('03_h_landing','04_corridor_landing','08_full_mission','10_low_coverage_full'):
            with self.subTest(folder=folder):
                s=self.settings(folder);original=copy.deepcopy(s);validate_settings(s)
                expected=dict(DEFAULTS,enabled=True,wait_sec=15.0 if folder=='04_corridor_landing' else 10.0)
                self.assertEqual(s['landing_coordinate_fallback'],expected)
                active=self.generated(s)
                self.assertEqual(active['control']['external_landing']['coordinate_fallback'],expected)
                off=copy.deepcopy(s);off['landing_coordinate_fallback']['enabled']=False
                baseline=self.generated(off)
                active['control']['external_landing']['coordinate_fallback']['enabled']=False
                self.assertEqual(active,baseline)
                self.assertEqual(s,original)

    def test_explicit_03_matches_08_and_generates_current_posctl_policy(self):
        standalone=yaml.safe_load((BASE/'03_h_landing/settings.yaml').read_text())
        full=yaml.safe_load((BASE/'08_full_mission/settings.yaml').read_text())
        policy=lambda s:{k:v for k,v in s.items() if k.startswith('landing_') and k!='landing_xy'}
        self.assertEqual(policy(standalone),policy(full))
        for folder in ('03_h_landing','08_full_mission'):
            with self.subTest(folder=folder):
                s=self.settings(folder)
                self.assertEqual((s['landing_alignment_mode'],s['landing_descent_mode'],
                                  s['landing_descent_speed_mps']),('vh',2,1.0))
                self.assertEqual((s['landing_posctl']['target_agl'],s['landing_posctl']['trigger_agl']),(.35,.50))
                control=self.generated(s)['control']['external_landing']
                self.assertEqual(control['alignment_mode'],'vh')
                self.assertEqual(control['descent']['mode'],2)
                self.assertEqual(control['descent']['speed_mps'],1.0)
                self.assertEqual(control['handoff_mode'],'POSCTL')
                self.assertAlmostEqual(control['auto_land_height']+.22,.50)
                self.assertEqual(control['coordinate_fallback']['wait_sec'],10.0)

    def test_site_override_and_posctl_descent_modes_unchanged(self):
        s=self.settings()
        apply_site_profile(s,{'landing_coordinate_fallback':{'enabled':True,'wait_sec':12}})
        for mode in (1,2,3):
            s['landing_descent_mode']=mode
            control=self.generated(s)['control']['external_landing']
            self.assertEqual(control['coordinate_fallback']['wait_sec'],12)
            self.assertEqual(control['descent']['mode'],mode)
            self.assertEqual(control['handoff_mode'],'POSCTL')
        apply_site_profile(s,{'landing_coordinate_fallback':{'enabled':False}})
        self.assertFalse(self.generated(s)['control']['external_landing']['coordinate_fallback']['enabled'])

    def test_other_handoffs_require_explicit_disable_preserve_mode4(self):
        s=self.settings()
        for handoff,mode in [('AUTO.LAND',1),('OFFBOARD',4)]:
            other=copy.deepcopy(s);other.update(landing_handoff_mode=handoff,landing_descent_mode=mode,landing_alignment_mode='non_vh')
            with self.assertRaisesRegex(ValueError,'coordinate_fallback'):validate_settings(other)
            other=apply_landing_override(other,{'landing_coordinate_fallback':{'enabled':False}})
            landing=self.generated(other)['control']['external_landing']
            self.assertEqual(landing['handoff_mode'],handoff)
            self.assertEqual(landing['descent']['mode'],mode)
            self.assertFalse(landing['coordinate_fallback']['enabled'])

    def test_non_h_path_never_enables_fallback(self):
        s=load_trial_settings(ROOT,'01_visual_interrupt')
        self.assertFalse(self.generated(s)['control']['external_landing']['coordinate_fallback']['enabled'])
        s['landing_coordinate_fallback']={'enabled':True}
        with self.assertRaises(ValueError):validate_settings(s)

    def test_simulation_entry_strips_field_enable_before_generator(self):
        class StopBeforeAssets(Exception):pass
        for name in ('landing','corridor_landing','full_mission'):
            with self.subTest(name=name),tempfile.TemporaryDirectory() as out:
                with patch.object(sys,'argv',['prepare_simulation.py',name,out]),patch.object(prepare_simulation,'generate',side_effect=StopBeforeAssets) as gen:
                    with self.assertRaises(StopBeforeAssets):prepare_simulation.main()
                self.assertFalse(gen.call_args.args[2]['landing_coordinate_fallback']['enabled'])

    def test_real_cli_offline_03_04_08_and_both_10_patterns(self):
        cases=[('landing',[]),('corridor_landing',[]),('full_mission',[])]
        cases += [('low_coverage_full',['--coverage-pattern',p]) for p in ('lawnmower','nested_rectangles')]
        env=dict(os.environ,PYTHONPATH=os.pathsep.join(sys.path[:3]))
        for trial,options in cases:
            with self.subTest(trial=trial,options=options),tempfile.TemporaryDirectory() as out:
                command=[sys.executable,str(TRIAL/'scripts/run_trial.py'),trial,'preview','--root',str(ROOT),
                         '--check-config','--generate-config',out,'--reference-fc','0','0','0',*options]
                result=subprocess.run(command,env=env,text=True,capture_output=True,timeout=30)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                data=json.loads(result.stdout.splitlines()[-1])
                self.assertTrue(data['settings']['landing_coordinate_fallback']['enabled'])
                control=yaml.safe_load((Path(out)/'control.yaml').read_text())
                expected=dict(DEFAULTS,enabled=True,wait_sec=15.0 if trial=='corridor_landing' else 10.0)
                self.assertEqual(data['settings']['landing_coordinate_fallback'],expected)
                self.assertEqual(control['external_landing']['coordinate_fallback'],expected)

if __name__=='__main__':unittest.main()
