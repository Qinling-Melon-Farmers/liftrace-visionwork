"""Generate low AGL delivery configurations with negative local Z."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[4]
MISSION=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path.insert(0,str(MISSION/'src'))


class LowDropHeightConfig(unittest.TestCase):
    def test_current_08_site_retains_low_drop_window_mode_two_rates_and_four_waypoints(self):
        base = ROOT/'deployment/board_trials_4x4'
        sys.path.insert(0, str(base/'common/uav_board_trials/scripts'))
        from trial_config import apply_site_profile, generate
        settings = yaml.safe_load((base/'08_full_mission/settings.yaml').read_text())
        site = yaml.safe_load((base/'08_full_mission/site_20261007_221730.yaml').read_text())
        apply_site_profile(settings, site)
        self.assertEqual([settings[k] for k in ('drop_agl','release_min_agl','release_max_agl')], [.15,.10,.43])
        self.assertEqual([settings[k] for k in ('drop_descent_mode','landing_descent_mode')], [2,2])
        self.assertEqual([settings[k] for k in ('drop_descent_speed_mps','landing_descent_speed_mps')], [1.,1.])
        self.assertEqual([p['y'] for p in settings['corridor_waypoints']], [1.9,1.9,1.,-3.45])
        rig = yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
        with tempfile.TemporaryDirectory() as out:
            ref = generate(ROOT, out, settings, (.02,-.01,.12), rig)
            control = yaml.safe_load((Path(out)/'control.yaml').read_text())
        drop = control['drop_system']
        for key, agl in [('release_setpoint_height',.15),('release_min_height',.10),('height_threshold',.43)]:
            self.assertAlmostEqual(drop[key]-ref['ground_z'], agl)
        self.assertEqual(drop['descent']['speed_mps'], 1.)
        self.assertEqual(control['external_landing']['descent']['speed_mps'], 1.)

    def entries(self):
        from uav_mission.competition_config import generate
        settings=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        settings['motion_optimization']['enabled']=False
        settings.update(site_confirmed=True,landing_xy=[8.5,-4.2],corridor_waypoints=[
            dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)])
        yield 'formal',generate,settings,yaml.safe_load((MISSION/'config/competition/known_rig.yaml').read_text())
        base=ROOT/'deployment/board_trials_4x4'
        sys.path.insert(0,str(base/'common/uav_board_trials/scripts'))
        from trial_config import generate
        settings=yaml.safe_load((base/'08_full_mission/settings.yaml').read_text())
        settings['motion_optimization']['enabled']=False
        settings.update(landing_xy=[2.5,0.],corridor_waypoints=[dict(x=.6,y=0.,agl=.9),dict(x=1.5,y=.4,agl=.9)])
        yield 'trial',generate,settings,yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())

    def test_requested_window_and_unchanged_other_control_fields(self):
        for name,generate,settings,rig in self.entries():
            for fc_z in (0.,.004678211873397231,.12):
                with self.subTest(entry=name,fc_z=fc_z):
                    original=copy.deepcopy(settings);controls=[]
                    for lower,target,upper in ((.35,.40,.45),(.15,.20,.40),(.10,.15,.43)):
                        selected=dict(settings,release_min_agl=lower,drop_agl=target,release_max_agl=upper)
                        with tempfile.TemporaryDirectory() as out:
                            ref=generate(ROOT,out,selected,(.02,-.01,fc_z),rig)
                            control=yaml.safe_load((Path(out)/'control.yaml').read_text())
                            overrides=yaml.safe_load((Path(out)/'overrides.yaml').read_text())
                        drop=control.pop('drop_system');ground=ref['ground_z']
                        for field,value in (('release_min_height',lower),('release_setpoint_height',target),('height_threshold',upper)):
                            self.assertAlmostEqual(drop[field]-ground,value)
                        self.assertAlmostEqual(overrides['/release_permission_arbiter/min_release_altitude']-ground,lower-.08)
                        self.assertAlmostEqual(overrides['/release_permission_arbiter/max_release_altitude']-ground,upper+.02)
                        controls.append(control)
                    self.assertTrue(all(c==controls[0] for c in controls[1:]));self.assertEqual(settings,original)

    def test_invalid_target_order_and_nonfinite_values_still_rejected(self):
        for name,generate,settings,rig in self.entries():
            for lower,target,upper in ((.10,.149,.43),(.21,.2,.4),(.15,.2,.19),(.15,float('nan'),.4)):
                with self.subTest(entry=name,values=(lower,target,upper)),tempfile.TemporaryDirectory() as out:
                    with self.assertRaises(ValueError):
                        generate(ROOT,out,dict(settings,release_min_agl=lower,drop_agl=target,release_max_agl=upper),(.02,-.01,0.),rig)


if __name__=='__main__':unittest.main()
