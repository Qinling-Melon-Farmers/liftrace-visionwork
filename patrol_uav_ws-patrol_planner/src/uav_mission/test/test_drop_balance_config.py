"""Verify generated release geometry and height agree across real entry points."""
import copy, importlib.util, sys, tempfile, unittest
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[4]
MISSION=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path.insert(0,str(MISSION/'src'))
class DropBalanceConfigTests(unittest.TestCase):
    def check_generated(self,fn,settings,rig):
        original=copy.deepcopy(settings)
        for fc_z in (-.05,.12):
            with tempfile.TemporaryDirectory() as out:
                ref=fn(ROOT,out,settings,(.02,-.01,fc_z),rig)
                c=yaml.safe_load((Path(out)/'control.yaml').read_text())['drop_system']
                o=yaml.safe_load((Path(out)/'overrides.yaml').read_text())
            self.assertEqual(c['settle']['xy_tolerance_m'],.10)
            self.assertEqual(c['settle']['red_cross_xy_tolerance_m'],.06)
            self.assertEqual(c['pre_descent']['xy_tolerance_m'],.06)
            self.assertEqual(c['pre_descent']['red_cross_xy_tolerance_m'],.04)
            if 'release_min_agl' not in settings:
                self.skipTest('Historical research profile has no independent release window; not a current flight entry')
            g=ref['ground_z']
            self.assertAlmostEqual(c['release_setpoint_height']-g,settings['drop_agl'])
            low=settings.get('release_min_agl',settings['drop_agl'])
            high=settings.get('release_max_agl',settings['drop_agl']+.10)
            self.assertAlmostEqual(c['release_min_height']-g,low)
            self.assertAlmostEqual(c['height_threshold']-g,high)
            # Every controller-accepted height must also fit the original arbiter.
            self.assertLessEqual(o['/release_permission_arbiter/min_release_altitude'],c['release_min_height'])
            self.assertGreaterEqual(o['/release_permission_arbiter/max_release_altitude'],c['height_threshold'])
        self.assertEqual(original,settings)
    def test_formal_profile(self):
        field=ROOT/'deployment/competition/field.example.yaml'
        if not field.exists(): self.skipTest('No formal entry in this research branch')
        from uav_mission.competition_config import generate
        s=yaml.safe_load(field.read_text());s.setdefault('motion_optimization',{})['enabled']=False
        s.update(site_confirmed=True,corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],landing_xy=[8.5,-4.2])
        rig=yaml.safe_load((MISSION/'config/competition/known_rig.yaml').read_text())
        self.check_generated(generate,s,rig)
    def test_trial_profiles(self):
        base=ROOT/'deployment/board_trials_4x4'
        path=base/'common/uav_board_trials/scripts'
        if not path.exists():self.skipTest('No trial entry in this navigation branch')
        sys.path.insert(0,str(path))
        from trial_config import generate
        rig=yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for group in ('05_low_multi','08_full_mission'):
            with self.subTest(group=group):
                s=yaml.safe_load((base/group/'settings.yaml').read_text())
                s.setdefault('motion_optimization',{})['enabled']=False
                if group=='08_full_mission':
                    s.update(corridor_waypoints=[dict(x=.6,y=0.,agl=.9),dict(x=1.5,y=.4,agl=.9)],landing_xy=[2.5,0.])
                self.check_generated(generate,s,rig)
if __name__=='__main__':unittest.main()
