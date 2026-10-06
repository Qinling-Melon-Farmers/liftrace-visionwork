from pathlib import Path
import copy,tempfile,unittest,yaml
from trial_config import generate,TRIAL_FOLDERS,flight_geometry
from uav_mission.motion_optimization import MotionOptimization
from uav_mission.corridor_speed import CorridorSpeedConfig
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
class MotionProfiles(unittest.TestCase):
    def config(self,trial):
        s=yaml.safe_load((BASE/TRIAL_FOLDERS[trial]/'settings.yaml').read_text())
        if trial in ('corridor_landing','full_mission'):
            s.update(corridor_waypoints=[dict(x=.6,y=0,agl=.9),dict(x=1.2,y=0,agl=.9),dict(x=1.8,y=0,agl=.9),dict(x=2.4,y=0,agl=.9)],landing_xy=[2.8,0])
        return s
    def generate(self,s):
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        with tempfile.TemporaryDirectory() as out:
            generate(ROOT,out,s,(0,0,0),rig)
            return [yaml.safe_load((Path(out)/f).read_text()) for f in ('runtime.yaml','control.yaml','overrides.yaml')]
    def test_all_nine_profiles_keep_release_and_height_limits(self):
        for trial in TRIAL_FOLDERS:
            s=self.config(trial)
            self.assertTrue(s['motion_optimization']['enabled'])
            s['motion_optimization']={**s['motion_optimization'],'enabled':False};before=self.generate(s)
            s['motion_optimization']={'enabled':True};after=self.generate(s)
            self.assertTrue(after[0]['motion_optimization']['enabled'])
            for key in ('align_height','drop_system','external_landing'):
                self.assertEqual(before[1][key],after[1][key],(trial,key))
            for key in ('/external_planner_max_command_z','/navigation/planner_bridge/execution/max_goal_z','/release_permission_arbiter/min_release_altitude','/release_permission_arbiter/max_release_altitude'):
                self.assertEqual(before[2][key],after[2][key],(trial,key))
            self.assertEqual(after[1]['uav_vision']['recovery_height'],after[2]['/navigation/planner_bridge/target/recovery_height'])
            self.assertEqual(before[1]['uav_vision']['standard_recovery_setpoint_height'],after[1]['uav_vision']['standard_recovery_setpoint_height'])
    def test_three_patterns_remain_inside_site_and_fixed_heading(self):
        for trial in ('high_view','high_priority','memory_only','full_mission','high_speed_capture'):
            s=self.config(trial);original=flight_geometry(s);xs=[p[0] for p in original['survey_xy']];ys=[p[1] for p in original['survey_xy']]
            for pattern,n in (('rectangle',5),('snake2',4),('snake3',6)):
                s['survey_pattern']=pattern;r,c,o=self.generate(s);points=r['high_view_probe']['config']['survey_xy']
                self.assertEqual(len(points),n)
                self.assertTrue(all(min(xs)<=x<=max(xs) and min(ys)<=y<=max(ys) for x,y in points))
                self.assertEqual(c['waypoints'][0]['yaw'],0)
    def test_low_module_rejects_survey_pattern(self):
        s=self.config('low_multi');s['survey_pattern']='snake3'
        with self.assertRaises(ValueError):self.generate(s)
    def test_corridor_merges_only_measured_low_prefix_and_preserves_h(self):
        s=self.config('full_mission');before=self.generate(s)[0]['mission']['post_delivery_route']
        s.update(motion_optimization={'enabled':True},corridor_geometry=dict(wall_axis=0,wall_coordinates=[1.4,2.1],entry_waypoints=1))
        r,c,o=self.generate(s);m=r['mission'];self.assertEqual(m['post_delivery_route'],[before[0],before[3],*before[4:]])
        CorridorSpeedConfig(**r['corridor_speed_schedule'])
        stages=m['post_delivery_parameter_stages'];self.assertEqual(stages[-2]['after_completed_waypoints'],1)
        self.assertEqual(stages[-1]['after_completed_waypoints'],2)
        self.assertLess(stages[-2]['parameters']['/external_planner_max_command_z'],stages[-1]['parameters']['/external_planner_max_command_z'])
    def test_corridor_without_measured_planes_keeps_all_points(self):
        s=self.config('full_mission');before=self.generate(s);s['motion_optimization']={'enabled':True};after=self.generate(s)
        self.assertEqual(before[0]['mission']['post_delivery_route'],after[0]['mission']['post_delivery_route'])
    def test_corridor_rejects_high_entry_and_invalid_braking(self):
        s=self.config('full_mission');s.update(motion_optimization={'enabled':True},corridor_geometry=dict(wall_axis=0,wall_coordinates=[1.4,2.1],entry_waypoints=1))
        s['corridor_waypoints'][2]['agl']=1.4
        with self.assertRaises(ValueError):self.generate(s)
        s=self.config('high_priority');s['motion_optimization']={'enabled':True,'braking_speed_mps':.1}
        with self.assertRaises(ValueError):self.generate(s)
if __name__=='__main__':unittest.main()
