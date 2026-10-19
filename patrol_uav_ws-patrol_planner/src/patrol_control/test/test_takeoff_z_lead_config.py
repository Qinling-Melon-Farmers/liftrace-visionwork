"""Generate candidate takeoff settings offline; do not start flight processes."""
from pathlib import Path
import copy, math, sys, tempfile, unittest
import yaml
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'deployment/board_trials_4x4/common/uav_board_trials/scripts'),str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),str(ROOT/'vision_ws/src/uav_high_view/src')]
from trial_config import generate as trial_generate, validate_settings
from uav_mission.competition_config import generate as formal_generate, validate
BASE=ROOT/'deployment/board_trials_4x4'
RIG=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())

def trial_settings(file):
    s=yaml.safe_load(file.read_text())
    if s.get('trial_kind') in ('corridor_landing','full_mission'):
        s.update(corridor_waypoints=[dict(x=.6,y=0.,agl=1.),dict(x=1.5,y=.4,agl=1.)],landing_xy=[2.5,0.])
    return s

def formal_settings():
    s=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
    s.update(site_confirmed=True,corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],landing_xy=[8.5,-4.2])
    return s

def generated(fn,s):
    with tempfile.TemporaryDirectory() as path:
        fn(ROOT,path,s,(0.,0.,0.),RIG)
        return [yaml.safe_load((Path(path)/(name+'.yaml')).read_text()) for name in ('runtime','control','overrides')]

class TakeoffSettings(unittest.TestCase):
    def test_all_nine_groups_default_to_legacy(self):
        files=sorted(BASE.glob('*/settings.yaml'));self.assertEqual(len(files),9)
        for file in files:
            with self.subTest(group=file.parent.name):
                _,c,o=generated(trial_generate,trial_settings(file))
                expected=0.
                self.assertEqual(c['takeoff_z_lead_m'],expected)
                self.assertEqual(o['/takeoff_z_lead_m'],expected)

    def test_trial_settings_override_and_missing_08_preserve_legacy(self):
        s=trial_settings(BASE/'08_full_mission/settings.yaml')
        before=generated(trial_generate,s)
        for lead in (0.,.4,.6):
            with self.subTest(lead=lead):
                changed=copy.deepcopy(s);changed['takeoff_z_lead_m']=lead
                after=generated(trial_generate,changed)
                self.assertEqual(after[0],before[0])
                self.assertEqual(after[1]['takeoff_z_lead_m'],lead)
                self.assertEqual(after[2]['/takeoff_z_lead_m'],lead)
                after[1]['takeoff_z_lead_m']=before[1]['takeoff_z_lead_m'];after[2]['/takeoff_z_lead_m']=before[2]['/takeoff_z_lead_m']
                self.assertEqual(after,before)
        del s['takeoff_z_lead_m']
        self.assertEqual(generated(trial_generate,s)[1]['takeoff_z_lead_m'],0.)

    def test_formal_default_and_settings_override(self):
        s=formal_settings();before=generated(formal_generate,s)
        self.assertEqual(before[1]['takeoff_z_lead_m'],0.)
        self.assertEqual(before[2]['/takeoff_z_lead_m'],0.)
        for lead in (0.,.4,.6):
            after=generated(formal_generate,{**s,'takeoff_z_lead_m':lead})
            self.assertEqual(after[0],before[0])
            self.assertEqual(after[1]['takeoff_z_lead_m'],lead)
            self.assertEqual(after[2]['/takeoff_z_lead_m'],lead)
            after[1]['takeoff_z_lead_m']=before[1]['takeoff_z_lead_m'];after[2]['/takeoff_z_lead_m']=before[2]['/takeoff_z_lead_m']
            self.assertEqual(after,before)

    def test_08_final_launch_and_both_profiles(self):
        import roslaunch, rospkg
        from unittest.mock import patch
        from trial_speed_profiles import apply_speed_profile
        scripts=BASE/'common/uav_board_trials/scripts'
        rospack=rospkg.RosPack(ros_paths=[str(BASE/'common'),str(ROOT/'vision_ws/src'),str(ROOT/'patrol_uav_ws-patrol_planner/src'),'/opt/ros/noetic/share'])
        for profile in ('limited','competition'):
            baseline=None
            for lead in (None,.6,0.,.4):
                with self.subTest(profile=profile,lead=lead),tempfile.TemporaryDirectory() as path:
                    s=trial_settings(BASE/'08_full_mission/settings.yaml')
                    apply_speed_profile(ROOT,s,'full_mission',profile)
                    if lead is not None:s['takeoff_z_lead_m']=lead
                    reference=trial_generate(ROOT,path,s,(0.,0.,0.),RIG)
                    args=['enable_control_output:=true','simulation:=false','mode:=high_view_full','model_path:=/offline-test.rknn','generated_dir:='+path,'ground_z:='+str(reference['ground_z']),'low_z:='+str(reference['low_z']),'cruise_speed:='+str(s['cruise_speed']),'cruise_acceleration:='+str(s['cruise_acceleration'])]
                    config=roslaunch.config.ROSLaunchConfig()
                    with patch.object(roslaunch.substitution_args,'_rospack',rospack):
                        roslaunch.xmlloader.XmlLoader().load(str(scripts.parent/'launch/application.launch'),config,argv=args,verbose=False)
                    values={key:param.value for key,param in config.params.items()}
                    self.assertEqual(values['/takeoff_z_lead_m'],0. if lead is None else lead)
                    self.assertEqual(values['/px4_max_distance'],.4 if profile=='competition' else .25)
                    if baseline is None:baseline=values
                    else:
                        changed={key for key in baseline.keys() | values.keys() if baseline.get(key)!=values.get(key)}
                        # Generated temporary paths are expected to differ between loads.
                        changed={key for key in changed if not (isinstance(values.get(key),str) and path in values[key])}
                        self.assertEqual(changed,{'/takeoff_z_lead_m'} if lead else set())

    def test_invalid_settings_are_rejected_before_launch(self):
        for lead in (-1.,.39,.600001,1.,float('nan'),float('inf'),True,'0.6',None):
            for check,s in ((validate_settings,trial_settings(BASE/'08_full_mission/settings.yaml')),(validate,formal_settings())):
                with self.subTest(lead=lead,check=check.__name__):
                    with self.assertRaisesRegex(ValueError,'takeoff_z_lead_m'):
                        check({**s,'takeoff_z_lead_m':lead})

if __name__=='__main__':unittest.main(verbosity=2)
