"""Generate real 04/08 CLI YAML offline; no ROS nodes or actuator processes."""
from pathlib import Path
import copy, json, os, subprocess, sys, tempfile, unittest
import yaml

ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
SCRIPTS=BASE/'common/uav_board_trials/scripts'
sys.path[:0]=[str(SCRIPTS),str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),
             str(ROOT/'vision_ws/src/uav_high_view/src')]
from trial_config import TRIAL_FOLDERS, generate, apply_site_profile
from uav_mission.corridor_speed import CorridorSpeed, CorridorSpeedConfig
from uav_mission.execution_speed import FollowingSpeed

SITE=BASE/'08_full_mission/site_20261007_221730.yaml'
REFERENCE=(-.011402054224163294,-.006272925063967705,-.04349584877490997)

class CompetitionSpeedCLI(unittest.TestCase):
    def cli(self, trial, profile=None, site=True):
        env=os.environ.copy()
        env['PYTHONPATH']=os.pathsep.join(str(p) for p in (SCRIPTS,ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src',ROOT/'vision_ws/src/uav_high_view/src'))
        env['BOARD_PYTHON']=sys.executable
        with tempfile.TemporaryDirectory() as tmp:
            args=['bash',str(BASE/TRIAL_FOLDERS[trial]/'start.sh'),'preview','--check-config',
                  '--generate-config',tmp,'--reference-fc',*[str(v) for v in REFERENCE]]
            if site: args+=['--site-config',str(SITE)]
            if profile: args+=['--speed-profile',profile]
            run=subprocess.run(args,env=env,text=True,capture_output=True,timeout=40)
            self.assertEqual(run.returncode,0,run.stdout+'\n'+run.stderr)
            self.assertIn('CONFIG_VALID; no ROS nodes started',run.stdout)
            evidence=json.loads(next(s for s in run.stdout.splitlines() if s.startswith('{')))
            configs=[yaml.safe_load((Path(tmp)/(n+'.yaml')).read_text()) for n in ('runtime','control','overrides')]
            for key in ('/navigation_recovery/enabled','/traj_server/navigation_recovery/enabled',
                        '/fast_planner_node/navigation_recovery/enabled',
                        '/navigation/planner_bridge/navigation_recovery/enabled',
                        '/fast_planner_node/sdf_map/recovery_layers_enabled'):
                self.assertIs(configs[2][key],False,key)
            return evidence,*configs

    def assert_corridor(self,runtime):
        following=FollowingSpeed(**runtime['following_speed_profile'])
        self.assertEqual(following.corridor_lead_m,.4)
        schedule=CorridorSpeedConfig(**runtime['corridor_speed_schedule'])
        self.assertEqual((schedule.open_lead_m,schedule.door_lead_m),(.6,.4))
        # Shift measured scene by the real FC reference; select real stage behaviour.
        speed=CorridorSpeed(schedule)
        home=runtime['mission']['home_xy'];h=runtime['mission']['landing_xy']
        self.assertEqual(speed.select((home[0]+7.7,home[1]+1.2),h,0)[1],.4)
        self.assertEqual(speed.select((home[0]+7.7,home[1]+1.2),h,2)[1],.6)
        self.assertEqual(speed.select((home[0]+7.7,home[1]+.1),h,2)[1],.4)
        self.assertEqual(speed.select(h,h,8)[1],.4)

    def assert_h(self,control,transit,capture):
        ground=REFERENCE[2]-.22
        self.assertAlmostEqual(control['external_landing']['capture_height']-ground,capture)
        self.assertAlmostEqual(control['external_landing']['auto_land_height']-ground,.37)
        posctl=control['external_landing']['posctl']
        self.assertEqual((posctl['max_horizontal_speed_mps'],posctl['max_vertical_speed_mps'],
                          posctl['stable_duration_sec']),(.08,.10,.15))
        self.assertEqual(control['external_landing']['handoff_mode'],'POSCTL')

    def assert_drop_settle(self,control):
        self.assertEqual(control['drop_system']['settle'],dict(
            xy_tolerance_m=.04,height_tolerance_m=.05,
            max_horizontal_speed_mps=.08,max_vertical_speed_mps=.10,
            stable_duration_sec=.30,max_odom_age_sec=.20,
            max_sample_gap_sec=.20,min_samples=3))
        self.assertIs(control['drop_system']['compensated_alignment'],True)
        self.assertIs(control['uav_vision']['require_release_permission'],True)

    def test_08_two_profiles_keep_last_scene_and_today_h(self):
        results={p:self.cli('full_mission',p) for p in ('limited','competition')}
        site=yaml.safe_load(SITE.read_text())
        for profile,(e,r,c,o) in results.items():
            self.assertEqual(e['source'],'generated_runtime')
            self.assertEqual(e['settings']['flight_area'],site['flight_area'])
            self.assertEqual(e['settings']['corridor_waypoints'],site['corridor_waypoints'])
            self.assertEqual(e['settings']['corridor_geometry'],site['corridor_geometry'])
            self.assertEqual(e['settings']['landing_xy'],[7.7,-3.45])
            self.assertEqual(e['high_agl'],2.)
            self.assertEqual(e['settings']['landing_transit_agl'],1.)
            self.assertEqual(e['landing_capture_agl'],1.2)
            self.assertTrue(r['motion_optimization']['enabled'])
            self.assertTrue(r['high_view_full']['policy']['resume_survey_enabled'])
            self.assert_corridor(r);self.assert_h(c,1.,1.2)
            self.assert_drop_settle(c)
            self.assertAlmostEqual(c['drop_system']['release_setpoint_height']-(REFERENCE[2]-.22),.40)
            self.assertAlmostEqual(c['drop_system']['release_min_height']-(REFERENCE[2]-.22),.35)
            self.assertAlmostEqual(c['drop_system']['height_threshold']-(REFERENCE[2]-.22),.45)
            self.assertAlmostEqual(o['/release_permission_arbiter/min_release_altitude']-(REFERENCE[2]-.22),.27)
            self.assertAlmostEqual(o['/release_permission_arbiter/max_release_altitude']-(REFERENCE[2]-.22),.47)
            vel,acc=(1.2,1.) if profile=='competition' else (.5,.35)
            for ns,v,a in [('manager','max_vel','max_acc'),('search','max_vel','max_acc'),
                           ('optimization','max_vel','max_acc'),('bspline','limit_vel','limit_acc')]:
                self.assertEqual(o['/fast_planner_node/'+ns+'/'+v],vel)
                self.assertEqual(o['/fast_planner_node/'+ns+'/'+a],acc)
            self.assertEqual(e['planning'],dict(max_vel=vel,max_acc=acc))
            initial=(.4,.4,1.2) if profile=='competition' else (.25,.25,.75)
            self.assertEqual((c['px4_max_distance'],o['/traj_server/traj_server/target_dist'],
                              o['/external_planner_start_max_distance']),initial)
        _,r,c,o=results['competition']
        from dataclasses import asdict
        from uav_mission.motion_optimization import MotionOptimization
        self.assertEqual(r['motion_optimization'],asdict(MotionOptimization(enabled=True)))
        self.assertEqual(r['motion_optimization']['braking_speed_mps'],1.2)
        self.assertEqual(r['motion_optimization']['braking_accel_mps2'],.6)
        self.assertEqual(o['/navigation/planner_bridge/motion_optimization']['braking_accel_mps2'],.6)
        self.assertEqual(results['competition'][0]['initial_distances'],dict(controller_limit_m=.4,traj_target_dist_m=.4,planner_start_max_distance_m=1.2))
        f=FollowingSpeed(**r['following_speed_profile'])
        self.assertEqual((f.cruise_lead_m,f.precision_lead_m,f.boundary_lead_m),(1.,.4,.2))
        self.assertEqual(results['limited'][1]['mission']['post_delivery_route'],
                         r['mission']['post_delivery_route'])
        self.assertEqual(results['limited'][1]['high_view_probe']['config'],
                         r['high_view_probe']['config'])
        # Metadata/archived route is optional on a clean checkout, never a test prerequisite.
        archive=ROOT/'试飞产物/board_full_mission_20261007_221730'
        if archive.exists():
            meta=json.loads((archive/'run_metadata.json').read_text())
            for key,value in site.items():self.assertEqual(value,meta['settings'][key],key)
            old=yaml.safe_load((archive/'runtime.yaml').read_text())
            self.assertEqual(old['mission']['post_delivery_route'],r['mission']['post_delivery_route'])
            self.assertEqual(old['high_view_probe']['config'],r['high_view_probe']['config'])

    def test_real_xml_loader_final_parameters_follow_generated_control(self):
        # XmlLoader expands all actual nested includes and rosparams; it starts no nodes.
        import roslaunch, rospkg
        from trial_speed_profiles import apply_speed_profile
        roslaunch.substitution_args._rospack=rospkg.RosPack(ros_paths=[
            str(ROOT/'deployment/board_trials_4x4/common'),str(ROOT/'vision_ws/src'),
            str(ROOT/'patrol_uav_ws-patrol_planner/src'),'/opt/ros/noetic/share'])
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for profile,expected in [('limited',(.25,.25,.75)),('competition',(.4,.4,1.2))]:
            with self.subTest(profile=profile), tempfile.TemporaryDirectory() as tmp:
                settings=yaml.safe_load((BASE/'08_full_mission/settings.yaml').read_text())
                apply_site_profile(settings,yaml.safe_load(SITE.read_text()))
                apply_speed_profile(ROOT,settings,'full_mission',profile)
                reference=generate(ROOT,tmp,settings,REFERENCE,rig)
                control=yaml.safe_load((Path(tmp)/'control.yaml').read_text())
                overrides=yaml.safe_load((Path(tmp)/'overrides.yaml').read_text())
                args=['enable_control_output:=true','simulation:=false','mode:=high_view_full',
                      'model_path:=/offline-test.rknn','generated_dir:='+tmp,
                      'ground_z:='+str(reference['ground_z']),'low_z:='+str(reference['low_z']),
                      'cruise_speed:='+str(settings['cruise_speed']),
                      'cruise_acceleration:='+str(settings['cruise_acceleration'])]
                config=roslaunch.config.ROSLaunchConfig()
                roslaunch.xmlloader.XmlLoader().load(
                    str(SCRIPTS.parent/'launch/application.launch'),config,argv=args,verbose=False)
                values={key:param.value for key,param in config.params.items()}
                self.assert_drop_settle(control)
                for key,value in control['drop_system']['settle'].items():
                    self.assertEqual(values['/drop_system/settle/'+key],value)
                for key,agl in [('release_setpoint_height',.40),('release_min_height',.35),('height_threshold',.45)]:
                    self.assertAlmostEqual(values['/drop_system/'+key]-reference['ground_z'],agl)
                self.assertEqual(values['/external_landing/posctl/xy_tolerance_m'],.05)
                self.assertEqual(values['/external_landing/posctl/stable_duration_sec'],.15)
                self.assertEqual((values['/px4_max_distance'],
                    values['/traj_server/traj_server/target_dist'],
                    values['/external_planner_start_max_distance']),expected)
                self.assertEqual(values['/px4_max_distance'],control['px4_max_distance'])
                self.assertEqual(overrides['/px4_max_distance'],control['px4_max_distance'])
                for ns,v,a in [('manager','max_vel','max_acc'),('search','max_vel','max_acc'),
                               ('optimization','max_vel','max_acc'),('bspline','limit_vel','limit_acc')]:
                    self.assertEqual(values['/fast_planner_node/'+ns+'/'+v],settings['cruise_speed'])
                    self.assertEqual(values['/fast_planner_node/'+ns+'/'+a],settings['cruise_acceleration'])

    def test_04_real_cli_preserves_its_h_and_initial_thresholds(self):
        e,r,c,o=self.cli('corridor_landing')
        self.assert_corridor(r);self.assert_h(c,1.4,1.8)
        self.assertEqual(e['settings']['landing_transit_agl'],1.4)
        self.assertEqual((c['px4_max_distance'],o['/traj_server/traj_server/target_dist'],
                          o['/external_planner_start_max_distance']),(.25,.25,.75))
        self.assertFalse(c['drop_system']['enable_drop'])

    def test_corridor_schedule_also_applies_with_motion_disabled(self):
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for trial in ('corridor_landing','full_mission'):
            s=yaml.safe_load((BASE/TRIAL_FOLDERS[trial]/'settings.yaml').read_text())
            apply_site_profile(s,yaml.safe_load(SITE.read_text()))
            s['motion_optimization']['enabled']=False
            with tempfile.TemporaryDirectory() as out:
                generate(ROOT,out,s,REFERENCE,rig)
                r=yaml.safe_load((Path(out)/'runtime.yaml').read_text())
                self.assert_corridor(r)
                self.assertEqual(len(r['mission']['post_delivery_route']),len(s['corridor_waypoints'])+2)

    def test_all_actual_delivery_groups_target_040_and_release_within_035_045(self):
        for trial in ('visual_interrupt','high_view','low_multi','high_priority'):
            e,r,c,o=self.cli(trial,site=False)
            self.assertEqual(e['drop_agl'],.40)
            self.assertAlmostEqual(c['drop_system']['release_setpoint_height']-(REFERENCE[2]-.22),.40)
            self.assertAlmostEqual(c['drop_system']['release_min_height']-(REFERENCE[2]-.22),.35)
            self.assertAlmostEqual(c['drop_system']['height_threshold']-(REFERENCE[2]-.22),.45)
            self.assertTrue(c['drop_system']['enable_drop'])
            self.assert_drop_settle(c)
            self.assertEqual((c['px4_max_distance'],o['/traj_server/traj_server/target_dist'],
                              o['/external_planner_start_max_distance']),(.25,.25,.75))

    def test_fixed_site_does_not_inherit_old_h_or_drop_override(self):
        from trial_config import validate_settings
        s=yaml.safe_load((BASE/'08_full_mission/settings.yaml').read_text())
        self.assertNotIn('landing_posctl',yaml.safe_load(SITE.read_text()))
        for invalid in (dict(drop_agl=.45),):
            with self.assertRaises(ValueError):apply_site_profile(copy.deepcopy(s),invalid)
        s['speed_profile']='competition';s['trial_kind']='corridor_landing'
        with self.assertRaises(ValueError):validate_settings(s)

if __name__=='__main__':unittest.main()
