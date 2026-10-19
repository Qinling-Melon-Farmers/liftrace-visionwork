"""Real offline CLI checks for registered field defaults; never start ROS."""
from pathlib import Path
import copy,json,os,shutil,subprocess,sys,tempfile,unittest,yaml
from trial_config import apply_site_profile,load_trial_settings,validate_settings

ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
SCRIPT=BASE/'common/uav_board_trials/scripts/run_trial.py'
NAME='site_waypoint_map_20261016.yaml'
def site_name(folder):
    return NAME if folder=='08_full_mission' else 'site_lawnmower_20261016.yaml'


class RegisteredSite(unittest.TestCase):
    def run_cli(self,trial,*args,root=ROOT):
        with tempfile.TemporaryDirectory() as out:
            command=[sys.executable,str(SCRIPT),trial,'preview','--root',str(root),
                     '--check-config','--generate-config',out,'--reference-fc','0','0','0',*args]
            result=subprocess.run(command,text=True,capture_output=True)
            data=json.loads(result.stdout.splitlines()[-1]) if result.returncode==0 else None
            if data and data['settings'].get('speed_profile')=='competition':
                overrides=yaml.safe_load((Path(out)/'overrides.yaml').read_text())
                for namespace,velocity,acceleration in (('manager','max_vel','max_acc'),('search','max_vel','max_acc'),('optimization','max_vel','max_acc'),('bspline','limit_vel','limit_acc')):
                    self.assertEqual(overrides['/fast_planner_node/'+namespace+'/'+velocity],1.2)
                    self.assertEqual(overrides['/fast_planner_node/'+namespace+'/'+acceleration],1.)
            runtime=yaml.safe_load((Path(out)/'runtime.yaml').read_text()) if data else None
            return result,data,runtime

    def test_no_site_all_four_coverage_combinations(self):
        for pattern,count in (('lawnmower',6),('nested_rectangles',11)):
            for height in (1.8,1.9):
                with self.subTest(pattern=pattern,height=height):
                    result,data,runtime=self.run_cli('low_coverage_full','--coverage-pattern',pattern,'--coverage-agl',str(height))
                    self.assertEqual(result.returncode,0,result.stderr)
                    value=data['settings']
                    self.assertEqual(value['low_agl'],height)
                    self.assertEqual(value['coverage_pattern'],pattern)
                    self.assertEqual(len(runtime['trial']['waypoints']),count)
                    self.assertEqual(value['landing_xy'],[7.51,-3.85])
                    self.assertEqual(value['drop_agl'],.25)
                    self.check_speed(value,out_runtime=runtime)

    def test_shiftback_is_explicit_only_and_preserves_corridor(self):
        for pattern in ('lawnmower','nested_rectangles'):
            result,default,rt=self.run_cli('low_coverage_full','--coverage-pattern',pattern)
            self.assertEqual(result.returncode,0,result.stderr)
            path=BASE/'10_low_coverage_full'/('site_'+pattern+'_shiftback_1m_20261016.yaml')
            result,shifted,srt=self.run_cli('low_coverage_full','--coverage-pattern',pattern,'--site-config',str(path))
            self.assertEqual(result.returncode,0,result.stderr)
            original=rt['trial']['waypoints'];candidate=srt['trial']['waypoints']
            self.assertEqual(len(original),len(candidate))
            for a,b in zip(original,candidate):
                self.assertAlmostEqual(a[0]-1.,b[0])
                self.assertEqual(a[1:],b[1:])
            self.assertAlmostEqual(min(p[0] for p in original),1.)
            self.assertAlmostEqual(max(p[0] for p in original),6.2)
            for key in ('corridor_waypoints','landing_xy','landing_capture_agl'):
                self.assertEqual(default['settings'][key],shifted['settings'][key])

    def test_corridor_default_matches_full_corridor_and_h(self):
        result,corridor,_=self.run_cli('corridor_landing')
        self.assertEqual(result.returncode,0,result.stderr)
        result,full,_=self.run_cli('full_mission')
        self.assertEqual(result.returncode,0,result.stderr)
        for key in ('corridor_waypoints','corridor_geometry','landing_xy','landing_transit_agl','landing_capture_agl'):
            self.assertEqual(corridor['settings'][key],full['settings'][key])
        self.assertEqual(corridor['settings']['cruise_speed'],load_trial_settings(ROOT,'04_corridor_landing')['cruise_speed'])

    def test_full_default_geometry_and_heights(self):
        result,data,runtime=self.run_cli('full_mission')
        self.assertEqual(result.returncode,0,result.stderr)
        value=data['settings']
        for key,expected in dict(high_agl=2.6,max_agl=3.2,low_agl=1.4,
                                 survey_pattern='rectangle',landing_capture_agl=1.1,drop_agl=.25).items():
            self.assertEqual(value[key],expected,key)
        self.assertEqual(value['flight_area']['search_bounds'],[-1.34,6.56,-4.85,4.85])
        self.assertEqual(value['flight_area']['survey_xy'],[[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5],[1,-3.5]])
        self.assertEqual(value['flight_area']['map_size'],[22,12,4])
        self.assertEqual(value['landing_xy'],[7.51,-3.85])
        self.assertEqual(len(value['corridor_waypoints']),6)
        self.assertIn('mission',runtime)
        self.check_speed(value,out_runtime=runtime)

    def test_missing_defaults_reject_and_explicit_override_wins(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            shutil.copytree(BASE,root/'deployment/board_trials_4x4')
            for name in ('docs','patrol_uav_ws-patrol_planner','vision_ws'):
                (root/name).symlink_to(ROOT/name,target_is_directory=True)
            for trial,folder in (('full_mission','08_full_mission'),('low_coverage_full','10_low_coverage_full')):
                with self.subTest(trial=trial):
                    default=root/'deployment/board_trials_4x4'/folder/site_name(folder)
                    override=yaml.safe_load(default.read_text())
                    override['landing_xy']=[7.6,-3.4]
                    explicit=root/(trial+'.yaml')
                    explicit.write_text(yaml.safe_dump(override))
                    result,data,_=self.run_cli(trial,'--site-config',str(explicit),root=root)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertEqual(data['settings']['landing_xy'],[7.6,-3.4])
                    default.unlink()
                    result,_,_=self.run_cli(trial,root=root)
                    self.assertNotEqual(result.returncode,0)
                    self.assertIn('Registered default site config not found',result.stderr)
                    self.assertIn(str(default),result.stderr)
                    result,data,_=self.run_cli(trial,'--site-config',str(explicit),root=root)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertEqual(data['settings']['landing_xy'],[7.6,-3.4])
                    result,_,_=self.run_cli(trial,'--site-config',str(root/'missing.yaml'),root=root)
                    self.assertNotEqual(result.returncode,0)


    def check_speed(self,value,out_runtime):
        self.assertEqual(value['speed_profile'],'competition')
        self.assertEqual(value['cruise_speed'],1.2)
        self.assertEqual(value['cruise_acceleration'],1.)
        self.assertEqual(value['landing_capture_agl'],1.1)
        self.assertEqual(value['landing_transit_agl'],1.)
        following=value['following_speed_profile']
        self.assertEqual(following['cruise_lead_m'],1.)
        for key in ('precision_lead_m','boundary_lead_m','low_coverage_lead_m','corridor_lead_m'):
            self.assertEqual(following[key],.6)

    def test_limited_override_keeps_module_speeds(self):
        for trial,folder in (('full_mission','08_full_mission'),('low_coverage_full','10_low_coverage_full')):
            result,data,_=self.run_cli(trial,'--speed-profile','limited')
            self.assertEqual(result.returncode,0,result.stderr)
            original=load_trial_settings(ROOT,folder)
            self.assertEqual(data['settings']['speed_profile'],'limited')
            for key in ('cruise_speed','cruise_acceleration'):
                self.assertEqual(data['settings'][key],original[key])

    def test_full_search_must_remain_in_target_bounds(self):
        value=load_trial_settings(ROOT,'08_full_mission')
        apply_site_profile(value,yaml.safe_load((BASE/'08_full_mission'/NAME).read_text()))
        value['flight_area']['search_bounds'][1]=8.
        with self.assertRaisesRegex(ValueError,'Search bounds must lie within target_bounds'):
            validate_settings(value)


    def test_both_modules_keep_all_waypoints_inside_flight_area(self):
        for folder in ('08_full_mission','10_low_coverage_full'):
            value=load_trial_settings(ROOT,folder)
            apply_site_profile(value,yaml.safe_load((BASE/folder/site_name(folder)).read_text()))
            if folder.startswith('10'):
                from trial_low_coverage import prepare_low_coverage
                prepare_low_coverage(value,site_provided=True)
            for kind in ('staging','survey','corridor','landing','search'):
                with self.subTest(folder=folder,kind=kind):
                    invalid=copy.deepcopy(value)
                    if kind=='staging': invalid['flight_area']['staging_xy'][0]=10.
                    elif kind=='survey': invalid['flight_area']['survey_xy'][0][0]=10.
                    elif kind=='corridor': invalid['corridor_waypoints'][0]['x']=10.
                    elif kind=='landing': invalid['landing_xy'][0]=10.
                    else: invalid['flight_area']['search_bounds'][1]=10.
                    with self.assertRaises(ValueError):
                        validate_settings(invalid)

if __name__=='__main__':
    unittest.main()
