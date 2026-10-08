import unittest,copy,json,sys,tempfile,subprocess,shlex,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import wb_board,wb_geometry
import test_review

class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.config=wb_board.load_config();self.group=next(g for g in self.config['groups'] if g['id']=='mod08')
        self.patch=dict(corridor_waypoints=[dict(x=.6,y=0.,agl=.9),dict(x=1.5,y=.4,agl=.9)],landing_xy=[2.5,0.])
    def test_invalid_and_mode_fields_are_rejected(self):
        for patch in [dict(self.patch,cruise_speed=1.2),dict(self.patch,landing_xy=[float('nan'),0]),dict(self.patch,corridor_waypoints=[dict(x=0,y=0,agl=True)]),dict(self.patch,corridor_geometry={'wall_axis':True,'wall_coordinates':[1.4],'entry_waypoints':1})]:
            with self.assertRaises(ValueError):wb_geometry.validate_geometry(patch)
        with self.assertRaises(ValueError):wb_geometry.overlay_command('site.yaml','03_h_landing',self.patch,'a')
    def test_same_geometry_uses_same_overlay_across_modes(self):
        paths=[]
        for mode,check,real in [('preview',True,False),('preview',False,False),('flight',False,True)]:
            cmd,_=wb_board.build_group_command(self.config,self.group,mode,check_config=check,real_release=real,site_geometry=self.patch,geometry_revision='a')
            args=shlex.split(cmd);paths.append(args[args.index('--site-config')+1]);self.assertIn('&&',args)
        self.assertEqual(len(set(paths)),1)
    def test_writer_preserves_profile_and_rejects_out_of_bounds(self):
        repo=Path(os.environ.get('LIFTRACE_BOARD_FIXTURE_ROOT',Path(__file__).resolve().parents[3]))
        if not (repo/'deployment/site_20260928/full_mission_test_area.yaml').is_file():
            self.skipTest('This branch has no board site fixture; set LIFTRACE_BOARD_FIXTURE_ROOT to the reviewed board checkout')
        import yaml
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);(root/'deployment').symlink_to(repo/'deployment',target_is_directory=True)
            (root/'patrol_uav_ws-patrol_planner').symlink_to(repo/'patrol_uav_ws-patrol_planner',target_is_directory=True)
            (root/'vision_ws').symlink_to(repo/'vision_ws',target_is_directory=True)
            (root/'docs').symlink_to(repo/'docs',target_is_directory=True)
            (root/'tools').symlink_to(repo/'tools',target_is_directory=True)
            source=root/'site.yaml';base=yaml.safe_load((repo/'deployment/site_20260928/full_mission_test_area.yaml').read_text());source.write_text(yaml.safe_dump(base))
            original=source.read_text()
            target,cmd,_=wb_geometry.overlay_command(str(source),'08_full_mission',self.patch,'a')
            subprocess.run(shlex.split(cmd),cwd=root,check=True,capture_output=True)
            generated=yaml.safe_load((root/target).read_text());self.assertEqual(generated['max_agl'],base['max_agl']);self.assertEqual(generated['flight_area'],base['flight_area']);self.assertEqual(generated['landing_xy'],[2.5,0.]);self.assertEqual(source.read_text(),original)
            subprocess.run(shlex.split(cmd),cwd=root,check=True,capture_output=True)
            base['max_agl']=1.9;source.write_text(yaml.safe_dump(base));self.assertNotEqual(subprocess.run(shlex.split(cmd),cwd=root,capture_output=True).returncode,0)
            bad=dict(self.patch,landing_xy=[200,0]);t,c,_=wb_geometry.overlay_command(str(source),'08_full_mission',bad,'b');self.assertNotEqual(subprocess.run(shlex.split(c),cwd=root,capture_output=True).returncode,0);self.assertFalse((root/t).exists())
    def test_full_generator_rejects_invalid_motion_geometry_before_overlay(self):
        repo=Path(os.environ.get('LIFTRACE_BOARD_FIXTURE_ROOT',Path(__file__).resolve().parents[3]))
        if not (repo/'deployment/site_20260928/full_mission_test_area.yaml').is_file():
            self.skipTest('This branch has no board site fixture; set LIFTRACE_BOARD_FIXTURE_ROOT to the reviewed board checkout')
        import yaml
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            for folder in ('patrol_uav_ws-patrol_planner','vision_ws','docs','tools'):
                (root/folder).symlink_to(repo/folder,target_is_directory=True)
            modules=root/'deployment/board_trials_4x4';modules.mkdir(parents=True)
            (modules/'common').symlink_to(repo/'deployment/board_trials_4x4/common',target_is_directory=True)
            module=modules/'08_full_mission';module.mkdir()
            settings=yaml.safe_load((repo/'deployment/board_trials_4x4/08_full_mission/settings.yaml').read_text())
            settings.setdefault('motion_optimization',{})['enabled']=True
            (module/'settings.yaml').write_text(yaml.safe_dump(settings))
            source=root/'site.yaml';source.write_text((repo/'deployment/site_20260928/full_mission_test_area.yaml').read_text())
            patch=dict(self.patch,corridor_waypoints=[dict(x=.6,y=0.,agl=.9),dict(x=2.,y=0.,agl=1.2)],corridor_geometry=dict(wall_axis=0,wall_coordinates=[1.4],entry_waypoints=1))
            target,cmd,_=wb_geometry.overlay_command(str(source),'08_full_mission',patch,'badmotion')
            result=subprocess.run(shlex.split(cmd),cwd=root,text=True,capture_output=True)
            self.assertNotEqual(result.returncode,0);self.assertIn('height cap',result.stderr)
            self.assertFalse((root/target).exists())
            patch['corridor_waypoints'][1]['agl']=.9
            target,cmd,_=wb_geometry.overlay_command(str(source),'08_full_mission',patch,'goodmotion')
            result=subprocess.run(shlex.split(cmd),cwd=root,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr);self.assertTrue((root/target).exists())

    def test_values_never_become_shell_program(self):
        with self.assertRaises(ValueError):wb_geometry.validate_geometry(dict(self.patch,landing_xy=['$(touch /tmp/never)',0]))
        _,cmd,_=wb_geometry.overlay_command("field name'; false; #.yaml",'08_full_mission',self.patch,'safe')
        self.assertEqual(shlex.split(cmd)[3],"field name'; false; #.yaml")

class CommandTests(unittest.TestCase):
    setUp=test_review.ReviewTests.setUp
    def request(self):return dict(group_id='mod08',mode='preview',check_config=True,real_release=False,geometry_revision='r1',site_geometry=dict(corridor_waypoints=[dict(x=.6,y=0,agl=.9),dict(x=2,y=0,agl=.9)],landing_xy=[2.5,0.]))
    def test_offline_command_plan_does_not_execute(self):
        body=self.request();plan=self.wb.trial_command(body);self.assertTrue(plan['ok']);self.wb.sessions.open.assert_not_called()
        body['expected_body']=plan['body'];res=self.wb.start_trial(body);self.assertEqual(res['trial']['command'],plan['body'])
    def test_changed_geometry_requires_new_preview(self):
        body=self.request();body['expected_body']=self.wb.trial_command(body)['body'];body['site_geometry']['landing_xy']=[3.,0.]
        with self.assertRaisesRegex(ValueError,'不一致'):self.wb.start_trial(body)
        self.wb.sessions.open.assert_not_called()
    def test_missing_preview_and_live_trial_are_rejected(self):
        with self.assertRaises(ValueError):self.wb.start_trial(self.request())
        self.wb.sessions.open.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)