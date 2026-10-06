import unittest,sys,math,subprocess,shlex,tempfile,json,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import wb_survey,wb_geometry,wb_board

class SurveyTests(unittest.TestCase):
    def spec(self,**kw):
        v=dict(bounds=[.6,5.5,-1.1,1.1],inset=.35,pattern='snake3',camera_agl=1.84,fc_to_camera_z=-.16,fov_x_deg=75.,fov_y_deg=60.);v.update(kw);return v
    def test_shapes_heights_and_inside(self):
        for pattern,count in [('rectangle',5),('snake2',4),('snake3',6)]:
            p=wb_survey.plan(self.spec(pattern=pattern));self.assertEqual(len(p['route']),count);self.assertAlmostEqual(p['high_agl'],2.)
            self.assertTrue(all(.95-1e-9<=x<=5.15+1e-9 and -.75-1e-9<=y<=.75+1e-9 for x,y in p['route']))
            self.assertTrue(0<=p['coverage_percent']<=100)
    def test_exact_full_coverage(self):
        p=wb_survey.plan(self.spec(bounds=[0,2,0,2],inset=.5,pattern='rectangle',camera_agl=1.,fov_x_deg=90.,fov_y_deg=90.))
        self.assertAlmostEqual(p['covered_m2'],4);self.assertAlmostEqual(p['coverage_percent'],100);self.assertEqual(p['blind_rectangles'],[])
    def test_blind_area_and_no_double_count(self):
        p=wb_survey.plan(self.spec(bounds=[0,10,0,10],inset=1.,pattern='snake2',camera_agl=.5,fov_x_deg=60.,fov_y_deg=60.))
        self.assertLess(p['coverage_percent'],40);self.assertGreater(len(p['blind_rectangles']),0)
        blind=sum((r[1]-r[0])*(r[3]-r[2]) for r in p['blind_rectangles'])
        self.assertAlmostEqual(blind+p['covered_m2'],100)
    def test_height_scales_footprint(self):
        a=wb_survey.plan(self.spec(camera_agl=1));b=wb_survey.plan(self.spec(camera_agl=2))
        self.assertEqual(b['footprint_xy'],[2*x for x in a['footprint_xy']])
    def test_invalid(self):
        for changes in [dict(inset=2),dict(camera_agl=0),dict(camera_agl=True),dict(fov_x_deg=180),dict(bounds=[0,0,0,1]),dict(fc_to_camera_z=float('nan')),dict(pattern='unknown')]:
            with self.assertRaises(ValueError):wb_survey.plan(self.spec(**changes))
    def test_command_high_only_and_pattern_consistency(self):
        c=wb_board.load_config();g=next(x for x in c['groups'] if x['id']=='site5')
        cmd,_=wb_board.build_group_command(c,g,'preview',check_config=True,site_geometry={'survey_plan':self.spec()},geometry_revision='a')
        self.assertIn('--survey-pattern snake3',cmd)
        with self.assertRaises(ValueError):wb_board.build_group_command(c,g,'preview',survey_pattern='rectangle',site_geometry={'survey_plan':self.spec()},geometry_revision='a')
        with self.assertRaises(ValueError):wb_geometry.overlay_command('site.yaml','04_corridor_landing',{'survey_plan':self.spec()},'a')
    def test_real_generator_offset_limit_and_source_preserved(self):
        repo=Path(os.environ.get('LIFTRACE_BOARD_FIXTURE_ROOT',Path(__file__).resolve().parents[3]))
        if not (repo/'deployment/site_20260928/test_area.yaml').exists():self.skipTest('Board site fixture unavailable')
        import yaml
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            for f in ('deployment','patrol_uav_ws-patrol_planner','vision_ws','docs','tools'):(root/f).symlink_to(repo/f,target_is_directory=True)
            src=root/'site.yaml';src.write_text((repo/'deployment/site_20260928/test_area.yaml').read_text());old=src.read_text()
            patch={'survey_plan':self.spec()};dst,cmd,_=wb_geometry.overlay_command(str(src),'06_high_priority',patch,'a')
            result=subprocess.run(shlex.split(cmd),cwd=root,text=True,capture_output=True);self.assertEqual(result.returncode,0,result.stderr)
            saved=yaml.safe_load((root/dst).read_text());self.assertAlmostEqual(saved['high_agl'],2);self.assertEqual(len(saved['flight_area']['survey_xy']),6);self.assertEqual(saved['max_agl'],2);self.assertEqual(src.read_text(),old)
            for changes,why in [(dict(camera_agl=2.),'height limit'),(dict(fc_to_camera_z=-.2),'offset')]:
                patch={'survey_plan':self.spec(**changes)};dst,cmd,_=wb_geometry.overlay_command(str(src),'06_high_priority',patch,'b')
                result=subprocess.run(shlex.split(cmd),cwd=root,text=True,capture_output=True);self.assertNotEqual(result.returncode,0,why);self.assertFalse((root/dst).exists())

if __name__=='__main__':unittest.main(verbosity=2)
