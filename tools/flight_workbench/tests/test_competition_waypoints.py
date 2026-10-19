"""Independent competition map overlays: real configuration validator, no ROS."""
import base64
import copy
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_waypoints import WaypointTests
import wb_board
import wb_waypoints as waypoints
import wb_competition_geometry as geometry

TOOL=Path(__file__).resolve().parents[1]
ROOT=TOOL.parents[1]

class CompetitionWaypoints(unittest.TestCase):
    setUp=WaypointTests.setUp
    tearDown=WaypointTests.tearDown

    def context(self,path=None):
        return waypoints.context(self.wb,dict(group_id='competition',**({'competition_config':path} if path else {})))

    def complete(self):
        c=self.context()
        site=copy.deepcopy(c['site'])
        site['corridor_waypoints']=[dict(x=8.,y=2.,agl=.9),dict(x=8.,y=-2.,agl=.9)]
        site['landing_xy']=[8.,-2.]
        return c,site

    def test_empty_template_and_explicit_confirmation(self):
        c=self.context()
        self.assertEqual(c['editor_kind'],'competition')
        self.assertFalse(c['site']['site_confirmed'])
        self.assertIsNone(c['site']['landing_xy'])
        c,site=self.complete()
        saved=waypoints.save(self.wb,dict(group_id='competition',scope=c['scope'],site=site))
        self.assertFalse(saved['patch']['site_confirmed'])
        self.assertFalse(self.context()['site']['site_confirmed'])

    def test_selected_source_isolation_and_source_scoping(self):
        c,site=self.complete()
        waypoints.save(self.wb,dict(group_id='competition',scope=c['scope'],site=site))
        other=self.context('deployment/competition/candidates/snake3_motion.yaml')
        self.assertNotEqual(c['scope'],other['scope'])
        self.assertIsNone(other['draft'])
        self.assertAlmostEqual(other['site']['high_agl'],2.16)
        with self.assertRaises(ValueError): self.context('deployment/competition/unavailable.yaml')
        with self.assertRaises(ValueError): self.context('deployment/competition/../../secrets.yaml')
        with self.assertRaises(ValueError):
            waypoints.save(self.wb,dict(group_id='competition',competition_config=other['source'],scope=c['scope'],site=site))

    def test_geometry_preserves_full_competition_settings(self):
        profile=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        c,site=self.complete();site['site_confirmed']=True
        patch=waypoints.patch_from_site(site,True)
        merged=geometry.merge_profile(profile,patch)
        changed=set(geometry.AREA_KEYS.values())|{'corridor_waypoints','landing_xy','site_confirmed','corridor_speed_schedule','following_speed_profile'}
        for key in set(profile)-changed: self.assertEqual(profile[key],merged[key],key)
        self.assertEqual(merged['corridor_speed_schedule']['open_lead_m'],profile['corridor_speed_schedule']['open_lead_m'])
        self.assertEqual(merged['following_speed_profile']['cruise_lead_m'],profile['following_speed_profile']['cruise_lead_m'])
        self.assertNotIn('flight_area',merged)
        self.assertNotIn('corridor_geometry',merged)
        self.assertEqual(merged['flight_bounds'],site['flight_area']['center_bounds'])

    def test_same_overlay_reaches_check_preview_flight_and_keeps_landing(self):
        c,site=self.complete();patch=waypoints.patch_from_site(site,True)
        group=next(g for g in self.wb.config['groups'] if g['id']=='competition')
        outputs=[]
        for mode,check in [('preview',True),('preview',False),('flight',False)]:
            command,_=wb_board.build_group_command(self.wb.config,group,mode,check_config=check,
                real_release=True,site_geometry=patch,geometry_revision='fixture1',
                landing_config='candidate.yaml',motion_optimization='off',resume_survey='on')
            argv=shlex.split(command);tail=argv[argv.index('&&')+1:]
            self.assertEqual(tail[:3],['bash','deployment/competition/start.sh',mode])
            self.assertIn('--landing-config',tail);self.assertIn('--resume-survey',tail)
            outputs.append(tail[tail.index('--site-config')+1])
            self.assertNotIn('board_trials',command)
        self.assertEqual(len(set(outputs)),1)

    def test_real_validator_runs_without_board_trials_or_installed_workbench(self):
        c,site=self.complete();site['site_confirmed']=True
        patch=waypoints.patch_from_site(site,True)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ('patrol_uav_ws-patrol_planner','vision_ws'): (root/name).symlink_to(ROOT/name,target_is_directory=True)
            source=root/'field.yaml';source.write_bytes((ROOT/'deployment/competition/field.example.yaml').read_bytes())
            before=source.read_bytes()
            relative,command,_=geometry.overlay_command('field.yaml',patch,'fixture2')
            argv=shlex.split(command);argv[0]=sys.executable
            result=subprocess.run(argv,cwd=root,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(source.read_bytes(),before)
            generated=yaml.safe_load((root/relative).read_text())
            self.assertEqual(generated['corridor_waypoints'],site['corridor_waypoints'])
            self.assertEqual(generated['drop_agl'],.40)
            self.assertFalse((root/'tools').exists())
            result=subprocess.run(argv,cwd=root,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            original=yaml.safe_load(source.read_text());original['cruise_speed']=1.1
            source.write_text(yaml.safe_dump(original))
            result=subprocess.run(argv,cwd=root,capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('source changed',result.stderr)

    def test_reject_geometry_and_non_geometry_injection(self):
        c,site=self.complete();patch=waypoints.patch_from_site(site,True)
        for key,value in [('drop_agl',.1),('landing_posctl',{}),('site_confirmed','true')]:
            with self.assertRaises(ValueError):geometry.validate_patch(dict(patch,**{key:value}))
        patch['flight_area']['coverage_bounds']=[-100,100,-100,100]
        with self.assertRaises(ValueError):geometry.validate_patch(patch)

    def test_routes_use_asymmetric_origin_without_changing_height(self):
        c,site=self.complete();site['flight_area']['search_bounds']=[.6,7.2,-3.7,1.6]
        before=copy.deepcopy(site)
        for pattern,count in [('rectangle',5),('snake2',4),('snake3',6)]:
            result=waypoints.route(self.wb,dict(group_id='competition',site=site,pattern=pattern,inset=.2))
            self.assertEqual(len(result['route']),count)
            self.assertAlmostEqual(min(p[0] for p in result['route']),.8)
            self.assertAlmostEqual(max(p[0] for p in result['route']),7.)
            self.assertAlmostEqual(min(p[1] for p in result['route']),-3.5)
            self.assertAlmostEqual(max(p[1] for p in result['route']),1.4)
            self.assertEqual(site,before)
        with self.assertRaises(ValueError):waypoints.route(self.wb,dict(group_id='competition',site=site,pattern='snake3',inset=10))

    def test_real_import_and_preview(self):
        c,site=self.complete()
        profile=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        profile=geometry.merge_profile(profile,waypoints.patch_from_site(site,True))
        imported=waypoints.import_site(self.wb,dict(group_id='competition',text=yaml.safe_dump(profile)))
        self.assertEqual(imported,site)
        self.assertEqual(waypoints.preview(self.wb,dict(group_id='competition',site=site))['errors'],[])

if __name__=='__main__':unittest.main(verbosity=2)