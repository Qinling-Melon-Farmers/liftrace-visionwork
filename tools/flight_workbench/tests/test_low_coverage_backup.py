"""Coverage backup map/command contracts; no SSH, ROS, PWM or flight."""
import copy,json,os,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
import yaml
TOOL=Path(__file__).resolve().parents[1];sys.path.insert(0,str(TOOL))
import wb_board,wb_low_coverage as low

class CoverageBackupTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.p=patch.object(wb_board,'DEFAULT_PROFILE_DIR',self.temp.name);self.p.start();self.addCleanup(self.p.stop)
        self.config=wb_board.load_config(str(TOOL/'workbench.yaml'))
        self.group=next(g for g in self.config['groups'] if g.get('folder')==low.FOLDER)
        self.raw=yaml.safe_load((TOOL/'waypoint_templates/module10.yaml').read_text(encoding='utf-8'))
        self.site=low.normalize_site(self.raw)
        self.wb=N(config=self.config,target=N(host='test',transport='local'),connection={'state':'unknown'},sessions=N(get=lambda _:None))
        self.wb.board=wb_board.BoardClient(self.config,self.wb.target)
    def command(self,agl=1.8,real=False,check=False,revision='map1',pattern='lawnmower'):
        return wb_board.build_group_command(self.config,self.group,'flight' if not check else 'preview',
            real_release=real,check_config=check,coverage_agl=agl,coverage_pattern=pattern,site_geometry=low.patch_from_site(self.site),geometry_revision=revision)[0]
    def test_original_map_geometry_preserved_without_high_route_conversion(self):
        self.assertEqual(self.site['flight_area']['center_bounds'],self.raw['flight_bounds'])
        self.assertEqual(self.site['flight_area']['target_bounds'],self.raw['target_bounds'])
        self.assertEqual(self.site['flight_area']['search_bounds'],self.raw['coverage_bounds'])
        self.assertEqual(self.site['flight_area']['staging_xy'],self.raw['staging_xy'])
        self.assertEqual(self.site['corridor_waypoints'],self.raw['corridor_waypoints'])
        self.assertEqual(self.site['landing_xy'],self.raw['landing_xy'])
        self.assertEqual(self.site['landing_capture_agl'],self.raw['landing_capture_agl'])
        self.assertEqual(self.site['_low_coverage_original'],self.raw)
    def test_height_explicit_check_before_flight_and_real_opt_in(self):
        for agl in (1.8,1.9):
            command=self.command(agl)
            self.assertIn('--coverage-agl %.1f'%agl,command)
            self.assertIn('preview --site-config',command)
            self.assertLess(command.index('--check-config'),command.index(' flight --site-config'))
            self.assertNotIn('--real-release',command)
            self.assertNotIn('--speed-profile',command)
            self.assertIn('--real-release',self.command(agl,True))
            self.assertNotIn('08_full_mission/start',command)
        self.assertNotEqual(self.command(1.8),self.command(1.9))
        self.assertEqual(self.command(),self.command())
    def test_search_pattern_changes_command_without_mutating_map_or_height(self):
        original=copy.deepcopy(self.site)
        for pattern in ('lawnmower','nested_rectangles'):
            command=self.command(1.9,pattern=pattern)
            self.assertEqual(command.count('--coverage-pattern '+pattern),2)
            self.assertIn('--coverage-agl 1.9',command)
            self.assertNotIn('--coverage-lane-spacing',command)
        self.assertNotEqual(self.command(pattern='lawnmower'),self.command(pattern='nested_rectangles'))
        self.assertEqual(self.site,original)
        for invalid in ('rectangle','nested_rectangles;bad',True,1):
            with self.assertRaises(ValueError):self.command(pattern=invalid)
    def test_unsupported_options_and_missing_geometry_do_not_fall_back(self):
        for agl in (.18,1.4,2.,True,float('nan')):
            with self.assertRaises(ValueError):self.command(agl)
        with self.assertRaises(ValueError):wb_board.build_group_command(self.config,self.group,'preview',coverage_agl=1.8)
        with self.assertRaises(ValueError):wb_board.build_group_command(self.config,self.group,'preview',coverage_agl=1.8,site_geometry=self.site,geometry_revision='r1',survey_pattern='rectangle')
        old=next(g for g in self.config['groups'] if g['id']=='mod08')
        with self.assertRaises(ValueError):wb_board.build_group_command(self.config,old,'preview',coverage_agl=1.8)
    def test_import_save_and_context_reuse_the_exact_saved_draft(self):
        body={'group_id':self.group['id']}
        imported=low.map_action(self.wb,'import',dict(body,text=yaml.safe_dump(self.raw)))
        context=low.map_action(self.wb,'context',body)
        result=low.map_action(self.wb,'save',dict(body,scope=context['scope'],site=imported['site']))
        restored=low.map_action(self.wb,'context',body)
        self.assertEqual(low.patch_from_site(restored['site']),result['patch'])
        self.assertEqual(result['patch']['corridor_waypoints'],self.raw['corridor_waypoints'])
        self.assertIn('10_low_coverage_full',self.command())
        self.wb.target.host='other'
        self.assertIsNone(low.map_action(self.wb,'draft',body)['draft'])
    def test_server_preview_and_start_bind_the_same_selected_height(self):
        import server
        with patch.object(server,'PROFILE_DIR',self.temp.name):
            workbench=server.Workbench(self.config,{'transport':'ssh'})
            workbench.connection['state']='ok'
            body=dict(group_id=self.group['id'],mode='flight',real_release=False,
                confirm='启动试飞',coverage_agl=1.9,coverage_pattern='nested_rectangles',site_geometry=low.patch_from_site(self.site),geometry_revision='serverMap1')
            expected=workbench.trial_command(body)['body']
            with patch.object(workbench.sessions,'open') as stale_open,self.assertRaisesRegex(ValueError,'不一致'):
                workbench.start_trial(dict(body,coverage_pattern='lawnmower',expected_body=expected))
            stale_open.assert_not_called()
            session=N(snapshot=lambda:{'state':'running'})
            with patch.object(workbench.board,'run',side_effect=AssertionError('no SSH calls')),patch.object(workbench.sessions,'open',return_value=session) as opened:
                result=workbench.start_trial(dict(body,expected_body=expected))
            opened.assert_called_once()
            self.assertEqual(result['trial']['coverage_agl'],1.9)
            self.assertEqual(result['trial']['coverage_pattern'],'nested_rectangles')
            self.assertIn('--coverage-pattern nested_rectangles',expected)
            self.assertEqual(result['trial']['command'],expected)
            self.assertIn('--coverage-agl 1.9',expected)
            self.assertLess(expected.index('--check-config'),expected.index(' flight --site-config'))
    def test_old_cards_and_default_selection_untouched(self):
        self.assertEqual(self.group['release'],'mock')
        self.assertNotEqual(self.config['groups'][0]['id'],self.group['id'])
        old=next(g for g in self.config['groups'] if g['id']=='mod08')
        self.assertEqual(old['folder'],'08_full_mission')
        self.assertNotIn('coverage_agl',wb_board.build_group_command(self.config,old,'preview')[0])
    def test_editor_has_height_and_apply_reminder(self):
        js=(TOOL/'web/low-coverage.js').read_text()
        self.assertIn("U.coverageHeight='1.8'",js)
        self.assertIn('不是镜头高度',js)
        self.assertIn('保存并用于本组',js)
        self.assertIn('coverage-agl',js)
        self.assertIn('low-coverage.js',(TOOL/'web/index.html').read_text())

if __name__=='__main__':unittest.main(verbosity=2)
