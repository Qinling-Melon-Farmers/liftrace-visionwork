"""Exact site selection, map source isolation and browser command parity; local only."""
import copy, json, os, subprocess, sys, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock, patch
import yaml
TOOL=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TOOL))
import wb_board, wb_low_coverage as low, wb_waypoints, wb_site_sources

class SiteSelectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        p=patch.object(wb_board,'DEFAULT_PROFILE_DIR',self.tmp.name);p.start();self.addCleanup(p.stop)
        self.config=wb_board.load_config(str(TOOL/'workbench.yaml'))
        self.group=next(g for g in self.config['groups'] if g.get('folder')==low.FOLDER)
        self.high=next(g for g in self.config['groups'] if g.get('folder')=='08_full_mission')
        self.wb=N(config=self.config,target=N(host='offline-test',transport='local'),connection={'state':'unknown'},sessions=N(get=lambda _:None))
        self.wb.board=wb_board.BoardClient(self.config,self.wb.target)
        self.run=patch.object(self.wb.board,'run',side_effect=AssertionError('No SSH allowed'));self.run.start();self.addCleanup(self.run.stop)
    def context(self,pattern,shiftback=False):
        return low.map_action(self.wb,'context',dict(group_id=self.group['id'],coverage_pattern=pattern,coverage_shiftback=shiftback))
    def test_corridor_group_uses_exact_new_site(self):
        group=next(g for g in self.config['groups'] if g['id']=='mod04')
        self.assertTrue(group['exact_site_reference'])
        ctx=wb_waypoints.context(self.wb,dict(group_id=group['id']))
        self.assertEqual(ctx['source'],group['site_config'])
        self.assertEqual(len(ctx['site']['corridor_waypoints']),6)
        self.assertEqual(ctx['site']['landing_xy'],[7.51,-3.85])
        self.assertEqual(ctx['site']['landing_capture_agl'],1.1)
        wb_waypoints.patch_from_site(ctx['site'])
        command=wb_board.build_group_command(self.config,group,'preview')[0]
        self.assertIn('--site-config '+ctx['source'],command)
    def test_context_matches_selected_source_and_explicit_route(self):
        a,b=[self.context(p) for p in ('lawnmower','nested_rectangles')]
        for result in (a,b):
            self.assertEqual(result['source'],self.group['site_configs_by_pattern'][result['coverage_pattern']])
            self.assertIn('未核对板端部署',result['origin'])
            self.assertEqual(result['site']['flight_area']['search_bounds'],[-1.34,6.56,-4.85,4.85])
        self.assertNotEqual(a['scope'],b['scope'])
        self.assertEqual(len(a['site']['search_waypoints_xy']),6)
        self.assertEqual([p[0] for p in a['site']['search_waypoints_xy']],[1,1,3.6,3.6,6.2,6.2])
        self.assertNotIn('search_waypoints_xy',b['site'])
        self.assertEqual(len(b['site']['coverage_rectangles']),2)
        high=wb_waypoints.context(self.wb,dict(group_id=self.high['id']))
        self.assertEqual(high['source'],self.high['site_config'])
        self.assertEqual(high['site']['flight_area']['search_bounds'],[-1.34,6.56,-4.85,4.85])
        self.assertEqual(high['site']['landing_xy'],a['site']['landing_xy'])
    def test_manual_route_survives_save_export_and_command(self):
        for pattern in ('lawnmower','nested_rectangles'):
            ctx=self.context(pattern);body=dict(group_id=self.group['id'],coverage_pattern=pattern)
            saved=low.map_action(self.wb,'save',dict(body,scope=ctx['scope'],site=ctx['site']))
            key='search_waypoints_xy' if pattern=='lawnmower' else 'coverage_rectangles'
            self.assertEqual(saved['patch'][key],ctx['site'][key])
            exported=low.map_action(self.wb,'export',dict(body,site=ctx['site']))
            self.assertEqual(yaml.safe_load(exported['yaml'])[key],ctx['site'][key])
            for selected in ('lawnmower','nested_rectangles'):
                command=wb_board.build_group_command(self.config,self.group,'preview',coverage_pattern=selected,site_geometry=saved['patch'],geometry_revision=saved['revision'])[0]
                self.assertIn('site_overlays/',command)
                self.assertNotIn('--site-config '+self.group['site_configs_by_pattern'][selected],command)
    def test_import_does_not_inherit_the_previous_explicit_route(self):
        lawn=self.context('lawnmower')['site'];nested=self.context('nested_rectangles')['site']
        result=low.map_action(self.wb,'import',dict(group_id=self.group['id'],coverage_pattern='nested_rectangles',text=yaml.safe_dump(nested),base_site=lawn))
        self.assertNotIn('search_waypoints_xy',result['site'])
        self.assertEqual(result['site']['coverage_rectangles'],nested['coverage_rectangles'])
    def test_drafts_do_not_leak_across_source_or_pattern(self):
        ctx=self.context('lawnmower');body=dict(group_id=self.group['id'],coverage_pattern='lawnmower')
        low.map_action(self.wb,'save',dict(body,scope=ctx['scope'],site=ctx['site']))
        self.assertIsNotNone(self.context('lawnmower')['draft'])
        self.assertIsNone(self.context('nested_rectangles')['draft'])
        self.group['site_configs_by_pattern']['lawnmower']='deployment/other.yaml'
        self.assertIsNone(low.map_action(self.wb,'draft',body)['draft'])
        with self.assertRaisesRegex(ValueError,'不回退旧地图'):self.context('lawnmower')
    def test_missing_board_file_never_falls_back_to_old_or_offline_map(self):
        self.wb.connection['state']='ok';self.wb.target.transport='ssh'
        with patch.object(self.wb.board,'run',return_value=(1,'missing')) as run:
            for pattern in ('lawnmower','nested_rectangles'):
                with self.assertRaisesRegex(ValueError,'未部署'):self.context(pattern)
                self.assertIn(self.group['site_configs_by_pattern'][pattern],run.call_args.args[0])
            with self.assertRaisesRegex(ValueError,'未部署'):wb_waypoints.context(self.wb,dict(group_id=self.high['id']))
    def test_connected_map_reads_current_pattern(self):
        self.wb.connection['state']='ok';self.wb.target.transport='ssh'
        for pattern in ('lawnmower','nested_rectangles'):
            source=self.group['site_configs_by_pattern'][pattern]
            raw=wb_site_sources.reference(source)
            with patch.object(self.wb.board,'run',return_value=(0,yaml.safe_dump(raw))) as run:
                ctx=self.context(pattern)
            self.assertIn(source,run.call_args.args[0]);self.assertEqual(ctx['source'],source)
            self.assertEqual(ctx['site']['flight_area'],raw['flight_area'])
    def test_shiftback_optional_sources_and_server_contract(self):
        import server
        with patch.object(server,'PROFILE_DIR',self.tmp.name):
            wb=server.Workbench(self.config,{'transport':'ssh'})
            wb.connection['state']='ok'
            for pattern in ('lawnmower','nested_rectangles'):
                a,b=self.context(pattern),self.context(pattern,True)
                self.assertNotEqual(a['scope'],b['scope'])
                self.assertIn('shiftback_1m',b['source'])
                self.assertEqual(a['site']['corridor_waypoints'],b['site']['corridor_waypoints'])
                self.assertEqual(a['site']['landing_xy'],b['site']['landing_xy'])
                body=dict(group_id=self.group['id'],mode='flight',real_release=False,confirm='启动试飞',coverage_pattern=pattern,coverage_shiftback=True)
                expected=wb.trial_command(body)['body']
                self.assertIn(b['source'],expected)
                with patch.object(wb.sessions,'open') as opened,self.assertRaises(ValueError):
                    wb.start_trial(dict(body,coverage_shiftback=False,expected_body=expected))
                opened.assert_not_called()
                with patch.object(wb.board,'run',side_effect=AssertionError('No SSH')),patch.object(wb.sessions,'open',return_value=N(snapshot=lambda:{'state':'running'})):
                    result=wb.start_trial(dict(body,expected_body=expected))
                self.assertTrue(result['trial']['coverage_shiftback'])
                self.assertEqual(result['trial']['command'],expected)
    def test_browser_command_and_map_contract(self):
        cases=[]
        for pattern in ('lawnmower','nested_rectangles'):
            for agl in (1.8,1.9):
                for mode,check,real in [('preview',False,False),('preview',True,False),('flight',False,False),('flight',False,True)]:
                    options=dict(coverage_pattern=pattern,coverage_agl=agl,speed_profile='competition')
                    command=wb_board.build_group_command(self.config,self.group,mode,check_config=check,real_release=real,**options)[0]
                    cases.append(dict(options=options,mode=mode,check=check,real=real,expected=command))
        for case in list(cases):
            shifted=copy.deepcopy(case);shifted['options']['coverage_shiftback']=True
            shifted['expected']=wb_board.build_group_command(self.config,self.group,shifted['mode'],check_config=shifted['check'],real_release=shifted['real'],**shifted['options'])[0]
            cases.append(shifted)
        data=dict(group=self.group,cases=cases,contexts=[self.context(p,shifted) for shifted in (False,True) for p in ('lawnmower','nested_rectangles')])
        result=subprocess.run(['node',str(TOOL/'tests/site_selection.test.cjs')],input=json.dumps(data),text=True,capture_output=True,cwd=TOOL)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

if __name__=='__main__':unittest.main(verbosity=2)
