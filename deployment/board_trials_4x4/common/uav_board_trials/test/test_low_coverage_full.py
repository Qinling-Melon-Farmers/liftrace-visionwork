"""Low-coverage adapter contracts and real MissionRuntime transaction sequences, no ROS."""
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import ast,copy,json,math,os,subprocess,sys,tempfile,unittest,yaml
from trial_config import load_trial_settings,apply_site_profile,validate_settings,generate,mapping_profile
from trial_low_coverage import inherit_full_settings,normalize_coverage_site,prepare_low_coverage,coverage_waypoints
from trial_speed_profiles import apply_speed_profile
from trial_result import evaluate
from uav_mission.coverage_route import CoverageRoute
from uav_mission.mission_core import MissionCore,MissionPhase,GoalSnapshot
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.search_types import Waypoint
from test_mission_runtime import profile,candidate,result_for,release_ack
from test_trials import config

ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
FOLDER='10_low_coverage_full'
SCRIPTS=BASE/'common/uav_board_trials/scripts'
RIG=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())


def settings(height=1.8):
    value=load_trial_settings(ROOT,FOLDER)
    apply_site_profile(value,yaml.safe_load((BASE/FOLDER/'site_waypoint_map_20261015.yaml').read_text()))
    prepare_low_coverage(value,site_provided=True,height=height)
    return value


def make_runtime(height=1.8):
    value=settings(height)
    with tempfile.TemporaryDirectory() as out:
        generate(ROOT,out,value,(0.,0.,0.),RIG)
        runtime=yaml.safe_load((Path(out)/'runtime.yaml').read_text())
    mission=runtime['mission']
    cfg=replace(config(),mission_timeout=600.,forced_return_at=510.,
        approach_altitude=mission['approach_altitude'],return_altitude=mission['return_altitude'],
        landing_xy=tuple(mission['landing_xy']),post_delivery_route=tuple(
            GoalSnapshot('camera_init',*point) for point in mission['post_delivery_route']))
    ordinary=MissionRuntime(MissionCore(profile(),cfg),CoverageRoute([Waypoint(.6,0,1.58)],'unused'))
    # Execute the production selector body without importing rospy or starting nodes.
    tree=ast.parse((SCRIPTS/'trial_manager.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='BoardManager')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_new_runtime')
    test_cls=ast.ClassDef(name='Adapter',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],body=[method],decorator_list=[])
    module=ast.fix_missing_locations(ast.Module(body=[test_cls],type_ignores=[]))
    params={'~trial/waypoints':runtime['trial']['waypoints'],
            '~trial/coverage_full_mission':runtime['trial']['coverage_full_mission']}
    namespace=dict(Base=type('Base',(),{'_new_runtime':lambda self:ordinary}),
        rospy=SimpleNamespace(get_param=lambda key,default=None:params.get(key,default)),
        CoverageRoute=CoverageRoute,Waypoint=Waypoint,MissionRuntime=MissionRuntime)
    exec(compile(module,str(SCRIPTS/'trial_manager.py'),'exec'),namespace)
    adapter=namespace['Adapter']();adapter.mode='low_multi_full';adapter._high_stage_parameters=None
    selected=adapter._new_runtime()
    assert type(selected) is MissionRuntime
    return selected


class LowCoverageConfig(unittest.TestCase):
    def test_inherits_current_08_drop_h_and_does_not_copy_05_defaults(self):
        value=settings()
        full=load_trial_settings(ROOT,'08_full_mission')
        for key in full:
            if key.startswith('drop_') or key.startswith('release_') or key.startswith('landing_'):
                if key not in ('drop_agl','landing_xy','landing_capture_agl','landing_transit_agl'):
                    self.assertEqual(value[key],full[key],key)
        self.assertEqual(value['drop_agl'],.25)
        self.assertEqual(value['landing_capture_agl'],1.1)
        self.assertEqual(value['delivery_count'],3)
        self.assertNotIn('resume_survey_enabled',value)
        self.assertTrue(value['auto_start_after_arm'])
    def test_no_geometry_leaks_from_future_08_defaults(self):
        full=load_trial_settings(ROOT,'08_full_mission')
        full.update(flight_area=settings()['flight_area'],search_waypoints_xy=[[.6,0],[1.,0]])
        overlay=yaml.safe_load((BASE/FOLDER/'settings.yaml').read_text())
        value=inherit_full_settings(full,overlay)
        self.assertNotIn('flight_area',value)
        self.assertNotIn('search_waypoints_xy',value)

    def test_no_silent_default_field(self):
        value=load_trial_settings(ROOT,FOLDER)
        with self.assertRaises(ValueError):prepare_low_coverage(value,site_provided=False)
        with self.assertRaises(ValueError):validate_settings(value)
        for missing in ('target_bounds','map_size','search_bounds'):
            value=settings();del value['flight_area'][missing]
            with self.assertRaises(ValueError):validate_settings(value)
        value=settings();value['corridor_waypoints']=[]
        with self.assertRaises(ValueError):validate_settings(value)
    def test_both_heights_full_route_map_caps_and_entity_protection(self):
        for height in (1.8,1.9):
            for speed in ('limited','competition'):
                with self.subTest(height=height,speed=speed),tempfile.TemporaryDirectory() as out:
                    value=settings(height);apply_speed_profile(ROOT,value,'low_coverage_full',speed)
                    ref=generate(ROOT,out,value,(.01,-.01,.02),RIG)
                    runtime=yaml.safe_load((Path(out)/'runtime.yaml').read_text())
                    control=yaml.safe_load((Path(out)/'control.yaml').read_text())
                    overrides=yaml.safe_load((Path(out)/'overrides.yaml').read_text())
                    self.assertEqual(mapping_profile(value),'coverage')
                    mapping=yaml.safe_load((BASE/'common/uav_board_trials/config/mapping_coverage.yaml').read_text())
                    self.assertGreater(mapping['sensor']['max_z'],height)
                    self.assertGreater(mapping['map']['raycast_max_z'],height)
                    self.assertTrue(runtime['motion_optimization']['enabled'])
                    self.assertTrue(runtime['motion_optimization']['moving_recovery'])
                    self.assertEqual(runtime['motion_optimization']['recovery_handoff_agl'],.9)
                    self.assertAlmostEqual(control['uav_vision']['recovery_height']-ref['ground_z'],.9,places=6)
                    self.assertAlmostEqual(overrides['/navigation/planner_bridge/target/recovery_height']-ref['ground_z'],.9,places=6)
                    self.assertAlmostEqual(control['uav_vision']['standard_recovery_setpoint_height']-ref['ground_z'],height+.1,places=6)
                    self.assertAlmostEqual(control['uav_vision']['cross_recovery_setpoint_height']-ref['ground_z'],height+.1,places=6)
                    self.assertTrue(runtime['trial']['coverage_full_mission'])
                    self.assertTrue(runtime['trial']['corridor_after_delivery'])
                    self.assertEqual(runtime['mission']['timeout'],600.)
                    self.assertFalse(runtime['mission']['early_return_enabled'])
                    for point in runtime['trial']['waypoints']:
                        self.assertAlmostEqual(point[2]-ref['ground_z'],height,places=6)
                    for key in ('approach_altitude','return_altitude'):
                        self.assertAlmostEqual(runtime['mission'][key]-ref['ground_z'],height,places=6)
                    self.assertAlmostEqual(control['align_height']-ref['ground_z'],height,places=6)
                    self.assertFalse(overrides['/fast_planner_node/sdf_map/horizontal_avoidance/enabled'])
                    self.assertEqual(overrides['/fast_planner_node/sdf_map/obstacles_inflation'],.25)
                    cap=overrides['/external_planner_max_command_z']
                    self.assertGreater(cap,ref['low_z']+.1)
                    low_stage={v['name']:v['value'] for v in runtime['high_view_probe']['low_stage_parameters']}
                    self.assertGreater(low_stage['/external_planner_max_command_z'],ref['low_z']+.2)
                    self.assertEqual(runtime['mission']['post_delivery_route'][0][:2],[6.91,3.39])
                    self.assertEqual(runtime['mission']['post_delivery_route'][-1][:2],[8.31,-3.51])
                    self.assertAlmostEqual(runtime['mission']['post_delivery_route'][-1][2]-ref['ground_z'],1.1)
                    self.assertEqual(control['external_landing']['handoff_mode'],'POSCTL')
                    self.assertEqual(value['drop_agl'],.25)
                    self.assertAlmostEqual(control['drop_system']['release_setpoint_height']-ref['ground_z'],.25)
                    self.assertEqual(value['release_min_agl'],load_trial_settings(ROOT,'08_full_mission')['release_min_agl'])
                    self.assertEqual(value['release_max_agl'],load_trial_settings(ROOT,'08_full_mission')['release_max_agl'])
    def test_y_sweep_default_and_x_sweep_optional(self):
        value=settings();area=value['flight_area']
        points,source=coverage_waypoints(value,area)
        self.assertEqual(len(points),6)
        self.assertEqual(points[:4],[[1.,-4.1],[1.,4.1],[3.6,4.1],[3.6,-4.1]])
        self.assertEqual(points[-1][0],6.2)
        self.assertTrue(all(abs(points[i+2][0]-points[i][0])<=2.6000001 for i in range(0,len(points)-2,2)))
        value['coverage_scan_axis']='x'
        other,_=coverage_waypoints(value,area)
        self.assertEqual(other[:2],[[.4,-3.5],[6.8,-3.5]])
        self.assertNotEqual(points,other)
    def test_derived_routes_match_coordinated_points_and_lengths(self):
        expected_lawn=[[1.,-4.1],[1.,4.1],[3.6,4.1],[3.6,-4.1],[6.2,-4.1],[6.2,4.1]]
        expected_nested=[[1.,-4.1],[1.,4.1],[6.2,4.1],[6.2,-4.1],[1.,-4.1],
                         [2.8,-4.1],[2.8,-3.2],[2.8,3.2],[4.4,3.2],[4.4,-3.2],[2.8,-3.2]]
        for pattern,expected,length in [('lawnmower',expected_lawn,29.8),('nested_rectangles',expected_nested,45.5)]:
            for height in (1.8,1.9):
                value=settings(height);original=copy.deepcopy(value['flight_area'])
                prepare_low_coverage(value,site_provided=True,pattern=pattern)
                validate_settings(value)
                points,_=coverage_waypoints(value,value['flight_area'])
                self.assertEqual(len(points),len(expected))
                for actual,wanted in zip(points,expected):
                    for a,w in zip(actual,wanted):self.assertAlmostEqual(a,w)
                self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(points,points[1:])),length)
                self.assertEqual(value['flight_area'],original)
                for a,b in zip(points,points[1:]):
                    self.assertTrue(abs(a[0]-b[0])<1e-9 or abs(a[1]-b[1])<1e-9)
        self.assertEqual(settings()['coverage_pattern'],'lawnmower')

    def test_cross_inset_spacing_and_unusable_nested_dimensions(self):
        for inset in (0.,-.1,True,float('nan'),float('inf'),3.2,4.):
            value=settings();value['coverage_cross_inset']=inset
            with self.assertRaises(ValueError):validate_settings(value)
        value=settings();value['coverage_lane_spacing']=3.0
        validate_settings(value)
        for spacing in (3.001,.199,True,float('inf')):
            value=settings();value['coverage_lane_spacing']=spacing
            with self.assertRaises(ValueError):validate_settings(value)
        value=settings();value['coverage_pattern']='nested_rectangles'
        value['flight_area']['search_bounds']=[1.,4.,-2.,2.]
        with self.assertRaisesRegex(ValueError,'unavailable'):validate_settings(value)
        # A caller may supply smaller valid measured rectangles instead of an inverted derived inner box.
        value['coverage_rectangles']=[[1.1,3.9,-1.9,1.9],[1.5,3.5,-1.,1.]]
        validate_settings(value)

    def test_loop_inset_independent_and_site_normalized(self):
        value=settings();value['coverage_pattern']='nested_rectangles'
        baseline=coverage_waypoints(value,value['flight_area'])[0]
        for spacing in (.2,1.8,2.6,3.0):
            value['coverage_lane_spacing']=spacing
            validate_settings(value)
            self.assertEqual(coverage_waypoints(value,value['flight_area'])[0],baseline)
        for bad in (0.,-.1,True,float('nan'),float('inf')):
            value=settings();value['coverage_loop_inset']=bad
            with self.assertRaises(ValueError):validate_settings(value)
        value=settings();apply_site_profile(value,{'coverage_loop_inset':1.7})
        self.assertEqual(value['coverage_loop_inset'],1.7)
        area=value['flight_area']
        raw=dict(flight_bounds=area['center_bounds'],target_bounds=area['target_bounds'],
            coverage_bounds=area['search_bounds'],staging_xy=area['staging_xy'],survey_xy=area['survey_xy'],
            map_size=area['map_size'],corridor_waypoints=value['corridor_waypoints'],landing_xy=value['landing_xy'],
            coverage_loop_inset=1.7)
        self.assertEqual(normalize_coverage_site(raw)['coverage_loop_inset'],1.7)

    def test_nested_rectangles_two_closed_loops_and_selected_height(self):
        # Small synthetic fixture, not a proposed measured route/default.
        boxes=[[1.,6.,-3.,3.],[2.,5.,-2.,2.]]
        for height in (1.8,1.9):
            value=settings(height)
            apply_site_profile(value,dict(coverage_pattern='nested_rectangles',coverage_rectangles=boxes))
            original=copy.deepcopy(value)
            with tempfile.TemporaryDirectory() as out:
                generate(ROOT,out,value,(.1,-.1,0.),RIG)
                actual=yaml.safe_load((Path(out)/'runtime.yaml').read_text())
            self.assertEqual(value,original)
            points=actual['trial']['waypoints']
            self.assertEqual(len(points),11)
            self.assertEqual(points[0],points[4])
            self.assertEqual(points[6],points[10])
            self.assertEqual(points[0][:2],[1.1,-3.1])
            self.assertEqual(points[5][:2],[2.1,-3.1])
            self.assertEqual(points[6][:2],[2.1,-2.1])
            self.assertEqual(actual['trial']['coverage_route_source'],'nested_rectangles(coverage_rectangles)')
            self.assertTrue(all(abs(p[2]+RIG['fc_ground_clearance']-height)<1e-6 for p in points))
            self.assertEqual(actual['mission']['landing_xy'],[8.4,-3.6])
            self.assertTrue(actual['motion_optimization']['moving_recovery'])

    def test_nested_bounds_validation_and_manual_route_priority(self):
        for rectangles in (None,[],[[1,6,-3,3]],[[1,6,-3,3],[0,5,-2,2]],
                           [[1,6,-3,3],[2,5,-2,float('nan')]],
                           [[.1,6,-3,3],[2,5,-2,2]]):
            value=settings();value.update(coverage_pattern='nested_rectangles',coverage_rectangles=rectangles)
            with self.assertRaises(ValueError):validate_settings(value)
        value=settings();value.update(coverage_pattern='nested_rectangles',search_waypoints_xy=[[1.,0.],[2.,1.]])
        self.assertEqual(coverage_waypoints(value,value['flight_area'])[1],'site.search_waypoints_xy')

    def test_nested_rectangles_public_cli_generates_config(self):
        with tempfile.TemporaryDirectory() as out:
            site=yaml.safe_load((BASE/FOLDER/'site_waypoint_map_20261015.yaml').read_text())
            site['coverage_rectangles']=[[1.,6.,-3.,3.],[2.,5.,-2.,2.]]
            path=Path(out)/'site.yaml';path.write_text(yaml.safe_dump(site))
            generated=Path(out)/'generated'
            result=subprocess.run([sys.executable,str(SCRIPTS/'run_trial.py'),'low_coverage_full','preview',
                '--root',str(ROOT),'--site-config',str(path),'--coverage-pattern','nested_rectangles',
                '--check-config','--generate-config',str(generated),'--reference-fc','0','0','0'],text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            response=json.loads(result.stdout.splitlines()[-1])
            self.assertEqual(response['coverage']['pattern'],'nested_rectangles')
            actual=yaml.safe_load((generated/'runtime.yaml').read_text())
            self.assertEqual(len(actual['trial']['waypoints']),11)

    def test_full_mission_025_override_only_relaxes_08(self):
        from uav_mission.flight_settings import validate_patch,apply_patch
        self.assertEqual(validate_patch({'drop_agl':.25},'08_full_mission'),{'drop_agl':.25})
        for bad in (.249,True,float('nan')):
            with self.assertRaises(ValueError):validate_patch({'drop_agl':bad},'08_full_mission')
        for folder in ('01_visual_interrupt','02_high_view_revisit','06_high_priority','competition'):
            with self.assertRaises(ValueError):validate_patch({'drop_agl':.25},folder)
            self.assertEqual(validate_patch({'drop_agl':.35},folder),{'drop_agl':.35})
        for folder in ('05_low_multi','10_low_coverage_full'):
            self.assertEqual(validate_patch({'drop_agl':.15},folder),{'drop_agl':.15})
        full=load_trial_settings(ROOT,'08_full_mission')
        original=copy.deepcopy(full)
        patched=apply_patch(full,{'drop_agl':.25},'08_full_mission')
        self.assertEqual(full,original)
        self.assertEqual(patched['drop_agl'],.25)
        self.assertEqual(patched['release_min_agl'],full['release_min_agl'])
        self.assertEqual(patched['release_max_agl'],full['release_max_agl'])

    def test_full_mission_real_cli_accepts_025_flight_override(self):
        with tempfile.TemporaryDirectory() as out:
            patch=Path(out)/'drop.yaml';patch.write_text('drop_agl: 0.25\n')
            result=subprocess.run([sys.executable,str(SCRIPTS/'run_trial.py'),'full_mission','preview',
                '--root',str(ROOT),'--site-config',str(BASE/'08_full_mission/site_20261007_221730.yaml'),
                '--flight-config',str(patch),'--check-config'],text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout.splitlines()[-1])['drop_agl'],.25)

    def test_manual_route_has_priority_and_is_checked_against_search_area(self):
        value=settings();custom=[[.5,-1.],[1.7,1.],[2.,-1.]]
        value['search_waypoints_xy']=custom
        self.assertEqual(coverage_waypoints(value,value['flight_area']),(custom,'site.search_waypoints_xy'))
        value['search_waypoints_xy']=[[8.3,-3.5],[8.3,1.9]]
        with self.assertRaises(ValueError):validate_settings(value)
    def test_waypoint_map_conversion_preserves_geometry_not_old_search_height(self):
        existing=yaml.safe_load((BASE/FOLDER/'site_waypoint_map_20261015.yaml').read_text())
        source=dict(flight_bounds=existing['flight_area']['center_bounds'],target_bounds=existing['flight_area']['target_bounds'],
            coverage_bounds=existing['flight_area']['search_bounds'],staging_xy=existing['flight_area']['staging_xy'],
            survey_xy=existing['flight_area']['survey_xy'],map_size=existing['flight_area']['map_size'],
            corridor_waypoints=existing['corridor_waypoints'],landing_xy=existing['landing_xy'],
            landing_transit_agl=1.,landing_capture_agl=existing['landing_capture_agl'],low_agl=1.4,high_agl=2.6,
            search_center_bounds=[-.1,7.05,-4.35,4.35],corridor_speed_schedule=dict(axis=1,wall_coordinates=[-1.6,1.6],entry_waypoints=1))
        original=copy.deepcopy(source)
        converted=normalize_coverage_site(source)
        self.assertEqual(source,original)
        self.assertEqual(converted,existing)
        self.assertNotIn('low_agl',converted)
    def test_existing_module_explicitly_resets_coverage_selector(self):
        value=load_trial_settings(ROOT,'05_low_multi')
        apply_site_profile(value,yaml.safe_load((BASE/'05_low_multi/site.yaml').read_text()))
        with tempfile.TemporaryDirectory() as out:
            generate(ROOT,out,value,(0.,0.,0.),RIG)
            actual=yaml.safe_load((Path(out)/'runtime.yaml').read_text())
            self.assertIs(actual['trial']['coverage_full_mission'],False)

    def test_height_switch_scoped_and_no_185_cap_gap(self):
        value=settings(1.9);value['max_agl']=2.0
        with self.assertRaises(ValueError):validate_settings(value)
        value=settings();value['low_agl']=1.85
        with self.assertRaises(ValueError):validate_settings(value)
        other=load_trial_settings(ROOT,'05_low_multi')
        with self.assertRaises(ValueError):prepare_low_coverage(other,site_provided=True,height=1.9)
    def test_cli_offline_generate_and_missing_site_fails_before_ros(self):
        command=[sys.executable,str(SCRIPTS/'run_trial.py'),'low_coverage_full','preview','--root',str(ROOT),'--check-config']
        result=subprocess.run(command,text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr)
        for height in (1.8,1.9):
            with tempfile.TemporaryDirectory() as out:
                result=subprocess.run(command+['--site-config',str(BASE/FOLDER/'site_waypoint_map_20261015.yaml'),
                    '--coverage-agl',str(height),'--generate-config',out,'--reference-fc','0','0','0'],text=True,capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr)
                actual=json.loads(result.stdout.splitlines()[-1])
                self.assertEqual(actual['source'],'generated_runtime')
                self.assertEqual(actual['settings']['low_agl'],height)
                self.assertEqual(actual['settings']['coverage_scan_axis'],'y')
                self.assertEqual(actual['settings']['coverage_lane_spacing'],2.6)
                self.assertEqual(actual['settings']['coverage_loop_inset'],1.8)
                self.assertEqual(actual['settings']['coverage_cross_inset'],.6)


class LowCoverageTransactions(unittest.TestCase):
    def test_single_high_weight_target_interrupts_with_red_cross_priority(self):
        for cls in ('bridge','panzer','red_cross'):
            runtime=make_runtime();runtime.start('mission-runtime',100.,(0.,0.))
            runtime.ingest([candidate(class_name=cls,now=101.,x=1.,y=0.)],101.)
            out=runtime.tick(101.1,(.6,0.))
            self.assertEqual(out.action.command,'APPROACH')
            self.assertEqual(out.action.target_class,cls)
            self.assertEqual(out.action.reason,'high_weight_search_interrupt')
        runtime=make_runtime();runtime.start('mission-runtime',100.,(0.,0.))
        runtime.ingest([candidate(target_id=i+1,class_name=cls,now=101.,x=1.+i,y=0.) for i,cls in enumerate(('bridge','panzer','red_cross'))],101.)
        self.assertEqual(runtime.tick(101.1,(.6,0.)).action.target_class,'red_cross')
    def test_low_weight_and_unconfirmed_do_not_early_interrupt(self):
        for target in (candidate(class_name='pillbox',now=101.),candidate(class_name='tent',now=101.),replace(candidate(now=101.),state=0)):
            runtime=make_runtime();runtime.start('mission-runtime',100.,(0.,0.));runtime.ingest([target],101.)
            self.assertIsNone(runtime.tick(101.1,(0.,0.)).action)
            self.assertEqual(runtime.core.active_action.command,'SEARCH')
    def test_three_releases_recover_then_corridor_and_h(self):
        runtime=make_runtime();runtime.start('mission-runtime',100.,(0.,0.));seq=0
        deadline=runtime.core.started_at+runtime.core.config.mission_timeout
        for i,cls in enumerate(('red_cross','panzer','bridge')):
            now=101.+i*4
            runtime.ingest([candidate(target_id=i+1,class_name=cls,now=now,x=1.+i,y=0.)],now)
            action=runtime.tick(now+.1,(.6,0.)).action
            self.assertEqual(action.command,'APPROACH');index=runtime.route.current_index
            seq+=1;runtime.apply_result(replace(release_ack(action,seq),event_stamp_ns=int((now+1)*1e9)),now+1,(1.+i,0.))
            self.assertNotEqual(runtime.core.phase,MissionPhase.POST_DELIVERY_ROUTE)
            seq+=1;out=runtime.apply_result(replace(result_for(action,seq,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=int((now+2)*1e9)),now+2,(1.+i,0.))
            self.assertEqual(runtime.route.current_index,index)
            self.assertEqual(runtime.core.started_at+runtime.core.config.mission_timeout,deadline)
            if i<2:self.assertEqual(out.action.command,'RESUME')
        self.assertEqual(runtime.core.committed_slots,3)
        self.assertEqual(runtime.core.phase,MissionPhase.POST_DELIVERY_ROUTE)
        now+=3
        for goal in runtime.core.config.post_delivery_route:
            self.assertEqual(out.action.goal,goal);seq+=1
            out=runtime.apply_result(replace(result_for(out.action,seq,status='SUCCEEDED',stage='PLANNER',terminal=True),event_stamp_ns=int(now*1e9)),now,(goal.x,goal.y));now+=1
        self.assertEqual(out.action.command,'LAND')
        self.assertEqual(runtime.core.config.landing_xy,(8.3,-3.5))
    def test_moving_recovery_resumes_at_selected_search_height(self):
        for height in (1.8,1.9):
            runtime=make_runtime(height);runtime.start('mission-runtime',100.,(0.,0.))
            runtime.ingest([candidate(class_name='panzer',now=101.)],101.)
            action=runtime.tick(101.1,(.6,0.)).action
            self.assertAlmostEqual(action.goal.z+RIG['fc_ground_clearance'],height,places=6)
            runtime.apply_result(replace(release_ack(action,1),event_stamp_ns=102000000000),102.,(1.,0.))
            out=runtime.apply_result(replace(result_for(action,2,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=103000000000),103.,(1.,0.))
            self.assertEqual(out.action.command,'RESUME')
            self.assertAlmostEqual(out.action.goal.z+RIG['fc_ground_clearance'],height,places=6)

    def test_failed_target_resumes_same_nominal_cursor(self):
        runtime=make_runtime();runtime.start('mission-runtime',100.,(0.,0.))
        runtime.ingest([candidate(class_name='panzer',now=101.)],101.)
        action=runtime.tick(101.1,(.6,0.)).action
        out=runtime.apply_result(replace(result_for(action,1,status='FAILED',stage='PLANNER',terminal=True,retryable=True),event_stamp_ns=102000000000),102.,(1.,0.))
        self.assertEqual(out.action.command,'RESUME')
        self.assertEqual(runtime.route.current_index,0)
        self.assertEqual(runtime.core.committed_slots,0)
    def test_completed_coverage_uses_existing_lower_weight_fallback(self):
        runtime=make_runtime();runtime.start('mission-runtime',100.,(0.,0.))
        now=101.
        for seq in range(1,len(runtime.route.waypoints)+1):
            action=runtime.core.active_action
            if seq==len(runtime.route.waypoints):
                runtime.ingest([candidate(class_name='pillbox',now=now,x=1.,y=0.)],now)
            out=runtime.apply_result(replace(result_for(action,seq,status='SUCCEEDED',stage='PLANNER',terminal=True),event_stamp_ns=int(now*1e9)),now,(action.goal.x,action.goal.y))
            now+=.1
        self.assertEqual(out.action.command,'APPROACH')
        self.assertEqual(out.action.target_class,'pillbox')
        self.assertEqual(out.action.reason,'coverage_complete_fallback')
        self.assertTrue(runtime.route.is_complete)

    def test_missing_targets_complete_coverage_returns_home_without_fake_failure(self):
        runtime=make_runtime();runtime.start('mission-runtime',100.,(0.,0.))
        now=101.
        for seq in range(1,len(runtime.route.waypoints)+1):
            action=runtime.core.active_action
            out=runtime.apply_result(replace(result_for(action,seq,status='SUCCEEDED',stage='PLANNER',terminal=True),event_stamp_ns=int(now*1e9)),now,(action.goal.x,action.goal.y))
            now+=.1
        self.assertEqual(out.action.command,'RETURN_HOME')
        self.assertEqual(out.action.reason,'coverage_complete_insufficient_candidates')
        self.assertEqual(runtime.core.committed_slots,0)
        self.assertFalse(runtime.core.mission_failed)
        self.assertNotEqual(runtime.core.phase,MissionPhase.POST_DELIVERY_ROUTE)

    def test_backup_flight_override_supports_explicit_025_target(self):
        from uav_mission.flight_settings import apply_patch
        patched=apply_patch(settings(),{'drop_agl':.25},FOLDER)
        self.assertEqual(patched['drop_agl'],.25)
        self.assertEqual(patched['release_min_agl'],settings()['release_min_agl'])
        self.assertEqual(patched['release_max_agl'],settings()['release_max_agl'])

    def test_result_requires_three_releases_and_completed_tail(self):
        args=dict(supervisor=dict(trial='low_coverage_full',end_reason='landed_after_flight'),
            latest=dict(mission=dict(phase='COMPLETE',committed_slots=3,post_delivery_route_complete=True,mission_failed=False)),settings=settings())
        self.assertEqual(evaluate(**args)['status'],'PASS')
        args['latest']['mission']['post_delivery_route_complete']=False
        self.assertEqual(evaluate(**args)['status'],'INCOMPLETE')
        args['latest']['mission']['committed_slots']=2
        self.assertEqual(evaluate(**args)['status'],'INCOMPLETE')

if __name__=='__main__':unittest.main()
