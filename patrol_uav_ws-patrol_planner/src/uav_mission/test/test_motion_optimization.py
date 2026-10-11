import ast
import copy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import yaml
from uav_mission.motion_optimization import MotionOptimization, MovingRecoveryWindow, optimize_post_route
from uav_mission.boundary_revisit import BoundaryRevisit
from uav_mission.corridor_speed import CorridorSpeed, CorridorSpeedConfig
from uav_mission.planner_execution import OdomSample

class MotionTests(unittest.TestCase):
    def test_boundary_far_target_does_not_slow_far_interior(self):
        o=MotionOptimization(enabled=True);b=BoundaryRevisit(True,(-.5,7.95,-5,5))
        self.assertTrue(b.near((7.4,0)))
        self.assertFalse(o.boundary_slow(b,(3.5,0),(7.4,0),True))
        self.assertTrue(o.boundary_slow(b,(6.5,0),(7.4,0),True))
        self.assertTrue(o.boundary_slow(b,(3.5,4.5),(7.4,0),True))
        self.assertTrue(o.boundary_slow(b,(3.5,0),(7.4,0),False))
        self.assertTrue(o.boundary_slow(b,(float('nan'),0),(7.4,0),True))
    def test_boundary_hysteresis(self):
        o=MotionOptimization(enabled=True);b=BoundaryRevisit(True,(-10,10,-10,10))
        x=10-b.margin-(1.2**2/1.2+1.2*.3+.15)-.1
        self.assertFalse(o.boundary_slow(b,(x,0),(0,0),True,False))
        self.assertTrue(o.boundary_slow(b,(x,0),(0,0),True,True))
    def sample(self,t,vz=.4):
        return OdomSample(int(t*1e9),'camera_init',0,0,1.,0,0,vz)
    def test_moving_recovery_requires_ack_delay_and_new_samples(self):
        w=MovingRecoveryWindow();ack=1_000_000_000
        for t in (1.1,1.2,1.5):self.assertFalse(w.update(self.sample(t),ack,int(t*1e9),.6,.6))
        self.assertFalse(w.update(self.sample(1.6),ack,1_600_000_000,.6,.6))
        self.assertFalse(w.update(self.sample(1.6),ack,1_700_000_000,.6,.6))
        self.assertFalse(w.update(self.sample(1.68),ack,1_680_000_000,.6,.6))
        self.assertTrue(w.update(self.sample(1.76),ack,1_760_000_000,.6,.6))
    def test_recovery_gap_downward_or_fast_resets(self):
        w=MovingRecoveryWindow()
        for t,v in ((2.,.4),(2.1,.8),(2.2,.4),(2.3,-.2),(2.4,.4),(2.8,.4)):
            self.assertFalse(w.update(self.sample(t,v),1_000_000_000,int(t*1e9),.6,.6))
        self.assertFalse(w.update(self.sample(2.88),1_000_000_000,2_880_000_000,.6,.6))
        self.assertTrue(w.update(self.sample(2.96),1_000_000_000,2_960_000_000,.6,.6))
    def test_relays_merge_without_erasing_turn_descent_h(self):
        points=[[6.7,4,1.4],[6.7,4,.9],[8.75,4,.9],[8.75,2.3,.9],
                [8.75,.9,.9],[8.75,0,.9],[8.75,-.9,.9],[8.75,-2.3,.9],[8.75,-4.2,1.]]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),8,1,[-1.6,1.6])
        self.assertEqual(out,[points[i] for i in (1,2,3,4,6,7,8)])
        self.assertEqual(meta['retained_original_indices'],[1,2,3,4,6,7,8])
        self.assertEqual(meta['protected_original_indices'],[3,4,6,7])
        self.assertTrue(meta['diagonal_entry']);self.assertEqual(meta['corridor_points_count'],6)
        self.assertEqual(points[0],[6.7,4,1.4])
    def test_field_seven_points_keep_both_doors_with_merge_enabled(self):
        points=[[6.5,2.1,.9]]+[[7.7,y,.9] for y in (2.1,1.,-.7,-1.1,-2.5,-3.45)]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),7,1,[0.,-1.4])
        self.assertEqual(out,points)
        self.assertEqual(meta['protected_original_indices'],[2,3,4,5])
        self.assertEqual([(c['before_original_index'],c['after_original_index'])
                          for c in meta['protected_crossings']],[(2,3),(4,5)])

    def test_reverse_repeated_crossings_protect_each_visit_not_coordinates(self):
        points=[[0,y,.9] for y in (3,2,1,-1,-2,-3,-2,-1,1,2,3)]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),len(points),1,[0.])
        self.assertEqual(meta['protected_original_indices'],[2,3,7,8])
        self.assertEqual(meta['retained_original_indices'],[0,2,3,5,7,8,10])
        self.assertEqual(out,[points[i] for i in (0,2,3,5,7,8,10)])

    def test_multiwall_same_segment_and_reverse_slanted_route(self):
        # Both walls lie in one slanted segment; each direction keeps its anchors.
        points=[[x,2*x,.9] for x in (-3,-2,-1,3,4,5)]
        for route in (points,list(reversed(points))):
            with self.subTest(route=route):
                out,meta=optimize_post_route(route,MotionOptimization(enabled=True),6,0,[0.,2.])
                self.assertEqual(out,[route[i] for i in (0,2,3,5)])
                self.assertEqual(meta['protected_original_indices'],[2,3])
                self.assertEqual(len(meta['protected_crossings']),2)

    def test_on_wall_runs_keep_nearest_strict_side_anchors(self):
        points=[[0,y,.9] for y in (3,2,1,0,0,-1,-2,-3)]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),8,1,[0.])
        self.assertEqual(meta['protected_original_indices'],[2,3,4,5])
        self.assertEqual(meta['protected_crossings'][0]['on_wall_original_indices'],[3,4])
        self.assertEqual(out,[points[i] for i in (0,2,3,4,5,7)])

    def test_wall_touch_is_retained_without_reporting_crossing(self):
        points=[[0,y,.9] for y in (2,1,0,1,2)]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),5,1,[0.])
        self.assertEqual(meta['protected_crossings'],[])
        self.assertEqual(meta['protected_original_indices'],[2])
        self.assertEqual(out,[points[i] for i in (0,2,4)])

    def test_diagonal_entry_on_wall_preserves_explicit_height_change(self):
        points=[[0,0,1.4],[0,0,.9],[0,1,.9],[0,2,1.2]]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),3,1,[0.])
        self.assertEqual(out,points)
        self.assertFalse(meta['diagonal_entry'])

    def test_height_changes_and_h_suffix_remain(self):
        points=[[0,y,z] for y,z in ((4,.9),(3,.9),(2,1.),(1,1.),(-1,1.),(-2,1.2),(-3,1.2))]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),6,1,[0.])
        self.assertEqual(out,points)
        self.assertEqual(meta['corridor_points_count'],6)

    def test_crossing_into_h_suffix_keeps_nearest_pre_wall_anchor(self):
        points=[[0,y,.9] for y in (3,2,1,0,-1,-2)]
        out,meta=optimize_post_route(points,MotionOptimization(enabled=True),4,1,[0.])
        self.assertEqual(out,[points[i] for i in (0,2,3,4,5)])
        self.assertEqual(meta['corridor_points_count'],3)
        self.assertEqual(meta['protected_original_indices'],[2,3,4])

    def test_disabled_preserves_route_and_reversal_not_removed(self):
        points=[[0,0,.9],[0,2,.9],[0,1,.9]]
        self.assertEqual(optimize_post_route(points,MotionOptimization(),3,1,[1.6])[0],points)
        self.assertEqual(optimize_post_route(points,MotionOptimization(enabled=True),3,1,[1.6])[0],points)
    def test_new_corridor_entry_count_still_brakes_at_doors_and_h(self):
        p=CorridorSpeed(CorridorSpeedConfig(entry_waypoints=1))
        self.assertEqual(p.select((8.75,4),(8.75,-4.2),1)[0],'CORRIDOR_OPEN')
        self.assertEqual(p.select((8.75,1.7),(8.75,-4.2),1)[0],'DOOR')
        self.assertEqual(p.select((8.75,-4),(8.75,-4.2),3)[0],'H_APPROACH')
    def test_invalid_options(self):
        for kw in ({'enabled':1},{'recovery_handoff_agl':.5},{'braking_accel_mps2':float('nan')},{'survey_line_weight':100}):
            with self.assertRaises(ValueError): MotionOptimization(**kw)

ROOT=Path(__file__).resolve().parents[4]
@unittest.skipUnless((ROOT/'deployment/competition/candidates/rectangle_motion.yaml').exists(),'integration frontend only')
class CandidateTests(unittest.TestCase):
    def setUp(self):
        from uav_mission.competition_config import generate
        self.generate=generate
        self.s=yaml.safe_load((ROOT/'deployment/competition/candidates/rectangle_motion.yaml').read_text())
        self.s.update(site_confirmed=True,corridor_waypoints=[
            dict(x=6.7,y=4,agl=1.4),dict(x=6.7,y=4,agl=.9),dict(x=8.75,y=4,agl=.9),
            dict(x=8.75,y=2.3,agl=.9),dict(x=8.75,y=.9,agl=.9),dict(x=8.75,y=0,agl=.9),
            dict(x=8.75,y=-.9,agl=.9),dict(x=8.75,y=-2.3,agl=.9)],
            landing_xy=[8.75,-4.2])
        self.rig=yaml.safe_load((ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
    def gen(self,fc_xyz=(0,0,0)):
        with tempfile.TemporaryDirectory() as p:
            ref=self.generate(ROOT,p,self.s,fc_xyz,self.rig)
            data={n:yaml.safe_load((Path(p)/(n+'.yaml')).read_text()) for n in ('runtime','control','overrides')}
        return ref,data
    def test_camera_height_recovery_and_post_entry_caps_wired(self):
        ref,d=self.gen();g=ref['ground_z']
        self.assertAlmostEqual(ref['high_z']-g-.16,2.6)
        self.assertAlmostEqual(d['control']['uav_vision']['recovery_height']-g,.9,places=6)
        self.assertAlmostEqual(d['control']['uav_vision']['standard_recovery_setpoint_height']-g,1.5,places=6)
        self.assertEqual(d['overrides']['/navigation/planner_bridge/target/recovery_height'],
                         d['control']['uav_vision']['recovery_height'])
        stages=d['runtime']['mission']['post_delivery_parameter_stages']
        self.assertAlmostEqual(stages[0]['parameters']['/external_planner_max_command_z']-g,self.s['max_agl'])
        self.assertEqual(stages[1]['after_completed_waypoints'],1)
        self.assertAlmostEqual(stages[1]['parameters']['/external_planner_max_command_z']-g,1.)
        self.assertEqual(d['runtime']['corridor_speed_schedule']['entry_waypoints'],1)
        self.assertEqual(d['runtime']['mission']['post_delivery_route'][0][:2],[6.7,4.])
        self.assertEqual(len(d['runtime']['mission']['post_delivery_route']),8)
        self.assertTrue(d['control']['uav_vision']['require_release_permission'])
    def test_protected_count_drives_h_stage_with_and_without_diagonal_entry(self):
        self.s['landing_capture_agl']=1.2
        for diagonal,count,entry in ((True,6,1),(False,7,2)):
            with self.subTest(diagonal=diagonal):
                self.s['motion_optimization']['diagonal_entry']=diagonal
                ref,d=self.gen((.2,-.2,.1))
                runtime=d['runtime'];meta=runtime['motion_optimization_metadata']
                m=runtime['mission'];stages=m['post_delivery_parameter_stages']
                self.assertEqual(meta['corridor_points_count'],count)
                self.assertEqual(stages[1]['after_completed_waypoints'],entry)
                self.assertEqual(stages[2]['after_completed_waypoints'],count)
                self.assertEqual(m['post_delivery_route'][count:],meta['original'][8:])
                low=ref['ground_z']+self.s['motion_optimization']['corridor_max_agl']
                for completed in range(entry,count):
                    active=[s for s in stages if s['after_completed_waypoints']<=completed][-1]
                    self.assertAlmostEqual(active['parameters']['/external_planner_max_command_z'],low)
                self.assertEqual([s for s in stages if s['after_completed_waypoints']<=count][-1],stages[2])
                self.assertAlmostEqual(stages[2]['parameters']['/external_planner_max_command_z'],
                    max(low,m['post_delivery_route'][-1][2]+.1))

    def test_field_route_generator_keeps_doors_merges_ordinary_relay_and_h_stage(self):
        self.s['corridor_waypoints']=[dict(x=6.5,y=2.1,agl=.9)]+[
            dict(x=7.7,y=y,agl=.9) for y in (2.1,1.,-.7,-1.1,-2.5,-3.,-3.45)]
        self.s['landing_xy']=[7.7,-3.45]
        self.s['corridor_speed_schedule']['wall_coordinates']=[0.,-1.4]
        for fc in ((0,0,0),(.2,-.2,.1)):
            with self.subTest(fc=fc):
                ref,d=self.gen(fc);r=d['runtime'];m=r['mission'];meta=r['motion_optimization_metadata']
                self.assertTrue(r['motion_optimization']['merge_collinear_relays'])
                self.assertEqual(meta['protected_original_indices'],[2,3,4,5])
                self.assertEqual(meta['corridor_points_count'],7)
                self.assertEqual(meta['retained_original_indices'][:7],[0,1,2,3,4,5,7])
                self.assertEqual(m['post_delivery_route'][7:],meta['original'][8:])
                self.assertEqual(m['post_delivery_parameter_stages'][-1]['after_completed_waypoints'],7)
                self.assertEqual(r['corridor_speed_schedule']['wall_coordinates'],[fc[1],fc[1]-1.4])
                self.assertAlmostEqual(m['post_delivery_parameter_stages'][-1]['parameters'][
                    '/external_planner_max_command_z'],max(ref['ground_z']+1.,m['post_delivery_route'][-1][2]+.1))

    def test_turn_off_diagonal_retains_height_stop(self):
        self.s['motion_optimization']['diagonal_entry']=False
        _,d=self.gen()
        self.assertEqual(d['runtime']['corridor_speed_schedule']['entry_waypoints'],2)
        self.assertEqual(d['runtime']['mission']['post_delivery_parameter_stages'][1]['after_completed_waypoints'],2)
    def test_reject_height_mismatch_unmeasured_and_invalid_recovery(self):
        for key,val in [('survey_camera_agl',2.0),('site_confirmed',False)]:
            old=self.s[key];self.s[key]=val
            with self.assertRaises(ValueError):self.gen()
            self.s[key]=old
        self.s['drop_agl']=.8
        with self.assertRaises(ValueError):self.gen()
    def test_reject_high_corridor_and_entrance_in_door_zone(self):
        self.s['corridor_waypoints'][3]['agl']=1.4
        with self.assertRaises(ValueError):self.gen()
        self.s['corridor_waypoints'][3]['agl']=.9
        self.s['corridor_waypoints'][0]['y']=1.7;self.s['corridor_waypoints'][1]['y']=1.7
        with self.assertRaises(ValueError):self.gen()

class BridgeMovingRecoveryTests(unittest.TestCase):
    def make_bridge(self, enabled=True):
        tree=ast.parse((ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_update_recovery')
        ns=dict(MotionOptimization=MotionOptimization)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'bridge recovery','exec'),ns)
        events=[]
        obj=SimpleNamespace(_transaction=SimpleNamespace(phase='RECOVERY',release_ack_ns=1_000_000_000),
            _motion_options=MotionOptimization(enabled=enabled),_moving_recovery=MovingRecoveryWindow(),
            _odom_velocity_available=True,
            _recovery_height=.68,_control_state=1,_control_state_receipt_ns=1_500_000_000,
            _align_mode='disabled',_align_mode_receipt_ns=1_500_000_000,
            _odom_rejection_reason=lambda s,n: '' if 0<=n-s.stamp_ns<=200_000_000 else 'stale',
            _recovery_settle=SimpleNamespace(reset=lambda *a:None,update=lambda *a:SimpleNamespace(ready=False)),
            _report_target_stage=lambda *a,**kw:events.append((a,kw)))
        return obj,events,ns['_update_recovery']
    def sample(self,t):
        return OdomSample(int(t*1e9),'camera_init',0,0,.8,0,0,.4)
    def test_handoff_while_rising_only_in_opt_in_mode(self):
        for enabled in (False,True):
            o,events,call=self.make_bridge(enabled)
            call(o,self.sample(2),2_000_000_000);call(o,self.sample(2.08),2_080_000_000);call(o,self.sample(2.16),2_160_000_000)
            self.assertEqual(len(events),int(enabled))
            if enabled:self.assertEqual(events[0][1]['reason'],'release_recovery_motion_handoff')
    def test_missing_raw_lio_velocity_cannot_use_moving_recovery(self):
        o,events,call=self.make_bridge()
        o._odom_velocity_available=False
        for t in (2.,2.08,2.16):
            call(o,self.sample(t),int(t*1e9))
        self.assertEqual(events,[])
        self.assertEqual(o._moving_recovery.count,0)

    def test_control_handoff_freshness_and_altitude_still_required(self):
        for attr,value in [('_control_state',2),('_control_state_receipt_ns',500_000_000),
                           ('_align_mode','drop_circle'),('_align_mode_receipt_ns',500_000_000),
                           ('_recovery_height',1.)]:
            o,events,call=self.make_bridge();setattr(o,attr,value)
            call(o,self.sample(2),2_000_000_000);call(o,self.sample(2.08),2_080_000_000);call(o,self.sample(2.16),2_160_000_000)
            self.assertEqual(events,[],attr)
        o,events,call=self.make_bridge()
        call(o,self.sample(2),2_000_000_000);call(o,self.sample(2.16),2_600_000_000)
        self.assertEqual(events,[])


class ManagerMotionTests(unittest.TestCase):
    def test_actual_speed_helper_preserves_global_line_cost_and_switches_boundary_speed(self):
        from uav_mission.execution_speed import FollowingSpeed
        source=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py'
        method=next(n for n in ast.walk(ast.parse(source.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_apply_following_speed')
        values={'/fast_planner_node/search/line_deviation_weight':2.,'~motion_optimization':{'enabled':True},
                '~following_speed_profile':{'cruise_lead_m':1.}}
        class Clock:
            def to_sec(self):return 100.
            def __sub__(self,other):return SimpleNamespace(to_sec=lambda:.1)
        ros=SimpleNamespace(get_param=lambda k,d=None:values.get(k,d),set_param=lambda k,v:values.update({k:v}),
                            Time=SimpleNamespace(now=lambda:Clock()),loginfo=lambda *a:None)
        ns=dict(rospy=ros,FollowingSpeed=FollowingSpeed,MotionOptimization=MotionOptimization)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'manager speed','exec'),ns)
        rt=SimpleNamespace(stage='SURVEY',ascent_verified=True,
            boundary_policy=BoundaryRevisit(True,(-.5,7.95,-5,5)),core=SimpleNamespace(post_delivery_route_index=0))
        xy=[3.5,0]
        o=SimpleNamespace(_runtime=rt,_pose=SimpleNamespace(header=SimpleNamespace(stamp=Clock())),
            _pose_max_age=.5,_current_xy=lambda:xy)
        a=SimpleNamespace(command='SEARCH',reason='survey',goal=SimpleNamespace(x=7.4,y=0))
        call=ns['_apply_following_speed'];call(o,a)
        self.assertEqual(values['/fast_planner_node/search/line_deviation_weight'],2.)
        rt.stage='REVISIT';call(o,a)
        self.assertEqual(values['/fast_planner_node/search/line_deviation_weight'],2.)
        self.assertEqual(o._following_speed_state,('CRUISE',1.))
        xy[:]=[7.,0];call(o,a)
        self.assertEqual(o._following_speed_state,('BOUNDARY_REVISIT',.2))
        a.command='LAND';call(o,a)
        self.assertEqual(o._following_speed_state[0],'TERMINAL')

if __name__=='__main__':unittest.main()
