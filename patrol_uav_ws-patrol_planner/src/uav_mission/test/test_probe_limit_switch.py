"""Execute the actual ProbeManager stage methods against a parameter-server double."""
import ast,math,unittest
from pathlib import Path
from types import SimpleNamespace as N
from contextlib import nullcontext

class Stamp:
    def __init__(self,t):self.t=t
    def to_sec(self):return self.t
class Clock:
    t=10.
    @classmethod
    def now(cls):return Stamp(cls.t)
class Base:
    def _height_stage_ready(self,action):return True
    def _readiness(self,now):return True,'ready'
    def _on_timer(self,event):self.parent_timer+=1

class LimitTests(unittest.TestCase):
    def manager(self):
        Clock.t=10.;ns='/navigation_height_constraint';cap='/external_planner_max_command_z';ceil='/ceiling'
        params={ns+'/enabled':True,ns+'/frame_id':'map',cap:2.78,ceil:-.1,
            '~high_view_probe/low_stage_parameters':[dict(name=cap,value=1.63),dict(name=ceil,value=-.1)]}
        missing=object()
        def get(k,default=missing):
            if k in params:return params[k]
            if default is missing:raise KeyError(k)
            return default
        rospy=N(get_param=get,set_param=lambda k,v:params.__setitem__(k,v),Time=Clock)
        p=Path(__file__).resolve().parents[1]/'src/uav_mission/high_view_stage_limits.py'
        tree=ast.parse(p.read_text());cls=next(x for x in tree.body if isinstance(x,ast.ClassDef))
        cls.bases=[ast.Name(id='NavigationMissionManager',ctx=ast.Load())]
        cls.body=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in ('_height_stage_ready','_on_timer')]
        env=dict(rospy=rospy,math=math,NavigationMissionManager=Base,MissionPhase=N(COMPLETE='COMPLETE',ABORTED='ABORTED'))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(p),'exec'),env)
        o=env['HighViewStageMixin']();o._runtime=N(stage='REVISIT',core=N(config=N(mission_frame='map',mission_timeout=600.),mission_id='m',started_at=1.,phase='SEARCH'))
        o._pose=N(header=N(frame_id='map',stamp=Stamp(10.)),pose=N(position=N(z=1.18)))
        o._pose_max_age=.5;o._low_limits_applied=False;o._high_stage_parameters=None;o._probe_limit_switch=None
        o._lock=nullcontext();o.parent_timer=0;o._pending_motion_action=None
        return o,params,ns,cap
    def action(self,seq):return N(command='SEARCH',decision_seq=seq,deadline_at=50.)
    def ack(self,p,ns):p[ns+'/ack']=dict(p[ns+'/request'],stamp=Clock.t)

    def test_low_high_low_each_requires_matching_ack_and_restores_disabled_ceiling(self):
        o,p,ns,cap=self.manager()
        self.assertFalse(o._height_stage_ready(self.action(1)));self.assertEqual(p[cap],1.63)
        self.ack(p,ns);self.assertTrue(o._height_stage_ready(self.action(1)))
        old=p[ns+'/ack']
        o._runtime.stage='RESUME_ASCEND'
        self.assertFalse(o._height_stage_ready(self.action(2)));self.assertEqual(p[cap],2.78)
        self.assertEqual(p['/ceiling'],-.1)
        self.assertEqual(p[ns+'/ack'],old)
        self.ack(p,ns);self.assertTrue(o._height_stage_ready(self.action(2)))
        o._runtime.stage='REVISIT';o._pose.pose.position.z=1.65
        self.assertFalse(o._height_stage_ready(self.action(3)));self.assertEqual(p[cap],2.78)
        o._pose.pose.position.z=1.18
        self.assertFalse(o._height_stage_ready(self.action(3)));self.assertEqual(p[cap],1.63)
        self.ack(p,ns);self.assertTrue(o._height_stage_ready(self.action(3)))

    def test_missing_frame_stale_pose_ack_and_disabled_constraint_cannot_authorize_climb(self):
        o,p,ns,cap=self.manager()
        self.assertFalse(o._height_stage_ready(self.action(1)));self.ack(p,ns)
        self.assertTrue(o._height_stage_ready(self.action(1)))
        o._runtime.stage='RESUME_ASCEND';o._pose.header.frame_id='other'
        self.assertFalse(o._height_stage_ready(self.action(2)))
        o._pose.header.frame_id='map';o._pose.header.stamp=Stamp(9.)
        self.assertFalse(o._height_stage_ready(self.action(2)))
        o._pose.header.stamp=Stamp(10.);p[ns+'/enabled']=False
        self.assertFalse(o._height_stage_ready(self.action(2)))
        p[ns+'/enabled']=True
        self.assertFalse(o._height_stage_ready(self.action(2)))
        self.ack(p,ns);p[ns+'/ack']['frame']='other'
        self.assertFalse(o._height_stage_ready(self.action(2)))
        self.ack(p,ns);self.assertTrue(o._height_stage_ready(self.action(2)))

    def test_missing_cap_cannot_skip_ack_even_when_high_stage_is_cached(self):
        for stage in ('RESUME_ASCEND','RESUME_JOIN'):
            with self.subTest(stage=stage):
                o,p,ns,cap=self.manager()
                o._runtime.stage=stage;o._runtime.resume_attempted=True
                p['~high_view_probe/low_stage_parameters']=[dict(name='/ceiling',value=-.1)]
                # False previously allowed an early cached-high return.
                o._low_limits_applied=False
                self.assertFalse(o._height_stage_ready(self.action(2)))
                self.assertEqual(o._last_reason,'probe_height_limit_parameter_missing')
                self.assertNotIn(ns+'/request',p)
                self.assertNotIn(ns+'/ack',p)
                self.assertEqual(p[cap],2.78)

    def test_resume_join_needs_its_own_ack_after_ascent(self):
        o,p,ns,cap=self.manager()
        self.assertFalse(o._height_stage_ready(self.action(1)));self.ack(p,ns)
        self.assertTrue(o._height_stage_ready(self.action(1)))
        o._runtime.stage='RESUME_ASCEND'
        self.assertFalse(o._height_stage_ready(self.action(2)));self.ack(p,ns)
        self.assertTrue(o._height_stage_ready(self.action(2)))
        o._runtime.stage='RESUME_JOIN'
        self.assertFalse(o._height_stage_ready(self.action(3)))
        self.ack(p,ns);self.assertTrue(o._height_stage_ready(self.action(3)))

    def test_map_wait_deadline_queues_return_through_parent_readiness(self):
        o,p,ns,cap=self.manager();o._runtime.stage='DESCENT_WAIT'
        o._runtime.descent_wait_until=9.;o._runtime.descent_wait_mode='local'
        a=self.action(3)
        def advance(now,mode):
            o._runtime.stage='RETURN_COLUMN';return N(action=a)
        o._runtime._wait_for_descent=advance
        o._on_timer(None)
        self.assertIs(o._pending_motion_action,a)
        self.assertEqual(o.parent_timer,1)

    def test_resume_timeout_abort_is_published_even_when_core_becomes_terminal(self):
        o,p,ns,cap=self.manager()
        o._runtime.stage='RESUME_ASCEND';o._runtime.resume_until=9.
        o._pending_motion_action=self.action(5)
        o._runtime._route_binding_matches=lambda a:True
        o._runtime.route=N(interrupt=lambda seq:None)
        abort=N(command='ABORT')
        def fail(now,reason):
            o._runtime.core.phase='ABORTED'
            return N(action=abort)
        o._runtime._resume_failed=fail
        published=[]
        o._publish_action=published.append
        o._publish_status=lambda **kw:None
        o._on_timer(None)
        self.assertEqual(published,[abort])
        self.assertEqual(o.parent_timer,0)
        self.assertIsNone(o._pending_motion_action)

    def test_terminal_state_does_not_issue_wait_abort_again(self):
        o,p,ns,cap=self.manager();o._runtime.stage='DESCENT_WAIT'
        o._runtime.core.phase='ABORTED';Clock.t=900.
        o._on_timer(None)
        self.assertEqual(o.parent_timer,1)

    def grace_manager(self):
        import test_high_view_full as full
        fixture=full.FullTests();fixture.setUp();fixture.top3();fixture.finish(103.)
        fixture.r.tick(104.,(0.,0.))
        o,p,ns,cap=self.manager();o._runtime=fixture.r
        published=[]
        def publish(action):
            published.append(action)
            if action.command=='ABORT':o._pending_motion_action=None
        o._publish_action=publish;o._publish_status=lambda **kw:None
        return o,fixture,published

    def test_active_map_grace_abort_publishes_before_parent_readiness(self):
        o,f,published=self.grace_manager()
        Clock.t=105.999;o._on_timer(None)
        self.assertEqual(published,[])
        self.assertEqual(o.parent_timer,1)
        Clock.t=106.;o._on_timer(None)
        self.assertEqual([a.command for a in published],['ABORT'])
        self.assertEqual(f.r.failure,'descent_motion_handoff_timeout')
        self.assertEqual(o.parent_timer,1)
        self.assertIsNone(f.r.route.active)

    def test_new_descent_pending_on_pose_or_ack_cannot_leave_old_flight_unbounded(self):
        o,f,published=self.grace_manager()
        f.map(105.);out=f.r.tick(105.,(0.,0.))
        self.assertEqual(f.r.stage,'DESCEND')
        o._pending_motion_action=out.action
        Clock.t=106.;o._on_timer(None)
        self.assertEqual([a.command for a in published],['ABORT'])
        self.assertEqual(f.r.failure,'descent_motion_handoff_timeout')
        self.assertEqual(o.parent_timer,0)

    def test_published_descent_replacement_releases_old_motion_watchdog(self):
        o,f,published=self.grace_manager()
        f.map(105.);f.r.tick(105.,(0.,0.))
        o._pending_motion_action=None
        Clock.t=106.;o._on_timer(None)
        self.assertEqual(published,[])
        self.assertIsNone(f.r.descent_motion_until)
        self.assertEqual(o.parent_timer,1)

    def test_dispatched_resume_budget_expires_even_without_parent_readiness(self):
        import test_high_view_resume as resume
        fixture=resume.ResumeTests();fixture.setUp();fixture.prepare()
        o,p,ns,cap=self.manager();o._runtime=fixture.r
        o._pending_motion_action=None
        fixture.r.resume_until=110.5;fixture.r.grid.stamp=None;Clock.t=110.5
        published=[]
        o._publish_action=published.append;o._publish_status=lambda **kw:None
        o._on_timer(None)
        self.assertEqual([a.command for a in published],['ABORT'])
        self.assertEqual(fixture.r.failure,'resume_active_motion_map_stale')
        self.assertTrue(fixture.r.resume_completed)
        self.assertEqual(o.parent_timer,0)
        self.assertIsNone(fixture.r.route.active)

    def test_fresh_map_return_is_published_at_grace_deadline(self):
        o,f,published=self.grace_manager()
        f.map(106.);f.r.grid.blocked[:]=True
        Clock.t=106.;o._on_timer(None)
        self.assertEqual([a.command for a in published],['SEARCH'])
        self.assertEqual(f.r.stage,'RETURN_COLUMN')
        self.assertEqual((published[0].goal.x,published[0].goal.y),f.r.ascent_xy)
        self.assertIsNone(f.r.descent_motion_until)
        self.assertEqual(o.parent_timer,0)

    def test_fresh_grid_cannot_bypass_parent_map_frame_readiness_on_return(self):
        o,f,published=self.grace_manager()
        f.map(106.);f.r.grid.blocked[:]=True
        o._readiness=lambda now:(False,'map_stale')
        Clock.t=106.;o._on_timer(None)
        self.assertEqual([a.command for a in published],['ABORT'])
        self.assertEqual(f.r.failure,'descent_motion_handoff_timeout')

    def test_fresh_resume_fallback_pending_ack_is_bounded(self):
        import test_high_view_resume as resume
        fixture=resume.ResumeTests();fixture.setUp();fixture.prepare()
        o,p,ns,cap=self.manager();o._runtime=fixture.r
        fixture.r.resume_until=110.5;fixture.r.pose_stamp=110.5;Clock.t=110.5
        published=[]
        def publish(action):
            published.append(action)
            if action.command=='ABORT':o._pending_motion_action=None
        o._publish_action=publish;o._publish_status=lambda **kw:None
        o._on_timer(None)
        self.assertEqual(fixture.r.stage,'LOW_COVERAGE')
        self.assertEqual(o._pending_motion_action.command,'SEARCH')
        self.assertEqual(fixture.r.descent_motion_until,112.5)
        self.assertEqual(published,[])
        Clock.t=112.5;o._on_timer(None)
        self.assertEqual([a.command for a in published],['ABORT'])
        self.assertEqual(fixture.r.failure,'descent_motion_handoff_timeout')

if __name__=='__main__':unittest.main()
