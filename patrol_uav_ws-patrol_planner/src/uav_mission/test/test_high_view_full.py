from dataclasses import replace
import unittest
import numpy as np
from uav_high_view.grid_cost import GridCost
from uav_mission.high_view_probe import ProbeConfig
from uav_mission.high_view_full import HighViewFull
from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint
from uav_mission.mission_core import MissionCore,MissionConfig,GoalSnapshot,MissionPhase,validate_candidate
from test_mission_runtime import profile,candidate,result_for,release_ack


class FullTests(unittest.TestCase):
    def test_deferred_target_does_not_discard_other_hints_without_coarse_order(self):
        self.to_capture();first=self.r.selected.class_name
        self.r.grid.stamp=None
        out=self.r.tick(self.r.wait_until+.1,self.r.selected.xy)
        self.assertEqual(self.r.stage,'REVISIT')
        self.assertNotEqual(self.r.selected.class_name,first)
        self.assertEqual(out.action.command,'SEARCH')
        self.assertFalse(out.action.has_target)
        self.assertEqual(self.r.orders[-1]['scope'],'COARSE_ORDER_UNAVAILABLE_REQUIRES_3D_PLANNER')
        self.assertEqual(self.r.core.committed_slots,0)

    def test_timeout_delivers_other_hints_before_retrying_deferred_target(self):
        self.to_capture();deferred=self.r.selected.class_name
        now=self.r.wait_until+.1;self.map(now)
        out=self.r.tick(now,self.r.selected.xy)
        self.assertEqual(self.r.stage,'REVISIT')
        self.assertNotEqual(self.r.selected.class_name,deferred)
        self.assertIsNone(self.r.fallback_started)
        self.finish(now+1);now+=2
        delivered=[]
        for slot in range(1,4):
            h=self.r.selected;delivered.append(h.class_name)
            self.r.update_pose((*h.xy,1.18),now,'camera_init')
            self.r.ingest([candidate(target_id=h.key.target_id,class_name=h.class_name,now=now,x=h.xy[0],y=h.xy[1])],now)
            action=self.r.tick(now+.1,h.xy).action
            self.assertEqual(action.payload_slot,slot)
            self.seq+=1
            self.r.apply_result(replace(release_ack(action,self.seq),event_stamp_ns=int((now+1)*1e9)),now+1,h.xy)
            self.seq+=1;self.map(now+2)
            out=self.r.apply_result(replace(result_for(action,self.seq,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=int((now+2)*1e9)),now+2,h.xy)
            if slot<3:self.finish(now+5);now+=10
        self.assertEqual(delivered[-1],deferred)
        self.assertEqual(len(set(delivered)),3)
        self.assertEqual(self.r.revisit_counts[deferred],2)
        self.assertIsNone(self.r.fallback_started)
        self.assertEqual(out.action.command,'RETURN_HOME')
        self.assertEqual(self.r.core.started_at,100.)

    def test_all_reacquisition_failures_are_bounded_before_coverage(self):
        self.to_capture();seen=[]
        for i in range(6):
            seen.append(self.r.selected.class_name)
            now=self.r.wait_until+.1;self.map(now)
            out=self.r.tick(now,self.r.selected.xy)
            if i<5:
                self.assertEqual(self.r.stage,'REVISIT')
                self.finish(now+1)
        self.assertEqual(len(set(seen[:3])),3)
        self.assertEqual(self.r.revisit_counts,dict.fromkeys(self.r.required,2))
        self.assertEqual(self.r.stage,'LOW_COVERAGE')
        self.assertEqual(self.r.core.committed_slots,0)
        self.assertEqual(self.r.core.started_at,100.)
        self.assertLessEqual(out.action.deadline_at,700.)

    def test_unreachable_revisit_uses_other_known_hint(self):
        self.to_capture();now=self.r.wait_until+.1;self.map(now)
        self.r.tick(now,self.r.selected.xy)
        failed_class=self.r.selected.class_name;action=self.r.core.active_action
        self.seq+=1;self.map(now+1)
        out=self.r.apply_result(replace(result_for(action,self.seq,status='FAILED',terminal=True),event_stamp_ns=int((now+1)*1e9)),now+1,self.r._current_xy)
        self.assertEqual(out.action.command,'SEARCH')
        self.assertNotEqual(self.r.selected.class_name,failed_class)
        self.assertIsNone(self.r.fallback_started)

    def test_coverage_rechecks_queued_candidate_boundary_before_reserving(self):
        from uav_mission.boundary_revisit import BoundaryRevisit
        self.finish(103.);active=self.r.core.active_action
        self.r.route.interrupt(active.decision_seq);self.r.core.active_action=None
        self.r.top_hints={};self.r._next_target(104.)
        self.r.ingest([candidate(target_id=5,class_name='red_cross',now=105.,x=4.6,y=3.)],105.)
        self.r.boundary_policy=BoundaryRevisit(enabled=True)
        out=self.r.tick(105.1,(0.,1.))
        self.assertIsNone(out.action)
        self.assertEqual(self.r.core.active_action.command,'SEARCH')
        self.assertTrue(all(s.candidate_key is None for s in self.r.core.slots))
        self.assertGreater(self.r.boundary_rejections,0)
        self.r.ingest([candidate(target_id=5,class_name='red_cross',now=106.,x=4.4,y=3.)],106.)
        out=self.r.tick(106.1,(0.,1.))
        self.assertEqual(out.action.command,'APPROACH')
        self.assertEqual(out.action.target_class,'red_cross')
        self.assertEqual(out.action.payload_slot,1)

    def test_coverage_rejection_does_not_starve_an_allowed_target(self):
        from uav_mission.boundary_revisit import BoundaryRevisit
        self.finish(103.);active=self.r.core.active_action
        self.r.route.interrupt(active.decision_seq);self.r.core.active_action=None
        self.r.top_hints={};self.r._next_target(104.)
        self.r.boundary_policy=BoundaryRevisit(enabled=True)
        self.r.ingest([candidate(target_id=5,class_name='red_cross',now=105.,x=4.6,y=3.),candidate(target_id=6,class_name='bridge',now=105.,x=1.,y=1.)],105.)
        out=self.r.tick(105.1,(0.,1.))
        self.assertEqual(out.action.target_class,'bridge')
        self.assertEqual(out.action.payload_slot,1)

    def test_boundary_hint_changes_viewpoint_not_target_identity(self):
        from uav_mission.boundary_revisit import BoundaryRevisit
        self.to_capture()
        self.r.boundary_policy=BoundaryRevisit(enabled=True)
        self.r.core.queue.delivered_classes={'bridge','panzer'}
        h=self.r.top_hints['red_cross']
        self.r.top_hints['red_cross']=replace(h,xy=(4.5,3.),uncertainty_m=.3)
        out=self.r._next_target(114.)
        self.assertLess(out.action.goal.x,4.2)
        self.assertEqual(self.r.selected.xy,(4.5,3.))
        self.assertFalse(out.action.has_target)
        self.assertEqual(self.r.core.committed_slots,0)

    def test_fresh_candidate_outside_boundary_cannot_start_delivery(self):
        from uav_mission.boundary_revisit import BoundaryRevisit
        self.to_capture();self.r.boundary_policy=BoundaryRevisit(enabled=True)
        h=self.r.selected
        self.r.fresh_candidate=candidate(target_id=h.key.target_id,class_name=h.class_name,now=114.,x=4.6,y=3.)
        self.r.reacquired={'target_id':h.key.target_id}
        out=self.r.tick(114.1,(4.,3.))
        self.assertIsNone(out.action)
        self.assertEqual(self.r.core.committed_slots,0)
        self.assertEqual(self.r.boundary_rejections,1)

    def setUp(self):
        cfg=MissionConfig(early_return_enabled=False,post_delivery_route=(GoalSnapshot('camera_init',-3.,6.,1.18),),landing_xy=(-3.,6.))
        self.r=HighViewFull(MissionCore(profile(),cfg),ProbeConfig(-.22,((1.,1.),(2.,1.))),
                            fallback_route=CoverageRoute((Waypoint(0.,1.,1.18),Waypoint(2.,1.,1.18)),'test-low'))
        self.r.start('mission-runtime',100.,(0.,0.));self.seq=0

    def finish(self,now):
        a=self.r.core.active_action;self.seq+=1
        return self.r.apply_result(replace(result_for(a,self.seq,status='SUCCEEDED',terminal=True),event_stamp_ns=int((now-.01)*1e9)),now,(a.goal.x,a.goal.y))

    def top3(self):
        for t in [101.,101.3,101.6]:
            self.r.update_pose((0.,0.,2.38),t,'camera_init')
            cs=[replace(candidate(target_id=i,class_name=c,now=t,x=float(i),y=1.),first_seen_ns=99_000_000_000)
                for i,c in enumerate(['bridge','panzer','red_cross'])]
            self.r.ingest(cs,t)

    def map(self,t):
        # Nonempty observed floor; coarse cost remains optimistic, not an approval.
        self.r.grid.update(np.array([[0.,0.,-.22]]),t,.18,2.8)

    def to_capture(self):
        self.top3();self.finish(103.)
        self.map(104.)
        out=self.r.tick(104.,(0.,0.))
        self.assertEqual(self.r.stage,'DESCEND')
        self.assertEqual((out.action.goal.x,out.action.goal.y),(0.,0.))
        self.map(110.);self.finish(110.)
        self.finish(113.);self.assertEqual(self.r.stage,'REACQUIRE')

    def test_no_fixed_45_second_cutoff(self):
        self.assertEqual(self.r.survey_until,700.)
        self.assertGreater(self.r.core.active_action.deadline_at,145.)

    def test_interrupt_requires_all_three_coordinates(self):
        self.top3();self.finish(103.);old=self.r.core.active_action
        self.map(104.)
        self.r.tick(104.,(0.,0.))
        self.assertGreater(self.r.core.active_action.decision_seq,old.decision_seq)
        self.assertFalse(self.r.core.active_action.has_target)
        self.assertEqual(set(self.r.top_hints),{'bridge','panzer','red_cross'})

    def test_missing_top3_returns_verified_column_then_low_coverage(self):
        self.finish(103.);self.finish(106.);self.finish(109.)
        self.assertFalse(self.r.done);self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.finish(112.);self.finish(115.)
        self.assertEqual(self.r.stage,'LOW_COVERAGE')
        self.assertEqual(self.r.core.started_at,100.)
        self.assertEqual(self.r.core.committed_slots,0)

    def test_confirmed_hint_survives_short_window_reset(self):
        self.top3()
        stamp=self.r._all_top(101.6)['red_cross'].last_seen_ns
        self.r.update_pose((0.,0.,2.38),104.,'camera_init')
        c=replace(candidate(target_id=2,class_name='red_cross',now=104.,x=2.,y=1.),first_seen_ns=99_000_000_000)
        self.r.ingest([c],104.)
        self.assertNotIn('red_cross',[h.class_name for h in self.r.catalog.hints(104_000_000_000)])
        self.assertIn('red_cross',self.r._all_top(104.))
        self.assertEqual(self.r._all_top(104.)['red_cross'].last_seen_ns,stamp)

    def test_memory_conflict_and_epoch_invalidation(self):
        self.top3();h=self.r._all_top(101.6)['red_cross']
        found=self.r.memory.update([replace(h,xy=(4.,4.))],h.epoch,102_000_000_000)
        self.assertNotIn('red_cross',found)
        self.assertEqual(self.r.memory.update([],None,103_000_000_000),{})
        self.assertEqual(self.r.memory.saved,{})

    def test_partial_hints_descend_without_inventing_missing_target(self):
        for t in [101.,101.3,101.6]:
            self.r.update_pose((0.,0.,2.38),t,'camera_init')
            self.r.ingest([replace(candidate(target_id=i,class_name=c,now=t,x=float(i),y=1.),first_seen_ns=99_000_000_000)
                           for i,c in enumerate(['bridge','red_cross'])],t)
        self.finish(103.);self.finish(106.);self.map(109.);self.finish(109.)
        self.assertEqual(self.r.stage,'DESCEND')
        self.assertEqual(set(self.r.top_hints),{'bridge','red_cross'})
        self.assertEqual(self.r.core.committed_slots,0)

    def test_high_stall_skips_with_no_map_and_rejects_late_success(self):
        self.finish(103.);old=self.r.core.active_action
        self.r.tick(104.,(0.,0.));self.r.tick(112.1,(0.,0.))
        self.assertFalse(self.r.done)
        self.assertNotEqual(self.r.core.active_action.decision_seq,old.decision_seq)
        self.assertEqual(self.r.core.active_action.goal.x,2.)
        out=self.r.apply_result(replace(result_for(old,999,status='SUCCEEDED',terminal=True),event_stamp_ns=112_000_000_000),112.2,(0.,0.))
        self.assertFalse(out.accepted)

    def test_failed_survey_gets_only_one_alternative(self):
        self.finish(103.);self.map(104.);self.r.tick(104.,(0.,0.))
        self.map(112.1);self.r.tick(112.1,(0.,0.))
        self.assertTrue(self.r._alternative)
        self.r.tick(113.,(0.,0.));self.map(122.);self.r.tick(122.,(0.,0.))
        self.assertFalse(self.r._alternative)
        self.assertEqual(self.r.core.active_action.goal.x,2.)

    def test_fallback_retains_delivered_state_and_uses_original_selection(self):
        self.finish(103.)
        active=self.r.core.active_action
        self.r.route.interrupt(active.decision_seq);self.r.core.active_action=None
        self.r.core.queue.delivered_classes={'bridge','red_cross'}
        self.r.top_hints={};self.r._next_target(104.)
        self.assertEqual(self.r.stage,'LOW_COVERAGE')
        self.r.ingest([candidate(target_id=5,class_name='panzer',now=105.,x=1.,y=1.)],105.)
        out=self.r.tick(105.1,(0.,1.))
        self.assertEqual(out.action.command,'APPROACH')
        self.assertEqual(out.action.target_class,'panzer')
        self.assertEqual(self.r.core.queue.delivered_classes,{'bridge','red_cross'})

    def test_fresh_capture_enters_original_delivery(self):
        self.to_capture();h=self.r.selected
        self.r.update_pose((*h.xy,1.18),114.,'camera_init')
        self.r.ingest([candidate(target_id=h.key.target_id,class_name=h.class_name,now=114.,x=h.xy[0],y=h.xy[1])],114.)
        out=self.r.tick(114.1,h.xy)
        self.assertEqual(out.action.command,'APPROACH');self.assertTrue(out.action.has_target)
        self.assertFalse(self.r.done)

    def test_three_releases_continue_original_corridor_and_land(self):
        self.to_capture();now=114.
        for slot in range(1,4):
            h=self.r.selected
            self.r.update_pose((*h.xy,1.18),now,'camera_init')
            self.r.ingest([candidate(target_id=h.key.target_id,class_name=h.class_name,now=now,x=h.xy[0],y=h.xy[1])],now)
            out=self.r.tick(now+.1,h.xy);action=out.action
            self.assertEqual(action.payload_slot,slot)
            self.seq+=1
            self.r.apply_result(replace(release_ack(action,self.seq),event_stamp_ns=int((now+1)*1e9)),now+1,h.xy)
            self.seq+=1;self.map(now+2)
            out=self.r.apply_result(replace(result_for(action,self.seq,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=int((now+2)*1e9)),now+2,h.xy)
            if slot<3:
                self.assertEqual(out.action.command,'SEARCH')
                self.finish(now+5);now+=10
            else:
                self.assertEqual(out.action.command,'RETURN_HOME')
                self.assertEqual(self.r.core.phase,MissionPhase.POST_DELIVERY_ROUTE)
        self.assertEqual(self.r.core.committed_slots,3)

    def test_no_map_cost_no_route_guess(self):
        self.top3();self.finish(103.);self.r.tick(104.,(0.,0.))
        self.assertEqual(self.r.failure,'no_local_descent_route')

    def test_blocked_terminal_does_not_prevent_clear_local_descent(self):
        self.top3();self.finish(103.);self.map(104.)
        self.r.grid.blocked[self.r.grid.cell((-3.,6.))]=True
        self.r.tick(104.,(0.,0.))
        self.assertEqual(self.r.stage,'DESCEND')
        self.assertTrue(self.r.descent_debug['exit_blocked'])
        self.map(110.);self.r.grid.blocked[self.r.grid.cell((-3.,6.))]=True
        out=self.finish(110.)
        self.assertEqual(self.r.stage,'REVISIT')
        self.assertEqual(out.action.command,'SEARCH')
        self.assertIn('ONE_REACHABLE_HINT',self.r.orders[-1]['scope'])

    def test_column_proposal_never_uses_stale_or_fully_blocked_map(self):
        from uav_high_view.local_descent import propose_column
        self.assertIsNone(propose_column(self.r.grid,(0.,0.),104.))
        self.map(104.);self.r.grid.blocked[:]=True
        self.assertIsNone(propose_column(self.r.grid,(0.,0.),104.))
        self.r.grid.blocked[:]=False
        self.assertIsNone(propose_column(self.r.grid,(0.,0.),107.))

    def test_high_hint_admission_is_not_low_delivery_admission(self):
        for t in [101.,101.1,101.2]:
            self.r.update_pose((0.,0.,2.38),t,'camera_init')
            c=replace(candidate(target_id=1,class_name='bridge',now=t,x=1.+t-101.,y=1.),consecutive_observe_count=1,first_seen_ns=99_000_000_000)
            self.r.ingest([c],t)
        self.assertIn('bridge',self.r._all_top(101.2))
        self.assertGreater(self.r._all_top(101.2)['bridge'].uncertainty_m,.25)
        self.assertEqual(self.r.core.config.min_streak,3)
        self.r.stage='REACQUIRE'
        self.assertEqual(self.r._candidate_validation_config().min_streak,3)
        self.assertEqual(validate_candidate(c,101.2,self.r.core.profile,self.r.core.config).reason,'streak_too_short')

    def test_single_remaining_target_uses_actual_planner(self):
        self.to_capture()
        self.r.core.queue.delivered_classes={'panzer','red_cross'}
        self.r.stage='DELIVERY';self.r.grid.stamp=None
        out=self.r._next_target(114.)
        self.assertEqual(out.action.command,'SEARCH')
        self.assertIsNone(self.r.orders[-1]['grid_length_m'])
        self.assertEqual(self.r.selected.class_name,'bridge')

    def test_old_candidate_cannot_be_released(self):
        self.to_capture();self.r.update_pose((0.,1.,1.18),114.,'camera_init')
        h=self.r.selected
        self.r.ingest([candidate(class_name=h.class_name,now=105.,x=h.xy[0],y=h.xy[1])],114.)
        out=self.r.tick(114.1,h.xy)
        self.assertIsNone(out.action);self.assertEqual(self.r.core.committed_slots,0)


class GridTests(unittest.TestCase):
    def test_wall_requires_detour(self):
        grid=GridCost((0,5,0,5),.25,0.)
        grid.update(np.array([[2.,y,1.] for y in np.arange(0,4,.25)]),10.,.5,2.)
        direct=grid.distances((1.,1.)).get(grid.cell((3.,1.)))
        self.assertGreater(direct,6.)

    def test_enumeration_includes_terminal(self):
        grid=GridCost((0,5,0,5),.25,0.)
        grid.update(np.array([[0.,0.,0.]]),10.,.5,2.)
        answer=grid.order((.5,.5),{'A':(1.,.5),'B':(3.,.5),'C':(4.,.5)},(4.5,.5),11.)
        self.assertEqual(answer[1],('A','B','C'))
        self.assertIsNone(grid.order((.5,.5),{'A':(1.,.5)},(4.5,.5),20.))

    def test_empty_map_is_not_free_space(self):
        grid=GridCost();grid.update(np.empty((0,3)),10.,0.,3.)
        self.assertIsNone(grid.order((0,0),{'A':(1,1)},(2,2),11.))


if __name__=='__main__':unittest.main()
