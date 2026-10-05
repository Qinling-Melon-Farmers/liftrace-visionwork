"""Production-runtime tests; no ROS master, simulator or payload hardware."""
from dataclasses import replace
import unittest
from uav_mission.high_view_full import HighViewFull
from uav_mission.search_types import Waypoint
from uav_mission.coverage_route import CoverageRoute
from uav_mission.mission_core import MissionPhase
from uav_high_view.core import Hint,Key
import test_high_view_full as full
from test_mission_runtime import result_for

class ResumeTests(unittest.TestCase):
    setUp=full.FullTests.setUp
    finish=full.FullTests.finish
    map=full.FullTests.map
    def prepare(self):
        self.r.policy=replace(self.r.policy,resume_survey_enabled=True,coarse_enabled=True)
        self.finish(103.)
        self.r._current_xy=(.5,.5)
        self.r._save_remaining_survey(104.)
        a=self.r.core.active_action
        self.r.route.interrupt(a.decision_seq);self.r.core.active_action=None
        self.r.stage='REACQUIRE'
        self.r.pose=(.5,.5,1.18);self.r.pose_stamp=110.
        self.map(110.)
        return self.r._start_fallback(110.,'known_hints_exhausted')

    def test_projection_rejoins_unfinished_segment_with_overlap(self):
        self.r.route=CoverageRoute([Waypoint(3.7,-3.95,1.94),Waypoint(6.4,-3.95,1.94),Waypoint(6.4,4.1,1.94)],'partial')
        self.r._survey_leg_start=(3.7,4.1);self.r._current_xy=(3.9,0.)
        self.r._save_remaining_survey(104.)
        self.assertAlmostEqual(self.r.survey_breakpoint[0],3.7)
        self.assertAlmostEqual(self.r.survey_breakpoint[1],.3)
        self.assertEqual(len(self.r.remaining_survey),3)
        self.assertEqual(self.r.remaining_survey[-1].y,4.1)
        self.assertEqual(self.r.core.started_at,100.)

    def test_failed_alternative_is_excluded_from_high_rescan(self):
        self.r._alternative=True;self.r._survey_original=(1.,1.)
        self.r.route=CoverageRoute([Waypoint(1.2,1.,2.38),Waypoint(2.,1.,2.38)],'alternative')
        self.r._current_xy=(.5,.5)
        self.r._save_remaining_survey(104.)
        self.assertIn((1.,1.),self.r.skipped_survey_xy)
        self.assertEqual(len(self.r.remaining_survey),1)
        self.assertEqual(self.r.remaining_survey[0].x,2.)

    def test_resume_climbs_then_rejoins_then_scans_not_diagonal_shortcut(self):
        out=self.prepare()
        self.assertEqual(self.r.stage,'RESUME_ASCEND')
        self.assertEqual((out.action.goal.x,out.action.goal.y),(.5,.5))
        self.assertEqual(out.action.goal.z,2.38)
        self.r.pose=(.5,.5,2.38);self.r.pose_stamp=111.;self.map(111.)
        out=self.finish(111.)
        self.assertEqual(self.r.stage,'RESUME_JOIN')
        self.assertEqual((out.action.goal.x,out.action.goal.y),self.r.survey_breakpoint)
        self.map(112.);out=self.finish(112.)
        self.assertEqual(self.r.stage,'SURVEY')
        self.assertEqual((out.action.goal.x,out.action.goal.y),(1.,1.))
        self.assertEqual(self.r.resume_started,112.)
        self.assertEqual(self.r.core.started_at,100.)
        self.assertLessEqual(out.action.deadline_at,700.)

    def test_resume_new_missing_class_uses_two_new_frames_not_old_or_delivered(self):
        self.prepare();self.r.pose=(.5,.5,2.38);self.r.pose_stamp=111.
        self.map(111.);self.finish(111.);self.map(112.);self.finish(112.)
        self.r.core.queue.delivered_classes={'red_cross','bridge'}
        epoch=self.r.catalog.epoch
        hint=lambda stamp:Hint(epoch,Key(3,100000000000,'bbox'),'panzer',(2.,1.),.45,stamp,1.,1)
        self.r.memory.update([hint(111000000000),hint(111200000000)],epoch,112000000000)
        self.assertFalse(self.r._resume_interrupt_hints(112.))
        self.r.memory.update([hint(112100000000)],epoch,112100000000)
        self.assertFalse(self.r._resume_interrupt_hints(112.1))
        self.assertFalse(self.r._resume_interrupt_hints(112.15))
        self.r.memory.update([hint(112300000000)],epoch,112300000000)
        self.assertEqual(set(self.r._resume_interrupt_hints(112.3)),{'panzer'})
        self.r.pose_stamp=112.3;self.map(112.3)
        out=self.r.tick(112.3,self.r._current_xy)
        self.assertEqual(self.r.stage,'DESCEND')
        self.assertEqual(out.action.command,'SEARCH')
        self.assertTrue(any(e['stage']=='SURVEY_RESUME_FOUND_MISSING' for e in self.r.events))
        self.assertEqual(self.r.core.committed_slots,0)

    def test_low_disproof_blocks_same_false_high_location(self):
        self.prepare();self.r.pose=(.5,.5,2.38);self.r.pose_stamp=111.
        self.map(111.);self.finish(111.);self.map(112.);self.finish(112.)
        epoch=self.r.catalog.epoch
        self.r.memory.update((),epoch,111000000000)
        self.assertTrue(self.r.memory.resolve_low(Hint(epoch,Key(4,100000000000),'bridge',(2.,1.),.2,111000000000,1.,3),112000000000))
        for stamp in (112100000000,112300000000):
            self.r.memory.update([Hint(epoch,Key(3,100000000000,'bbox'),'panzer',(2.,1.),.45,stamp,1.,1)],epoch,stamp)
            self.assertFalse(self.r._resume_interrupt_hints(stamp/1e9))
        self.assertNotIn('panzer',self.r._all_top(112.3))

    def test_climb_failure_returns_low_fallback_once(self):
        self.prepare();a=self.r.core.active_action;self.seq+=1
        event=replace(result_for(a,self.seq,status='FAILED',terminal=True),event_stamp_ns=110500000000)
        self.r.pose_stamp=110.5
        out=self.r.apply_result(event,110.5,(.5,.5))
        self.assertEqual(self.r.stage,'LOW_COVERAGE')
        self.assertTrue(self.r.resume_attempted)
        self.assertTrue(self.r.resume_completed)
        self.assertEqual(out.action.command,'SEARCH')
        self.assertIsNone(self.r._try_resume_survey(110.6))

    def test_blocked_or_stale_column_does_not_climb(self):
        for blocked in (False,True):
            self.setUp();self.r.policy=replace(self.r.policy,resume_survey_enabled=True)
            self.r.remaining_survey=(Waypoint(1.,1.,2.38),);self.r.survey_breakpoint=(.5,.5)
            self.r.route.interrupt(self.r.core.active_action.decision_seq);self.r.core.active_action=None
            self.r.pose=(0.,0.,1.18);self.r.pose_stamp=110.
            if blocked:
                self.map(110.);self.r.grid.blocked[:]=True
            self.assertIsNone(self.r._try_resume_survey(110.))
            self.assertTrue(self.r.resume_attempted)

    def test_resume_budget_is_not_new_mission_budget(self):
        self.prepare();self.r.resume_until=110.5;self.r.pose_stamp=110.5
        out=self.r.tick(110.5,(.5,.5))
        self.assertEqual(self.r.stage,'LOW_COVERAGE')
        self.assertTrue(self.r.resume_completed)
        self.assertEqual(self.r.core.started_at,100.)
        self.assertEqual(out.action.command,'SEARCH')
        self.assertLessEqual(out.action.deadline_at,700.)

    def test_descent_map_wait_recovers_or_returns_column_after_bound(self):
        full.FullTests.top3(self);self.finish(103.)
        old=self.r.core.active_action
        out=self.r.tick(104.,(0.,0.))
        self.assertEqual(self.r.stage,'DESCENT_WAIT')
        self.assertIsNone(out.action)
        self.assertIs(self.r.core.active_action,old)
        self.assertTrue(self.r._route_binding_matches(old))
        self.assertEqual(out.reason,'continuing_accepted_leg_for_descent_map')
        self.assertEqual(self.r.probe_status()['descent_wait_motion'],'CONTINUING_ACCEPTED_LEG')
        self.assertEqual(self.r.descent_motion_until,106.)
        self.map(105.);out=self.r.tick(105.,(0.,0.))
        self.assertEqual(self.r.stage,'DESCEND')
        self.assertEqual(out.action.goal.z,1.18)
        self.assertGreater(out.action.decision_seq,old.decision_seq)
        self.assertTrue(self.r._route_binding_matches(out.action))
        self.assertEqual(self.r.core.started_at,100.)

    def test_active_map_grace_ends_with_abort_at_two_seconds(self):
        self.r.policy=replace(self.r.policy,descent_wait_seconds=5.)
        full.FullTests.top3(self);self.finish(103.)
        self.r.tick(104.,(0.,0.));old=self.r.core.active_action
        self.assertIsNone(self.r.tick(105.999,(0.,0.)).action)
        self.assertIs(self.r.core.active_action,old)
        out=self.r.tick(106.,(0.,0.))
        self.assertEqual(out.action.command,'ABORT')
        self.assertGreater(out.action.decision_seq,old.decision_seq)
        self.assertEqual(self.r.core.phase,MissionPhase.ABORTED)
        self.assertEqual(self.r.failure,'descent_motion_handoff_timeout')
        self.assertIsNone(self.r.route.active)
        self.assertEqual(self.r.core.started_at,100.)

    def test_map_grace_cannot_extend_old_action_deadline(self):
        full.FullTests.top3(self);self.finish(103.)
        self.r.core.active_action=replace(self.r.core.active_action,deadline_at=104.8)
        self.r.tick(104.,(0.,0.))
        self.assertEqual(self.r.descent_motion_until,104.8)
        out=self.r.tick(104.8,(0.,0.))
        self.assertEqual(out.action.command,'ABORT')
        self.assertEqual(self.r.failure,'descent_motion_handoff_timeout')

    def test_successful_old_leg_during_grace_can_return_without_restarting_survey(self):
        full.FullTests.top3(self);self.finish(103.)
        self.r.tick(104.,(0.,0.))
        out=self.finish(105.)
        self.assertIsNone(out.action)
        self.assertIsNone(self.r.core.active_action)
        self.assertIsNone(self.r.route.active)
        self.assertIsNone(self.r.descent_motion_until)
        self.assertEqual(self.r.stage,'DESCENT_WAIT')
        out=self.r.tick(106.,self.r._current_xy)
        self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.assertEqual(out.action.command,'SEARCH')
        self.assertEqual((out.action.goal.x,out.action.goal.y),self.r.ascent_xy)
        self.assertEqual(self.r.core.started_at,100.)

    def test_failed_old_leg_during_grace_issues_abort(self):
        full.FullTests.top3(self);self.finish(103.)
        self.r.tick(104.,(0.,0.));old=self.r.core.active_action;self.seq+=1
        event=replace(result_for(old,self.seq,status='FAILED',terminal=True),event_stamp_ns=105000000000)
        out=self.r.apply_result(event,105.,(0.,0.))
        self.assertEqual(out.action.command,'ABORT')
        self.assertEqual(self.r.failure,'continued_descent_leg_failed')
        self.assertIsNone(self.r.route.active)

    def test_active_resume_timeout_at_high_altitude_does_not_orphan_old_leg(self):
        self.prepare();self.r.pose=(.5,.5,2.38);self.r.pose_stamp=110.5
        self.r.resume_until=110.5
        out=self.r.tick(110.5,(.5,.5))
        self.assertEqual(out.action.command,'SEARCH')
        self.assertEqual(self.r.stage,'DESCEND')
        self.assertTrue(self.r._route_binding_matches(out.action))
        self.assertEqual(self.r.descent_motion_until,112.5)
        self.assertEqual(self.r.core.started_at,100.)

    def test_fresh_blocked_local_columns_return_to_ascent_after_two_seconds(self):
        full.FullTests.top3(self);self.finish(103.);self.map(104.)
        self.r.grid.blocked[:]=True
        self.r.tick(104.,(0.,0.));old=self.r.core.active_action
        self.assertEqual(self.r.stage,'DESCENT_WAIT')
        self.map(106.);self.r.grid.blocked[:]=True
        out=self.r.tick(106.,(0.,0.))
        self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.assertEqual(out.action.command,'SEARCH')
        self.assertGreater(out.action.decision_seq,old.decision_seq)
        self.assertEqual((out.action.goal.x,out.action.goal.y),self.r.ascent_xy)
        self.assertTrue(self.r._route_binding_matches(out.action))
        self.assertFalse(self.r.done)
        self.assertEqual(self.r.core.started_at,100.)

    def test_active_resume_timeout_returns_column_with_fresh_blocked_map(self):
        self.prepare();old=self.r.core.active_action
        self.r.pose=(.5,.5,2.38);self.r.pose_stamp=110.5
        self.r.grid.blocked[:]=True;self.r.resume_until=110.5
        out=self.r.tick(110.5,(.5,.5))
        self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.assertEqual(out.action.command,'SEARCH')
        self.assertGreater(out.action.decision_seq,old.decision_seq)
        self.assertEqual((out.action.goal.x,out.action.goal.y),self.r.ascent_xy)
        self.assertEqual(self.r.descent_motion_until,112.5)
        self.assertTrue(self.r.resume_completed)

    def test_active_resume_timeout_stale_map_issues_abort(self):
        self.prepare();self.r.resume_until=110.5;self.r.grid.stamp=None
        out=self.r.tick(110.5,(.5,.5))
        self.assertEqual(out.action.command,'ABORT')
        self.assertEqual(self.r.failure,'resume_active_motion_map_stale')
        self.assertIsNone(self.r.route.active)

    def test_return_column_must_have_current_clear_swept_map(self):
        self.finish(103.);self.finish(106.);self.finish(109.)
        self.r.tick(111.,self.r._current_xy)
        self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.finish(112.)
        self.assertEqual(self.r.stage,'DESCENT_WAIT')
        self.assertEqual(self.r.descent_wait_mode,'return')
        self.r.tick(114.,self.r._current_xy)
        self.assertTrue(self.r.done)
        self.assertEqual(self.r.failure,'verified_return_column_unavailable')

if __name__=='__main__':unittest.main()
