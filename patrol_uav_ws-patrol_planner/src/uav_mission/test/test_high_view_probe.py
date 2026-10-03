from dataclasses import replace
import math
import unittest
from uav_mission.high_view_probe import HighViewProbe, ProbeConfig
from uav_mission.mission_core import MissionCore, MissionConfig, MissionPhase
from test_mission_runtime import profile, candidate, result_for


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.r=HighViewProbe(MissionCore(profile(),MissionConfig(early_return_enabled=False,motion_action_timeout=30.)),
                            ProbeConfig(-.22,((1.,1.),)))
        self.seq=0
        self.r.start('mission-runtime',100.,(0.,0.))

    def finish(self,now,success=True):
        action=self.r.core.active_action;self.seq+=1
        event=replace(result_for(action,self.seq,status='SUCCEEDED' if success else 'FAILED',terminal=True),
                      event_stamp_ns=int((now-.01)*1e9))
        return self.r.apply_result(event,now,(action.goal.x,action.goal.y))

    def observe_high(self,target_id=1):
        for t in [101.,101.3,101.6]:
            self.r.update_pose((0.,0.,2.38),t,'camera_init')
            c=replace(candidate(now=t,class_name='red_cross',x=1.,y=1.),target_id=target_id,first_seen_ns=99_000_000_000)
            self.r.ingest([c],t)

    def to_reacquire(self):
        self.observe_high()
        self.finish(103.);self.finish(106.)
        self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.finish(109.);self.assertEqual(self.r.stage,'DESCEND')
        self.finish(112.);self.assertEqual(self.r.stage,'REVISIT')
        self.finish(115.);self.assertEqual(self.r.stage,'REACQUIRE')

    def test_full_probe_fresh_low_observation(self):
        self.to_reacquire()
        self.r.update_pose((1.,1.,1.18),116.,'camera_init')
        self.r.ingest([candidate(now=116.,class_name='red_cross',x=1.1,y=1.)],116.)
        out=self.r.tick(116.1,(1.,1.))
        self.assertTrue(self.r.succeeded)
        self.assertEqual(out.action.command,'ABORT')
        self.assertEqual(self.r.core.committed_slots,0)
        self.assertLess(self.r.reacquired['hint_delta'],.2)

    def test_never_ingests_survey_into_delivery_queue(self):
        self.observe_high()
        self.assertFalse(self.r.core.queue.entries)
        self.assertFalse(self.r.core.active_action.has_target)

    def test_zero_id_survives_real_candidate_adapter(self):
        self.observe_high(target_id=0);self.finish(103.);self.finish(106.)
        self.assertEqual(self.r.selected.key.target_id,0)

    def test_ascent_failure_stops(self):
        self.finish(103.,False)
        self.assertTrue(self.r.done);self.assertFalse(self.r.ascent_verified)
        self.assertEqual(self.r.core.phase,MissionPhase.ABORTED)

    def test_mid_survey_failure_not_silently_skipped(self):
        self.finish(103.);self.finish(106.,False)
        self.assertTrue(self.r.done);self.assertFalse(self.r.succeeded)

    def test_budget_expiry_returns_verified_column(self):
        self.finish(103.)
        self.r.tick(146.,(1.,.5))
        self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.assertEqual(self.r.core.active_action.goal.x,0.)

    def test_budget_is_original_deadline(self):
        self.assertEqual(self.r.core.active_action.deadline_at,130.)
        self.finish(125.)
        self.assertEqual(self.r.core.active_action.deadline_at,145.)

    def test_no_hint_still_descends_then_reports_failure(self):
        self.finish(103.);self.finish(106.);self.finish(109.);self.finish(112.)
        self.assertTrue(self.r.done);self.assertEqual(self.r.failure,'no_high_view_hint')

    def test_expired_hint_rejected(self):
        self.observe_high();self.finish(103.);self.finish(106.)
        self.r.selected=replace(self.r.selected,last_seen_ns=1)
        self.finish(109.);self.finish(112.)
        self.assertEqual(self.r.failure,'hint_expired_before_revisit')

    def test_old_high_candidate_not_reacquisition(self):
        self.to_reacquire();self.r.update_pose((1.,1.,1.18),116.,'camera_init')
        self.r.ingest([candidate(now=101.,class_name='red_cross',x=1.,y=1.)],116.)
        self.r.tick(131.,(1.,1.));self.assertEqual(self.r.failure,'reacquisition_timeout')

    def test_wrong_class_position_or_height_rejected(self):
        for changes in [dict(class_name='bridge'),dict(x=3.),dict(y=3.)]:
            self.setUp();self.to_reacquire();self.r.update_pose((1.,1.,1.18),116.,'camera_init')
            c=replace(candidate(now=116.,class_name='red_cross',x=1.,y=1.),**changes)
            self.r.ingest([c],116.);self.assertIsNone(self.r.reacquired)

    def test_duplicate_low_candidates_ambiguous(self):
        self.to_reacquire();self.r.update_pose((1.,1.,1.18),116.,'camera_init')
        a=candidate(now=116.,class_name='red_cross',x=1.,y=1.)
        self.r.ingest([a,replace(a,target_id=2)],116.)
        self.assertIsNone(self.r.reacquired)

    def test_field_pose_jump_is_rejected_without_replacing_last_pose(self):
        before=(3.1740057468,-1.1063992977,1.803057313)
        after=(3.2236683369,-1.0406501293,2.196893454)
        self.r.update_pose(before,101.,'camera_init')
        with self.assertRaisesRegex(ValueError,'probe pose discontinuity'):
            self.r.update_pose(after,101.04,'camera_init')
        self.assertEqual(self.r.pose,before)
        self.assertEqual(self.r.pose_stamp,101.)
        # A subsequent consistent sample is evaluated against the accepted pose.
        normal=(before[0]+.1,before[1],before[2])
        self.r.update_pose(normal,101.1,'camera_init')
        self.assertEqual(self.r.pose,normal)

    def test_field_pose_guard_uses_three_dimensional_displacement(self):
        self.r.update_pose((0.,0.,2.),101.,'camera_init')
        # dt=.25 gives a 1m boundary. A diagonal sample exceeds it even
        # though its separate axis deltas are all below the boundary.
        with self.assertRaisesRegex(ValueError,'probe pose discontinuity'):
            self.r.update_pose((.6,.6,2.6),101.25,'camera_init')
        self.r.update_pose((1.,0.,2.),101.25,'camera_init')
        self.assertEqual(self.r.pose,(1.,0.,2.))

    def test_invalid_pose_and_clock_still_rejected(self):
        self.r.update_pose((0.,0.,2.38),101.,'camera_init')
        for xyz,stamp,frame in [
                ((0.,0.,2.38),102.,'map'),
                ((math.nan,0.,2.38),102.,'camera_init'),
                ((0.,0.,2.38),math.inf,'camera_init'),
                ((0.,0.,2.38),100.,'camera_init')]:
            with self.assertRaises(ValueError):
                self.r.update_pose(xyz,stamp,frame)
        self.assertEqual(self.r.pose_stamp,101.)

    def test_abort_status_preserves_original_reason(self):
        self.r.abort('board_pose_exception:ValueError',101.)
        self.r.tick(101.1,(0.,0.))
        self.assertEqual(self.r.failure,'board_pose_exception:ValueError')

    def test_old_result_does_not_advance_new_stage(self):
        action=self.r.core.active_action;self.observe_high();self.finish(103.);self.finish(106.)
        self.seq+=1
        out=self.r.apply_result(replace(result_for(action,self.seq,status='SUCCEEDED',terminal=True),event_stamp_ns=106_000_000_000),107.,(0.,0.))
        self.assertEqual(self.r.stage,'RETURN_COLUMN');self.assertEqual(self.r.route.current_index,0)

    def test_probe_configuration_bound(self):
        for args in [dict(high_agl=4.),dict(survey_budget=100.),dict(hint_radius=.5),
                     dict(staging_xy=(1.,)),dict(staging_xy=(math.nan,0.))]:
            with self.assertRaises(ValueError):ProbeConfig(-.22,((1.,1.),),**args)

    def test_low_staging_precedes_ascent_and_becomes_verified_column(self):
        runtime=HighViewProbe(
            MissionCore(profile(),MissionConfig(early_return_enabled=False,motion_action_timeout=30.)),
            ProbeConfig(-.22,((1.,1.),),staging_xy=(.6,.05)))
        runtime.start('mission-runtime',100.,(0.,0.))
        self.assertEqual((runtime.core.active_action.goal.x,runtime.core.active_action.goal.y,
                          runtime.core.active_action.goal.z),(.6,.05,1.18))
        self.r=runtime;self.seq=0
        self.finish(102.)
        self.assertFalse(runtime.ascent_verified)
        self.assertEqual((runtime.core.active_action.goal.x,runtime.core.active_action.goal.y,
                          runtime.core.active_action.goal.z),(.6,.05,2.38))
        self.finish(105.)
        self.assertTrue(runtime.ascent_verified)
        self.assertEqual((runtime.core.active_action.goal.x,runtime.core.active_action.goal.y,
                          runtime.core.active_action.goal.z),(1.,1.,2.38))
        self.finish(108.)
        self.assertEqual(runtime.core.active_action.goal.x,.6)
        self.assertEqual(runtime.core.active_action.goal.y,.05)

    def test_low_staging_observation_is_not_high_view_memory(self):
        runtime=HighViewProbe(
            MissionCore(profile(),MissionConfig(early_return_enabled=False,motion_action_timeout=30.)),
            ProbeConfig(-.22,((1.,1.),),staging_xy=(.6,.05)))
        runtime.start('mission-runtime',100.,(0.,0.))
        runtime.update_pose((.6,.05,1.18),101.,'camera_init')
        runtime.ingest([candidate(now=101.,class_name='red_cross',x=1.,y=1.)],101.)
        self.assertFalse(runtime.catalog.entries)


if __name__=='__main__':unittest.main()
