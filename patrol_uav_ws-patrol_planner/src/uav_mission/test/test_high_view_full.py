from dataclasses import replace
import unittest
import numpy as np
from uav_high_view.grid_cost import GridCost
from uav_mission.high_view_probe import ProbeConfig
from uav_mission.high_view_full import HighViewFull
from uav_mission.mission_core import MissionCore,MissionConfig,GoalSnapshot,MissionPhase
from test_mission_runtime import profile,candidate,result_for,release_ack


class FullTests(unittest.TestCase):
    def setUp(self):
        cfg=MissionConfig(early_return_enabled=False,post_delivery_route=(GoalSnapshot('camera_init',-3.,6.,1.18),),landing_xy=(-3.,6.))
        self.r=HighViewFull(MissionCore(profile(),cfg),ProbeConfig(-.22,((1.,1.),(2.,1.))))
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
        out=self.r.tick(104.,(0.,0.))
        self.assertEqual(self.r.stage,'RETURN_COLUMN')
        self.finish(107.);self.map(110.);self.finish(110.)
        self.finish(113.);self.assertEqual(self.r.stage,'REACQUIRE')

    def test_no_fixed_45_second_cutoff(self):
        self.assertEqual(self.r.survey_until,700.)
        self.assertGreater(self.r.core.active_action.deadline_at,145.)

    def test_interrupt_requires_all_three_coordinates(self):
        self.top3();self.finish(103.);old=self.r.core.active_action
        self.r.tick(104.,(0.,0.))
        self.assertGreater(self.r.core.active_action.decision_seq,old.decision_seq)
        self.assertFalse(self.r.core.active_action.has_target)
        self.assertEqual(set(self.r.top_hints),{'bridge','panzer','red_cross'})

    def test_missing_top3_at_route_end_is_failure_not_cow_path(self):
        self.finish(103.);self.finish(106.);self.finish(109.)
        self.assertTrue(self.r.done);self.assertEqual(self.r.failure,'survey_complete_missing_top3')

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
        self.top3();self.finish(103.);self.r.tick(104.,(0.,0.));self.finish(107.);self.finish(110.)
        self.assertEqual(self.r.failure,'no_fresh_grid_route_to_required_targets')

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
