from dataclasses import replace
import unittest
from uav_high_view.core import Epoch,Key,Hint
from uav_high_view.navigation_memory import NavigationMemory
from uav_mission.boundary_revisit import BoundaryRevisit
import test_high_view_full as fixtures
from test_mission_runtime import candidate,result_for


class ConflictMemoryTest(unittest.TestCase):
    def hint(self,x,stamp=10_000_000_000):
        return Hint(Epoch('m','l','c'),Key(1,1),'panzer',(x,1.),.2,stamp,3,1.)
    def test_two_hypotheses_are_separate_from_qualified_hints(self):
        m=NavigationMemory(['panzer'],30_000_000_000)
        h=self.hint(1.);m.update([h],h.epoch,10_000_000_000)
        self.assertEqual(m.update([self.hint(3.)],h.epoch,11_000_000_000),{})
        hs=m.verification_hints(11_000_000_000)['panzer']
        self.assertEqual({v.xy for v in hs},{(1.,1.),(3.,1.)})
        m.update([self.hint(5.)],h.epoch,12_000_000_000)
        self.assertEqual(len(m.verification_hints(12_000_000_000)['panzer']),2)
        self.assertEqual(m.verification_hints(41_000_000_000)['panzer'],())
        m.update([],None,42_000_000_000);self.assertEqual(m.conflict_hints,{})


class ConflictMotionTest(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.FullTests();self.fixture.setUp();self.fixture.to_capture();self.r=self.fixture.r
        h=self.r.top_hints['panzer'];self.r.memory.update([replace(h,xy=(3.,1.))],h.epoch,114_000_000_000)
        self.r.top_hints={};self.r.core.queue.delivered_classes={'bridge','red_cross'}
        self.r._current_xy=(0.,1.)

    def test_check_near_first_then_other_without_coverage_or_release(self):
        out=self.r._next_target(114.)
        self.assertEqual(out.action.command,'SEARCH');self.assertFalse(out.action.has_target)
        self.assertEqual(self.r.selected.xy,(1.,1.));self.assertTrue(self.r.conflict_active)
        self.assertLessEqual(out.action.deadline_at,139.)
        self.fixture.finish(116.)
        self.r.update_pose((1.,1.,1.18),117.,'camera_init')
        self.r.ingest([candidate(class_name='pillbox',now=117.,x=1.,y=1.)],117.)
        self.assertIsNone(self.r.tick(117.1,(1.,1.)).action)
        out=self.r.tick(131.1,(1.,1.))
        self.assertEqual(out.action.command,'SEARCH');self.assertEqual(self.r.selected.xy,(3.,1.))
        self.assertIsNone(self.r.fallback_started);self.assertEqual(self.r.core.committed_slots,0)
        self.fixture.finish(133.)
        self.r.update_pose((3.,1.,1.18),134.,'camera_init')
        self.r.ingest([candidate(target_id=8,class_name='panzer',now=134.,x=3.,y=1.)],134.)
        out=self.r.tick(134.1,(3.,1.))
        self.assertEqual(out.action.command,'APPROACH');self.assertEqual(out.action.target_class,'panzer')
        self.assertEqual(self.r.core.started_at,100.)

    def test_unreachable_hypothesis_advances_and_late_success_is_rejected(self):
        self.r._next_target(114.);old=self.r.core.active_action
        self.fixture.seq+=1
        failed=replace(result_for(old,self.fixture.seq,status='FAILED',terminal=True),event_stamp_ns=115_000_000_000)
        out=self.r.apply_result(failed,115.1,(0.,1.))
        self.assertEqual(out.action.command,'SEARCH');self.assertEqual(self.r.selected.xy,(3.,1.))
        late=replace(result_for(old,999,status='SUCCEEDED',terminal=True),event_stamp_ns=116_000_000_000)
        self.assertFalse(self.r.apply_result(late,116.1,(0.,1.)).accepted)

    def test_both_checks_exhaust_before_bounded_coverage(self):
        self.r._next_target(114.);self.fixture.finish(116.)
        self.r.tick(131.1,(1.,1.));self.fixture.finish(133.)
        self.r.tick(148.1,(3.,1.))
        self.assertEqual(self.r.stage,'LOW_COVERAGE');self.assertEqual(len(self.r.conflict_checked),2)
        self.assertEqual(self.r.core.started_at,100.);self.assertEqual(self.r.core.committed_slots,0)


class CoverageBrakingTest(unittest.TestCase):
    def test_only_edge_or_final_approach_is_slow(self):
        b=BoundaryRevisit(enabled=True,bounds=(-.5,7.4,-4.8,4.8))
        self.assertFalse(b.slow_coverage((4.,1.),(0.,1.)))
        self.assertTrue(b.slow_coverage((.8,1.),(0.,1.)))
        self.assertTrue(b.slow_coverage((.1,1.),(6.,1.)))
        self.assertFalse(b.slow_coverage((1.2,1.),(6.,1.)))
        self.assertFalse(BoundaryRevisit().slow_coverage((-4.6,1.),(-4.3,1.)))

if __name__=='__main__':unittest.main()
