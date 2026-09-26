"""Offline navigation checks: coarse hints must not become release candidates."""
from dataclasses import replace
import unittest
from uav_high_view.core import Hint, Key
from uav_high_view.navigation_memory import NavigationMemory
import test_high_view_full as full_tests

class CoarseTests(unittest.TestCase):
    def setUp(self):
        full_tests.FullTests.setUp(self)
        self.r.policy=replace(self.r.policy,coarse_enabled=True)
        self.r.update_pose((0.,0.,2.38),101.,'camera_init')
    finish=full_tests.FullTests.finish
    map=full_tests.FullTests.map

    def cue(self, **kwargs):
        data=dict(class_name='panzer',xy=(1.,1.),stamp_ns=101000000000,
                  frame='camera_init',confidence=.61,transform_age_sec=0.,
                  map_valid=True,now=101.)
        data.update(kwargs)
        return self.r.ingest_coarse(**data)

    def test_one_bbox_can_be_remembered_without_delivery_candidate(self):
        self.assertEqual(self.cue(),'coarse_accepted')
        hint=self.r._all_hints(101.)['panzer']
        self.assertEqual(hint.evidence_count,1)
        self.assertEqual(hint.key.source,'bbox')
        self.assertEqual(hint.role,'REVISIT_HINT_ONLY')
        self.assertEqual(self.r.core.committed_slots,0)
        self.assertTrue(all(s.candidate_key is None for s in self.r.core.slots))
        self.assertEqual(self.r.core.active_action.command,'SEARCH')

    def test_three_coarse_hints_only_descend_and_wait_for_low_confirmation(self):
        for index,cls in enumerate(('bridge','panzer','red_cross')):
            self.assertEqual(self.cue(class_name=cls,xy=(float(index),1.)),'coarse_accepted')
        self.finish(103.)
        self.map(104.)
        self.r.tick(104.,(0.,0.))
        self.assertEqual(self.r.stage,'DESCEND')
        self.map(110.);self.finish(110.);self.finish(113.)
        self.assertEqual(self.r.stage,'REACQUIRE')
        out=self.r.tick(113.1,self.r.selected.xy)
        self.assertIsNone(out.action)
        self.assertIsNone(self.r.reacquired)
        self.assertTrue(all(s.candidate_key is None for s in self.r.core.slots))

    def test_freshness_class_geometry_and_phase_guards(self):
        cases=[
            dict(stamp_ns=100000000000),dict(stamp_ns=102000000000),
            dict(transform_age_sec=.11),dict(transform_age_sec=-1.),
            dict(xy=(float('nan'),1.)),dict(confidence=float('nan')),
            dict(confidence=.59),dict(confidence=1.1),dict(map_valid=False),
            dict(frame='wrong'),dict(class_name='circle')]
        for case in cases:
            with self.subTest(case=case):
                self.assertNotEqual(self.cue(**case),'coarse_accepted')
                self.assertFalse(self.r._all_hints(101.))
        self.r.policy=replace(self.r.policy,coarse_enabled=False)
        self.assertEqual(self.cue(),'coarse_inactive')
        self.r.policy=replace(self.r.policy,coarse_enabled=True)
        self.r.stage='REACQUIRE'
        self.assertEqual(self.cue(),'coarse_inactive')

    def test_low_height_is_not_a_high_hint(self):
        self.r.pose=(0.,0.,1.18)
        self.assertEqual(self.cue(),'coarse_not_at_high_view')
        self.r.pose=(0.,0.,2.38);self.r.pose_stamp=100.
        self.assertEqual(self.cue(),'coarse_pose_unavailable')

    def test_duplicate_cannot_refresh_source_time(self):
        self.cue()
        self.assertEqual(self.cue(now=101.1),'coarse_duplicate_or_older')
        self.assertEqual(self.r._all_hints(101.1)['panzer'].last_seen_ns,101000000000)

    def test_same_class_two_distant_boxes_require_low_verification(self):
        self.cue()
        self.assertEqual(self.cue(xy=(3.,1.)),'coarse_conflict')
        self.assertNotIn('panzer',self.r._all_hints(101.))
        self.assertEqual(len(self.r.memory.verification_hints(101000000000)['panzer']),2)

    def test_two_classes_on_one_position_cannot_complete_top_three(self):
        self.cue()
        self.assertEqual(self.cue(class_name='bridge',xy=(1.1,1.)),'coarse_conflict')
        self.assertNotIn('panzer',self.r._all_hints(101.))
        self.assertNotIn('bridge',self.r._all_hints(101.))

    def test_refined_upgrade_is_not_overwritten_by_a_later_bbox(self):
        self.cue()
        hint=self.r._all_hints(101.)['panzer']
        fine=replace(hint,key=Key(42,100000000000),xy=(1.1,1.),uncertainty_m=.2,evidence_count=3)
        self.r.memory.update([fine],self.r.catalog.epoch,101000000000)
        self.cue(stamp_ns=101100000000,now=101.1,xy=(1.3,1.))
        self.assertEqual(self.r._all_hints(101.1)['panzer'],fine)

    def test_expired_hint_cannot_be_reinserted(self):
        self.cue()
        h=self.r._all_hints(101.)['panzer']
        mem=NavigationMemory(['panzer'],1000000000)
        self.assertFalse(mem.update([h],h.epoch,h.last_seen_ns+1000000001))

if __name__=='__main__':unittest.main()
