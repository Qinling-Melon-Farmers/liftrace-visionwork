"""Regression coverage for finite evidence updates and interruption support."""
from dataclasses import replace
import unittest
from uav_high_view.core import Epoch,Hint,Key
from uav_high_view.navigation_memory import NavigationMemory

N=1_000_000_000
class MemoryTests(unittest.TestCase):
    def setUp(self):
        self.epoch=Epoch('m','loc','cal')
        self.m=NavigationMemory(['panzer','pillbox','bridge'],30*N)
    def h(self,xy=(0.,0.),t=10.,cls='panzer',source='bbox',key=1):
        return Hint(self.epoch,Key(key,1,source),cls,xy,.2,int(t*N),1.,1 if source=='bbox' else 3)
    def put(self,*hints):
        return self.m.update(hints,self.epoch,max(h.last_seen_ns for h in hints))
    def test_two_wrong_then_refined_third_replaces_weak_location(self):
        self.put(self.h())
        self.put(self.h((.33,3.36),11.))
        good=self.h((3.12,-.34),12.,source='vision')
        self.assertEqual(self.put(good)['panzer'],good)
        self.assertFalse(self.m.suspended)
        self.assertIn('panzer',self.m.interrupt_hints(12*N))
        self.assertLessEqual(len(self.m.support_status()['panzer']),2)
    def test_refined_is_not_overwritten_or_suspended_by_single_wrong_box(self):
        good=self.h(source='vision');self.put(good)
        self.put(self.h((3.,3.),11.))
        self.put(self.h((.1,0.),12.,cls='pillbox'))
        self.assertEqual(self.m.interrupt_hints(12*N),{'panzer':good})
    def test_two_equal_refined_locations_remain_ambiguous(self):
        self.put(self.h(source='vision'),self.h((3.,3.),source='vision',key=2))
        self.assertFalse(self.m.interrupt_hints(10*N))
        self.assertEqual(len(self.m.verification_hints(10*N)['panzer']),2)
    def test_coarse_competing_locations_remain_ambiguous_even_with_two_frames(self):
        self.put(self.h(),self.h((3.,3.),key=2))
        self.put(self.h(t=10.2),self.h((3.,3.),10.2,key=2))
        self.assertFalse(self.m.interrupt_hints(int(10.2*N)))
    def test_same_image_new_id_does_not_add_support(self):
        self.put(self.h());self.put(self.h(key=99))
        self.assertEqual(self.m.saved['panzer'].evidence_count,1)
        self.assertFalse(self.m.interrupt_hints(10*N))
    def test_two_independent_images_form_support_not_release_evidence(self):
        self.put(self.h());self.put(self.h((.1,0.),10.1,key=99))
        h=self.m.interrupt_hints(int(10.1*N))['panzer']
        self.assertEqual(h.evidence_count,2)
        self.assertEqual(h.role,'REVISIT_HINT_ONLY')
        self.assertEqual(h.last_seen_ns,int(10.1*N))
    def test_short_long_and_spatially_inconsistent_pairs_do_not_support(self):
        for dt,xy in ((.09,(0.,0.)),(1.01,(0.,0.)),(.2,(.55,0.))):
            with self.subTest(dt=dt,xy=xy):
                self.setUp();self.put(self.h());self.put(self.h(xy,10.+dt))
                self.assertFalse(self.m.interrupt_hints(int((10.+dt)*N)))
    def test_dense_frames_count_by_source_time_not_publication_count(self):
        for i in range(12):self.put(self.h(t=10.+i*.01))
        stamps=self.m.support_status()['panzer'][0]['distinct_image_stamps']
        self.assertEqual(len(stamps),2)
        self.assertGreaterEqual(stamps[1]-stamps[0],100_000_000)
    def test_later_outlier_cannot_reuse_old_consistent_pair(self):
        self.put(self.h());self.put(self.h(t=10.2))
        self.put(self.h((.55,0.),10.4))
        self.assertFalse(self.m.interrupt_hints(int(10.4*N)))
    def test_expiration_clears_conflict_and_accepts_new_evidence(self):
        self.put(self.h(),self.h((3.,3.),key=2))
        self.assertFalse(self.m.verification_hints(41*N))
        self.assertFalse(self.m.suspended)
        good=self.h((4.,2.),42.,source='vision')
        self.assertEqual(self.put(good),{'panzer':good})
    def test_low_resolution_removes_wrong_label_only_at_same_place(self):
        wrong=self.h((1.,1.));other=self.h((3.,3.),key=2)
        pb=self.h((1.,1.),cls='pillbox')
        self.put(wrong,other,pb)
        low=self.h((1.,1.),11.,cls='pillbox',source='vision')
        self.assertTrue(self.m.resolve_low(low,11*N))
        self.assertEqual(self.m.update([wrong,pb],self.epoch,11*N)['panzer'].xy,(3.,3.))
        self.assertFalse(self.m.suspended)
        self.assertEqual(self.m.saved['pillbox'].xy,(1.,1.))
    def test_coarse_or_old_low_resolution_is_rejected(self):
        self.put(self.h())
        self.assertFalse(self.m.resolve_low(self.h(),10*N))
        low=self.h(t=11.,source='vision')
        self.assertTrue(self.m.resolve_low(low,11*N))
        self.assertFalse(self.m.resolve_low(replace(low,class_name='pillbox'),11*N))
    def test_two_low_instances_of_same_class_are_not_silently_collapsed(self):
        self.m.update([],self.epoch,10*N)
        self.m.resolve_low(self.h(source='vision'),10*N)
        self.m.resolve_low(self.h((3.,3.),11.,source='vision',key=2),11*N)
        self.assertIn('panzer',self.m.suspended)
    def test_clock_and_epoch_reset_drop_certificates_and_resolutions(self):
        self.put(self.h());self.put(self.h(t=10.2))
        self.assertFalse(self.m.update([],self.epoch,9*N))
        self.assertFalse(self.m.support_status())
        self.put(self.h(source='vision'))
        self.assertFalse(self.m.update([],Epoch('new','loc','cal'),11*N))
    def test_retirement_blocks_catalog_replay_but_not_other_same_class_location(self):
        bad=self.h();other=self.h((3.,3.),key=2)
        self.put(bad,other);self.m.retire_location(bad,11*N)
        self.assertEqual(self.m.update([bad],self.epoch,11*N),{'panzer':other})
        self.assertEqual(self.put(self.h(t=12.)),{'panzer':other})
        fine=self.h(t=13.,source='vision')
        self.assertEqual(self.put(fine),{'panzer':fine})

    def test_retired_old_formal_hint_cannot_restore_without_new_image(self):
        h=self.h(source='vision');self.put(h);self.m.retire_location(h,11*N)
        self.assertFalse(self.m.update([h],self.epoch,12*N))
        self.assertFalse(self.m.resolve_low(h,12*N))
        fresh=self.h(t=12.,source='vision')
        self.assertTrue(self.m.resolve_low(fresh,12*N))
        self.assertEqual(self.m.saved['panzer'],fresh)

    def test_retirement_has_bounded_capacity_and_resets_with_epoch_clock_and_ttl(self):
        for i in range(20):
            h=self.h((2.*i,0.),10.+i)
            self.put(h);self.m.retire_location(h,h.last_seen_ns)
        self.assertLessEqual(len(self.m._retired),2*len(self.m.classes))
        self.m.update([],self.epoch,61*N);self.assertFalse(self.m._retired)
        self.put(self.h(t=62.));self.m.retire_location(self.h(t=62.),63*N)
        self.m.update([],Epoch('new','loc','cal'),64*N);self.assertFalse(self.m._retired)
        self.m.update([],self.epoch,65*N)
        h=self.h(t=65.);self.put(h);self.m.retire_location(h,66*N)
        self.m.update([],self.epoch,64*N);self.assertFalse(self.m._retired)

    def test_capacity_is_bounded(self):
        for i in range(100):self.put(self.h((i*2.,0.),10.+i*.1,key=i))
        self.assertLessEqual(len(self.m.support_status()['panzer']),2)
        self.assertLessEqual(len(self.m.events),64)

class RevisitMemoryTests(unittest.TestCase):
    setUp=MemoryTests.setUp
    h=MemoryTests.h
    put=MemoryTests.put
    def test_supported_place_survives_newer_competing_single_image(self):
        for i in range(6):self.put(self.h(t=10.+i*.2))
        self.put(self.h((3.,3.),12.,key=2))
        hints=self.m.revisit_hints(12*N)
        self.assertEqual(hints['panzer'].xy,(0.,0.))
        self.assertEqual(hints['panzer'].evidence_count,2)
        self.assertFalse(self.m.interrupt_hints(12*N))
        self.assertEqual(len(self.m.verification_hints(12*N)['panzer']),2)
        self.assertEqual(self.m.support_status()['panzer'][0]['revisit_support_count'],6)

    def test_sustained_place_beats_newer_two_image_competitor(self):
        for i in range(6):self.put(self.h(t=10.+i*.2))
        for t in (12.,12.2):self.put(self.h((3.,3.),t,key=2))
        self.assertEqual(self.m.revisit_hints(int(12.2*N))['panzer'].xy,(0.,0.))
        self.assertFalse(self.m.interrupt_hints(int(12.2*N)))

    def test_single_image_conflicts_stay_for_local_verification(self):
        self.put(self.h(),self.h((3.,3.),key=2))
        self.assertFalse(self.m.revisit_hints(10*N))
        self.assertEqual(len(self.m.verification_hints(10*N)['panzer']),2)

    def test_same_place_wrong_class_does_not_remove_supported_revisit(self):
        for t in (10.,10.2,10.4):self.put(self.h(t=t))
        self.put(self.h(t=11.,cls='bridge',key=2))
        self.assertEqual(set(self.m.revisit_hints(11*N)),{'panzer'})
        self.assertFalse(self.m.interrupt_hints(11*N))
        low=self.h(t=12.,source='vision')
        self.assertTrue(self.m.resolve_low(low,12*N))
        self.assertEqual(self.m.revisit_hints(12*N),{'panzer':low})
        self.assertNotIn('bridge',self.m.saved)

    def test_duplicate_or_dense_images_cannot_inflate_revisit_support(self):
        for i in range(9):self.put(self.h(t=10.+i*.01,key=i))
        self.assertEqual(self.m.support_status()['panzer'][0]['revisit_support_count'],1)
        self.put(self.h(t=10.2))
        for i in range(10):self.put(self.h(t=10.2,key=100+i))
        self.assertEqual(self.m.support_status()['panzer'][0]['revisit_support_count'],2)

    def test_low_confirmation_cleans_local_labels_and_suppresses_remote_weak_place(self):
        for t in (10.,10.2):
            self.put(self.h((0.,0.),t),self.h((3.,0.),t,key=2),
                     self.h((3.,0.),t,cls='bridge',key=3),
                     self.h((6.,0.),t,cls='bridge',key=4))
        low=self.h((3.,0.),11.,source='vision',key=2)
        self.assertTrue(self.m.resolve_low(low,11*N))
        preferred=self.m.revisit_hints(11*N)
        self.assertEqual(preferred['panzer'],low)
        self.assertEqual(preferred['bridge'].xy,(6.,0.))
        self.assertEqual(len(self.m.support_status()['bridge']),1)
        self.assertFalse(self.m.verification_hints(11*N))
        # Older wrong labels cannot reappear as navigation candidates.
        self.put(self.h((0.,0.),12.),self.h((3.,0.),12.,cls='bridge',key=3))
        preferred=self.m.revisit_hints(12*N)
        self.assertEqual(preferred['panzer'],low)
        self.assertEqual(preferred['bridge'].xy,(6.,0.))

    def test_revisit_hints_respect_expiry_and_clock_reset(self):
        for t in (10.,10.2):self.put(self.h(t=t))
        self.put(self.h((3.,3.),11.,key=2))
        self.assertTrue(self.m.revisit_hints(11*N))
        self.assertFalse(self.m.revisit_hints(42*N))
        self.put(self.h(t=43.))
        self.assertFalse(self.m.revisit_hints(42*N))

    def test_retire_preferred_exposes_alternative_without_resurrecting_it(self):
        for t in (10.,10.2):self.put(self.h(t=t))
        self.put(self.h((3.,3.),11.,key=2))
        preferred=self.m.revisit_hints(11*N)['panzer']
        self.m.retire_location(preferred,12*N)
        self.assertEqual(self.m.revisit_hints(12*N)['panzer'].xy,(3.,3.))
        self.put(self.h(t=13.))
        self.assertEqual(self.m.revisit_hints(13*N)['panzer'].xy,(3.,3.))

if __name__=='__main__':unittest.main()
