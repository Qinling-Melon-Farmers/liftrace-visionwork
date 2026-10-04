import math
import unittest
from mapping_startup import PoseAgreement, MapWarmup, VisionReadiness, startup_transport_pending

C = dict(pose_max_age=.3, pair_max_skew=.1, position_tolerance=.2,
         yaw_tolerance_deg=5., stable_seconds=2., map_max_age=1.5,
         map_warmup_seconds=3.)

def pose(t, x=0., yaw=0.):
    return (x, 0., -.05, yaw, t)

class StartupTests(unittest.TestCase):
    def setUp(self):
        self.g = PoseAgreement(C)

    def update(self, t, x=0., yaw=0., disarmed=True):
        return self.g.update([pose(t)], [pose(t, x, yaw)], t, disarmed)

    def test_118_degree_initialization_never_admitted(self):
        for t in range(1, 150):
            ok, detail = self.update(float(t), yaw=math.radians(118.))
            self.assertFalse(ok)
            self.assertEqual(detail['reason'], 'fc_lio_disagreement')

    def test_stationary_but_translated_estimates_rejected(self):
        for t in range(1, 5):
            self.assertFalse(self.update(float(t), x=2.)[0])

    def test_continuous_convergence_window(self):
        for i in range(20):
            self.assertFalse(self.update(1.+i/10.)[0])
        self.assertTrue(self.update(3.)[0])

    def test_disagreement_restarts_window(self):
        self.update(1.);self.update(2.)
        self.assertFalse(self.update(2.1, x=.3)[0])
        self.assertFalse(self.update(3.)[0])
        self.assertFalse(self.update(4.)[0])
        self.assertTrue(self.update(5.)[0])

    def test_arming_revokes_preflight_admission(self):
        self.update(1.);self.assertTrue(self.update(3.)[0])
        self.assertFalse(self.update(3.1, disarmed=False)[0])
        self.assertFalse(self.update(3.2)[0])

    def test_duplicate_old_stamps_do_not_hold_ready(self):
        self.update(1.)
        self.assertFalse(self.g.update([pose(1.)],[pose(1.)],3.,True)[0])

    def test_nearest_source_time_pair(self):
        self.assertEqual(self.g.update([pose(1.)],[pose(.99),pose(1.1,2.)],1.1,True)[1]['reason'],'settling')
        self.assertEqual(self.g.update([pose(1.)],[pose(1.2)],1.2,True)[1]['reason'],'pose_stamp_skew')

    def test_delayed_lio_pairs_to_historical_fc_without_relaxing_skew(self):
        for i in range(21):
            t=10.+i/10.
            fc=[pose(t-.2),pose(t-.15),pose(t-.1),pose(t)]
            ok,detail=self.g.update(fc,[pose(t-.15)],t,True)
        self.assertTrue(ok,detail)
        self.assertFalse(self.g.update(fc,[pose(t-.5)],t,True)[0])

    def test_nonfinite_future_and_clock_reset(self):
        self.assertFalse(self.update(1.,x=math.nan)[0])
        self.assertFalse(self.g.update([pose(2.)],[pose(2.)],1.,True)[0])
        self.update(2.);self.assertEqual(self.update(1.)[1]['reason'],'clock_reset')

    def test_wrapped_yaw(self):
        a=[pose(1.,yaw=math.radians(179))];b=[pose(1.,yaw=math.radians(-179))]
        self.assertAlmostEqual(self.g.update(a,b,1.,True)[1]['yaw_delta_deg'],2.)

    def test_empty_old_cloud_cannot_initialize(self):
        w=MapWarmup(10.,C)
        for t in (8.,9.,10.):w.observe(t,10.,True)
        w.observe(11.,11.,False)
        self.assertFalse(w.ready(14.))

    def test_map_requires_live_observation_window(self):
        w=MapWarmup(10.,C)
        for t in (11.,12.,13.):w.observe(t,t,True)
        self.assertFalse(w.ready(13.))
        w.observe(14.,14.,True)
        self.assertTrue(w.ready(14.))
        self.assertFalse(w.ready(16.))

    def test_map_gap_and_duplicate_restart(self):
        w=MapWarmup(10.,C)
        w.observe(11.,11.,True);w.observe(11.,11.,True)
        w.observe(14.,14.,True)
        self.assertFalse(w.ready(14.))
        for t in (15.,16.,17.):w.observe(t,t,True)
        self.assertTrue(w.ready(17.))

class StartupTransportTests(unittest.TestCase):
    def test_launch_gap_waits_but_does_not_reuse_old_stability(self):
        g=PoseAgreement(C)
        g.update([pose(1.)],[pose(1.)],1.,True)
        self.assertTrue(g.update([pose(3.)],[pose(3.)],3.,True)[0])
        ready,info=g.update([pose(3.)],[pose(3.)],3.4,True)
        self.assertFalse(ready);self.assertTrue(startup_transport_pending(info))
        for t in (3.5,4.,5.):
            ready,info=g.update([pose(t)],[pose(t)],t,True)
            self.assertFalse(ready);self.assertTrue(startup_transport_pending(info))
        self.assertTrue(g.update([pose(5.6)],[pose(5.6)],5.6,True)[0])
    def test_future_and_geometry_mismatch_still_require_stop(self):
        for fc,lio,now,disarmed in [([pose(2.)],[pose(2.)],1.,True),([pose(1.)],[pose(1.,x=.3)],1.,True),([pose(1.)],[pose(1.,yaw=math.pi)],1.,True),([pose(1.)],[pose(1.)],1.,False)]:
            ready,info=PoseAgreement(C).update(fc,lio,now,disarmed)
            self.assertFalse(ready);self.assertFalse(startup_transport_pending(info))
        self.assertFalse(startup_transport_pending(dict(reason='clock_reset')))
        self.assertFalse(startup_transport_pending(dict(reason='pose_stale_or_future')))

class VisionTests(unittest.TestCase):
    def test_yolo_alone_or_one_startup_array_cannot_admit(self):
        v=VisionReadiness(['yolo','refined','mapped','targets'],1.)
        for t in (10.,10.2):v.observe('yolo',t,t)
        v.observe('targets',10.1,10.2)
        self.assertEqual(v.missing(10.2),['refined','mapped','targets'])

    def test_two_empty_pipeline_arrays_suffice_without_visible_target(self):
        v=VisionReadiness(['yolo','refined','mapped','targets'],1.)
        for t in (10.,10.2):
            for topic in v.streams:v.observe(topic,t,t)
        self.assertFalse(v.missing(10.2))
        self.assertTrue(v.missing(11.3))

    def test_replayed_stamp_cannot_refresh_stream(self):
        v=VisionReadiness(['targets'],1.)
        for now in (10.,10.1,10.2):v.observe('targets',10.,now)
        self.assertEqual(v.missing(10.2),['targets'])
        v.observe('targets',10.3,10.3)
        self.assertFalse(v.missing(10.3))

if __name__=='__main__':unittest.main()
