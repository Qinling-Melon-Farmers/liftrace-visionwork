import math,unittest
from uav_mission.flight_envelope import projected_bounds,within_xy
from test_navigation_vcl06_assertion import MODULE

class InnerEnvelopeTest(unittest.TestCase):
    def test_body_crossing_is_rejected_before_center_enters_corridor(self):
        region=dict(min_x=-.5,max_x=7.4,min_y=-4.8,max_y=4.8)
        self.assertTrue(within_xy(projected_bounds((7.,0.,3.),(0.,0.,0.,1.)),region))
        self.assertFalse(within_xy(projected_bounds((7.2,0.,3.),(0.,0.,0.,1.)),region))
    def test_yaw_and_tilt_are_included(self):
        b=projected_bounds((0,0,1),(0,0,math.sin(math.pi/8),math.cos(math.pi/8)))
        self.assertAlmostEqual(b[1],.275*math.sqrt(2))
        with self.assertRaises(ValueError):projected_bounds((0,0,1),(0,0,0,0))
    def gate(self):
        bounds=dict(min_x=-.5,max_x=9.1,min_y=-4.8,max_y=4.8)
        low=dict(min_x=7.6,max_x=9.1,min_y=-4.8,max_y=4.8,max_height=.7)
        h=dict(min_x=7.6,max_x=9.1,min_y=-4.8,max_y=-2.1,max_height=1.2)
        return MODULE.Vcl06GateReducer(field_bounds=bounds,low_height_region=low,landing_observation_region=h,search_envelope_region=dict(min_x=-.5,max_x=7.4,min_y=-4.8,max_y=4.8))
    def test_H_exemption_needs_phase_and_region(self):
        g=self.gate();g.land_decision_issued_ns=1;g.observe_pose(8.5,-4.2,1.,g.mission_frame)
        self.assertNotIn('corridor_height_limit_violation',g.errors)
        g.observe_pose(8.5,0.,1.,g.mission_frame)
        self.assertIn('corridor_height_limit_violation',g.errors)
        g=self.gate();g.observe_pose(8.5,-4.2,1.,g.mission_frame)
        self.assertIn('corridor_height_limit_violation',g.errors)
    def test_search_guard_stays_active_in_delivery_and_releases_for_tail(self):
        g=self.gate();g.statuses['manager']={'phase':'EXECUTING'}
        g.observe_search_envelope((7.2,0.,3.),(0,0,0,1))
        self.assertEqual(g.search_envelope_violations,1)
        g.statuses['manager']={'phase':'POST_DELIVERY_ROUTE'}
        g.observe_search_envelope((8.,0.,.9),(0,0,0,1))
        self.assertEqual(g.search_envelope_violations,1)

if __name__=='__main__':unittest.main()
