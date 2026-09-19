import unittest
from uav_mission.boundary_revisit import BoundaryRevisit

class BoundaryTests(unittest.TestCase):
    def test_seed38_40_outward_hint_replaced_by_interior_view(self):
        p=BoundaryRevisit(enabled=True)
        for x in (4.410+.042,4.435+.065):
            view=p.viewpoint((x,3.),.3)
            self.assertLess(view[0],x-.1)
            self.assertLessEqual(x-view[0],p.max_view_offset_m)
            self.assertTrue(p.admissible(view))
        self.assertTrue(p.admissible((4.435,3.)))  # true target retained

    def test_already_inflated_box_counted_once_and_yaw_matters(self):
        p=BoundaryRevisit(enabled=True,yaw_budget_deg=0,tracking_reserve_m=0)
        self.assertAlmostEqual(p.margin,.275)
        q=BoundaryRevisit(enabled=True,yaw_budget_deg=45)
        self.assertFalse(q.admissible((4.435,3.)))

    def test_outside_field_or_nan_not_repaired_into_fake_target(self):
        p=BoundaryRevisit(enabled=True)
        self.assertIsNone(p.viewpoint((6.,3.),.3))
        self.assertIsNone(p.viewpoint((float('nan'),3.),.3))
        self.assertFalse(p.admissible((4.6,3.)))

    def test_center_cue_unchanged(self):
        p=BoundaryRevisit(enabled=True)
        self.assertEqual(p.viewpoint((0.,3.),.4),(0.,3.))
        self.assertFalse(p.near((0.,3.)))

if __name__=='__main__':unittest.main()
