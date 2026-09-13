import unittest
import numpy as np
from uav_high_view.grid_cost import GridCost
from uav_high_view.local_descent import propose


class LocalDescentTests(unittest.TestCase):
    def grid(self):
        g=GridCost(bounds=(-3.,3.,-3.,3.),resolution=.15,inflation=.35)
        g.update(np.array([[20.,20.,1.]]),10.,.4,3.)
        return g

    def test_clear_interruption_point_does_not_return_home(self):
        g=self.grid();targets={'a':(2.,1.),'b':(-2.,1.),'c':(0.,2.)}
        p=propose(g,(1.,1.),targets,(-2.,2.),10.2)
        self.assertEqual(p['xy'],(1.,1.));self.assertEqual(p['kind'],'CURRENT_COLUMN')
        self.assertEqual(tuple(p['classes']),g.order((1.,1.),targets,(-2.,2.),10.2)[1])

    def test_tree_under_aircraft_uses_bounded_nearby_column(self):
        g=self.grid();g.update(np.array([[1.,1.,2.]]),10.,.4,3.)
        p=propose(g,(1.,1.),{'a':(2.,2.)},(-2.,2.),10.1)
        self.assertEqual(p['kind'],'NEARBY_COLUMN')
        self.assertFalse(g.blocked[g.cell(p['xy'])]);self.assertLessEqual(p['candidates'],25)
        self.assertLessEqual(np.linalg.norm(np.array(p['xy'])-[1.,1.]),1.5)

    def test_stale_map_and_occupied_neighborhood_reject(self):
        g=self.grid();self.assertIsNone(propose(g,(0.,0.),{'a':(2.,2.)},(-2.,2.),13.))
        g.blocked[:]=True
        self.assertIsNone(propose(g,(0.,0.),{'a':(2.,2.)},(-2.,2.),10.))

    def test_unreachable_target_is_not_replaced_by_euclidean_cost(self):
        g=self.grid();g.blocked[g.cell((2.,2.))]=True
        self.assertIsNone(propose(g,(1.,1.),{'a':(2.,2.)},(-2.,2.),10.))


if __name__=='__main__':unittest.main()
