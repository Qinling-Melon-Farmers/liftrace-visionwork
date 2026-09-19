import math,unittest
import numpy as np
from uav_high_view.presentation import FollowView,look_at,intersects

class PresentationTests(unittest.TestCase):
    def test_look_at_uses_gazebo_positive_x(self):
        for camera,target in [((1,2,3),(0,0,0)),((0,0,7),(0,0,0)),((2,0,2),(0,0,1))]:
            x,y,z,w=look_at(camera,target)
            forward=np.array([1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)])
            wanted=np.array(target)-camera;wanted=wanted/np.linalg.norm(wanted)
            np.testing.assert_allclose(forward,wanted,atol=1e-9)

    def test_inward_view_keeps_camera_inside_tall_boundary(self):
        f=FollowView()
        p=f.propose((4.4,3,1),math.pi,None,.1)
        self.assertLessEqual(p['xyz'][0],4.55)
        self.assertGreaterEqual(p['xyz'][0],-4.55)
        self.assertTrue(p['wall_los_clear'])

    def test_occluded_behind_view_chooses_another_angle(self):
        wall=(-.9,-.7,-1,1,0,4)
        f=FollowView();p=f.propose((0,0,1),0,None,.1,[wall])
        self.assertFalse(intersects(p['xyz'],(0,0,1),wall))
        self.assertTrue(p['wall_los_clear'])

    def test_smoothing_cannot_cross_occluding_wall(self):
        f=FollowView();wall=(-.9,-.7,-1,1,0,4)
        p=f.propose((0,0,1),0,(-1.8,0,2),.1,[wall])
        self.assertFalse(intersects(p['xyz'],(0,0,1),wall))

    def test_invalid_state_rejected_and_z_bounded(self):
        f=FollowView()
        with self.assertRaises(ValueError):f.propose((0,0,float('nan')),0,None,.1)
        self.assertLessEqual(f.propose((0,0,3.9),0,None,.1)['xyz'][2],4.6)

if __name__=='__main__':unittest.main()
