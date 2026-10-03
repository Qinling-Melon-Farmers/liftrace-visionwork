import unittest
from uav_mission.corridor_speed import CorridorSpeed,CorridorSpeedConfig

class CorridorSpeedTests(unittest.TestCase):
    def setUp(self):self.p=CorridorSpeed(CorridorSpeedConfig());self.land=(8.5,-4.2)
    def test_pose_switches_inside_a_leg_and_hysteresis(self):
        self.assertEqual(self.p.select((8.35,3.),self.land,3)[0],'CORRIDOR_OPEN')
        self.assertEqual(self.p.select((8.35,2.3),self.land,3)[0],'DOOR')
        self.assertEqual(self.p.select((8.35,2.45),self.land,3)[0],'DOOR')
        self.assertEqual(self.p.select((8.35,2.6),self.land,3)[0],'CORRIDOR_OPEN')
    def test_negative_direction_other_gate_and_landing(self):
        self.assertEqual(self.p.select((8.35,0.),self.land,6)[0],'CORRIDOR_OPEN')
        self.assertEqual(self.p.select((8.35,-1.0),self.land,7)[0],'DOOR')
        self.assertEqual(self.p.select((8.5,-3.8),self.land,8)[0],'H_APPROACH')
    def test_no_fast_staging_descent_and_invalid_feedback(self):
        self.assertEqual(self.p.select((6.7,4.05),self.land,1)[1],.15)
        self.assertEqual(self.p.select(None,self.land,4)[1],.15)
    def test_invalid_config(self):
        with self.assertRaises(ValueError):CorridorSpeedConfig(enter_distance_m=1.,exit_distance_m=.8)

if __name__=='__main__':unittest.main()
