import importlib.util
from pathlib import Path
import threading,unittest
from types import SimpleNamespace as NS

class SupportContactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('support_monitor',Path(__file__).resolve().parents[1]/'scripts/gazebo_contact_monitor.py')
        cls.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.module)
    def setUp(self):
        self.m=self.module.GazeboContactMonitor.__new__(self.module.GazeboContactMonitor)
        for k,v in dict(_lock=threading.RLock(),_ignored=('ground_plane','landing_h::link::collision'),_support_active=False,_support_count=0,_support_events=[],_guard_xy=None,_active=False,_actual_collision_count=0,_events=[],_sample_count=0).items():setattr(self.m,k,v)
        self.m._publish=lambda:None
    def state(self,name):
        return NS(collision1_name='iris::competition_guard_collision',collision2_name=name,depths=[.002],info='guard_xy_m=0.500000',total_wrench=NS(force=NS(x=0,y=0,z=20)),contact_positions=[NS(x=8.5,y=-4.2,z=.005)],contact_normals=[NS(x=0,y=0,z=1)])
    def send(self,states):self.m._on_contacts(NS(header=NS(stamp=NS(to_sec=lambda:100.)),states=states))
    def test_support_is_recorded_but_not_obstacle_collision(self):
        self.send([self.state('landing_h::link::collision')])
        self.assertEqual(self.m._actual_collision_count,0);self.assertEqual(self.m._support_count,1)
        self.assertEqual(self.m._guard_xy,.5);self.assertEqual(self.m._support_events[0]['peak_sampled_force_n'],20.)
    def test_wall_cannot_be_hidden_by_simultaneous_ground_contact(self):
        self.send([self.state('ground_plane'),self.state('toudi2::Wall_9::collision')])
        self.assertEqual(self.m._actual_collision_count,1);self.assertEqual(self.m._support_count,1)
        self.assertEqual(len(self.m._events[0]['details']),1)
        self.send([self.state('toudi2::Wall_9::collision')]);self.assertEqual(self.m._actual_collision_count,1)

if __name__=='__main__':unittest.main()
