import importlib.util
from pathlib import Path
import threading,unittest
from types import SimpleNamespace as NS
from unittest import mock
import json
import tempfile

class SupportContactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('support_monitor',Path(__file__).resolve().parents[1]/'scripts/gazebo_contact_monitor.py')
        cls.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.module)
    def setUp(self):
        self.m=self.module.GazeboContactMonitor.__new__(self.module.GazeboContactMonitor)
        for k,v in dict(_lock=threading.RLock(),_ignored=('ground_plane','landing_h::link::collision'),_support_active=False,_support_count=0,_support_events=[],_guard_xy=None,_active=False,_actual_collision_count=0,_events=[],_sample_count=0).items():setattr(self.m,k,v)
        self.m._publish=lambda:None
    def state(self,name,depth=.002,force=20,position=8.5):
        return NS(collision1_name='iris::competition_guard_collision',collision2_name=name,depths=[depth] if depth is not None else [],info='guard_xy_m=0.500000',total_wrench=NS(force=NS(x=0,y=0,z=force)),contact_positions=[NS(x=position,y=-4.2,z=.005)],contact_normals=[NS(x=0,y=0,z=1)])
    def send(self,states,stamp=100.):self.m._on_contacts(NS(header=NS(stamp=NS(to_sec=lambda:stamp)),states=states))
    def test_support_is_recorded_but_not_obstacle_collision(self):
        self.send([self.state('landing_h::link::collision')])
        self.assertEqual(self.m._actual_collision_count,0);self.assertEqual(self.m._support_count,1)
        self.assertEqual(self.m._guard_xy,.5);self.assertEqual(self.m._support_events[0]['peak_sampled_force_n'],20.)
    def test_wall_cannot_be_hidden_by_simultaneous_ground_contact(self):
        self.send([self.state('ground_plane'),self.state('toudi2::Wall_9::collision')])
        self.assertEqual(self.m._actual_collision_count,1);self.assertEqual(self.m._support_count,1)
        self.assertEqual(len(self.m._events[0]['details']),1)
        self.send([self.state('toudi2::Wall_9::collision')]);self.assertEqual(self.m._actual_collision_count,1)

    def test_first_light_then_deep_contact_records_independent_peaks(self):
        wall = 'toudi2::Wall_9::collision'
        self.send([self.state(wall, .001, 3)], 100.)
        self.send([self.state(wall, .030, 25, position=9.)], 100.2)
        self.send([self.state(wall, .005, 90)], 100.5)
        self.send([self.state(wall, .002, 2)], 100.8)
        self.send([], 101.)
        self.assertEqual(self.m._actual_collision_count, 1)
        event = self.m._events[0]
        self.assertEqual(event['sample_count'], 4)
        self.assertAlmostEqual(event['duration_sec'], .8)
        self.assertEqual(event['last_ros_stamp'], 100.8)
        self.assertEqual(event['ended_ros_stamp'], 101.)
        self.assertEqual(event['peak_sampled_depth_m'], .030)
        self.assertEqual(event['peak_sampled_force_n'], 90.)
        detail = event['details'][0]
        self.assertEqual(detail['max_depth_m'], .030)
        self.assertEqual(detail['total_force_norm_n'], 90.)
        self.assertEqual(detail['peak_depth_ros_stamp'], 100.2)
        self.assertEqual(detail['peak_force_ros_stamp'], 100.5)
        self.assertEqual(detail['positions'][0][0], 9.)
        self.assertFalse(self.m._active)
        self.send([self.state(wall, .003, 4)], 102.)
        self.assertEqual(self.m._actual_collision_count, 2)
        self.assertEqual(self.m._events[0], event)
        self.assertEqual(self.m._events[1]['sample_count'], 1)

    def test_duplicate_and_reversed_pairs_merge_across_frames(self):
        wall = 'toudi2::Wall_9::collision'
        other = 'toudi2::Wall_11::collision'
        reversed_state = self.state(wall, .020, 5)
        reversed_state.collision1_name, reversed_state.collision2_name = (
            reversed_state.collision2_name, reversed_state.collision1_name)
        self.send([self.state(wall, .002, 40), reversed_state], 100.)
        self.send([self.state(other, .050, 10), self.state(wall, .010, 60)], 100.3)
        self.send([self.state(other, .004, 2)], 100.7)
        event = self.m._events[0]
        self.assertEqual(self.m._actual_collision_count, 1)
        self.assertEqual(event['sample_count'], 3)
        self.assertEqual(len(event['pairs']), 2)
        by_name = {item['pair'][1]: item for item in event['details']}
        self.assertEqual(by_name[wall]['sample_count'], 2)
        self.assertEqual(by_name[wall]['max_depth_m'], .020)
        self.assertEqual(by_name[wall]['total_force_norm_n'], 60.)
        self.assertAlmostEqual(by_name[wall]['duration_sec'], .3)
        self.assertEqual(by_name[other]['sample_count'], 2)
        self.assertAlmostEqual(by_name[other]['duration_sec'], .4)
        self.assertEqual(event['peak_sampled_depth_m'], .050)
        self.assertEqual(event['peak_sampled_force_n'], 60.)

    def test_support_depth_and_force_peaks_do_not_hide_obstacles(self):
        support = 'landing_h::link::collision'
        wall = 'toudi2::Wall_9::collision'
        self.send([self.state(support, .001, 60)], 100.)
        self.send([self.state(support, .020, 5), self.state(wall, .007, 8)], 100.4)
        self.send([self.state(support, .003, 90)], 100.6)
        self.send([], 101.)
        event = self.m._support_events[0]
        self.assertEqual(event['peak_sampled_depth_m'], .020)
        self.assertEqual(event['peak_sampled_force_n'], 90.)
        self.assertAlmostEqual(event['duration_sec'], .6)
        self.assertEqual(event['sample_count'], 3)
        self.assertEqual(event['ended_ros_stamp'], 101.)
        self.assertEqual(self.m._actual_collision_count, 1)
        self.assertEqual(self.m._support_count, 1)
        self.assertEqual(self.m._events[0]['peak_sampled_depth_m'], .007)
        self.assertEqual(self.m._events[0]['pairs'][0][1], wall)

    def test_no_depth_then_depth_and_all_support_episodes_retained(self):
        wall = 'toudi2::Wall_9::collision'
        self.send([self.state(wall, None, 50)], 100.)
        self.send([self.state(wall, .004, 5)], 100.2)
        detail = self.m._events[0]['details'][0]
        self.assertEqual(detail['max_depth_m'], .004)
        self.assertEqual(detail['total_force_norm_n'], 50.)
        self.assertEqual(detail['peak_depth_ros_stamp'], 100.2)
        for index in range(65):
            self.send([self.state('ground_plane')], 101. + index)
            self.send([], 101.5 + index)
        self.assertEqual(len(self.m._support_events), 65)
        self.assertEqual(self.m._support_count, 65)

    def test_empty_ignore_pattern_cannot_hide_wall_contact(self):
        self.m._ignored = ('', 'ground_plane')
        self.send([self.state('ground_plane'), self.state('toudi2::Wall_9::collision')])
        self.assertEqual(self.m._actual_collision_count, 1)
        self.assertEqual(len(self.m._support_events[0]['details']), 1)

    def test_publish_persists_updated_episode_not_just_first_sample(self):
        wall = 'toudi2::Wall_9::collision'
        self.send([self.state(wall, .001, 2)], 100.)
        self.send([self.state(wall, .020, 50)], 100.4)
        self.m._raw_topic = '/test/contacts'
        self.m._publisher = mock.Mock()
        with tempfile.TemporaryDirectory() as directory:
            self.m._status_path = str(Path(directory) / 'contacts.json')
            self.module.GazeboContactMonitor._publish(self.m)
            payload = json.loads(Path(self.m._status_path).read_text())
        self.assertEqual(payload['actual_collision_count'], 1)
        self.assertEqual(payload['events'][0]['peak_sampled_depth_m'], .020)
        self.assertAlmostEqual(payload['events'][0]['duration_sec'], .4)
        published = json.loads(self.m._publisher.publish.call_args[0][0].data)
        self.assertEqual(published, payload)

if __name__=='__main__':unittest.main()
