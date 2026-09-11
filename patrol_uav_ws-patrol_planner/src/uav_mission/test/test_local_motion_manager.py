"""Offline ROS-shell checks: call methods on an unstarted manager only."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock,patch
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_mission.mission_core import MissionConfig
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_local_motion import make_runtime,proposal,memory,POSE

PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('local_manager_test',PACKAGE/'scripts/navigation_mission_manager.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class LocalManagerTest(unittest.TestCase):
    def setUp(self):
        n=module.NavigationMissionManager.__new__(module.NavigationMissionManager)
        self.node=n;n._lock=threading.RLock();n._local_enabled=True;n._runtime=make_runtime()
        n._local_memory=memory();n._local_input_reason='';n._last_reason='search'
        n._pose=PoseStamped();n._pose.header.frame_id='camera_init'
        n._pose.header.stamp=rospy.Time.from_sec(100.95)
        n._pose.pose.position.x,n._pose.pose.position.y,n._pose.pose.position.z=POSE
        n._readiness=Mock(return_value=(True,'ready'))
        n._publish_action=Mock();n._publish_status=Mock()

    def send(self,data):
        text=data if isinstance(data,str) else json.dumps(data)
        with patch.object(rospy.Time,'now',return_value=rospy.Time(101)):
            self.node._on_local_proposal(String(data=text))

    def test_advice_uses_existing_manager_action_publisher(self):
        self.send(proposal(self.node._runtime.core.active_action))
        action=self.node._publish_action.call_args[0][0]
        self.assertTrue(action.reason.startswith('research_local_entry:'))
        self.assertEqual(self.node._runtime.route.current_index,0)
        self.node._publish_action.assert_called_once()

    def test_disabled_or_unready_does_not_publish_new_action(self):
        packet=proposal(self.node._runtime.core.active_action)
        self.node._local_enabled=False;self.send(packet)
        self.node._local_enabled=True;self.node._readiness.return_value=(False,'pose_stale');self.send(packet)
        self.node._publish_action.assert_not_called()

    def test_invalid_memory_or_message_preserves_running_action(self):
        old=self.node._runtime.core.active_action
        self.node._on_local_memory(String(data='[]'))
        self.assertIsNone(self.node._local_memory)
        for packet in ['{broken','x'*65537,proposal(old)]:self.send(packet)
        self.node._publish_action.assert_not_called();self.assertIs(self.node._runtime.core.active_action,old)

    def test_restarted_memory_rejects_queued_advice(self):
        packet=proposal(self.node._runtime.core.active_action)
        self.node._on_local_memory(String(data=json.dumps(memory(generation=3))))
        self.send(packet);self.node._publish_action.assert_not_called()

    def test_runtime_uses_actual_search_bounds_and_immutable_enable_choice(self):
        n=self.node;n._mission_config=Mock(return_value=MissionConfig())
        n._profile_path=str(PACKAGE/'config/competition_profiles.yaml');n._profile_name='r2026'
        values={'~search/min_x':-4.3,'~search/max_x':4.3,'~search/min_y':0.,'~search/max_y':7.1}
        with patch.object(rospy,'get_param',side_effect=lambda key,default=None:values.get(key,default)):
            runtime=n._new_runtime()
        self.assertTrue(runtime.local_config.enabled)
        self.assertEqual(runtime.local_config.bounds,(-4.3,4.3,0.,7.1))


if __name__=='__main__':unittest.main()
