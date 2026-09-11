"""Offline transport/identity tests, never instantiate ROS subscriptions."""
import ast
import importlib.util
import json
import threading
import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from uav_mission.msg import NavigationDecision
from uav_coverage_memory.local_proposals import ProposalConfig,LocalProposer
from uav_coverage_memory.queries import MissionLedger

PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('local_adviser_test',PACKAGE/'scripts/local_search_adviser.py')
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)


class AdviserTests(unittest.TestCase):
    def setUp(self):
        self.node=adapter.LocalSearchAdviser.__new__(adapter.LocalSearchAdviser)
        n=self.node;n.lock=threading.RLock();n.proposer=LocalProposer(ProposalConfig((-4,4,-.5,7)))
        n.grids=deque(maxlen=4);n.statuses=deque(maxlen=4)
        n.ledger=MissionLedger();n.snapshot=None;n.blocked_memory=None
        n.planner_session=None;n.planner_revision=0
        n.pending=None;n.last_wall=-float('inf');n.last_ros=10.;n.rate=1.
        n.record=None;n.record_count=0;n.record_limit=1500
        n.pose=PoseStamped();n.pose.header.frame_id='map';n.pose.header.stamp=rospy.Time(10)
        n.pose.pose.position.x=-3;n.pose.pose.position.y=2;n.pose.pose.position.z=1.2
        n.command=NavigationDecision();m=n.command
        m.schema_version=1;m.mission_id='mission';m.decision_seq=1;m.command=m.SEARCH;m.has_goal=True
        m.goal.header.frame_id='map';m.goal.pose.position.x=4;m.goal.pose.position.y=2;m.goal.pose.position.z=1.2;m.deadline=rospy.Time(50)
        n.mission=dict(mission_id='mission',phase='SEARCH',active_command='SEARCH',active_decision_seq=1)
        g=OccupancyGrid();g.header.frame_id='map';g.header.stamp=rospy.Time(10)
        g.info.width=100;g.info.height=90;g.info.resolution=float(np.float32(.1))
        g.info.origin.position.x=-5;g.info.origin.position.y=-1;g.info.origin.orientation.w=1
        g.data=[0]*9000;n.grids.append(g)
        n.statuses.append(dict(epoch='mission',generation=2,receipt_ros=10.,frame_id='map',
            grid_geometry=dict(min_x=-5,max_x=5,min_y=-1,max_y=8,resolution=.1,ground_z=0,frame_id='map')))

    def test_float32_grid_resolution_and_matched_identity_accepted(self):
        context=self.node.context(10.1)
        self.assertEqual(self.node.observation(context,10.1).states.shape,(90,100))

    def test_mismatched_grid_and_status_not_fused(self):
        self.node.grids[0].header.stamp=rospy.Time(11)
        with self.assertRaises(ValueError):self.node.observation(self.node.context(10.1),10.1)

    def test_nan_or_wrong_origin_rejected(self):
        for x in [float('nan'),2.]:
            self.node.grids[0].info.origin.position.x=x
            with self.assertRaises(ValueError):self.node.observation(self.node.context(10.1),10.1)

    def test_phase_or_decision_change_blocks_advice(self):
        for update in [dict(phase='APPROACH'),dict(active_decision_seq=2),dict(mission_id='other')]:
            saved=self.node.mission.copy();self.node.mission.update(update)
            with self.assertRaises(ValueError):self.node.context(10.1)
            self.node.mission=saved

    def test_input_caches_bounded_and_oversized_cleared(self):
        for i in range(10):
            self.node.on_grid(self.node.grids[-1]);self.node.on_status(String(data='{}'))
        self.assertEqual(len(self.node.grids),4);self.assertEqual(len(self.node.statuses),4)
        self.node.on_status(String(data='x'*65537));self.assertEqual(len(self.node.statuses),0)
        huge=OccupancyGrid();huge.data=[0]*20001;self.node.on_grid(huge)
        self.assertEqual(len(self.node.grids),0)

    def test_pending_rpc_never_starts_a_second_worker(self):
        self.node.pending=SimpleNamespace(is_alive=lambda:True)
        with patch.object(rospy.Time,'now',return_value=rospy.Time(10)), \
             patch.object(self.node,'query',side_effect=AssertionError('second RPC forbidden')), \
             patch.object(self.node,'publish') as pub:
            self.node.tick(None)
        self.assertEqual(pub.call_args[0][0]['reason'],'probe_pending_no_second_request')

    def test_planner_restart_requires_new_memory_generation(self):
        n=self.node;context=n.context(10.1);snapshot=n.observation(context,10.1)
        batch=n.proposer.prepare(snapshot,context,10.1)
        n.planner_session='old';n.planner_revision=50
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.2)),patch.object(n,'publish') as pub:
            n.complete(batch,dict(planner_session='new',map_revision=1))
        self.assertIn('memory_reset',pub.call_args[0][0]['reason'])
        with self.assertRaises(ValueError):n.observation(context,10.2)
        n.statuses[-1]['generation']=3
        self.assertEqual(n.observation(context,10.2).generation,3)

    def test_only_json_advice_publisher_no_flight_command_authority(self):
        source=(PACKAGE/'scripts/local_search_adviser.py').read_text()
        tree=ast.parse(source)
        calls=[x for x in ast.walk(tree) if isinstance(x,ast.Call) and isinstance(x.func,ast.Attribute)]
        self.assertEqual([x.args[0].value for x in calls if x.func.attr=='Publisher'],['~proposal'])
        self.assertNotIn('/fastplanner/goal',source)
        self.assertFalse(any(x.func.attr=='Service' for x in calls))

    def test_recording_is_bounded_and_keeps_failure_reason(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'advice.jsonl';n=self.node
            n.pub=Mock();n.record=path.open('x');n.record_limit=1
            for i in range(3):n.publish(dict(accepted=False,reason='probe_pending'),10.+i)
            n.close()
            rows=[json.loads(x) for x in path.read_text().splitlines()]
            self.assertEqual(len(rows),1);self.assertEqual(rows[0]['reason'],'probe_pending')
            last=json.loads(n.pub.publish.call_args[0][0].data)
            self.assertTrue(last['record_limit_reached']);self.assertFalse(last['flight_authorized'])

    def test_recording_io_failure_does_not_suppress_advice_status(self):
        n=self.node;n.pub=Mock();n.record=Mock()
        n.record.write.side_effect=OSError('fixture disk full')
        n.record.close.side_effect=OSError('fixture close failed')
        with patch.object(rospy,'logwarn_throttle'):
            n.publish(dict(accepted=False,reason='no_candidate'),10.)
        self.assertIsNone(n.record);n.pub.publish.assert_called_once()

    def test_local_execution_and_used_budget_suppress_redundant_queries(self):
        self.node.command.reason='research_local_entry:1'
        with self.assertRaises(ValueError):self.node.context(10.1)
        self.node.command.reason='coverage_waypoint'
        self.node.mission['local_motion']=dict(enabled=True,remaining_for_waypoint=0)
        with self.assertRaises(ValueError):self.node.context(10.1)


if __name__=='__main__':unittest.main()
