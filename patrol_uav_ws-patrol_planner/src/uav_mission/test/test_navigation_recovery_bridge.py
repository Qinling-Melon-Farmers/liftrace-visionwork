#!/usr/bin/env python3
"""Execute production bridge methods with real ROS message/time classes.

No ROS node/master is started. Publishers and the mission executor are local
test doubles; publication method bodies are extracted verbatim from production.
"""
import ast
import copy
from contextlib import nullcontext
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rospy
from geometry_msgs.msg import PoseStamped
from navigation_recovery_msgs.msg import NavigationRecoveryContext


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


@dataclass
class Config:
    arrival_distance_m: float = .18
    arrival_dwell_ns: int = 200000000


class BridgeRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'scripts/navigation_planner_bridge.py'
        tree = ast.parse(path.read_text())
        bodies = []
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == '_ns_to_stamp':
                bodies.append(node)
            if isinstance(node, ast.ClassDef) and node.name == 'NavigationPlannerBridge':
                bodies.extend(n for n in node.body if isinstance(n, ast.FunctionDef) and
                              n.name in ('_publish_planner_goal', '_on_decision'))
        namespace = dict(rospy=rospy, PoseStamped=PoseStamped,
                         NavigationRecoveryContext=NavigationRecoveryContext,
                         replace=replace, WIRE_GOAL_COMMANDS={'SEARCH', 'RESUME', 'APPROACH', 'RETURN_HOME'},
                         _seconds_to_ns=lambda name, value: int(value*1e9))
        exec(compile(ast.fix_missing_locations(ast.Module(body=bodies, type_ignores=[])), str(path), 'exec'), namespace)
        cls.publish = staticmethod(namespace['_publish_planner_goal'])
        cls.on_decision = staticmethod(namespace['_on_decision'])

    def decision(self, command='SEARCH'):
        return SimpleNamespace(decision_seq=41, issued_at_ns=10000000001,
                               deadline_ns=45000000009, command=command,
                               goal=SimpleNamespace(frame_id='camera_init', x=1., y=2., z=2.7,
                                                    qx=0., qy=0., qz=0., qw=1.))

    def bridge(self):
        return SimpleNamespace(_output_enabled=True, _goal_pub=Publisher(), _recovery_context_pub=Publisher())

    def test_original_identity_and_deadline_preserved_exactly(self):
        bridge, decision = self.bridge(), self.decision()
        self.publish(bridge, decision)
        goal = bridge._goal_pub.messages[0]
        context = bridge._recovery_context_pub.messages[0]
        self.assertTrue(context.active)
        self.assertEqual(context.header, goal.header)
        self.assertEqual(context.header.stamp.to_nsec(), decision.issued_at_ns)
        self.assertEqual(context.deadline.to_nsec(), decision.deadline_ns)

    def test_republication_does_not_start_a_new_budget(self):
        bridge, decision = self.bridge(), self.decision()
        self.publish(bridge, decision)
        self.publish(bridge, decision)
        a, b = bridge._recovery_context_pub.messages
        self.assertEqual(a, b)
        self.assertEqual(b.deadline.to_nsec(), 45000000009)

    def test_default_disabled_does_not_publish_recovery_context(self):
        bridge = self.bridge()
        bridge._recovery_context_pub = None
        self.publish(bridge, self.decision())
        self.assertEqual(len(bridge._goal_pub.messages), 1)

    def test_accepted_non_navigation_decision_revokes_context(self):
        for command in ('HOLD', 'ABORT', 'ALIGN', 'LAND'):
            bridge, decision = self.bridge(), self.decision(command)
            bridge._lock = nullcontext()
            bridge._now_ns = lambda: 11000000000
            bridge._decision_from_message = lambda m, t: decision
            bridge._mission_frame = 'camera_init'
            bridge._executor = SimpleNamespace(config=Config(), submit_decision=lambda d, t: SimpleNamespace(accepted=True))
            bridge._apply_outcome = lambda *a, **k: None
            bridge._start_decision_handoff = lambda *a: None
            bridge._publish_status = lambda **k: None
            bridge._transaction = None
            failures = []
            bridge._handle_callback_exception = lambda *a: failures.append(a)
            with patch.object(rospy, 'get_param', side_effect=lambda name, default: default):
                self.on_decision(bridge, SimpleNamespace(reason='test'))
            self.assertFalse(failures)
            context = bridge._recovery_context_pub.messages[0]
            self.assertFalse(context.active)
            self.assertEqual(context.deadline.to_nsec(), decision.deadline_ns)


if __name__ == '__main__':
    unittest.main()
