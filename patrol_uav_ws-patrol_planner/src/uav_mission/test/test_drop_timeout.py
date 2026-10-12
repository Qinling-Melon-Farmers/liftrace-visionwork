"""ALIGN age never creates authority: run the normal arbiter/proxy/Bridge.

The controller alone owns the XY fallback clock. These continuous tests use
real commitment/fence policy and production transport methods without ROS nodes.
"""
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace as N

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import release_commitment as policy
from uav_mission.motion_optimization import MotionOptimization
from test_release_transaction_followups import (
    ARB, ROS, Stamp, Clock, Publisher, permission, context, proxy, bridge,
    COMPLETED, action_identity, candidate,
)

for name in ('MODE_TARGET_CLASS', 'strict_context_source', 'strict_commitment_fence',
             'commitment_context_rejection_is_terminal', 'commitment_matches_fence',
             'commitment_rejection_is_terminal'):
    ARB._evaluate.__globals__[name] = getattr(policy, name)


class TimeoutAuthority(unittest.TestCase):
    def setUp(self):
        previous_time = ROS.Time
        class RosTime(Stamp):
            def __init__(self, secs=0, nsecs=0):
                super().__init__(secs + nsecs / 1e9)
            now = staticmethod(Clock.now)
        ROS.Time = RosTime
        self.addCleanup(setattr, ROS, 'Time', previous_time)
        Clock.value = 100.
        self.m = permission()
        self.a = a = ARB.__new__(ARB)
        a._require_evidence_context = True
        a._class_profile = 'r2026'
        a._next_slot, a._payload_slots = 1, 3
        a._blocked_slots, a._revoked_actions = set(), set()
        a._authorized_actions, a._called_actions = {}, {}
        a._released_targets, a._completed_slots = set(), set()
        a._commitment = None
        a._lock = threading.RLock()
        a._required_control_state = a._control_state = 2
        a._align_mode = 'drop_circle'
        a._control_state_stamp = Stamp(100.)
        a._control_state_timeout = a._pose_timeout = a._evidence_timeout = .5
        a._min_altitude, a._max_altitude = .35, .45
        a._pose = N(header=N(stamp=Stamp(100.), frame_id='camera_init'),
                    pose=N(position=N(x=0., y=0., z=.4)))
        a._permission_epoch, a._permission_revision = 'test', 0
        a._permission_lifetime = .25
        a._permission_pub, a._authorization_pub = Publisher(), Publisher()
        a._permission_state_pub, a._commitment_evidence_pub = Publisher(), Publisher()
        a._commitment_policy = policy.ReleaseCommitmentPolicy(2, 45., .20)
        self.c = N(
            context_header=N(stamp=Stamp(100.)), deadline=Stamp(190.),
            context_active=True, context_valid=True, context_schema_version=1,
            context_source='planner_bridge', mission_id=self.m.mission_id,
            decision_seq=self.m.decision_seq, attempt=self.m.attempt, command=2,
            payload_slot=1, class_profile='r2026', align_mode='drop_circle',
            has_semantic_target=True, semantic_geometry_match=True,
            semantic_target_id=self.m.target_id, semantic_target_class='bridge',
            semantic_target_first_seen=self.m.target_first_seen,
            geometry_target_present=True, geometry_map_valid=True,
            geometry_target_id=3, geometry_target_class='circle',
            geometry_target_first_seen=Stamp(98.),
            evidence=N(header=N(stamp=Stamp(100.)), evidence_valid=True,
                       aligned=True, align_mode='drop_circle', target_id=3,
                       target_class='circle', stable_frames=5))
        a._evidence_context, a._evidence = self.c, self.c.evidence

    def advance(self, t, height=.4, visual=False):
        Clock.value = t
        self.c.context_header.stamp = Stamp(t)
        self.a._control_state_stamp = self.a._pose.header.stamp = Stamp(t)
        self.a._pose.pose.position.z = height
        if visual:
            self.c.evidence.header.stamp = Stamp(t)

    def establish(self):
        self.a._maybe_establish_commitment()
        self.assertIsNotNone(self.a._commitment)

    def evaluate(self, t=120., height=.4, visual=False):
        self.advance(t, height, visual)
        return self.a._evaluate(Stamp(t))

    def publish(self):
        ROS.Duration = Stamp
        try:
            self.a._publish_permission(None)
        finally:
            del ROS.Duration
        return self.a._permission_pub.messages[-1]

    def test_twenty_seconds_without_valid_visual_or_commitment_never_grants(self):
        self.c.evidence.evidence_valid = False
        for t in (100., 119.999, 120., 140., 189.):
            self.assertFalse(self.evaluate(t)[0])
        self.assertIsNone(self.a._commitment)

    def test_original_commitment_survives_visual_loss_for_standard_and_cross(self):
        for cross in (False, True):
            self.setUp()
            if cross:
                self.a._align_mode = self.c.align_mode = self.c.evidence.align_mode = 'drop_cross'
                self.c.semantic_target_class = self.c.geometry_target_class = 'red_cross'
                self.c.evidence.target_class = 'red_cross'
                self.c.geometry_target_id = self.c.evidence.target_id = self.c.semantic_target_id
                self.c.geometry_target_first_seen = self.c.semantic_target_first_seen
            self.establish()
            allowed, reason, source = self.evaluate()
            self.assertTrue(allowed)
            self.assertEqual(reason, 'permission_granted_from_commitment')
            self.assertEqual(source['target_id'], self.c.semantic_target_id)

    def test_precision_permission_is_immediate_before_timeout(self):
        allowed, reason, _ = self.evaluate(100.01, visual=True)
        self.assertTrue(allowed)
        self.assertEqual(reason, 'permission_granted')

    def test_high_and_below_release_interval_never_grant_after_twenty_seconds(self):
        self.establish()
        for z in (.349, .451, 1.2, 2.6):
            self.assertFalse(self.evaluate(120., z)[0])
        for z in (.35, .4, .45):
            self.assertTrue(self.evaluate(120., z)[0])

    def test_original_commitment_drift_guard_remains(self):
        self.establish()
        self.a._pose.pose.position.x = .201
        allowed, reason, _ = self.evaluate()
        self.assertFalse(allowed)
        self.assertEqual(reason, 'commitment_position_drift')
        self.a._pose.pose.position.x = .19
        self.assertTrue(self.evaluate(120.1)[0])

    def test_cancel_deadline_changed_identity_and_slot_remain_blocked(self):
        for field in ('cancel', 'deadline', 'target', 'attempt', 'slot', 'duplicate'):
            self.setUp(); self.establish(); self.advance(120.)
            if field == 'cancel': self.a._on_alignment_context(context(self.m))
            if field == 'deadline': self.c.deadline = Stamp(120.)
            if field == 'target': self.c.semantic_target_id += 1
            if field == 'attempt': self.c.attempt += 1
            if field == 'slot': self.a._blocked_slots.add(1)
            if field == 'duplicate': self.a._released_targets.add(('drop_circle', self.c.semantic_target_id))
            self.assertFalse(self.a._evaluate(Stamp(120.))[0], field)

    def test_feedback_freshness_control_and_future_stamps_remain_blocked(self):
        for field in ('pose', 'control', 'stale_control', 'nan', 'future', 'future_control'):
            self.setUp(); self.establish(); self.advance(120.)
            if field == 'pose': self.a._pose.header.stamp = Stamp(100.)
            if field == 'control': self.a._control_state = 1
            if field == 'stale_control': self.a._control_state_stamp = Stamp(100.)
            if field == 'nan': self.a._pose.pose.position.z = float('nan')
            if field == 'future': self.a._pose.header.stamp = Stamp(120.1)
            if field == 'future_control': self.a._control_state_stamp = Stamp(120.1)
            self.assertFalse(self.a._evaluate(Stamp(120.))[0], field)

    def test_authorized_once_ack_recovery_resumes_next_target(self):
        self.establish(); self.evaluate(); p = self.publish()
        self.assertTrue(p.permitted)
        self.assertEqual(p.reason, 'permission_granted_from_commitment')
        self.assertEqual(action_identity(p), action_identity(self.m))
        o = proxy(); o._permission = p
        ok, fact, _ = o._execute_request(1, p)
        self.assertTrue(ok); self.assertEqual(fact, COMPLETED)
        self.assertFalse(o._execute_request(1, p)[0]); self.assertEqual(o._raw_client.calls, 1)
        self.a._on_result(o._result_pub.messages[-1]); self.assertEqual(self.a._next_slot, 2)
        b = bridge(); b._transaction.phase = 'ALIGN_COMMAND_SENT'
        b._transaction.strict_evidence_stamp_ns = 0
        target = b._transaction.decision.target
        for msg in o._result_pub.messages:
            msg.mission_id = b._transaction.decision.mission_id
            msg.decision_seq = b._transaction.decision.decision_seq
            msg.attempt, msg.payload_slot, msg.target_id = target.attempt, target.payload_slot, target.target_id
            msg.target_first_seen = Stamp(target.first_seen_ns / 1e9)
            msg.target_class, msg.align_mode = target.class_name, b._transaction.align_mode
            b._on_release_result(msg)
        self.assertEqual(b._transaction.phase, 'RECOVERY')
        b._update_recovery.__globals__['MotionOptimization'] = MotionOptimization
        b._recovery_height, b._control_state, b._align_mode = 1., 1, 'disabled'
        b._control_state_receipt_ns = b._align_mode_receipt_ns = 120_010_000_000
        b._odom_rejection_reason = lambda sample, now: ''
        for t in (120.1, 120.4, 120.7):
            Clock.value = t; ns = round(t * 1e9)
            b._update_recovery(N(stamp_ns=ns, x=0., y=0., z=1.4), ns)
        self.assertEqual(b._transaction.phase, 'TERMINAL')
        self.assertFalse(b.errors); self.assertEqual(b.core.committed_slots, 1)
        self.assertEqual(b.core.phase.value, 'SEARCH')
        next_action = b.core.choose_confirmed(candidate(target_id=8, class_name='panzer', now=121.), 121., (0., 0.))
        self.assertIsNotNone(next_action); self.assertEqual(next_action.payload_slot, 2)


if __name__ == '__main__':
    unittest.main()
