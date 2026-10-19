"""Exercise the production arbiter/proxy with controller descent timeout feedback."""
import copy
from types import SimpleNamespace as N
import unittest

from test_drop_timeout import TimeoutAuthority
from test_release_transaction_followups import Stamp, Clock, proxy, COMPLETED, action_identity


class DescentTimeoutRelease(TimeoutAuthority):
    def signal(self, t=120., height=1.2):
        self.advance(t, height)
        self.a._descent_timeout_enabled = True
        c = self.c
        self.a._on_alignment_context(N(
            header=N(stamp=Stamp(t)), active=True, has_target=True, schema_version=1,
            command=2, source='planner_bridge', class_profile='r2026', deadline=c.deadline,
            target_pose=N(header=N(frame_id='camera_init')),
            mission_id=c.mission_id, decision_seq=c.decision_seq, attempt=c.attempt,
            payload_slot=c.payload_slot, semantic_target_id=c.semantic_target_id,
            semantic_target_class=c.semantic_target_class, align_mode=c.align_mode,
            semantic_target_first_seen=c.semantic_target_first_seen))
        self.feedback = N(
            header=N(stamp=Stamp(t), frame_id='camera_init'), odom_stamp=Stamp(t), observation_stamp=Stamp(100.),
            valid=True, frozen=True, reason='descent_timeout_ready',
            horizontal_error_m=.8, horizontal_speed_mps=.6,
            mission_id=c.mission_id, decision_seq=c.decision_seq, attempt=c.attempt,
            payload_slot=c.payload_slot, semantic_target_id=c.semantic_target_id,
            semantic_target_class=c.semantic_target_class, align_mode=c.align_mode,
            semantic_target_first_seen=c.semantic_target_first_seen)
        self.a._on_drop_alignment_feedback(self.feedback)

    def test_timeout_bypasses_height_drift_and_lost_visual_for_both_classes(self):
        for cross in (False, True):
            for height in (-.1, .349, .451, 1.2, 2.6):
                self.setUp()
                if cross:
                    self.a._align_mode = self.c.align_mode = 'drop_cross'
                    self.c.semantic_target_class = 'red_cross'
                self.c.evidence.evidence_valid = False
                self.a._pose.pose.position.x = .8
                self.signal(height=height)
                allowed, reason, source = self.a._evaluate(Stamp(120.))
                self.assertTrue(allowed)
                self.assertEqual(reason, 'descent_timeout_release')
                self.assertEqual(source['target_id'], self.c.semantic_target_id)

    def test_no_controller_timeout_feedback_never_creates_authority(self):
        self.c.evidence.evidence_valid = False
        self.signal()
        for reason in ('compensated_not_ready', 'frozen_capture_geometry_valid'):
            self.feedback.reason = reason
            self.assertFalse(self.a._evaluate(Stamp(120.))[0])
        self.feedback.reason = 'descent_timeout_ready'
        self.feedback.frozen = False
        self.assertFalse(self.a._evaluate(Stamp(120.))[0])

    def test_timeout_keeps_identity_freshness_deadline_cancel_and_slot_guards(self):
        for failure in ('cancel', 'deadline', 'context_stale', 'feedback_stale', 'odom_stale',
                        'future', 'invalid', 'identity', 'first_seen', 'slot', 'duplicate',
                        'no_pose', 'no_control', 'nonfinite_pose', 'disabled', 'old_entry', 'frame'):
            self.setUp(); self.c.evidence.evidence_valid = False; self.signal()
            a, c, f = self.a, self.a._alignment_context, self.feedback
            if failure == 'cancel':
                cancelled = copy.deepcopy(c); cancelled.active = False
                a._on_alignment_context(cancelled)
            if failure == 'deadline': c.deadline = Stamp(120.)
            if failure == 'context_stale': c.header.stamp = Stamp(119.)
            if failure == 'feedback_stale': f.header.stamp = Stamp(119.)
            if failure == 'odom_stale': f.odom_stamp = Stamp(119.)
            if failure == 'future': f.header.stamp = Stamp(120.1)
            if failure == 'invalid': f.valid = False
            if failure == 'identity': f.attempt += 1
            if failure == 'first_seen': f.semantic_target_first_seen = Stamp(98.)
            if failure == 'slot': a._blocked_slots.add(1)
            if failure == 'duplicate': a._released_targets.add(('drop_circle', c.semantic_target_id))
            if failure == 'no_pose': a._pose = None
            if failure == 'no_control': a._control_state = 1
            if failure == 'nonfinite_pose': a._pose.pose.position.z = float('nan')
            if failure == 'disabled': a._descent_timeout_enabled = False
            if failure == 'old_entry': a._require_evidence_context = False
            if failure == 'frame': f.header.frame_id = 'other_frame'
            self.assertFalse(a._evaluate(Stamp(120.))[0], failure)

    def test_permission_preserves_timeout_identity_and_raw_call_happens_once(self):
        self.signal()
        # A queued visual wrapper for another target must not overwrite timeout identity.
        self.c.semantic_target_id += 1
        p = self.publish()
        self.assertTrue(p.permitted)
        self.assertEqual(p.reason, 'descent_timeout_release')
        self.assertEqual(action_identity(p), action_identity(self.m))
        actuator = proxy(); actuator._permission = p
        success, fact, _ = actuator._execute_request(1, p)
        self.assertTrue(success); self.assertEqual(fact, COMPLETED)
        self.assertFalse(actuator._execute_request(1, p)[0])
        self.assertEqual(actuator._raw_client.calls, 1)
        self.a._on_result(actuator._result_pub.messages[-1])
        self.assertEqual(self.a._next_slot, 2)

    def test_future_feedback_cannot_poison_timestamp_watermark(self):
        self.signal()
        future = copy.deepcopy(self.feedback); future.header.stamp = Stamp(500.)
        self.a._on_drop_alignment_feedback(future)
        self.assertIs(self.a._drop_alignment_feedback, self.feedback)
        Clock.value = 120.1
        valid = copy.deepcopy(self.feedback); valid.header.stamp = valid.odom_stamp = Stamp(120.1)
        self.a._on_drop_alignment_feedback(valid)
        self.assertIs(self.a._drop_alignment_feedback, valid)

    def test_future_or_old_context_cannot_replace_active_action(self):
        self.signal()
        current = self.a._alignment_context
        for stamp in (500., 100.):
            invalid = copy.deepcopy(current); invalid.header.stamp = Stamp(stamp)
            invalid.semantic_target_id += 1
            self.a._on_alignment_context(invalid)
            self.assertIs(self.a._alignment_context, current)


if __name__ == '__main__':
    unittest.main()
