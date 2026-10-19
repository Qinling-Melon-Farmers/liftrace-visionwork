"""Production Bridge/Core progression without a Servo ACK; ROS-free."""
import copy
from dataclasses import replace
import unittest

from test_release_transaction_followups import (
    BR, Clock, Stamp, NS, bridge as original_bridge, permission, result, COMPLETED, RAW_CALL_STARTED, NOT_STARTED,
)
from uav_mission.motion_optimization import MotionOptimization
from uav_mission.planner_execution import OdomSample
from uav_mission.release_transactions import invocation_consumes_slot

BR['NavigationPlannerBridge']._update_recovery.__globals__['MotionOptimization'] = MotionOptimization


def bridge():
    node = original_bridge()
    node._recover_without_ack = True
    return node


def invocation(**changes):
    message = permission()
    message.request_id = 1
    for key, value in changes.items(): setattr(message, key, value)
    return message


class InvocationRecovery(unittest.TestCase):
    def setUp(self): Clock.value = 100.10

    def test_only_explicit_invocation_policy_source_can_consume_without_ack(self):
        reason='invocation_consumes_slot_ack_not_required'
        self.assertTrue(invocation_consumes_slot(reason,'patrol_control_invocation:1'))
        for source in ('', 'guarded_servo_proxy:1', 'patrol_control_invocation:0',
                       'patrol_control_invocation:bad', 'patrol_control_invocation:1:extra'):
            self.assertFalse(invocation_consumes_slot(reason,source))
        self.assertFalse(invocation_consumes_slot('release_request_sent','patrol_control_invocation:1'))

    def test_invocation_consumes_once_and_starts_recovery_without_ack(self):
        node = bridge()
        node._on_release_invocation(invocation())
        self.assertEqual(node.errors, [])
        self.assertEqual(node._transaction.phase, 'RECOVERY')
        self.assertTrue(node._transaction.release_invocation_observed)
        self.assertFalse(node._transaction.raw_call_observed)
        self.assertEqual(node._transaction.release_ack_ns, 0)
        self.assertEqual(node.core.committed_slots, 1)
        self.assertEqual(node.context_events, [])  # admission context is not prematurely revoked
        node._on_release_invocation(invocation())
        self.assertEqual(len([e for e in node.events if e.payload_committed]), 1)

    def test_foreign_stale_future_expired_and_disabled_invocations_are_ignored(self):
        for changes in [dict(mission_id='other'), dict(decision_seq=2), dict(attempt=2),
                        dict(payload_slot=2), dict(target_id=8), dict(target_class='red_cross'),
                        dict(target_first_seen=Stamp(98)), dict(align_mode='drop_cross'),
                        dict(request_id=0)]:
            node = bridge(); node._on_release_invocation(invocation(**changes))
            self.assertEqual(node.events, [])
        for stamp in (99.9, 101.):
            node = bridge(); message = invocation(); message.header.stamp = Stamp(stamp)
            node._on_release_invocation(message); self.assertEqual(node.events, [])
        for phase in ('EXPIRED', 'CANCEL_PENDING', 'TERMINAL'):
            node = bridge(); node._transaction.phase = phase
            node._on_release_invocation(invocation()); self.assertEqual(node.events, [])
        node=bridge(); node._output_enabled=False
        node._on_release_invocation(invocation()); self.assertEqual(node.events, [])
        node=bridge(); node._recover_without_ack=False
        node._on_release_invocation(invocation()); self.assertEqual(node.events, [])

    def test_late_ack_failure_or_not_started_cannot_reverse_recovery_or_double_count(self):
        for fact in (COMPLETED, RAW_CALL_STARTED, NOT_STARTED):
            node=bridge(); node._on_release_invocation(invocation())
            count=len(node.events)
            node._on_release_result(result(permission(), fact))
            self.assertEqual(node.errors, [])
            self.assertEqual(node._transaction.phase, 'RECOVERY')
            self.assertEqual(len(node.events), count)
            self.assertEqual(node.core.committed_slots, 1)

    def test_ack_or_failure_arriving_before_invocation_cannot_abort_no_ack_policy(self):
        for fact in (COMPLETED, RAW_CALL_STARTED, NOT_STARTED):
            node=bridge(); node._on_release_result(result(permission(),fact))
            self.assertEqual(node.errors, [])
            self.assertEqual(node._transaction.phase,'ALIGNMENT')
            self.assertEqual(node.core.committed_slots,0)
            node._on_release_invocation(invocation())
            self.assertEqual(node.errors, [])
            self.assertEqual(node._transaction.phase,'RECOVERY')
            self.assertEqual(node.core.committed_slots,1)

    def test_recovery_completion_needs_fresh_run_state_height_and_alignment_disabled(self):
        node=bridge(); node._on_release_invocation(invocation())
        node._recovery_height=.95
        node._odom_rejection_reason=lambda sample, now: ''
        node._control_state=1; node._align_mode='disabled'
        node._control_state_receipt_ns=node._align_mode_receipt_ns=round(100.2*NS)
        sample=OdomSample(round(100.2*NS),'camera_init',1.,0.,1.,0.,0.,0.)
        Clock.value=100.2
        node._update_recovery(sample,round(100.2*NS))
        self.assertEqual(node._transaction.phase,'RECOVERY')
        for when in (100.3,100.4,100.5,100.6,100.7,100.8):
            Clock.value=when
            node._update_recovery(replace(sample,stamp_ns=round(when*NS)),round(when*NS))
        self.assertEqual(node.errors, [])
        self.assertEqual(node._transaction.phase,'TERMINAL')
        self.assertTrue(any(e.status=='SUCCEEDED' and e.stage=='RECOVERY' for e in node.events))
        self.assertEqual(node.context_events,[False])
        self.assertEqual(node._transaction.release_ack_ns,0)


if __name__ == '__main__': unittest.main()
