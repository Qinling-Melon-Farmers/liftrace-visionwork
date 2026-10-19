"""Direct landing must finish on confirmed touchdown, not an OFFBOARD label."""
import unittest
from types import SimpleNamespace as NS
import test_landing_posctl_handoff as fixtures


class OffboardCompletion(unittest.TestCase):
    def setup_bridge(self):
        helper = fixtures.Handoff()
        bridge = helper.bridge('OFFBOARD')
        helper.state(bridge, mode='OFFBOARD')
        return helper, bridge

    def sample(self, bridge):
        return NS(x=1., y=2., z=.22, stamp_ns=bridge.t)

    def ground(self, bridge):
        bridge._landed_state = 1
        bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = bridge.t

    def test_remaining_offboard_is_not_completion(self):
        helper, bridge = self.setup_bridge()
        helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
        bridge.t += 3_000_000_000
        helper.state(bridge, mode='OFFBOARD')
        bridge._update_landing(self.sample(bridge), bridge.t)
        self.assertFalse(bridge.outcomes)  # No POSCTL transition timeout here.
        self.assertIsNotNone(bridge._landing)

    def test_armed_ground_is_not_completion_even_with_event(self):
        helper, bridge = self.setup_bridge()
        self.ground(bridge)
        helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
        helper.event(bridge, 'OBSERVED', mode='OFFBOARD')
        bridge._update_landing(self.sample(bridge), bridge.t)
        self.assertFalse(bridge.outcomes)

    def test_disarmed_ground_without_controller_completion_waits(self):
        helper, bridge = self.setup_bridge()
        self.ground(bridge)
        helper.state(bridge, mode='OFFBOARD', armed=False)
        helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
        bridge._update_landing(self.sample(bridge), bridge.t)
        self.assertFalse(bridge.outcomes)

    def test_complete_accepts_either_callback_order_and_is_idempotent(self):
        for events_first in (False, True):
            helper, bridge = self.setup_bridge()
            def events():
                helper.event(bridge, 'OBSERVED', mode='OFFBOARD')
                helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
            if events_first: events()
            self.ground(bridge)
            helper.state(bridge, mode='OFFBOARD', armed=False)
            if not events_first: events()
            bridge._update_landing(self.sample(bridge), bridge.t)
            bridge._update_landing(self.sample(bridge), bridge.t)
            self.assertEqual(len(bridge.outcomes), 1)
            self.assertEqual(bridge.outcomes[0][2], 'SUCCEEDED')

    def test_wrong_identity_does_not_complete(self):
        helper, bridge = self.setup_bridge()
        self.ground(bridge)
        helper.state(bridge, mode='OFFBOARD', armed=False)
        helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
        helper.event(bridge, 'OBSERVED', mode='OFFBOARD', decision_seq=41)
        bridge._update_landing(self.sample(bridge), bridge.t)
        self.assertFalse(bridge.outcomes)

    def test_stale_ground_or_state_does_not_complete(self):
        for stale_field in ('_landed_state_source_ns', '_flight_state_source_ns'):
            helper, bridge = self.setup_bridge()
            self.ground(bridge)
            helper.state(bridge, mode='OFFBOARD', armed=False)
            helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
            helper.event(bridge, 'OBSERVED', mode='OFFBOARD')
            setattr(bridge, stale_field, bridge.t - 3_000_000_000)
            bridge._update_landing(self.sample(bridge), bridge.t)
            self.assertFalse(bridge.outcomes)

    def test_confirmed_disarm_mode_change_does_not_erase_touchdown(self):
        helper, bridge = self.setup_bridge()
        helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
        self.ground(bridge)
        helper.state(bridge, mode='MANUAL', armed=False)
        helper.event(bridge, 'OBSERVED', mode='OFFBOARD')
        bridge._update_landing(self.sample(bridge), bridge.t)
        self.assertEqual(bridge.outcomes[0][2], 'SUCCEEDED')

    def test_airborne_takeover_cancels(self):
        helper, bridge = self.setup_bridge()
        helper.event(bridge, 'REQUESTED', mode='OFFBOARD')
        helper.state(bridge, mode='POSCTL')
        self.assertIsNone(bridge._landing)
        self.assertEqual(bridge.outcomes[0][2], 'CANCELLED')

if __name__ == '__main__':
    unittest.main()
