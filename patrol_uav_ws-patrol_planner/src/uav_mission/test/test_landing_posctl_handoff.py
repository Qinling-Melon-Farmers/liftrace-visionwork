"""Execute production bridge methods without starting ROS or flight hardware."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import unittest
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from uav_mission.position_settle import PositionSettleWindow


SOURCE = Path(__file__).resolve().parents[1] / 'scripts/navigation_planner_bridge.py'
TREE = ast.parse(SOURCE.read_text(encoding='utf-8'))
CLASS = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'NavigationPlannerBridge')
METHODS = {'_on_landing_handoff', '_on_flight_state', '_check_landing_mode_handoff',
           '_cancel_landing_handoff', '_update_landing', '_mission_command_message',
           '_publish_mission_command'}
def command_message():
    return NS(header=NS(), command=0, target_id=0, target_class='',
              goal=NS(header=NS(), pose=NS(position=NS(), orientation=NS())))
NAMESPACE = dict(json=json, math=math, _stamp_to_ns=lambda value: value,
                 ExtendedState=NS(LANDED_STATE_ON_GROUND=1), MissionCommand=command_message,
                 MISSION_COMMAND_VALUES={'LAND': 5, 'ALIGN': 2},
                 rospy=NS(Time=NS(now=lambda: 100_000_000_000)))
for method in CLASS.body:
    if isinstance(method, ast.FunctionDef) and method.name in METHODS:
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), NAMESPACE)
Bridge = type('ProductionBridge', (), {name: NAMESPACE[name] for name in METHODS})


class Handoff(unittest.TestCase):
    def bridge(self, mode='POSCTL'):
        bridge = Bridge()
        bridge.t = 100_000_000_000
        bridge._now_ns = lambda: bridge.t
        bridge._lock = threading.RLock()
        bridge._landing_handoff_mode = mode
        bridge._landing_handoff_wait_ns = 500_000_000
        bridge._landing_mode_transition_timeout_ns = 2_500_000_000
        bridge._landing_state_max_age_ns = 2_500_000_000
        bridge._flight_state = None
        bridge._flight_state_receipt_ns = bridge._flight_state_source_ns = 0
        bridge._landed_state = 2
        bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = 0
        bridge._landing = NS(started=True, decision=NS(mission_id='mission', decision_seq=42),
            wire_command_stamp_ns=99_000_000_000, command_sent_ns=99_000_000_000,
            target_pose=NS(x=1., y=2.), handoff_requested_ns=0, handoff_observed_ns=0,
            posctl_wait_started_ns=0, posctl_authorized=False)
        bridge.outcomes = []
        def report(*args):
            bridge.outcomes.append(args)
            return NS(accepted=True)
        bridge._executor = NS(config=NS(odom_max_age_ns=500_000_000), report_landing=report)
        bridge._apply_outcome = lambda outcome: None
        bridge.resets = []
        bridge._landing_settle = NS(reset=bridge.resets.append, update=lambda *args: NS(ready=True))
        bridge._handle_callback_exception = lambda source, error: (_ for _ in ()).throw(error)
        bridge._odom_rejection_reason = lambda sample, now: ''
        bridge._landing_radius = .15
        bridge._landing_height = .3
        bridge._control_state = 3
        bridge._control_state_receipt_ns = 99_000_000_000
        return bridge

    def state(self, bridge, mode='POSCTL', **kwargs):
        message = NS(header=NS(stamp=bridge.t), mode=mode, connected=True, armed=True)
        for name, value in kwargs.items(): setattr(message, name, value)
        bridge._on_flight_state(message)

    def event(self, bridge, stage_name, **changes):
        data = dict(mission_id='mission', decision_seq=42, command_stamp_ns=99_000_000_000,
                    event_stamp_ns=bridge.t, mode='POSCTL', stage=stage_name)
        data.update(changes)
        for key in ('command_stamp_ns', 'event_stamp_ns'):
            data[key] = str(data[key])
        bridge._on_landing_handoff(NS(data=json.dumps(data)))

    def authorize(self, bridge, state_first=False, reverse_status=False):
        if state_first: self.state(bridge)
        for stage in (('OBSERVED', 'REQUESTED') if reverse_status else ('REQUESTED', 'OBSERVED')):
            self.event(bridge, stage)
        if not state_first: self.state(bridge)
        self.assertTrue(bridge._landing.posctl_authorized)
        self.assertFalse(bridge.outcomes)

    def test_normal_and_cross_topic_ordering_wait_for_actual_landing(self):
        for state_first in (False, True):
            for reverse in (False, True):
                bridge = self.bridge()
                self.authorize(bridge, state_first, reverse)
                bridge._update_landing(NS(x=1., y=2., z=.2, stamp_ns=bridge.t), bridge.t)
                self.assertFalse(bridge.outcomes)  # Mode handoff is not landed evidence.
                bridge._landed_state = 1
                bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = bridge.t
                bridge._update_landing(NS(x=1., y=2., z=.2, stamp_ns=bridge.t), bridge.t)
                self.assertEqual(bridge.outcomes[0][2], 'SUCCEEDED')

    def test_land_wire_identity_is_bound_without_changing_alignment_target_class(self):
        bridge = self.bridge()
        bridge._mission_frame = 'camera_init'
        messages = []
        bridge._mission_command_pub = NS(publish=messages.append)
        decision = NS(mission_id='mission', decision_seq=42, target=None, goal=None)
        bridge._publish_mission_command(decision, 'LAND')
        self.assertEqual(messages[0].target_class, 'mission')
        self.assertEqual(messages[0].goal.header.seq, 42)
        self.assertEqual(bridge._landing.wire_command_stamp_ns, messages[0].goal.header.stamp)
        decision.target = NS(target_id=7, class_name='panzer')
        bridge._publish_mission_command(decision, 'ALIGN')
        self.assertEqual(messages[1].target_class, 'panzer')

    def test_request_ack_without_observed_mode_expires_on_timer_check(self):
        bridge = self.bridge()
        self.event(bridge, 'REQUESTED')
        self.state(bridge)
        bridge.t += 499_999_999
        bridge._check_landing_mode_handoff(bridge.t)
        self.assertFalse(bridge.outcomes)
        bridge.t += 1
        bridge._check_landing_mode_handoff(bridge.t)
        self.assertIsNone(bridge._landing)
        self.assertEqual(bridge.outcomes[0][2], 'CANCELLED')

    def test_early_pilot_posctl_and_ground_before_confirmation_do_not_succeed(self):
        bridge = self.bridge()
        self.state(bridge)
        bridge._landed_state = 1
        bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = bridge.t
        bridge._update_landing(NS(x=1., y=2., z=.2, stamp_ns=bridge.t), bridge.t)
        self.assertFalse(bridge.outcomes)
        bridge.t += 500_000_000
        bridge._check_landing_mode_handoff(bridge.t)
        self.assertIsNone(bridge._landing)

    def test_bad_identity_old_future_and_unknown_status_do_not_authorize(self):
        for bad in (dict(mission_id='old'), dict(decision_seq=41), dict(decision_seq=True),
                    dict(command_stamp_ns=98_000_000_000), dict(event_stamp_ns=99_499_999_999),
                    dict(event_stamp_ns=100_000_000_001), dict(event_stamp_ns=100000000000.0),
                    dict(mode='AUTO.LAND'), dict(stage='OTHER')):
            bridge = self.bridge()
            self.event(bridge, 'REQUESTED', **bad)
            self.event(bridge, 'OBSERVED', **bad)
            self.state(bridge)
            bridge.t += 500_000_000
            bridge._check_landing_mode_handoff(bridge.t)
            self.assertIsNone(bridge._landing, bad)

    def test_cached_matching_status_before_control_acceptance_is_retained(self):
        bridge = self.bridge()
        bridge._landing.started = False
        self.event(bridge, 'REQUESTED')
        self.event(bridge, 'OBSERVED')
        bridge._landing.started = True
        self.state(bridge)
        self.assertTrue(bridge._landing.posctl_authorized)

    def test_cached_status_must_still_be_fresh_at_mode_authorization(self):
        bridge = self.bridge()
        self.event(bridge, 'REQUESTED'); self.event(bridge, 'OBSERVED')
        bridge.t += 500_000_001
        self.state(bridge)
        self.assertFalse(bridge._landing.posctl_authorized)
        bridge.t += 500_000_000
        bridge._check_landing_mode_handoff(bridge.t)
        self.assertIsNone(bridge._landing)

    def test_ground_and_disarm_first_do_not_bypass_posctl_identity(self):
        bridge = self.bridge()
        bridge._landed_state = 1
        bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = bridge.t
        self.state(bridge, armed=False)
        bridge._update_landing(NS(x=1., y=2., z=.2, stamp_ns=bridge.t), bridge.t)
        self.assertFalse(bridge.outcomes)
        bridge.t += 500_000_000
        bridge._check_landing_mode_handoff(bridge.t)
        self.assertIsNone(bridge._landing)

    def test_offboard_return_or_other_airborne_mode_cancels_authorized_transaction(self):
        for mode in ('OFFBOARD', 'MANUAL', 'AUTO.LAND'):
            bridge = self.bridge()
            self.authorize(bridge)
            self.state(bridge, mode)
            self.assertIsNone(bridge._landing)
            self.assertEqual(bridge.outcomes[0][2], 'CANCELLED')

    def test_controller_cancellation_and_stale_actual_mode_remain_closed(self):
        bridge = self.bridge()
        self.event(bridge, 'CANCELLED')
        self.assertIsNone(bridge._landing)
        bridge = self.bridge()
        self.event(bridge, 'REQUESTED'); self.event(bridge, 'OBSERVED')
        self.state(bridge)
        self.assertTrue(bridge._landing.posctl_authorized)
        bridge.t += 2_500_000_001
        bridge._landed_state = 1
        bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = bridge.t
        bridge._update_landing(NS(x=1., y=2., z=.2, stamp_ns=bridge.t), bridge.t)
        self.assertFalse(bridge.outcomes)  # A stale flight state cannot complete.

    def test_normal_ground_disarm_keeps_handoff_but_stale_ground_cannot_complete(self):
        bridge = self.bridge()
        self.authorize(bridge)
        self.state(bridge, armed=False)
        self.assertIsNotNone(bridge._landing)
        bridge._landed_state = 1
        bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = bridge.t - 2_500_000_001
        bridge._update_landing(NS(x=1., y=2., z=.2, stamp_ns=bridge.t), bridge.t)
        self.assertFalse(bridge.outcomes)

    def test_default_auto_land_and_early_manual_cancel_are_preserved(self):
        bridge = self.bridge('AUTO.LAND')
        self.state(bridge, 'AUTO.LAND')
        self.assertIsNotNone(bridge._landing)
        self.assertFalse(bridge.outcomes)
        self.state(bridge, 'POSCTL')
        self.assertIsNone(bridge._landing)

    def test_one_hz_mavros_with_fifty_hz_odom_completes_existing_settle_dwell(self):
        for dwell in (800_000_000, 1_000_000_000):
            bridge = self.bridge()
            bridge._landing_height = .05
            self.authorize(bridge)
            bridge._landing_settle = PositionSettleWindow(dwell, .15, 500_000_000)
            origin = bridge.t
            for i in range(61):
                bridge.t = origin + i * 20_000_000
                if i % 50 == 0:
                    self.state(bridge)
                    bridge._landed_state = 1
                    bridge._landed_state_receipt_ns = bridge._landed_state_source_ns = bridge.t
                bridge._update_landing(NS(x=1., y=2., z=0., stamp_ns=bridge.t), bridge.t)
                if bridge.outcomes: break
            self.assertEqual(bridge.outcomes[0][2], 'SUCCEEDED')
            self.assertGreaterEqual(bridge.t-origin, dwell)
            self.assertEqual(bridge._executor.config.odom_max_age_ns, 500_000_000)


if __name__ == '__main__': unittest.main()
