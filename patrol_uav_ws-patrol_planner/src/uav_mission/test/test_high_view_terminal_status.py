"""Full-mission terminal state follows accepted production result reduction."""
from dataclasses import replace
import unittest

from uav_mission.high_view_full import HighViewFull
from uav_mission.high_view_probe import ProbeConfig
from uav_mission.mission_core import GoalSnapshot, MissionConfig, MissionCore, MissionPhase
from test_mission_runtime import profile, result_for


class HighViewTerminalStatusTests(unittest.TestCase):
    def setUp(self):
        config = MissionConfig(
            early_return_enabled=False,
            post_delivery_route=(GoalSnapshot('camera_init', 0., 1., 1.18),),
            landing_xy=(0., 1.))
        self.runtime = HighViewFull(MissionCore(profile(), config),
                                    ProbeConfig(-.22, ((1., 1.),)))
        # Exercise real tail/LAND transactions without simulating payload IO.
        self.runtime.start_post_delivery_validation('mission-runtime', 100., (0., 0.))
        self.event_seq = 0

    def result(self, now, status='SUCCEEDED', stage='PLANNER', reason='finished'):
        self.event_seq += 1
        event = replace(result_for(self.runtime.core.active_action, self.event_seq,
                                   status=status, stage=stage, terminal=True,
                                   reason=reason),
                        event_stamp_ns=int((now - .01) * 1e9))
        return self.runtime.apply_result(event, now, (0., 1.))

    def land(self):
        out = self.result(101.)
        self.assertTrue(out.accepted, out.reason)
        self.assertEqual(out.action.command, 'LAND')
        self.assertFalse(self.runtime.done)
        return out

    def test_landing_result_immediately_synchronizes_success_without_tick(self):
        self.land()
        out = self.result(102., stage='LANDING', reason='landed')
        self.assertTrue(out.accepted, out.reason)
        self.assertEqual(out.snapshot.phase, MissionPhase.COMPLETE)
        self.assertTrue(self.runtime.done)
        self.assertTrue(self.runtime.succeeded)
        self.assertEqual(self.runtime.failure, '')
        status = self.runtime.probe_status()
        self.assertTrue(status['done'])
        self.assertTrue(status['succeeded'])
        self.assertEqual(status['stage'], 'TAIL')
        self.assertIsNone(out.action)

    def test_landing_failure_is_immediately_done_but_not_successful(self):
        self.land()
        out = self.result(102., status='FAILED', stage='LANDING', reason='landing_failed')
        self.assertTrue(out.accepted, out.reason)
        self.assertEqual(out.snapshot.phase, MissionPhase.ABORTED)
        self.assertTrue(self.runtime.done)
        self.assertFalse(self.runtime.succeeded)
        self.assertEqual(self.runtime.failure, 'landing_failed')
        self.assertFalse(self.runtime.probe_status()['succeeded'])

    def test_manual_abort_and_ack_never_report_success(self):
        out = self.runtime.abort('manual_abort_requested', 101.)
        self.assertTrue(out.accepted, out.reason)
        self.assertTrue(self.runtime.done)
        self.assertFalse(self.runtime.succeeded)
        self.assertEqual(self.runtime.failure, 'manual_abort_requested')
        self.result(102., stage='CONTROL', reason='held')
        self.assertEqual(self.runtime.core.phase, MissionPhase.ABORTED)
        self.assertTrue(self.runtime.probe_status()['done'])
        self.assertFalse(self.runtime.probe_status()['succeeded'])
        self.assertEqual(self.runtime.failure, 'manual_abort_requested')

    def test_successful_return_home_alone_is_not_successful_mission(self):
        self.land()
        status = self.runtime.probe_status()
        self.assertEqual(self.runtime.core.phase, MissionPhase.LAND)
        self.assertFalse(status['done'])
        self.assertFalse(status['succeeded'])
        self.assertEqual(status['stage'], 'TAIL')

    def test_rejected_landing_result_cannot_create_success(self):
        self.land()
        out = self.result(102., stage='PLANNER', reason='wrong_terminal_stage')
        self.assertFalse(out.accepted)
        self.assertEqual(self.runtime.core.phase, MissionPhase.LAND)
        self.assertFalse(self.runtime.done)
        self.assertFalse(self.runtime.probe_status()['succeeded'])

    def test_status_derives_core_terminal_state_if_runtime_flags_are_stale(self):
        self.land()
        self.result(102., stage='LANDING')
        self.runtime.done = False
        self.runtime.succeeded = False
        status = self.runtime.probe_status()
        self.assertTrue(status['done'])
        self.assertTrue(status['succeeded'])
        self.assertTrue(self.runtime.done)
        self.assertTrue(self.runtime.succeeded)

    def test_probe_assignments_cannot_override_core_completion_properties(self):
        self.runtime.done = True
        self.runtime.succeeded = True
        self.assertFalse(self.runtime.done)
        self.assertFalse(self.runtime.succeeded)
        self.land()
        self.result(102., stage='LANDING')
        self.runtime.done = False
        self.runtime.succeeded = False
        self.assertTrue(self.runtime.done)
        self.assertTrue(self.runtime.succeeded)
        self.assertNotIn('done', self.runtime.__dict__)
        self.assertNotIn('succeeded', self.runtime.__dict__)

    def test_no_argument_reader_sees_terminal_core_without_status_or_tick(self):
        read = lambda: (self.runtime.done, self.runtime.succeeded)
        self.assertEqual(read(), (False, False))
        self.land()
        self.result(102., status='FAILED', stage='LANDING')
        self.assertEqual(read(), (True, False))

    def test_full_mission_abort_overrides_probe_segment_success_flag(self):
        self.runtime._finish(True, 'probe_segment_only', 101.)
        self.assertEqual(self.runtime.core.phase, MissionPhase.ABORTED)
        self.assertTrue(self.runtime.done)
        self.assertFalse(self.runtime.succeeded)
        self.assertFalse(self.runtime.probe_status()['succeeded'])


if __name__ == '__main__':
    unittest.main()