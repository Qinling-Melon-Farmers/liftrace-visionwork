"""H landing observations may continue after collision without passing Gate."""

import importlib.util
from pathlib import Path
import sys
import tempfile
import json
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'h_contact_gate_fixtures', Path(__file__).with_name(
        'test_navigation_vcl06_assertion.py'))
FIXTURES = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = FIXTURES
SPEC.loader.exec_module(FIXTURES)
GATE = FIXTURES.MODULE


class HContactRecordOnlyTest(unittest.TestCase):
    def test_collision_during_h_approach_allows_landing_evidence_but_raw_fail(self):
        for observe_full_trial in (False, True):
            with self.subTest(observe_full_trial=observe_full_trial):
                reducer = FIXTURES.build_corridor_reducer(h_evidence=False)
                reducer.observe_status('contact', {
                    'status': 'READY', 'ready': True, 'actual_collision_count': 1})
                node = GATE.NavigationVcl06AssertionNode.__new__(
                    GATE.NavigationVcl06AssertionNode)
                node.reducer = reducer
                node._finished = False
                node._stop_on_collision = False
                node._observe_full_trial = observe_full_trial
                node._mission_started_ros = 10.
                node._recorded_collision_count = 0
                node.exit_code = 1
                with tempfile.TemporaryDirectory() as directory, \
                        mock.patch.object(GATE, 'rospy', mock.Mock()) as fake_ros:
                    fake_ros.Time.now.return_value.to_sec.return_value = 550.
                    node._report_path = str(Path(directory) / 'gate_status.json')
                    node._check_terminal()
                    self.assertFalse(node._finished)
                    fake_ros.signal_shutdown.assert_not_called()
                    raw = json.loads(Path(node._report_path).read_text())
                    self.assertEqual(raw['status'], 'FAIL')
                    self.assertIn('actual_collision', raw['errors'])
                    self.assertEqual(raw['metrics']['actual_collision_count'], 1)

                    issued_ns = reducer.land_decision_issued_ns
                    receipt = reducer.land_decision_receipt_wall + 1.
                    reducer.observe_landing_h_mark(
                        *FIXTURES.LANDING_XY, 0., 'camera_init', issued_ns + 1,
                        receipt_wall=receipt, fresh=True)
                    reducer.observe_align_mode('landing', receipt_wall=receipt)
                    node._check_terminal()
                    self.assertFalse(node._finished)
                    reducer.observe_landed_state(
                        GATE.LANDED_STATE_ON_GROUND, receipt_wall=receipt + 1.)
                    node._check_terminal()
                    self.assertFalse(node._finished)
                    reducer.observe_vehicle_state(False, receipt_wall=receipt + 2.)
                    node._check_terminal()
                    self.assertTrue(node._finished)
                    fake_ros.signal_shutdown.assert_called_once()
                    self.assertEqual(node.exit_code, 1)
                    final = json.loads(Path(node._report_path).read_text())
                    self.assertEqual(final['status'], 'FAIL')
                    self.assertEqual(final['metrics']['actual_collision_count'], 1)
                    self.assertEqual(final['errors'], ['actual_collision'])
                    self.assertFalse(final['stop_on_collision'])
                    for check in ('landing_h_mark_valid', 'landing_align_mode_seen',
                                  'final_landed_on_ground', 'final_vehicle_disarmed'):
                        self.assertTrue(final['checks'][check], check)


if __name__ == '__main__':
    unittest.main()
