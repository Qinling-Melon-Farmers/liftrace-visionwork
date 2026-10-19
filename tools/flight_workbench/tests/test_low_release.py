"""Offline launch/confirmation/stop contracts. All transports are mocked."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import server
import wb_board
import wb_low_release


def command_fixture():
    config = wb_board.load_config()
    group = next(g for g in config['groups'] if wb_low_release.is_group(g))
    cases = []
    for slot in (1, 2, 3):
        for mode in ('preview', 'flight'):
            for real in (False, True):
                for check in (False, True):
                    body = dict(slot=slot, mode=mode, real_release=real, check_config=check)
                    cases.append(dict(body=body, expected=wb_board.build_group_command(config, group, **body)[0]))
    return dict(group=group, cases=cases)


class LowReleaseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        for owner, name in ((server, 'PROFILE_DIR'), (wb_board, 'DEFAULT_PROFILE_DIR')):
            guard = patch.object(owner, name, self.tmp.name)
            guard.start()
            self.addCleanup(guard.stop)
        self.config = wb_board.load_config()
        self.group = next(g for g in self.config['groups'] if wb_low_release.is_group(g))
        self.wb = server.Workbench(self.config, {'transport': 'ssh'})
        self.wb.connection['state'] = 'ok'
        for owner, name in ((self.wb.board, 'run'), (server, 'run_once')):
            guard = patch.object(owner, name, side_effect=AssertionError('No transport allowed'))
            guard.start()
            self.addCleanup(guard.stop)
        guard = patch.object(self.wb.sessions, 'open')
        self.opened = guard.start()
        self.addCleanup(guard.stop)
        self.session = N(state='running', snapshot=lambda: {'state': 'running'}, send_key=Mock())
        self.opened.return_value = self.session

    def body(self, **overrides):
        return dict(dict(group_id=self.group['id'], mode='flight', slot=1,
                         real_release=False, confirm='启动试飞'), **overrides)

    def test_slots_modes_and_preview_are_configuration_only(self):
        self.assertEqual(self.group['release'], 'mock')
        self.assertEqual(self.group['default_slot'], 1)
        for slot in (1, 2, 3):
            for mode in ('preview', 'flight'):
                for real in (False, True):
                    for check in (False, True):
                        body = self.body(mode=mode, slot=slot, real_release=real, check_config=check)
                        command = self.wb.trial_command(body)['body']
                        expected = 'bash deployment/board_trials_4x4/11_low_release_test/start.sh %s --slot %d' % (
                            'preview' if check else mode, slot)
                        if mode == 'flight' and real and not check:
                            expected += ' --real-release'
                        if check:
                            expected += ' --check-config'
                        self.assertEqual(command, expected)
        self.assertTrue(self.wb.trial_command({'group_id': self.group['id']})['body'].endswith('preview --slot 1'))
        self.opened.assert_not_called()

    def test_no_target_exception_is_scoped_and_options_rejected(self):
        for slot in (True, False, 0, 4, '1', '1; touch /tmp/no', 1.0):
            with self.subTest(slot=slot), self.assertRaises(ValueError):
                self.wb.trial_command(self.body(slot=slot))
        for options in ({'site_geometry': {}}, {'landing_config': 'landing.yaml'}, {'route': 'site'},
                        {'speed_profile': 'competition'}, {'motion_optimized': True},
                        {'coverage_agl': 1.8}, {'real_release': 'true'}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.wb.trial_command(self.body(**options))
        for group in self.config['groups']:
            if group.get('id') != self.group['id']:
                with self.subTest(group=group['id']), self.assertRaises(ValueError):
                    wb_board.build_group_command(self.config, group, 'preview', slot=1)
        for change in ({'low_release_test': False}, {'channel': 'site'}, {'folder': '08_full_mission'}):
            with self.assertRaises(ValueError):
                wb_board.build_group_command(self.config, dict(self.group, **change), 'flight')
        with self.assertRaises(ValueError):
            wb_board.build_group_command(dict(self.config, simulation={'enabled': True}), self.group, 'flight')

    def test_confirm_and_changed_slot_are_checked_before_start(self):
        for real, confirm in ((True, None), (True, '启动试飞'), (False, None)):
            with self.assertRaises(ValueError):
                self.wb.start_trial(self.body(real_release=real, confirm=confirm))
        body = self.body(real_release=True, confirm='实投')
        reviewed = self.wb.trial_command(body)['body']
        with self.assertRaisesRegex(ValueError, '不一致'):
            self.wb.start_trial(dict(body, slot=2, expected_body=reviewed))
        self.opened.assert_not_called()
        result = self.wb.start_trial(dict(body, expected_body=reviewed))
        self.assertEqual(result['trial']['command'], reviewed)
        self.assertTrue(result['trial']['low_release_test'])
        self.assertEqual(result['trial']['route'], 'module')
        self.assertEqual(result['trial']['slot'], 1)
        self.assertTrue(self.opened.call_args.kwargs['graceful_only'])
        self.assertTrue(self.wb.stage.observation_mode)

    def test_check_needs_no_flight_confirmation_and_cannot_release(self):
        result = self.wb.start_trial(self.body(check_config=True, real_release=True, confirm=None))
        self.assertEqual(result['trial']['release'], 'none')
        self.assertNotIn('--real-release', result['trial']['command'])
        self.assertNotIn('graceful_only', self.opened.call_args.kwargs)

    def test_stop_retains_localization_and_only_requests_interrupt(self):
        self.wb.start_trial(self.body())
        with patch.object(self.wb.sessions, 'get', return_value=self.session):
            for sid in (None, 'roscore', 'mavros', 'lidar', 'observation_localization'):
                with self.subTest(sid=sid), self.assertRaises(ValueError):
                    self.wb.protect_observation_dependencies(sid)
            with patch.object(self.wb.sessions, 'close_all') as close_all:
                with self.assertRaises(ValueError):
                    self.wb.stop_all()
                close_all.assert_not_called()
            self.assertTrue(self.wb.stop_trial()['ok'])
            self.session.send_key.assert_called_once_with('C-c')
            with self.assertRaisesRegex(ValueError, '无需另点启动任务'):
                self.wb.mission_start({'confirm': '启动任务'})
            self.session.state = 'exited'
            self.wb.protect_observation_dependencies()

    def test_observation_protocol_ready_hold_and_finish(self):
        self.wb.start_trial(self.body())
        tracker = self.wb.stage
        tracker.feed_line('READY_FOR_MANUAL_ARM_AND_OFFBOARD')
        self.assertEqual(tracker.name, 'READY')
        for phase in ('RUN', 'FINISHED_HOVER', 'HOLD_FOR_PILOT', 'TAKEN_OVER'):
            tracker.feed_line(json.dumps({'stage': phase, 'armed': True, 'mode': 'OFFBOARD'}))
            self.assertEqual(tracker.phase, phase)
        tracker.feed_line('OBSERVATION_CLOSED logs/test_mock')
        self.assertEqual(tracker.name, 'STOPPED')

    def test_frontend_backend_command_parity_and_wrappers(self):
        node = os.environ.get('NODE_BINARY') or shutil.which('node')
        self.assertTrue(node, 'Set NODE_BINARY to run frontend parity')
        result = subprocess.run([node, str(ROOT / 'tests/low_release_ui.test.cjs')],
                                cwd=str(ROOT), input=json.dumps(command_fixture()),
                                text=True, capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--fixture':
        Path(sys.argv[2]).write_text(json.dumps(command_fixture(), ensure_ascii=False), encoding='utf-8')
    else:
        unittest.main(verbosity=2)
