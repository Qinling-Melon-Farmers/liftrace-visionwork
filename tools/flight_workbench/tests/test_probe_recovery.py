"""Workbench retry policy with fake sessions/timers only; no SSH or ROS."""
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
import wb_board
import wb_status


class Timer:
    def __init__(self, interval, function):
        self.interval = interval; self.function = function; self.cancelled = False; self.daemon = False
    def start(self): pass
    def cancel(self): self.cancelled = True
    def fire(self): self.function()


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        with patch.object(server, 'PROFILE_DIR', self.tmp.name), patch.object(wb_board, 'load_profile', return_value={}):
            self.wb = server.Workbench(wb_board.load_config(), {'transport': 'ssh'})
        self.wb.connection['state'] = 'ok'
        self.wb.board = Mock(root='/fixture')
        self.wb.board.probe_command.return_value = 'fixture-probe-only'
        self.wb._journal = Mock()
        self.session = None
        self.wb.sessions = Mock()
        self.wb.sessions.get.side_effect = lambda sid: self.session if sid == 'probe' else None
        self.wb.sessions.snapshots.return_value = {}
        def opened(*args, **kwargs):
            self.session = SimpleNamespace(state='running', exit_code=None, started_at=time.time())
            self.session.snapshot = lambda: {'state': self.session.state, 'exit_code': self.session.exit_code,
                                            'started_at': self.session.started_at}
            return self.session
        self.wb.sessions.open.side_effect = opened
        self.timers = []
        def factory(*args):
            timer = Timer(*args); self.timers.append(timer); return timer
        patcher = patch.object(server.threading, 'Timer', side_effect=factory)
        patcher.start(); self.addCleanup(patcher.stop)

    def end(self, code):
        self.session.state = 'failed'; self.session.exit_code = code
        self.wb._on_state('probe', self.session.snapshot())

    def test_nonmanual_failure_only_reconnects_probe_with_backoff(self):
        self.assertTrue(self.wb.ensure_probe(explicit=True))
        self.assertTrue(self.wb.ensure_probe())
        self.wb.sessions.open.assert_called_once()
        self.end(255); self.assertEqual(self.timers[-1].interval, 2)
        self.timers[-1].fire()
        self.assertEqual(self.wb.sessions.open.call_count, 2)
        self.end(255); self.assertEqual(self.timers[-1].interval, 5)
        self.assertTrue(self.wb.snapshot()['probe']['link']['retrying'])
        self.wb.board.upload_probe.assert_called_once()
        self.assertTrue(all(c.args[0] == 'probe' for c in self.wb.sessions.open.call_args_list))
        self.wb.sessions.close_all.assert_not_called()

    def test_stopall_cancels_retry_even_if_ssh_reports_255(self):
        self.wb.ensure_probe(explicit=True); self.end(255)
        timer = self.timers[-1]; count = self.wb.sessions.open.call_count
        self.wb.stop_all(); self.assertTrue(timer.cancelled); timer.fire()
        self.end(255)
        self.assertEqual(self.wb.sessions.open.call_count, count)
        self.assertEqual(self.wb.probe_link()['status'], 'stopped')

    def test_exit130_never_retries_and_only_explicit_button_clears_stop(self):
        self.wb.ensure_probe(explicit=True); self.end(130)
        self.assertEqual(self.timers, [])
        self.assertFalse(self.wb.ensure_probe())
        self.assertTrue(self.wb.reconnect_probe()['ok'])
        self.assertEqual(self.wb.sessions.open.call_count, 2)

    def test_exit75_preserves_owner_without_retry(self):
        self.wb.ensure_probe(explicit=True); self.end(75)
        self.assertEqual(self.timers, [])
        self.assertFalse(self.wb.ensure_probe())
        self.assertEqual(self.wb.probe_link()['status'], 'conflict')
        self.wb.sessions.close.assert_not_called(); self.wb.sessions.close_all.assert_not_called()

    def test_fresh_cache_from_failed_probe_does_not_satisfy_ready_or_report(self):
        self.wb.ensure_probe(explicit=True)
        self.wb.telemetry = {'at': time.time(), 'master': True, 'state': {'connected': True},
                             'topics': {'/mavros/state': {'age': .01}}, 'nodes': []}
        self.end(255)
        tel = self.wb.snapshot()['telemetry']
        self.assertFalse(tel['probe_link']['usable'])
        self.assertFalse(wb_status.ready_check('ros_master', {}, tel)[0])
        report = self.wb.make_report()['markdown']
        self.assertIn('当前设备状态未观测', report)
        self.assertNotIn('遥测飞控状态：connected=True', report)

    def test_logs_only_disables_probe_and_device_execution(self):
        self.wb.options['logs_only'] = True
        self.assertFalse(self.wb.ensure_probe(explicit=True))
        self.wb.board.upload_probe.assert_not_called(); self.wb.sessions.open.assert_not_called()
        with self.assertRaises(ValueError): self.wb._require_command_transport()
        self.wb.board.test_connection.return_value = {'ok': True}
        with patch.object(self.wb, 'ensure_probe') as ensure, patch.object(self.wb, 'refresh_preflight') as preflight:
            self.assertTrue(self.wb.connect({})['ok'])
            ensure.assert_not_called(); preflight.assert_not_called()
        self.assertEqual(self.wb.sessions.open.call_count, 0)

    def test_5a_rejects_running_5b_without_killing_or_reopening(self):
        servo = SimpleNamespace(state='running')
        self.wb.sessions.get.side_effect = lambda sid: servo if sid == 'servo' else None
        with self.assertRaisesRegex(ValueError, '禁止再次5a'):
            self.wb.open_session({'id': 'servo_init', 'confirm': '确认'})
        self.wb.sessions.open.assert_not_called(); self.wb.sessions.close.assert_not_called()


if __name__ == '__main__': unittest.main(verbosity=2)
