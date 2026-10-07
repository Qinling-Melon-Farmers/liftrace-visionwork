#!/usr/bin/env python3
"""Offline probe stop/freshness/JSON regressions. No SSH or device operations."""
import json
import os
import subprocess
import sys
import tempfile
import textwrap
import unittest
from unittest.mock import patch

import test_probe as fixtures

sys.path.insert(0, fixtures.TOOL_DIR)
import wb_status


class ProbeStopTest(unittest.TestCase):
    def test_existing_owner_is_not_replaced_and_lock_releases_on_stop(self):
        with tempfile.TemporaryDirectory(prefix='wb-probe-owner-') as root:
            fixtures.make_shim(root, True)
            command = [sys.executable, os.path.join(fixtures.TOOL_DIR, 'board_probe.py'),
                       '--lock-path', os.path.join(root, 'owner.lock')]
            env = dict(os.environ, PYTHONPATH=root)
            owner = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True)
            try:
                self.assertTrue(json.loads(owner.stdout.readline())['master'])
                duplicate = subprocess.run(command + ['--once'], env=env,
                    capture_output=True, text=True, timeout=4)
                self.assertEqual(duplicate.returncode, 75)
                self.assertIn('PROBE_LOCK_UNAVAILABLE', duplicate.stderr)
                self.assertEqual(duplicate.stdout, '')
                self.assertIsNone(owner.poll())
                owner.send_signal(__import__('signal').SIGINT)
                owner.communicate(timeout=4)
                self.assertEqual(owner.returncode, 130)
                successor = subprocess.run(command + ['--once'], env=env,
                    capture_output=True, text=True, timeout=4)
                self.assertEqual(successor.returncode, 0, successor.stderr)
            finally:
                if owner.poll() is None:
                    owner.kill()
                    owner.communicate(timeout=4)

    def test_ctrl_c_has_explicit_marker_and_does_not_restart(self):
        with tempfile.TemporaryDirectory(prefix="wb-probe-stop-") as root:
            fixtures.make_shim(root, True)
            with open(os.path.join(root, "rospy.py"), "a", encoding="utf-8") as handle:
                handle.write(textwrap.dedent('''
                    import time
                    _init = init_node
                    _attempts = 0
                    def init_node(name, **kwargs):
                        global _attempts
                        _attempts += 1
                        assert _attempts == 1
                        assert kwargs['disable_rosout'] is True
                        _init(name, **kwargs)
                    def interrupted_sleep(seconds):
                        raise KeyboardInterrupt()
                    time.sleep = interrupted_sleep
                '''))
            result = subprocess.run([sys.executable,
                os.path.join(fixtures.TOOL_DIR, "board_probe.py")],
                env=dict(os.environ, PYTHONPATH=root), capture_output=True, text=True, timeout=4)
            self.assertEqual(result.returncode, 130, result.stderr)
            self.assertIn("PROBE_STOPPED reason=keyboard_interrupt", result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            self.assertEqual(len([line for line in result.stdout.splitlines() if line.startswith('{')]), 1)


class ProbeLinkTest(unittest.TestCase):
    def setUp(self):
        self.now = 1791359339.6496432
        self.old = {"at": 1791358726.338206, "master": True,
                    "state": {"connected": True, "armed": False, "mode": "MANUAL"},
                    "topics": {"/mavros/state": {"age": .01, "count": 100},
                               "/camera/image_raw": {"age": .01, "count": 100}},
                    "services": {"/legacy/Servo_raw": "patrol_control/Servo"}}

    def test_field_snapshot_is_stale_after_613_seconds(self):
        link = wb_status.probe_link_status(self.old, {"state": "running"}, now=self.now)
        self.assertEqual(link['status'], 'stale')
        self.assertAlmostEqual(link['age'], 613.3114372, places=5)
        self.assertFalse(link['usable'])
        self.assertFalse(link['fresh'])

    def test_failed_probe_overrides_fresh_cached_telemetry(self):
        telemetry = dict(self.old, at=self.now)
        link = wb_status.probe_link_status(telemetry, {"state": "failed", "exit_code": 255}, now=self.now)
        self.assertEqual(link['status'], 'disconnected')
        self.assertEqual(link['exit_code'], 255)
        self.assertFalse(link['fresh'])
        self.assertFalse(link['usable'])

    def test_user_stop_is_distinct_from_broken_pipe_255(self):
        session = {"state": "failed", "exit_code": 255}
        stopped = wb_status.probe_link_status(self.old, session, now=self.now, stop_requested=True)
        broken = wb_status.probe_link_status(self.old, session, now=self.now)
        self.assertEqual(stopped['status'], 'stopped')
        self.assertEqual(broken['status'], 'disconnected')
        self.assertFalse(stopped['usable'])
        session['stop_requested'] = True
        self.assertEqual(wb_status.probe_link_status(self.old, session, now=self.now)['status'], 'stopped')

    def test_interrupt_exit_130_preserves_stop_without_server_flag(self):
        link = wb_status.probe_link_status(dict(self.old, at=self.now),
            {'state': 'failed', 'exit_code': 130}, now=self.now)
        self.assertEqual(link['status'], 'stopped')
        self.assertFalse(link['usable'])

    def test_fresh_receive_time_does_not_depend_on_board_clock(self):
        telemetry = dict(self.old, at=self.now-.1, t=1)
        link = wb_status.probe_link_status(telemetry, {"state": "running"}, now=self.now)
        self.assertEqual(link['status'], 'live')
        self.assertTrue(link['usable'])
        self.assertTrue(link['fresh'])

    def test_fresh_probe_without_master_is_not_device_ready(self):
        link = wb_status.probe_link_status({"at": self.now, "master": False}, now=self.now)
        self.assertEqual(link['status'], 'no_master')
        self.assertTrue(link['fresh'])
        self.assertFalse(link['usable'])
        with patch.object(wb_status.time, 'time', return_value=self.now):
            self.assertFalse(wb_status.ready_check('topic', {'topic': '/camera/image_raw'},
                dict(self.old, at=self.now, master=False))[0])

    def test_reconnected_session_waits_for_its_own_first_packet(self):
        telemetry = dict(self.old, at=self.now-.1)
        link = wb_status.probe_link_status(telemetry,
            {'state': 'running', 'started_at': self.now}, now=self.now)
        self.assertEqual(link['status'], 'waiting')
        self.assertFalse(link['usable'])

    def test_missing_future_and_nonfinite_receive_times(self):
        self.assertEqual(wb_status.probe_link_status({}, now=self.now)['status'], 'waiting')
        for at in (self.now+10, float('nan'), float('inf'), True, 'bad'):
            with self.subTest(at=at):
                self.assertFalse(wb_status.probe_link_status(dict(self.old, at=at), now=self.now)['usable'])

    def test_stale_sample_cannot_pass_any_device_readiness_check(self):
        with patch.object(wb_status.time, 'time', return_value=self.now):
            for kind, spec in (('ros_master', {}), ('mavros_connected', {}),
                               ('topic', {'topic': '/camera/image_raw'}),
                               ('service', {'service': '/legacy/Servo_raw'})):
                with self.subTest(kind=kind):
                    self.assertFalse(wb_status.ready_check(kind, spec, self.old)[0])

    def test_stopped_link_cannot_pass_readiness_even_if_at_is_fresh(self):
        telemetry = dict(self.old, at=self.now, probe_link={'status': 'stopped', 'usable': False})
        with patch.object(wb_status.time, 'time', return_value=self.now):
            self.assertFalse(wb_status.ready_check('ros_master', {}, telemetry)[0])

    def test_report_marks_stale_and_omits_cached_live_device_claims(self):
        with patch.object(wb_status.time, 'time', return_value=self.now):
            report = wb_status.StageTracker().report(telemetry=self.old)
        self.assertIn('缓存仅供历史查看', report)
        self.assertNotIn('connected=True', report)


class JsonStatusTest(unittest.TestCase):
    def test_terminal_controls_and_bom(self):
        row = {"master": False, "error": "network unavailable"}
        for prefix in ("\ufeff", "\x1b[?2004l\r", "\x1b]0;board\x07", "\x1b]0;board\x1b\\"):
            with self.subTest(prefix=repr(prefix)):
                self.assertEqual(wb_status.parse_probe_line(prefix + json.dumps(row) + "\x1b[0m\r"), row)

    def test_nonfinite_values_remain_browser_serializable(self):
        row = wb_status.parse_probe_line('{"master":true,"observe":{"battery":{"current":NaN}},"values":[Infinity,-Infinity,1e400]}')
        self.assertIsNone(row['observe']['battery']['current'])
        self.assertEqual(row['values'], [None, None, None])
        json.dumps(row, allow_nan=False)

    def test_bad_shapes_and_truncated_json_are_ignored(self):
        for line in (None, 1, [], "[]", "{", '{"master":"false"}',
                     '{"topics":[]}', '{"state":"bad"}', '{"topics":{"/camera":1}}'):
            with self.subTest(line=line):
                self.assertIsNone(wb_status.parse_probe_line(line))

    def test_invalid_age_does_not_mark_topic_or_mavros_ready(self):
        for age in (-1, True, '0.1', float('nan'), None):
            row = {'state': {'connected': True}, 'topics': {'/mavros/state': {'age': age, 'count': 10}}}
            with self.subTest(age=age):
                self.assertFalse(wb_status.ready_check('topic', {'topic': '/mavros/state'}, row)[0])
                self.assertFalse(wb_status.ready_check('mavros_connected', {}, row)[0])


if __name__ == '__main__':
    unittest.main()
