#!/usr/bin/env python3
"""Offline reproductions of master ordering and rospy partial initialization.

All ROS imports use local file shims. Re-execution only replaces the test probe;
no SSH, real ROS process, service, publisher or hardware operation is used.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest

import test_probe as fixtures


class ProbeInitializationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='wb-probe-init-')
        self.root = Path(self.temporary.name)
        fixtures.make_shim(str(self.root), True)
        self.command = [sys.executable, str(Path(fixtures.TOOL_DIR) / 'board_probe.py'),
                        '--interval', '0.2', '--lock-path', str(self.root / 'probe.lock')]
        self.env = dict(os.environ, PYTHONPATH=str(self.root))

    def tearDown(self):
        self.temporary.cleanup()

    def append_rospy(self, source):
        with (self.root / 'rospy.py').open('a', encoding='utf-8') as handle:
            handle.write(textwrap.dedent(source))

    def run_probe(self, extra=(), timeout=7):
        result = subprocess.run(self.command + list(extra), env=self.env,
            capture_output=True, text=True, timeout=timeout)
        rows = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
        self.assertTrue(rows, result.stderr)
        return result, rows

    def test_probe_waits_until_master_ready_before_first_init(self):
        fixtures.write(str(self.root / 'rosnode.py'), textwrap.dedent('''
            reads = 0
            def get_node_names():
                global reads
                reads += 1
                if reads < 3:
                    raise OSError('master not running yet')
                return ['/mavros']
        '''))
        self.append_rospy('''
            import rosnode
            _init = init_node
            _init_calls = 0
            _checks = 0
            def init_node(name, **kwargs):
                global _init_calls
                _init_calls += 1
                assert rosnode.reads >= 3, 'init ran before master ready'
                assert _init_calls == 1, 'same process initialized twice'
                _init(name, **kwargs)
            def is_shutdown():
                global _checks
                _checks += 1
                return _checks > 3
        ''')
        result, rows = self.run_probe()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([row['master'] for row in rows], [False, False, True])
        self.assertTrue(all('waiting for ROS master' in row['error'] for row in rows[:2]))

    def test_partial_clock_init_reexecs_instead_of_same_process_false_success(self):
        self.append_rospy('''
            from pathlib import Path
            _root = Path(__file__).parent
            _init = init_node
            _init_calls = 0
            _clock_initialized = False
            _checks = 0
            _now = Time.now
            def init_node(name, **kwargs):
                global _init_calls, _clock_initialized
                _init_calls += 1
                with (_root / 'init_calls').open('a') as handle:
                    handle.write(str(_init_calls) + '\\n')
                if _init_calls > 1:
                    return  # Emulate rospy._init_node_args early return after failed init.
                if not (_root / 'failed_once').exists():
                    (_root / 'failed_once').touch()
                    raise RuntimeError('Failed to initialize time. Please check logs for additional details')
                _clock_initialized = True
                _init(name, **kwargs)
            def now():
                if not _clock_initialized:
                    raise RuntimeError('time is not initialized. Have you called init_node()?')
                return _now()
            Time.now = staticmethod(now)
            def is_shutdown():
                global _checks
                _checks += 1
                return _clock_initialized and _checks > 1
        ''')
        topics = {key:'/observe/'+key for key in ('fc_pose','lio_pose','ev_pose','setpoint')}
        result, rows = self.run_probe(['--observe-topics', json.dumps(topics)])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([row['master'] for row in rows], [False, True])
        self.assertIn('Failed to initialize time', rows[0]['error'])
        self.assertEqual(rows[0]['init_state'], 'restarting')
        self.assertEqual(rows[0]['restart_delay'], 2.)
        self.assertEqual((self.root / 'init_calls').read_text().splitlines(), ['1', '1'])
        for key in topics:
            self.assertEqual(rows[1]['observe'][key]['x'], 1.)
            self.assertEqual(rows[1]['observe_status'][key]['status'], 'receiving')
        self.assertNotIn('PROBE_PARSE_ERROR', result.stderr)
        # Old lock descriptor must close on exec; otherwise the replacement would fail with 75.
        self.assertNotIn('PROBE_LOCK_UNAVAILABLE', result.stderr)

    def test_once_reports_initialization_failure_without_reexec(self):
        self.append_rospy('''
            def init_node(name, **kwargs):
                raise RuntimeError('Failed to initialize time')
        ''')
        result, rows = self.run_probe(['--once'])
        self.assertEqual(result.returncode, 1)
        self.assertEqual(len(rows), 1)
        self.assertFalse(rows[0]['master'])
        self.assertEqual(rows[0]['init_state'], 'failed')
        self.assertIsNone(rows[0]['restart_delay'])
        self.assertNotIn('state', rows[0])

    def test_once_missing_master_never_calls_init(self):
        fixtures.write(str(self.root / 'rosnode.py'), "def get_node_names():\n    raise OSError('master offline')\n")
        self.append_rospy('''
            def init_node(name, **kwargs):
                raise AssertionError('init must not run without a master')
        ''')
        result, rows = self.run_probe(['--once'])
        self.assertEqual(result.returncode, 1)
        self.assertIn('waiting for ROS master: master offline', rows[0]['error'])
        self.assertNotIn('init_state', rows[0])

    def test_ctrl_c_during_restart_delay_does_not_reexec(self):
        self.append_rospy('''
            import time
            def init_node(name, **kwargs):
                raise RuntimeError('Failed to initialize time')
            def stop_during_delay(seconds):
                raise KeyboardInterrupt()
            time.sleep = stop_during_delay
        ''')
        result, rows = self.run_probe()
        self.assertEqual(result.returncode, 130)
        self.assertEqual(len(rows), 1)
        self.assertFalse(rows[0]['master'])
        self.assertIn('PROBE_STOPPED reason=keyboard_interrupt', result.stderr)
        self.assertNotIn('Traceback', result.stderr)


if __name__ == '__main__':
    unittest.main()
