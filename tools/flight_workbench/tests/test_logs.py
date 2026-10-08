"""Isolated log tests: no board connection, ROS, recording, or existing service."""
import copy
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
import wb_board
import wb_logs
import wb_ssh


class RemoteTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.config = copy.deepcopy(wb_board.load_config())
        self.config['connection']['board_root'] = str(self.root)
        self.client = wb_logs.LogsClient(wb_board.BoardClient(self.config, wb_ssh.Target(transport='local')))
        self.ns = {}
        exec(wb_logs.REMOTE, self.ns)
        self.ns['ROUTINE_WORKER'] = wb_logs.ROUTINE_WORKER
        self.ns['processes'] = lambda: []

    def request(self, operation, **kwargs):
        return self.client.payload(operation, **kwargs)

    def test_status_read_only_and_external_closed_file(self):
        status = self.ns['action'](self.request('status'))
        self.assertTrue(status['ok'])
        self.assertEqual(list(self.root.iterdir()), [])  # No mkdir/config writes on refresh.
        run = self.root / 'logs' / 'board_visual_interrupt_20261007_154003'
        run.mkdir(parents=True)
        (run / 'flight.bag').write_bytes(b'bag-fixture')
        (run / 'flight.bag.active').write_bytes(b'active')
        status = self.ns['action'](self.request('status'))
        self.assertEqual(len(status['files']), 2)
        self.assertTrue(next(f for f in status['files'] if f['path'].endswith('.bag'))['downloadable'])
        self.assertFalse(next(f for f in status['files'] if f['path'].endswith('.active'))['downloadable'])

    def test_path_validation_and_symlink_escape(self):
        for path in ('', '/tmp/file.bag', '../secret', 'run/../secret', 'run//file', 'run/.secret',
                     'run/a\r\nheader', 'run/a;cat', 'run/a"', 'run/a\\file', None):
            with self.assertRaises(ValueError, msg=repr(path)):
                wb_logs.relative_path(path)
        self.assertEqual(wb_logs.relative_path('board_trial/recording/diagnostics.bag'),
                         'board_trial/recording/diagnostics.bag')
        logs = self.root / 'logs'; logs.mkdir()
        (self.root / 'outside.bag').write_bytes(b'outside')
        (logs / 'escape.bag').symlink_to(self.root / 'outside.bag')
        with self.assertRaises(ValueError):
            self.ns['action'](self.request('download', path='escape.bag'))

    def test_start_refuses_existing_legacy_bag_before_popen(self):
        self.ns['processes'] = lambda: [{'kind': 'bag', 'pid': 12345}]
        with patch('subprocess.Popen') as spawn:
            with self.assertRaisesRegex(ValueError, 'already running'):
                self.ns['action'](self.request('start', kind='bag', profile='routine', duration=60))
            spawn.assert_not_called()

    def test_stop_refuses_unowned_or_reused_pid(self):
        job = 'bag_20261007_120000_1234abcd'
        run = self.root / 'logs' / 'flight_workbench_recordings' / job
        run.mkdir(parents=True)
        (run / 'job.json').write_text(json.dumps({'kind': 'bag', 'pid': os.getpid(),
                                                 'identity': {'start': 'wrong', 'argv': []}}))
        with patch('os.kill') as kill:
            with self.assertRaisesRegex(ValueError, 'No matching owned'):
                self.ns['action'](self.request('stop', id=job))
            kill.assert_not_called()

    def test_stop_signals_only_the_owned_pid_with_sigint(self):
        job = 'bag_20261007_120000_1234abcd'
        run = self.root / 'logs' / 'flight_workbench_recordings' / job; run.mkdir(parents=True)
        ident = self.ns['identity'](os.getpid())
        (run / 'job.json').write_text(json.dumps({'kind': 'bag', 'pid': os.getpid(), 'identity': ident}))
        self.ns['owned'] = Mock(side_effect=[True, False, False])
        with patch('os.kill') as kill:
            result = self.ns['action'](self.request('stop', id=job))
            kill.assert_called_once_with(os.getpid(), signal.SIGINT)
        self.assertFalse(result['stopping'])

    def test_managed_bag_needs_closed_summary(self):
        run = self.root / 'logs/flight_workbench_recordings/bag_20261007_120000_1234abcd/recording'
        run.mkdir(parents=True)
        bag = run / 'diagnostics.bag'; bag.write_bytes(b'fixture')
        root = (self.root / 'logs').resolve()
        self.assertFalse(self.ns['file_closed'](root, bag, []))
        (run / 'summary.json').write_text(json.dumps({'bag_closed': True, 'error': None}))
        self.assertTrue(self.ns['file_closed'](root, bag, []))
        self.assertFalse(self.ns['file_closed'](root, bag, [{'id': run.parent.name, 'active': True}]))

    def test_cli_contract_without_starting_processes(self):
        for kind, profile, filename in [('bag', 'diagnostic', 'bag_script'), ('ulog', None, 'ulog_script'),
                                         ('bag', 'routine', 'routine_script')]:
            payload = self.request('start', kind=kind, duration=60, log_id=17, profile=profile)
            script = self.root / payload[filename]; script.parent.mkdir(parents=True, exist_ok=True); script.write_text('# fixture')
            cfg = self.root / payload['bag_config']; cfg.parent.mkdir(parents=True, exist_ok=True); cfg.touch()
            fake = Mock(pid=999999)
            with patch('subprocess.Popen', return_value=fake) as spawn:
                result = self.ns['action'](payload)
                args = spawn.call_args.args[0]
                self.assertTrue(result['ok'])
                self.assertTrue(spawn.call_args.kwargs['start_new_session'])
                if kind == 'ulog':
                    self.assertIn('--log-id', args); self.assertIn('17', args)
                    self.assertNotIn('--latest', args); self.assertNotIn('logging_start', args)
                elif profile == 'diagnostic':
                    self.assertIn('--duration', args); self.assertIn('--config', args)
                else:
                    self.assertEqual(args[-1], 'wb_log_worker')

    def test_old_ulog_index_is_not_misrepresented_as_log_ids(self):
        request = self.request('index')
        script = self.root / request['ulog_script']; script.parent.mkdir(parents=True)
        script.write_text('parser.add_argument("--list-dir")')
        with patch('subprocess.Popen') as spawn:
            with self.assertRaisesRegex(ValueError, 'does not support'):
                self.ns['action'](request)
            spawn.assert_not_called()

    def test_request_validation_and_failures_do_not_become_success(self):
        for body in ({'kind': 'streaming'}, {'kind': 'bag', 'confirm': 'wrong'},
                     {'kind': 'bag', 'confirm': '开始轻量录制', 'duration': True},
                     {'kind': 'bag', 'confirm': '开始轻量录制', 'duration': 901},
                     {'kind': 'ulog', 'confirm': '取回飞控日志', 'log_id': -1},
                     {'kind': 'ulog', 'confirm': '取回飞控日志', 'log_id': '17'}):
            with self.assertRaises(ValueError): self.client.start(body)
        with patch.object(self.client.board, 'run', return_value=(124, 'secret transport failure')):
            with self.assertRaisesRegex(ValueError, 'exit=124'): self.client.status()
        with patch.object(self.client.board, 'run', return_value=(0, '')):
            with self.assertRaises(ValueError): self.client.status()
        with patch.object(self.client.board, 'run', return_value=(1, 'WB_LOGS_JSON={"ok": false, "error": "already running"}')):
            with self.assertRaisesRegex(ValueError, 'already running'): self.client.status()

    def test_binary_download_content_and_open_writer_rejection(self):
        env = self.root / 'environment.sh'; env.write_text('echo environment-chatter\n')
        self.config['connection']['env_script'] = 'environment.sh'
        self.config['connection']['board_python'] = sys.executable
        run = self.root / 'logs/board_fixture'; run.mkdir(parents=True)
        expected = b'#ROSBAG V2.0\n' + bytes(range(256)) * 300
        (run / 'closed.bag').write_bytes(expected)
        code, output = self.client.download('board_fixture/closed.bag')
        self.assertEqual(code, 0, output[:100]); self.assertEqual(output, expected)
        with (run / 'writing.bag').open('wb') as handle:
            handle.write(b'not closed'); handle.flush()
            code, output = self.client.download('board_fixture/writing.bag')
            self.assertNotEqual(code, 0); self.assertNotIn(b'not closed', output)

    def test_binary_frame_strips_startup_output_and_rejects_truncation(self):
        nonce = Mock(hex='fixture')
        frame = b'shell startup\nWB_BINARY_fixture 3\n\x00\n\xff\nWB_END_fixture\n'
        with patch.object(wb_logs.uuid, 'uuid4', return_value=nonce), patch.object(wb_logs, 'run_bytes', return_value=(0, frame)):
            self.assertEqual(self.client.download('board_fixture/closed.bag'), (0, b'\x00\n\xff'))
        for invalid in (b'partial-file-no-header', frame[:-1], frame.replace(b'fixture 3', b'fixture 4')):
            with patch.object(wb_logs.uuid, 'uuid4', return_value=nonce), patch.object(wb_logs, 'run_bytes', return_value=(0, invalid)):
                code, data = self.client.download('board_fixture/closed.bag')
                self.assertNotEqual(code, 0); self.assertNotIn(b'partial-file', data)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        with patch.object(server, 'PROFILE_DIR', self.tmp.name), patch.object(wb_board, 'load_profile', return_value={}):
            self.wb = server.Workbench(wb_board.load_config(), {'transport': 'ssh', 'password': 'memory-only'})
        self.wb.logs = Mock()
        self.wb.logs.lock = threading.RLock()
        self.wb.logs.active.return_value = False
        self.wb.logs.status.return_value = {'ok': True, 'jobs': [], 'files': [], 'recorders': []}
        self.wb.logs.download.return_value = (0, b'\x00\n\xfffixture')
        class TestHandler(server.Handler): pass
        TestHandler.workbench = self.wb
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), TestHandler)
        thread = threading.Thread(target=self.http.serve_forever, daemon=True); thread.start()
        def cleanup():
            self.http.shutdown(); self.http.server_close(); thread.join()
        self.addCleanup(cleanup)
        self.base = 'http://127.0.0.1:%s' % self.http.server_port

    def request(self, path, body=None, headers=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.base + path, data=data, headers=headers or {})
        try:
            with urllib.request.urlopen(req, timeout=2) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as response:
            return response.code, response.read()

    def test_static_page_does_not_connect_or_record(self):
        for path in ('/logs', '/static/logs.js', '/static/logs.css'):
            self.assertEqual(self.request(path)[0], 200)
        self.wb.logs.status.assert_not_called(); self.wb.logs.start.assert_not_called()
        self.assertEqual(self.wb.sessions.snapshots(), {})

    def test_offline_cross_origin_and_invalid_body_are_refused(self):
        self.assertEqual(self.request('/api/recording/status', {})[0], 400)
        self.wb.connection['state'] = 'ok'
        self.assertEqual(self.request('/api/recording/start', {}, {'Origin': 'http://evil.invalid'})[0], 400)
        self.assertEqual(self.request('/api/recording/status', [], {'Sec-Fetch-Site': 'cross-site'})[0], 400)
        self.assertEqual(self.request('/api/recording/status', [])[0], 400)
        self.wb.logs.start.assert_not_called(); self.wb.logs.status.assert_not_called()

    def test_download_binary_and_failure_status(self):
        self.wb.connection['state'] = 'ok'
        status, payload = self.request('/api/recording/download?path=board_fixture/closed.bag')
        self.assertEqual(status, 200); self.assertEqual(payload, b'\x00\n\xfffixture')
        self.wb.logs.download.return_value = (124, b'memory-only')
        status, payload = self.request('/api/recording/download?path=board_fixture/closed.bag')
        self.assertEqual(status, 502); self.assertNotIn(b'memory-only', payload)

    def test_connection_change_during_log_operation_is_refused(self):
        self.wb.logs.active.return_value = True
        with self.assertRaisesRegex(ValueError, '日志操作'):
            self.wb.update_config({'host': 'orangepi@10.75.120.193'})

    def test_logs_only_mode_blocks_all_control_endpoints(self):
        self.wb.options['logs_only'] = True
        self.wb.connection['state'] = 'ok'
        for path in ('/api/action/probe_reconnect', '/api/action/start_all', '/api/action/stop_all',
                     '/api/session/open', '/api/session/key', '/api/disconnect', '/api/trial/start'):
            self.assertEqual(self.request(path, {})[0], 404, path)
        status, body = self.request('/')
        self.assertEqual(status, 200); self.assertIn('独立日志连接'.encode(), body)
        self.assertEqual(self.request('/api/events')[0], 404)
        self.wb.logs.status.assert_not_called()


if __name__ == '__main__': unittest.main(verbosity=2)
