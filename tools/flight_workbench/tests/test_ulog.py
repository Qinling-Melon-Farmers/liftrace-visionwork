"""Real synthetic ULog → HTTP upload → isolated parser/Agg → artifacts; no ROS."""
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch, Mock
import urllib.request
import urllib.error
from urllib.parse import quote
import zipfile

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
import server
import wb_board
import wb_ulog


def synthetic_ulog():
    output = bytearray(wb_ulog.MAGIC + b'\x01' + struct.pack('<Q', 1000000))
    def message(kind, payload):
        output.extend(struct.pack('<HB', len(payload), ord(kind)) + payload)
    formats = [('actuator_motors', 'uint64_t timestamp;float[4] control;', '<Q4f'),
               ('vehicle_local_position', 'uint64_t timestamp;float x;float y;float z;bool xy_valid;bool z_valid;uint8_t xy_reset_counter;uint8_t z_reset_counter;', '<Q3f4B'),
               ('actuator_armed', 'uint64_t timestamp;bool armed;bool lockdown;bool manual_lockdown;bool force_failsafe;', '<Q4B'),
               ('vehicle_land_detected', 'uint64_t timestamp;bool landed;bool ground_contact;bool maybe_landed;', '<Q3B')]
    for name, fields, _ in formats:
        message('F', (name + ':' + fields).encode())
    for identifier, (name, _, _) in enumerate(formats):
        message('A', struct.pack('<BH', 0, identifier) + name.encode())
    for i in range(121):
        stamp = 1000000 + i * 100000
        values = [(stamp, .3, .4, .5, .6), (stamp, i * .01, i * .005, -.6, 1, 1, i >= 80, 0),
                  (stamp, 1, 0, 0, 0), (stamp, i < 10 or i > 110, i < 10 or i > 110, 0)]
        for identifier, (_, _, fmt) in enumerate(formats):
            message('D', struct.pack('<H', identifier) + struct.pack(fmt, *values[identifier]))
    return bytes(output)


class ULogHttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ulog-http-')
        self.addCleanup(self.tmp.cleanup)
        with patch.object(server, 'PROFILE_DIR', self.tmp.name):
            self.wb = server.Workbench(wb_board.load_config(), {'transport': 'local', 'logs_only': True})
        self.wb.logs = Mock()
        class Handler(server.Handler):
            workbench = self.wb
        self.http = server.WorkbenchHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()
        def cleanup():
            self.http.shutdown(); self.http.server_close(); self.thread.join()
            if self.wb.ulogs.pool:
                self.wb.ulogs.pool.shutdown(wait=True)
        self.addCleanup(cleanup)
        self.base = 'http://127.0.0.1:%s' % self.http.server_port

    def request(self, path, payload=None, headers=None):
        req = urllib.request.Request(self.base + path, data=payload, headers=headers or {})
        try:
            with urllib.request.urlopen(req, timeout=10) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as response:
            return response.code, response.read()

    def wait_ready(self, identifier):
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            _, payload = self.request('/api/ulog/library')
            records = json.loads(payload)['records']
            record = next(r for r in records if r['id'] == identifier)
            if record['state'] in ('ready', 'failed'):
                self.assertEqual(record['state'], 'ready', record)
                return record
            time.sleep(.1)
        self.fail('Offline analysis timed out')

    def test_local_upload_parse_history_and_artifacts_without_connection(self):
        payload = synthetic_ulog()
        status, result = self.request('/api/ulog/upload?name=' + quote('本地测试.ulg'), payload)
        self.assertEqual(status, 200, result)
        identifier = json.loads(result)['id']
        record = self.wait_ready(identifier)
        self.assertEqual(record['name'], '本地测试.ulg')
        self.assertEqual(record['summary']['source_file'], '本地测试.ulg')
        self.assertTrue(record['summary']['inventory']['actuator_motors[0]']['samples'] > 100)
        for name in ('trajectory.png', 'overview.png', 'trajectory.csv', 'motor_statistics.csv'):
            self.assertIn(name, record['files'])
            code, content = self.request('/api/ulog/file?id=' + identifier + '&name=' + name)
            self.assertEqual(code, 200); self.assertTrue(content)
        code, content = self.request('/api/ulog/file?id=' + identifier + '&name=input.ulg')
        self.assertEqual(content, payload)
        code, content = self.request('/api/ulog/file?id=' + identifier + '&name=analysis.zip')
        import io
        with zipfile.ZipFile(io.BytesIO(content)) as archive:
            self.assertIsNone(archive.testzip())
            self.assertIn('summary.json', archive.namelist())
            self.assertNotIn('input.ulg', archive.namelist())
        self.assertEqual(wb_ulog.ULogLibrary(self.tmp.name).catalog()['records'][0]['state'], 'ready')
        self.assertEqual(self.wb.sessions.snapshots(), {})
        self.wb.logs.download.assert_not_called()

    def test_invalid_upload_traversal_cross_origin_and_offline_board_refused(self):
        for name, data in [('x.part', synthetic_ulog()), ('x.ulg', b'incomplete'), ('x.ulg', b'wrong header!!!!!')]:
            self.assertEqual(self.request('/api/ulog/upload?name=' + name, data)[0], 400)
        self.assertEqual(self.request('/api/ulog/upload?name=x.ulg', synthetic_ulog(), {'Origin': 'http://evil.invalid'})[0], 400)
        self.assertEqual(self.request('/api/ulog/file?id=../profile&name=profile.json')[0], 400)
        self.assertEqual(self.request('/api/ulog/board', b'{"path":"closed.ulg"}')[0], 400)
        self.assertEqual(self.wb.ulogs.catalog()['records'], [])
        self.wb.logs.download.assert_not_called()

    def test_selected_board_file_reuses_binary_download_only(self):
        self.wb.connection['state'] = 'ok'
        self.wb.options['allow_local_commands'] = True
        self.wb.logs.download.return_value = (0, synthetic_ulog())
        with patch.object(self.wb.ulogs, 'ingest', return_value={'ok': True, 'id': 'fixture'}) as ingest:
            code, data = self.request('/api/ulog/board', b'{"path":"board_run/flight.ulg"}')
            self.assertEqual(code, 200, data)
            self.wb.logs.download.assert_called_once_with('board_run/flight.ulg')
            ingest.assert_called_once_with('flight.ulg', synthetic_ulog())
        self.assertEqual(self.wb.sessions.snapshots(), {})
        self.wb.logs.start.assert_not_called()

    @unittest.skipUnless(os.environ.get('WORKBENCH_TEST_ULOG'), 'No external real ULog supplied')
    def test_real_local_log_http_upload_and_exports(self):
        source = Path(os.environ['WORKBENCH_TEST_ULOG'])
        payload = source.read_bytes()
        status, result = self.request('/api/ulog/upload?name=' + quote(source.name), payload)
        self.assertEqual(status, 200, result)
        identifier = json.loads(result)['id']
        record = self.wait_ready(identifier)
        self.assertEqual(record['size'], source.stat().st_size)
        self.assertEqual(record['summary']['source_file'], source.name)
        self.assertGreater(record['summary']['inventory']['vehicle_local_position[0]']['samples'], 100)
        self.assertIn('trajectory.png', record['files'])
        self.assertIn('trajectory.csv', record['files'])
        code, content = self.request('/api/ulog/file?id=' + identifier + '&name=trajectory.png')
        self.assertEqual(code, 200)
        self.assertTrue(content.startswith(b'\x89PNG\r\n\x1a\n'))
        code, content = self.request('/api/ulog/file?id=' + identifier + '&name=input.ulg')
        self.assertEqual(content, payload)
        self.assertEqual(wb_ulog.ULogLibrary(self.tmp.name).catalog()['records'][0]['state'], 'ready')
        self.assertEqual(self.wb.sessions.snapshots(), {})
        self.wb.logs.download.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
