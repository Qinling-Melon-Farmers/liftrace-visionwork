"""Replay HTTP, local library, exclusion and dependency regressions; no devices."""
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request
from urllib.parse import quote, urljoin

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
import server
import wb_board
import wb_replay


def make_result(directory):
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'index.html').write_text('<video controls src="dashboard.mp4"></video>', encoding='utf-8')
    (directory / 'dashboard.mp4').write_bytes(bytes(range(256)) * 8)
    (directory / 'summary.json').write_text(json.dumps({'video_files': ['dashboard.mp4']}))
    (directory / 'validation.json').write_text(json.dumps({'dashboard.mp4': {'size': '2048'}}))


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='wb-replay-space-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.data = self.root / 'data space'
        self.data.mkdir()
        self.result = self.data / '1008 analysis'
        make_result(self.result)
        self.bag = self.data / 'flight space.bag'
        self.bag.write_bytes(b'#ROSBAG V2.0\nfixture')
        config = wb_board.load_config()
        config['replay'] = {'data_roots': {'local': str(self.data)}}
        with patch.object(server, 'PROFILE_DIR', str(self.root / 'profile')):
            self.wb = server.Workbench(config, {'transport': 'local', 'logs_only': True})
        self.library = self.wb.replays
        self.library.cached_capabilities = {'generate': True, 'view': True, 'detail': 'fixture only'}
        class Handler(server.Handler):
            workbench = self.wb
        self.http = server.WorkbenchHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        thread.start()
        def cleanup():
            self.library.close()
            self.http.shutdown()
            self.http.server_close()
            thread.join()
        self.addCleanup(cleanup)
        self.base = 'http://127.0.0.1:%d' % self.http.server_port

    def request(self, path, body=None, headers=None, method=None):
        request = urllib.request.Request(self.base + path,
                                         data=None if body is None else json.dumps(body).encode(),
                                         headers=headers or {}, method=method)
        try:
            response = urllib.request.urlopen(request, timeout=25)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            return response.status, dict(response.headers), response.read()

    def start(self, **values):
        body = {'root': 'local', 'path': self.bag.name, 'fps': 10, 'frame': 'camera_init'}
        body.update(values)
        return self.request('/api/replay/start', body)

    def test_verified_existing_analysis_discovery_without_generation(self):
        incomplete = self.data / 'unfinished'
        make_result(incomplete)
        (incomplete / 'validation.json').unlink()
        (self.data / 'arbitrary').mkdir()
        (self.data / 'arbitrary/index.html').write_text('not replay')
        with patch.object(wb_replay.subprocess, 'Popen') as spawn:
            status, _, payload = self.request('/api/replay/library')
            self.assertEqual(status, 200)
            data = json.loads(payload)
            self.assertEqual([r['path'] for r in data['results']], ['1008 analysis'])
            self.assertEqual([b['path'] for b in data['bags']], [self.bag.name])
            self.assertFalse(data['truncated'])
            spawn.assert_not_called()
        defaults = wb_replay.ReplayLibrary(self.root / 'defaults')
        self.assertNotIn('logs', defaults.roots)
        self.assertEqual(next(iter(defaults.roots)), 'flights')
        self.assertEqual(self.wb.sessions.snapshots(), {})

    def test_result_html_relative_resources_range_and_opaque_media_origin(self):
        player = self.library.player_url('local', '1008 analysis')
        status, headers, html = self.request(player)
        self.assertEqual(status, 200)
        self.assertIn(b'src="dashboard.mp4"', html)
        self.assertIn('sandbox allow-scripts;', headers['Content-Security-Policy'])
        self.assertIn('connect-src \'none\'', headers['Content-Security-Policy'])
        media = urljoin(player, 'dashboard.mp4')
        raw = (self.result / 'dashboard.mp4').read_bytes()
        for request_range, expected, content_range in [
            ('bytes=100-199', raw[100:200], 'bytes 100-199/2048'),
            ('bytes=2000-', raw[2000:], 'bytes 2000-2047/2048'),
            ('bytes=-20', raw[-20:], 'bytes 2028-2047/2048'),
            ('bytes=2000-9999', raw[2000:], 'bytes 2000-2047/2048')]:
            status, headers, body = self.request(media, headers={
                'Range': request_range, 'Origin': 'null', 'Sec-Fetch-Site': 'cross-site'})
            self.assertEqual(status, 206, body)
            self.assertEqual(body, expected)
            self.assertEqual(headers['Content-Range'], content_range)
            self.assertEqual(headers['Content-Length'], str(len(expected)))
            self.assertEqual(headers['Content-Type'], 'video/mp4')
        status, headers, body = self.request(media, method='HEAD', headers={'Range': 'bytes=0-99', 'Origin': 'null'})
        self.assertEqual((status, body, headers['Content-Length']), (206, b'', '100'))
        self.assertEqual(self.request(media)[2], raw)
        status, headers, body = self.request(media, headers={'Range': 'bytes=100-199', 'Sec-Fetch-Site': 'cross-site'})
        self.assertEqual((status, body), (206, raw[100:200]))
        for invalid in ('bytes=9999-', 'bytes=20-10', 'bytes=-0', 'bytes=0-1,4-6', 'wrong'):
            status, headers, body = self.request(media, headers={'Range': invalid})
            self.assertEqual((status, body), (416, b''))
            self.assertEqual(headers['Content-Range'], 'bytes */2048')

    def test_path_escape_symlinks_and_origin_rejections(self):
        for path in ('../profile', '/etc/passwd', 'C:/file.bag', 'dir\\x.bag', './flight.bag', 'x//y.bag'):
            self.assertEqual(self.start(path=path)[0], 400)
        for path in ('/replay/result/local/%2e%2e/profile/profile.json',
                     '/replay/result/unknown/index.html',
                     '/replay/result/local/1008%20analysis/%2e%2e/%2e%2e/profile.json',
                     '/replay/result/local/1008%20analysis/dashboard.mp4%5c..%5csecret',
                     '/replay/result/local/flight%20space.bag'):
            self.assertEqual(self.request(path)[0], 400, path)
        for encoder in ('gpu', 1, [], None):
            self.assertEqual(self.start(encoder=encoder)[0], 400)
        for fps in (0, 31, True, '10'):
            self.assertEqual(self.start(fps=fps)[0], 400)
        media = self.library.player_url('local', '1008 analysis') + 'dashboard.mp4'
        self.assertEqual(self.request(media, headers={'Origin': 'http://evil.invalid'})[0], 400)
        self.assertEqual(self.request('/api/replay/library', headers={'Origin': 'null'})[0], 400)
        self.assertEqual(self.request('/api/replay/library', headers={'Sec-Fetch-Site': 'cross-site'})[0], 400)
        self.assertEqual(self.request('/api/replay/start', {}, {'Origin': 'null'})[0], 400)
        self.assertEqual(self.request('/api/trial/start', {}, {'Origin': 'null'})[0], 400)
        self.assertEqual(self.request('/api/replay/library', headers={'Host': 'evil.invalid:123'})[0], 400)
        if os.name != 'nt':
            outside = self.root / 'outside'
            make_result(outside)
            (self.data / 'escape').symlink_to(outside, target_is_directory=True)
            self.assertEqual(self.request('/replay/result/local/escape/index.html')[0], 400)
            (self.result / 'dashboard.mp4').unlink()
            (self.result / 'dashboard.mp4').symlink_to(outside / 'dashboard.mp4')
            self.assertEqual(self.library.catalog()['results'], [])
            self.assertEqual(self.request(media)[0], 400)

    def test_native_windows_view_only_and_pythonpath_cleanup(self):
        self.library.cached_capabilities = None
        with patch.object(wb_replay, 'IS_WINDOWS', True), patch.object(wb_replay.subprocess, 'run') as run:
            caps = self.library.capabilities()
            self.assertFalse(caps['generate'])
            self.assertTrue(caps['view'])
            self.assertIn('Windows', caps['detail'])
            self.assertEqual(self.start()[0], 400)
            run.assert_not_called()
            self.assertEqual(len(self.library.catalog()['results']), 1)
        with patch.dict(os.environ, {'PYTHONPATH': '/custom:/usr/lib/python3/dist-packages/:/another'}):
            env = wb_replay.worker_environment()
            self.assertEqual(env['PYTHONPATH'], '/custom:/another')
            self.assertEqual(env['PYTHONUNBUFFERED'], '1')
            self.assertIn('/usr/lib/python3/dist-packages', os.environ['PYTHONPATH'])
        with patch.dict(os.environ, {'PYTHONPATH': '/usr/lib/python3/dist-packages'}):
            self.assertNotIn('PYTHONPATH', wb_replay.worker_environment())

    def test_frontend_view_only_polling_cpu_default_and_explicit_start(self):
        node = shutil.which('node')
        if not node:
            for drive in ('/mnt/c', '/mnt/d'):
                candidate = Path(drive) / 'Program Files/nodejs/node.exe'
                if candidate.is_file():
                    node = str(candidate)
                    break
        if not node:
            self.skipTest('No existing Node; no packages installed')
        source = (TOOL / 'web/replay.js').read_text(encoding='utf-8') + '\n' + (
            TOOL / 'tests/test_replay_frontend.js').read_text(encoding='utf-8')
        result = subprocess.run([node, '-'], input=source, text=True, encoding='utf-8',
                                capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS:', result.stdout)
        html = (TOOL / 'web/replay.html').read_text(encoding='utf-8')
        self.assertLess(html.index('<option value="cpu">'), html.index('<option value="auto">'))

    @unittest.skipIf(os.name == 'nt', 'Generation is WSL/Linux only')
    def test_owned_worker_progress_encoder_exclusion_and_verified_completion(self):
        tool = self.root / 'fake_tool'
        tool.mkdir()
        (tool / 'run.sh').write_text('exec "$REPLAY_PYTHON" "${BASH_SOURCE[0]%/*}/worker.py" "$@"\n')
        (tool / 'worker.py').write_text('''import json, os, pathlib, sys, time
out = pathlib.Path(sys.argv[2]); out.mkdir()
options = dict(zip(sys.argv[3::2], sys.argv[4::2]))
print('Exported 2 camera images')
print('Render 1/2')
(out / 'summary.json').write_text(json.dumps(dict(video_files=['dashboard.mp4'], options=options,
                                                 pythonpath=os.environ.get('PYTHONPATH'), unbuffered=os.environ.get('PYTHONUNBUFFERED'))))
(out / 'dashboard.mp4').write_bytes(b'fixture-video')
(out / 'index.html').write_text('<video src="dashboard.mp4" controls></video>')
(out / 'validation.json').write_text(json.dumps({'dashboard.mp4': {'size': '13'}}))
time.sleep(1.5)
print('All videos decoded; duration checks passed.')
''')
        with patch.object(wb_replay, 'REPLAY_TOOL', tool), patch.dict(os.environ, {
            'REPLAY_PYTHON': sys.executable, 'PYTHONPATH': '/usr/lib/python3/dist-packages'}):
            code, _, payload = self.start(encoder='nvenc')
            self.assertEqual(code, 200, payload)
            identifier = json.loads(payload)['id']
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                jobs = json.loads(self.request('/api/replay/jobs')[2])['jobs']
                if jobs[0]['tail'].startswith('Exported'):
                    break
                time.sleep(.02)
            self.assertEqual(jobs[0]['progress'], 50)
            self.assertEqual(jobs[0]['state'], 'running')
            self.assertEqual(self.start()[0], 400)
            self.assertFalse(any(r['root'] == 'generated' for r in self.library.catalog()['results']))
            self.assertEqual(self.request(jobs[0]['url'])[0], 400)
            second = wb_replay.ReplayLibrary(self.root / 'profile', {'data_roots': {'local': str(self.data)}})
            second.cached_capabilities = self.library.cached_capabilities
            with self.assertRaisesRegex(ValueError, '已有'):
                second.start({'root': 'local', 'path': self.bag.name})
            self.library.thread.join(10)
        job = self.library.jobs()[0]
        self.assertEqual(job['state'], 'ready', job)
        summary = json.loads((self.library.output / identifier / 'result/summary.json').read_text())
        self.assertEqual(summary['options']['--encoder'], 'nvenc')
        self.assertEqual(summary['unbuffered'], '1')
        self.assertIsNone(summary['pythonpath'])
        self.assertEqual(self.request(job['url'])[0], 200)
        reopened = wb_replay.ReplayLibrary(self.root / 'profile')
        self.assertEqual(reopened.jobs()[0]['state'], 'ready')
        self.assertEqual(self.wb.sessions.snapshots(), {})

    @unittest.skipIf(os.name == 'nt', 'POSIX inherited lock')
    def test_child_keeps_exclusion_lock_after_parent_handle_close(self):
        claim = self.library._claim()
        child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(2)'], pass_fds=(claim.fileno(),))
        try:
            claim.close()
            self.assertTrue(self.library._occupied())
            with self.assertRaisesRegex(ValueError, '已有'):
                self.library._claim()
        finally:
            child.terminate()
            child.wait(timeout=5)
        self.assertFalse(self.library._occupied())

    @unittest.skipUnless(Path('/opt/ros/noetic/setup.bash').is_file(), 'No existing ROS Noetic')
    def test_real_dependency_probe_ignores_workbench_system_pythonpath(self):
        library = wb_replay.ReplayLibrary(self.root / 'dependency-profile')
        with patch.dict(os.environ, {'PYTHONPATH': '/usr/lib/python3/dist-packages'}):
            caps = library.capabilities()
        self.assertTrue(caps['generate'], caps)


if __name__ == '__main__':
    unittest.main(verbosity=2)
