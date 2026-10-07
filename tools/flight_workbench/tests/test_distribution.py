"""Verify the ZIP as a standalone extracted workbench, without SSH or devices."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
import zipfile

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
import build_zip


class DistributionTests(unittest.TestCase):
    def test_allowlist_and_standalone_launch(self):
        with tempfile.TemporaryDirectory(prefix='wb-distribution-') as tmp:
            tmp = Path(tmp)
            archive_path = tmp / 'frontline.zip'
            subprocess.run([sys.executable, str(TOOL / 'build_zip.py'), '--output',
                            str(archive_path)], check=True, capture_output=True)
            with zipfile.ZipFile(archive_path) as archive:
                self.assertIsNone(archive.testzip())
                names = archive.namelist()
                prefix = names[0].split('/')[0]
                expected = {prefix + '/' + ('README_FIRST.md' if f == 'README_DISTRIBUTION.md' else f)
                            for f in build_zip.FILES} | {prefix + '/manifest.json'}
                self.assertEqual(set(names), expected)
                self.assertTrue(all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in names))
                manifest = json.loads(archive.read(prefix + '/manifest.json'))
                self.assertFalse(manifest['board_deployment_included'])
                self.assertIn('uncommitted', manifest['source_scope'])
                self.assertIn(b'\r\n', archive.read(prefix + '/start_windows.cmd'))
                self.assertTrue(archive.read(prefix + '/start_windows.ps1').startswith(b'\xef\xbb\xbf'))
                archive.extractall(tmp / 'extracted space')
            package = tmp / 'extracted space' / prefix
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            base = 'http://127.0.0.1:%d' % port
            env = dict(os.environ, WORKBENCH_PYTHON=sys.executable)
            # Start from outside the source repository; no allow-local-commands.
            with (tmp / 'server.log').open('w') as log:
                process = subprocess.Popen(['bash', str(package / 'start_workbench.sh'),
                                            '--host', '127.0.0.1', '--port', str(port),
                                            '--transport', 'local', '--profile-dir', str(tmp / 'state')],
                                           cwd=tmp, env=env, stdout=log, stderr=subprocess.STDOUT)
                try:
                    for _ in range(100):
                        if process.poll() is not None:
                            self.fail((tmp / 'server.log').read_text())
                        try:
                            with urllib.request.urlopen(base + '/api/snapshot', timeout=.5) as response:
                                snapshot = json.load(response)
                            break
                        except OSError:
                            time.sleep(.1)
                    else:
                        self.fail('Extracted service did not become ready')
                    self.assertEqual(snapshot['connection']['state'], 'unknown')
                    self.assertEqual(snapshot['sessions'], {})
                    self.assertEqual(len(snapshot['groups']), 13)
                    observation = [g for g in snapshot['groups'] if g.get('channel') == 'low_observation']
                    self.assertEqual([g['profile'] for g in observation], ['hover', 'forward', 'square'])
                    for url in ('/', '/observe', '/motor', '/static/app.js', '/static/observe.js',
                                '/static/style.css', '/static/observe.css', '/static/motor_wiring.json'):
                        with urllib.request.urlopen(base + url, timeout=2) as response:
                            self.assertEqual(response.status, 200)
                            self.assertTrue(response.read())
                    body = json.dumps(dict(group_id='observation_hover', mode='preview',
                                           check_config=True)).encode()
                    req = urllib.request.Request(base + '/api/trial/command', data=body,
                                                 headers={'Content-Type': 'application/json'})
                    with urllib.request.urlopen(req, timeout=2) as response:
                        planned = json.load(response)
                    self.assertTrue(planned['ok'])
                    self.assertIn('preview hover', planned['body'])
                    # Previewing a command must not open a terminal; attempting to
                    # execute even a preview is refused in this offline service.
                    req = urllib.request.Request(base + '/api/trial/start', data=body,
                                                 headers={'Content-Type': 'application/json'})
                    with self.assertRaises(urllib.error.HTTPError) as caught:
                        urllib.request.urlopen(req, timeout=2)
                    self.assertEqual(caught.exception.code, 400)
                    with urllib.request.urlopen(base + '/api/snapshot', timeout=2) as response:
                        self.assertEqual(json.load(response)['sessions'], {})
                finally:
                    process.terminate()
                    process.wait(timeout=5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
