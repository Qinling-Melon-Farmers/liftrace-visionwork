"""Exercise extracted package via native PowerShell/Python; no SSH or devices."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--report-dir', type=Path, required=True)
    parser.add_argument('--launcher', choices=('ps1', 'bat'), default='bat')
    args = parser.parse_args()
    if os.name != 'nt':
        parser.error('Run with native Windows Python')
    args.report_dir.mkdir(parents=True, exist_ok=True)
    report = {'platform': sys.platform, 'python': sys.version, 'executable': sys.executable,
              'package': str(args.package), 'launcher': args.launcher,
              'checks': [], 'board_connected': False}
    def check(name, condition):
        if not condition:
            raise AssertionError(name)
        report['checks'].append(name)
    with tempfile.TemporaryDirectory(prefix='wb-native-ui-') as tmp:
        log = args.report_dir / 'windows_launcher.log'
        env = dict(os.environ, PYTHONUTF8='1', WORKBENCH_PYTHON=sys.executable)
        with log.open('wb') as output:
            launch_args = ['-Transport', 'local', '-Port', '8771', '-ProfileDir', tmp]
            if args.launcher == 'bat':
                launch = ['cmd.exe', '/d', '/c', 'call', str(args.package / 'start_windows.bat'), *launch_args]
            else:
                launch = ['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                          '-File', str(args.package / 'start_windows.ps1'), *launch_args]
            process = subprocess.Popen(launch, cwd=args.package, stdout=output,
                stderr=subprocess.STDOUT, env=env, creationflags=subprocess.CREATE_NO_WINDOW)
            try:
                base = None
                for _ in range(200):
                    if process.poll() is not None:
                        raise AssertionError(log.read_text(encoding='utf-8', errors='replace'))
                    matches = re.findall(r'http://127\.0\.0\.1:\d+/', log.read_text(encoding='utf-8', errors='replace'))
                    if matches:
                        base = matches[-1].rstrip('/')
                        try:
                            snapshot = json.load(urllib.request.urlopen(base + '/api/snapshot', timeout=.5))
                            break
                        except OSError:
                            pass
                    time.sleep(.1)
                else:
                    raise AssertionError('Native launcher did not become ready')
                report['url'] = base
                check('no automatic connection or sessions', snapshot['connection']['state'] == 'unknown' and snapshot['sessions'] == {})
                groups = snapshot['groups']
                comp = next(g for g in groups if g['id'] == 'competition')
                check('independent unconfirmed competition template', comp['site_config'] == 'deployment/competition/field.example.yaml')
                for path in ('/', '/logs', '/observe', '/static/app.js', '/static/observe.js', '/static/logs.js'):
                    with urllib.request.urlopen(base + path, timeout=2) as response:
                        data = response.read()
                        check('GET ' + path, response.status == 200 and bool(data))
                        if path == '/':
                            check('same-origin logs link and no motor page link', b'href="/logs"' in data and b'href="/motor"' not in data)
                with urllib.request.urlopen(base + '/motor', timeout=2) as response:
                    check('legacy motor route redirects to realtime observation', response.url == base + '/observe')
                body = dict(group_id='competition', mode='preview', check_config=True,
                            motion_optimization='off', obstacle_columns='on')
                request = urllib.request.Request(base + '/api/trial/command', data=json.dumps(body).encode(),
                                                headers={'Content-Type': 'application/json'})
                planned = json.load(urllib.request.urlopen(request, timeout=2))
                check('native command preview reaches real CLI without backslashes', planned['body'] ==
                    'bash deployment/competition/start.sh preview --site-config deployment/competition/field.example.yaml --motion-optimization off --obstacle-columns on --check-config')
                request = urllib.request.Request(base + '/api/trial/start', data=json.dumps(body).encode(),
                                                headers={'Content-Type': 'application/json'})
                try:
                    urllib.request.urlopen(request, timeout=2)
                    raise AssertionError('offline command unexpectedly executed')
                except urllib.error.HTTPError as error:
                    check('offline execution blocked', error.code == 400)
                check('still no sessions', json.load(urllib.request.urlopen(base + '/api/snapshot'))['sessions'] == {})
                # Run existing real Chromium DOM regressions against this native
                # extracted backend. All launch actions in those tests are mocked.
                browser = Path(os.environ.get('PROGRAMFILES(X86)', 'C:/Program Files (x86)')) / 'Microsoft/Edge/Application/msedge.exe'
                test = Path(__file__).with_name('browser_regression.mjs')
                result = subprocess.run(['node', str(test), base, str(browser), str(args.report_dir / 'windows_ui.png')], capture_output=True, text=True, timeout=90)
                (args.report_dir / 'windows_browser.log').write_text(result.stdout + result.stderr, encoding='utf-8')
                check('native Edge DOM regression', result.returncode == 0)
                result = subprocess.run(['node', str(Path(__file__).with_name('browser_observe.mjs')), base, str(browser)],
                                        capture_output=True, text=True, timeout=90)
                (args.report_dir / 'windows_observe.log').write_text(result.stdout + result.stderr, encoding='utf-8')
                check('native Edge realtime observer regression', result.returncode == 0)
                # The main backend occupies 8771. A second, strictly offline
                # backend must report its actual fallback port, with /logs
                # still relative to that origin. No connect/start API is used.
                fallback_log = args.report_dir / 'windows_port_fallback.log'
                with fallback_log.open('wb') as fallback_output:
                    fallback = subprocess.Popen([sys.executable, str(args.package / 'server.py'),
                        '--transport', 'local', '--port', base.rsplit(':', 1)[1],
                        '--profile-dir', str(Path(tmp) / 'fallback')], cwd=args.package,
                        stdout=fallback_output, stderr=subprocess.STDOUT, env=env,
                        creationflags=subprocess.CREATE_NO_WINDOW)
                    try:
                        fallback_base = None
                        for _ in range(100):
                            if fallback.poll() is not None:
                                raise AssertionError('Fallback backend exited')
                            found = re.findall(r'http://127\.0\.0\.1:\d+/',
                                fallback_log.read_text(encoding='utf-8', errors='replace'))
                            if found:
                                fallback_base = found[-1].rstrip('/')
                                break
                            time.sleep(.1)
                        check('occupied port selects a different actual URL', bool(fallback_base) and fallback_base != base)
                        with urllib.request.urlopen(fallback_base + '/', timeout=2) as response:
                            page = response.read()
                        check('fallback logs link uses the actual backend origin', b'href="/logs"' in page and b'8791' not in page)
                        with urllib.request.urlopen(fallback_base + '/logs', timeout=2) as response:
                            check('fallback same-origin logs page serves successfully', response.status == 200)
                        report['fallback_url'] = fallback_base
                    finally:
                        fallback.terminate()
                        fallback.wait(timeout=5)
                report['result'] = 'PASS'
            finally:
                # Only this test's offline launcher tree, never existing services.
                subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], capture_output=True)
                process.wait(timeout=5)
                (args.report_dir / 'windows_smoke.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
