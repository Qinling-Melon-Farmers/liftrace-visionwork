"""Extract unchanged final2 to a stable directory and capture four offline pages."""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import urllib.request
import zipfile

source = Path(r'\\wsl.localhost\Ubuntu-20.04\home\xhj\liftrace-worktrees\r2026-board-vision-tests')
out = source / 'docs/verification/workbench_release_20261008/screenshots'
archive = Path(r'C:\Users\ASUS\Downloads\liftrace_flight_workbench_windows_20261008_final2.zip')
target = Path(r'C:\Users\ASUS\Downloads\liftrace_flight_workbench_windows_20261008')
out.mkdir(parents=True, exist_ok=True)
report = {'archive': str(archive), 'startup_directory': str(target), 'launcher': str(target / 'start_windows.bat'), 'existing_directory': target.exists()}
with zipfile.ZipFile(archive) as z:
    files = {}
    for entry in z.infolist():
        if entry.is_dir():
            continue
        name = Path(*entry.filename.split('/')[1:])
        dest = (target / name).resolve()
        if not dest.is_relative_to(target.resolve()):
            raise RuntimeError('Invalid package path')
        files[name] = z.read(entry)
    differing = [str(n) for n, b in files.items() if (target / n).exists() and (target / n).read_bytes() != b]
    if differing:
        raise RuntimeError('Preserved existing differing files: ' + repr(differing))
    for name, payload in files.items():
        dest = target / name
        if not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            with dest.open('xb') as f:
                f.write(payload)
    mismatches = []
    checked = []
    for name, payload in files.items():
        if str(name).lower() == 'manifest.json':
            continue
        source_name = Path('README_DISTRIBUTION.md') if str(name) == 'README_FIRST.md' else name
        original = (source / 'tools/flight_workbench' / source_name).read_bytes()
        if name.suffix in ('.bat', '.cmd'):
            original = original.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
        if name.suffix == '.ps1':
            original = b'\xef\xbb\xbf' + original.removeprefix(b'\xef\xbb\xbf')
        checked.append(str(name))
        if payload != original:
            mismatches.append(str(name))
    report.update(extraction_matches_zip=all((target / n).read_bytes() == b for n, b in files.items()), source_payloads_checked=checked, source_mismatches=mismatches)
    if mismatches:
        raise RuntimeError('Package/source differ: ' + repr(mismatches))

with tempfile.TemporaryDirectory(prefix='liftrace-screenshot-offline-') as profile:
    env = dict(os.environ, PYTHONUTF8='1', WORKBENCH_PYTHON=sys.executable)
    with (out / 'launcher.log').open('wb') as log:
        proc = subprocess.Popen(['cmd.exe', '/d', '/c', 'call', str(target / 'start_windows.bat'), '-Transport', 'local', '-Port', '8771', '-ProfileDir', profile], cwd=target, env=env, stdout=log, stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW)
        report['owned_launcher_pid'] = proc.pid
        try:
            for _ in range(200):
                if proc.poll() is not None:
                    raise RuntimeError('Launcher exited')
                urls = re.findall(r'http://127\.0\.0\.1:\d+/', (out / 'launcher.log').read_text(encoding='utf-8', errors='replace'))
                if urls:
                    base = urls[-1].rstrip('/')
                    try:
                        snap = json.load(urllib.request.urlopen(base + '/api/snapshot', timeout=1))
                        break
                    except OSError:
                        pass
                time.sleep(.1)
            else:
                raise RuntimeError('Offline startup timeout')
            if snap['connection']['transport'] != 'local' or snap['connection']['state'] != 'unknown' or snap['sessions']:
                raise RuntimeError('Expected offline empty state')
            report['url'] = base
            report['initial_connection'] = snap['connection']['state']
            report['sessions'] = snap['sessions']
            with (out / 'browser.log').open('wb') as browser_log:
                subprocess.run(['node', str(out / 'capture.mjs'), base, str(out)], cwd=target, env=env, stdout=browser_log, stderr=subprocess.STDOUT, check=True, timeout=90, creationflags=subprocess.CREATE_NO_WINDOW)
            with urllib.request.urlopen(base + '/motor', timeout=2) as response:
                report['motor_redirect'] = response.url
            report['routes'] = ['/', '/observe', '/logs']
        finally:
            subprocess.run(['taskkill', '/PID', str(proc.pid), '/T', '/F'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=subprocess.CREATE_NO_WINDOW)
            proc.wait(timeout=10)
            report['owned_backend_stopped'] = True
            (out / 'capture_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
