#!/usr/bin/env python3
"""Build a source-only frontline ZIP from an explicit file allowlist."""
import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import zipfile

TOOL = Path(__file__).resolve().parent
ROOT = TOOL.parents[1]
FILES = (
    'server.py', 'wb_ssh.py', 'wb_native_ssh.py', 'wb_board.py', 'wb_status.py', 'wb_logs.py', 'wb_ulog.py',
    'wb_geometry.py', 'wb_survey.py', 'board_probe.py', 'workbench.yaml',
    'start_workbench.sh', 'start_windows.cmd', 'start_windows.bat', 'start_windows.ps1',
    'requirements.txt', 'README_DISTRIBUTION.md', 'README.md',
    'web/index.html', 'web/app.js', 'web/style.css',
    'web/observe.html', 'web/observe.js', 'web/observe.css',
    'web/motor_wiring.json',
    'web/logs.html', 'web/logs.js', 'web/logs.css', 'web/ulog.js',
)
SOURCES = {relative: TOOL / relative for relative in FILES}
VIEWER = ROOT / 'tools/flight_logs/ulg_motor_viewer'
for relative in ('analysis.py', 'plotting.py', 'trajectory.py', 'README.md',
                 'vendor/pyulog/__init__.py', 'vendor/pyulog/core.py', 'vendor/pyulog/px4.py',
                 'vendor/pyulog/LICENSE.md', 'vendor/README.md'):
    SOURCES['ulog_viewer/' + relative] = VIEWER / relative
for relative in ('px4_log_index.py', 'fetch_px4_ulog.py'):
    SOURCES['ulog_viewer/' + relative] = ROOT / 'tools/flight_logs' / relative


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Destination .zip (must not exist)')
    args = parser.parse_args()
    timestamp = datetime.now(timezone.utc)
    prefix = 'liftrace_flight_workbench_' + timestamp.strftime('%Y%m%d_%H%M%S')
    output = args.output or ROOT / 'deliverables' / (prefix + '.zip')
    if output.suffix.lower() != '.zip':
        parser.error('--output must end in .zip')
    for relative, source in SOURCES.items():
        if source.is_symlink() or not source.is_file():
            raise SystemExit('Missing or symlinked package source: ' + relative)
    manifest = {
        'created_utc': timestamp.isoformat(),
        'source_branch': git('branch', '--show-current'),
        'source_head': git('rev-parse', 'HEAD'),
        'source_scope': 'current workbench files, including uncommitted edits',
        'release_channel': 'local_candidate; not deployed to board',
        'modified_package_files': [p for p, source in SOURCES.items() if git('status', '--porcelain', '--', str(source))],
        'platform': 'Same source for WSL/Linux SSH and native Windows Python + Paramiko; offline ULog viewer',
        'default_port': 8771,
        'auto_connect': False,
        'board_deployment_included': False,
        'files': ['README_FIRST.md' if p == 'README_DISTRIBUTION.md' else p for p in SOURCES],
    }
    manifest['release_status']=('local_candidate_pending_commit' if manifest['modified_package_files'] else 'committed_source')+'; not a board deployment'
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for relative, source in SOURCES.items():
            name = 'README_FIRST.md' if relative == 'README_DISTRIBUTION.md' else relative
            payload = source.read_bytes()
            if relative in ('start_windows.cmd', 'start_windows.bat'):
                payload = payload.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
            elif relative == 'start_windows.ps1':
                # Windows PowerShell 5.1 needs a BOM for Chinese extraction paths/docs.
                payload = b'\xef\xbb\xbf' + payload.removeprefix(b'\xef\xbb\xbf')
            archive.writestr(prefix + '/' + name, payload)
        archive.writestr(prefix + '/manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    with zipfile.ZipFile(output) as archive:
        if archive.testzip():
            raise SystemExit('ZIP integrity check failed')
    print(json.dumps({'zip': str(output.resolve()), 'entries': len(SOURCES) + 1,
                      'bytes': output.stat().st_size}, ensure_ascii=False))


if __name__ == '__main__':
    main()
