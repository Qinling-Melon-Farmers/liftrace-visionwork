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
    'server.py', 'wb_ssh.py', 'wb_board.py', 'wb_status.py', 'wb_logs.py',
    'wb_geometry.py', 'wb_survey.py', 'board_probe.py', 'workbench.yaml',
    'start_workbench.sh', 'start_windows.cmd', 'start_windows.ps1',
    'requirements.txt', 'README_DISTRIBUTION.md', 'README.md',
    'web/index.html', 'web/app.js', 'web/style.css',
    'web/observe.html', 'web/observe.js', 'web/observe.css',
    'web/motor_wiring.json',
    'web/logs.html', 'web/logs.js', 'web/logs.css',
)


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
    for relative in FILES:
        source = TOOL / relative
        if source.is_symlink() or not source.is_file():
            raise SystemExit('Missing or symlinked package source: ' + relative)
    manifest = {
        'created_utc': timestamp.isoformat(),
        'source_branch': git('branch', '--show-current'),
        'source_head': git('rev-parse', 'HEAD'),
        'source_scope': 'current workbench files, including uncommitted edits',
        'modified_package_files': [p for p in FILES if git('status', '--porcelain', '--', str(TOOL / p))],
        'platform': 'Linux / Windows with WSL; not native Windows Python',
        'board_deployment_included': False,
        'files': ['README_FIRST.md' if p == 'README_DISTRIBUTION.md' else p for p in FILES],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for relative in FILES:
            name = 'README_FIRST.md' if relative == 'README_DISTRIBUTION.md' else relative
            payload = (TOOL / relative).read_bytes()
            if relative == 'start_windows.cmd':
                payload = payload.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
            elif relative == 'start_windows.ps1':
                # Windows PowerShell 5.1 needs a BOM for Chinese extraction paths/docs.
                payload = b'\xef\xbb\xbf' + payload.removeprefix(b'\xef\xbb\xbf')
            archive.writestr(prefix + '/' + name, payload)
        archive.writestr(prefix + '/manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    with zipfile.ZipFile(output) as archive:
        if archive.testzip():
            raise SystemExit('ZIP integrity check failed')
    print(json.dumps({'zip': str(output.resolve()), 'entries': len(FILES) + 1,
                      'bytes': output.stat().st_size}, ensure_ascii=False))


if __name__ == '__main__':
    main()
