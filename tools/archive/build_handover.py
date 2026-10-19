#!/usr/bin/env python3
"""Create the onboard runtime source + inference models handover tarball."""
import argparse
import io
import json
import subprocess
import tarfile
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# Current runtime and build dependencies; no legacy copies or historical archives.
SCOPE_PREFIXES = (
    'patrol_uav_ws-patrol_planner/src/',
    'vision_ws/src/uav_vision/', 'vision_ws/src/uav_high_view/',
    'vision_ws/src/camera_sdk/',
    'deployment/board_trials_4x4/', 'deployment/competition/',
    'deployment/site_20260928/', 'deployment/site_20261006/',
    'deployment/low_hover_observation/',
    'tools/', 'top_level_scripts/',
    'docs/competition/',
)
SCOPE_FILES = {
    'README_FIRST.md', 'VISION_WORKSPACE_GUIDE.md', 'AGENTS.md',
    'vision_ws/src/CMakeLists.txt', 'vision_ws/src/uav_board_trials',
    'docs/ENVIRONMENT.md', 'docs/INTERFACES.md', 'docs/CAMERA_AND_FLIGHT.md',
    'docs/2026实机相机与投递机构安装外参基线_20260905.md',
    'docs/closeout_20261019/NEXT_TEAM_README.md',
    'docs/closeout_20261019/ONBOARD_QUICKSTART.md',
    'docs/closeout_20261019/TOOLS.md', 'docs/closeout_20261019/VALIDATION.md',
    'docs/closeout_20261019/SOURCES.json',
    'docs/verification/fov_landing_inner_20260919/seed_2672/fast_runtime.yaml',
    'deployment/README_ONBOARD.md', 'deployment/MODEL_CONTENTS.md',
}

def in_runtime_scope(name):
    return name in SCOPE_FILES or name.startswith(SCOPE_PREFIXES)


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--models', type=Path, default=ROOT / 'runtime_models')
    p.add_argument('--workbench-zip', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Output already exists; select a new filename.')
    if git('diff', '--name-only') or git('diff', '--cached', '--name-only'):
        p.error('Commit tracked source changes before packaging.')
    model_root = a.models.resolve()
    models = sorted(x for x in model_root.rglob('*')
                    if x.is_file() and x.suffix.lower() in ('.rknn', '.yaml', '.yml', '.md'))
    if not any(x.suffix == '.rknn' for x in models) or not any(x.name == 'metadata.yaml' for x in models):
        p.error('The model directory must contain RKNN and metadata.yaml artifacts.')
    if not a.workbench_zip.is_file():
        p.error('Workbench ZIP is missing.')
    files = []
    for raw in git('ls-files', '-z').split(b'\0'):
        if raw:
            name = raw.decode('utf-8')
            if not in_runtime_scope(name):
                continue
            if name.startswith('docs/career_'):
                raise RuntimeError('Personal materials must not be tracked in the handover revision.')
            src = ROOT / name
            if src.is_symlink():
                src.resolve(strict=True).relative_to(ROOT)
            files.append((src, name))
    for src in models:
        files.append((src, 'runtime_models/' + src.relative_to(model_root).as_posix()))
    files.append((a.workbench_zip, 'distributions/' + a.workbench_zip.name))
    prefix = 'liftrace_competition_20261019'
    manifest = {'created_utc': datetime.now(timezone.utc).isoformat(),
                'source_head': git('rev-parse', 'HEAD').decode().strip(),
                'source_branch': git('branch', '--show-current').decode().strip(),
                'external_dependencies': 'See docs/closeout_20261019/NEXT_TEAM_README.md',
                'scope': 'onboard_runtime_source', 'raw_bags_included': False, 'build_outputs_included': False,
                'old_engineering_archives_included': False, 'training_weights_included': False,
                'files': [{'path': name, 'bytes': src.lstat().st_size,
                           'type': 'symlink' if src.is_symlink() else 'file'} for src, name in files]}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(a.output, 'w:gz', compresslevel=6) as arc:
        for src, name in files:
            arc.add(src, arcname=prefix + '/' + name, recursive=False)
        payload = (json.dumps(manifest, ensure_ascii=False, indent=2) + '\n').encode()
        info = tarfile.TarInfo(prefix + '/ARCHIVE_MANIFEST.json')
        info.size = len(payload); info.mode = 0o644
        arc.addfile(info, io.BytesIO(payload))
    # Read every member through gzip and compare regular file byte counts.
    with tarfile.open(a.output, 'r:gz') as arc:
        seen = []
        for member in arc:
            seen.append(member.name)
            if member.isfile():
                stream = arc.extractfile(member)
                size = 0
                while True:
                    block = stream.read(1024 * 1024)
                    if not block:
                        break
                    size += len(block)
                if size != member.size:
                    raise RuntimeError('Truncated member: ' + member.name)
        if len(seen) != len(files) + 1 or len(set(seen)) != len(seen):
            raise RuntimeError('Archive membership mismatch')
    print(json.dumps({'archive': str(a.output.resolve()), 'bytes': a.output.stat().st_size,
                      'members': len(seen), 'source_head': manifest['source_head']}, indent=2))

if __name__ == '__main__':
    main()
