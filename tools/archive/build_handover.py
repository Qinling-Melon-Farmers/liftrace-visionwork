#!/usr/bin/env python3
"""Create the complete source + externally installed models handover tarball."""
import argparse
import io
import json
import subprocess
import tarfile
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

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
    models = sorted(x for x in model_root.rglob('*') if x.is_file())
    if not all(any(x.suffix == ext for x in models) for ext in ('.pt', '.onnx', '.rknn')):
        p.error('The model directory must contain PT, ONNX and RKNN artifacts.')
    if not a.workbench_zip.is_file():
        p.error('Workbench ZIP is missing.')
    files = []
    for raw in git('ls-files', '-z').split(b'\0'):
        if raw:
            name = raw.decode('utf-8')
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
                'raw_bags_included': False, 'build_outputs_included': False,
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
