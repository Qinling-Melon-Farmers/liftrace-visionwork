#!/usr/bin/env python3
"""Export committed onboard/simulation source bundles; never starts ROS or hardware."""
import argparse
import io
import json
import re
from pathlib import Path
import subprocess
import tarfile
from datetime import datetime, timezone


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--label', default='r62', help='Filename version label')
    parser.add_argument('--onboard-rknn', type=Path, required=True)
    parser.add_argument('--sitl-weights', type=Path, required=True)
    parser.add_argument('--optional-model-root', type=Path,
                        help='PX4 model root for historical camera fixtures only')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.label):
        parser.error('--label must be a simple filename label')
    root = Path(__file__).resolve().parents[1]
    output = (args.output or root / 'deliverables').resolve()
    if git(root, 'status', '--porcelain'):
        raise SystemExit('Commit the source and documents before packaging.')
    for model in (args.onboard_rknn, args.sitl_weights):
        if not model.is_file():
            raise SystemExit('Missing explicitly selected model: ' + str(model))
    revision = git(root, 'rev-parse', 'HEAD')
    tracked = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z']).decode('utf-8').rstrip('\0').split('\0')
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for kind, weights in [('onboard', args.onboard_rknn), ('simulation', args.sitl_weights)]:
        name = 'liftrace_' + args.label + '_' + kind + '_' + revision[:8]
        archive = output / (name + '.tar.gz')
        if archive.exists():
            raise SystemExit('Archive already exists: ' + str(archive))
        entries = []
        for relative in tracked:
            p = Path(relative)
            common = relative.startswith(('patrol_uav_ws-patrol_planner/src/',
                                         'vision_ws/src/uav_vision/',
                                         'vision_ws/src/camera_sdk/'))
            current_docs = relative in ['README.md', 'AGENTS.md', 'VISION_2026_ROADMAP.md'] or relative.startswith(('docs/', 'deployment/'))
            onboard = common or current_docs or relative in (
                'vision_ws/src/CMakeLists.txt', 'top_level_scripts/build_competition.sh')
            if kind == 'simulation' or onboard:
                entries.append((root / p, relative))
        model_name = 'merged_standard_fp32.rknn' if kind == 'onboard' else 'merged_standard.pt'
        entries.append((weights, 'runtime_models/' + model_name))
        entries.append((root / 'deployment' / ('README_' + kind.upper() + '.md'), 'README_FIRST.md'))
        optional = []
        if kind == 'simulation' and args.optional_model_root:
            for model in ['D435i', 'fpv_cam', 'tanke']:
                source = args.optional_model_root / model
                if not source.is_dir():
                    raise SystemExit('Missing requested historical model: ' + str(source))
                optional.append(model)
                for f in sorted(source.rglob('*')):
                    if f.is_file() and '.git' not in f.parts:
                        entries.append((f, str(Path('simulation_assets/optional_models') / model / f.relative_to(source))))
        manifest = {
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'kind': kind, 'source_revision': revision,
            'source_branch': git(root, 'branch', '--show-current'),
            'status': 'SOURCE_PACKAGE_NOT_FLIGHT_ACCEPTANCE',
            'onboard_architecture': 'aarch64 / RK3588; build on target, no x86 build/devel copied',
            'weights_source': str(weights.resolve()), 'weights_destination': 'runtime_models/' + model_name,
            'optional_historical_models': optional,
            'external_runtime_dependencies': ['ROS Noetic and system libraries',
                'hardware: MAVROS/FC, device Livox SDK/driver, RKNN Lite2/NPU runtime, mechanical Servo implementation'
                if kind == 'onboard' else
                'PX4 SITL including iris_mid360 autostart, Gazebo Classic/PX4 plugins, MID360 plugin with scan CSV, Python inference environment'],
            'files': [{'path': p, 'bytes': f.stat().st_size} for f, p in entries],
        }
        with tarfile.open(archive, 'w:gz', dereference=True) as tar:
            for source, relative in entries:
                tar.add(source, arcname=name + '/' + relative, recursive=False)
            data = json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8')
            info = tarfile.TarInfo(name + '/BUNDLE_MANIFEST.json'); info.size = len(data); info.mode = 0o644
            tar.addfile(info, io.BytesIO(data))
        # Read every member, check portable paths, and force gzip CRC validation.
        with tarfile.open(archive, 'r:gz') as tar:
            count = 0
            for member in tar:
                if member.issym() or member.islnk() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:
                    raise RuntimeError('Non-portable archive member: ' + member.name)
                if member.isfile():
                    stream = tar.extractfile(member)
                    while stream.read(1024 * 1024):
                        pass
                    count += 1
        result = {'kind': kind, 'archive': str(archive), 'bytes': archive.stat().st_size,
                  'source_revision': revision, 'file_count': count, 'archive_readback': 'PASS'}
        results.append(result)
        print(json.dumps(result, ensure_ascii=False))
    (output / (args.label + '_bundles_' + revision[:8] + '.json')).write_text(
        json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
