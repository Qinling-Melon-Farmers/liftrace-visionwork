#!/usr/bin/env python3
"""Explicit maintainer-only source refresh. Never applies patches or builds."""
import argparse
import difflib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
BOARD = HERE.parents[1]
FAST = 'patrol_uav_ws-patrol_planner/src/FAST_LIO/'
MISSION = 'patrol_uav_ws-patrol_planner/src/uav_mission/'
FILES = [
    FAST+'scripts/ev_predictor.py', FAST+'scripts/ev_shadow.py',
    FAST+'config/ev_shadow.yaml', FAST+'msg/PredictionState.msg',
    MISSION+'src/uav_mission/__init__.py',
    MISSION+'src/uav_mission/task_frame_continuity.py',
    MISSION+'scripts/ev_task_boundary.py',
    MISSION+'config/ev_task_boundary.example.yaml',
    MISSION+'test/test_task_frame_continuity.py',
    MISSION+'test/test_ev_task_boundary.py',
] + ['tools/ev_continuity/'+name+'.py' for name in (
    'test_predictor', 'test_transport', 'offline_demo', 'replay_bag',
    'bag_timing', 'task_boundary_case')]


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def write(relative, data):
    path = HERE/relative
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(data, bytes):
        path.write_bytes(data)
    else:
        with path.open('w', encoding='utf-8', newline='') as handle:
            handle.write(data)


def delta(path, before, after):
    return ''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                      fromfile='a/'+path, tofile='b/'+path))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root', type=Path, required=True)
    p.add_argument('--refresh', action='store_true', required=True)
    args = p.parse_args()
    if (HERE/'integration/fast_lio_snapshot.patch').exists():
        raise SystemExit('Frozen integration patch already exists; do not refresh after main-agent integration')
    ev = args.source_root.resolve()
    records = []
    for relative in FILES:
        data = (ev/relative).read_bytes()
        write('source/'+relative, data)
        records.append(dict(destination='source/'+relative, origin_path=relative,
                            origin_commit=git(ev, 'rev-parse', 'HEAD'), verbatim=True))
    release = MISSION+'msg/ReleasePermission.msg'
    write('contracts/ReleasePermission.msg', (BOARD/release).read_bytes())
    records.append(dict(destination='contracts/ReleasePermission.msg', origin_path=release,
                        origin_commit=git(BOARD, 'rev-parse', 'HEAD'), verbatim=True,
                        runtime='generated from this same board workspace; not vendored code'))
    # Exact raw comparison, including unchanged implementation files; no caches/builds.
    names = sorted(set(git(BOARD, 'ls-files', FAST).splitlines()) |
                   set(git(ev, 'ls-files', FAST).splitlines()))
    comparison, raw = [], []
    for name in names:
        b, e = BOARD/name, ev/name
        before = b.read_text(encoding='utf-8') if b.is_file() else ''
        after = e.read_text(encoding='utf-8') if e.is_file() else ''
        kind = ('same' if before == after else 'add' if not b.exists()
                else 'remove' if not e.exists() else 'modify')
        comparison.append(dict(path=name, difference=kind))
        if kind != 'same': raw.append(delta(name, before, after))
    write('integration/full_fast_lio_comparison.diff', ''.join(raw))
    write('integration/comparison.json', json.dumps(comparison, indent=2)+'\n')
    # Minimal producer integration: preserve board diagnostic_msgs export and threads.
    cmake_path = FAST+'CMakeLists.txt'
    cmake = (BOARD/cmake_path).read_text(encoding='utf-8')
    minimal = cmake.replace('  Pose6D.msg\n', '  Pose6D.msg\n  PredictionState.msg\n', 1)
    minimal = minimal.replace(' DEPENDENCIES\n geometry_msgs\n)',
                              ' DEPENDENCIES\n geometry_msgs\n std_msgs\n)', 1)
    if minimal == cmake:
        raise SystemExit('Base CMake changed; manually review integration, no automatic apply')
    patch = delta(cmake_path, cmake, minimal)
    for relative in (FAST+'src/IMU_Processing.hpp', FAST+'src/laserMapping.cpp'):
        patch += delta(relative, (BOARD/relative).read_text(encoding='utf-8'),
                       (ev/relative).read_text(encoding='utf-8'))
    msg = FAST+'msg/PredictionState.msg'
    patch += delta(msg, '', (ev/msg).read_text(encoding='utf-8'))
    write('integration/fast_lio_snapshot.patch', patch)
    low_path = 'deployment/low_hover_observation/localization.launch'
    low = (BOARD/low_path).read_text(encoding='utf-8')
    enabled = low.replace('<launch>\n', '<launch>\n'
        '  <arg name="enable_prediction_state" default="false"/>\n'
        '  <arg name="prediction_imu_frame" default="livox_frame"/>\n', 1)
    marker = '  <node pkg="fast_lio" type="fastlio_mapping" name="laserMapping" output="screen">\n'
    if marker not in low:
        raise SystemExit('Low-hover LIO node changed; review optional launch patch manually')
    enabled = enabled.replace(marker, marker+
        '    <param name="prediction_state_enabled" value="$(arg enable_prediction_state)"/>\n'
        '    <param name="prediction_imu_frame" value="$(arg prediction_imu_frame)"/>\n', 1)
    write('integration/low_hover_prediction_optional.patch', delta(low_path, low, enabled))
    manifest = dict(date='2026-10-06', source_root_recorded=str(ev),
        source_branch=git(ev, 'branch', '--show-current'), source_commit=git(ev, 'rev-parse', 'HEAD'),
        board_base_commit=git(BOARD, 'rev-parse', 'HEAD'),
        source_worktree_status=git(ev, 'status', '--porcelain'),
        files=records, deployment_executed=False, nodes_started=False,
        default_mode='preview', new_workspace=False, production_ev_enabled=False,
        prediction_state_enabled_default=False, calibration_verified=False,
        required_match_threads=3, generated_message_authority='current board workspace',
        missing_online_contracts=['authoritative FC reset', 'independent LIO health'],
        excluded=['EV ReleasePermission.msg', 'build', 'devel', '__pycache__',
                  'formal launch/config changes', 'LIO startup', 'MAVROS/flight/actuator commands'])
    write('manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print('Prepared verbatim sources and review-only integration patch in', HERE)


if __name__ == '__main__':
    main()
