#!/usr/bin/env python3
"""Offline tests/cases: no master, node initialization, ROS publications or hardware."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import runpy
import sys
import tempfile
import unittest

from ev_suite.bootstrap import HERE, SOURCE, TOOLS, MISSION_TESTS, components, message_checks


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ros', action='store_true')
    p.add_argument('--kind', choices=['shadow', 'reset', 'all'], default='all')
    p.add_argument('--demo', action='store_true')
    p.add_argument('--out', type=Path, default=HERE/'verification/offline_result.json')
    p.add_argument('--bag', type=Path)
    p.add_argument('--timing-only', action='store_true')
    args = p.parse_args()
    sys.dont_write_bytecode = True
    scratch = HERE/'verification/.tmp'
    scratch.mkdir(parents=True, exist_ok=True)
    tempfile.tempdir = str(scratch)
    os.environ['TMPDIR'] = str(scratch)
    os.environ['MPLCONFIGDIR'] = str(scratch/'matplotlib')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    components(ros=args.ros)
    suite = unittest.TestSuite()
    schemas = {}
    if args.ros:
        # No init_node; fail closed on stale/foreign generated messages.
        if args.bag and args.timing_only:
            import rosbag  # No PredictionState required for original timing.
        else:
            schemas = message_checks(prediction=args.kind != 'reset')
        if not args.bag:
            if args.kind in ('reset', 'all'):
                suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(
                    module(MISSION_TESTS/'test_ev_task_boundary.py', 'suite_ros_reset')))
                suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(
                    module(HERE/'tests/test_ros_wrapper.py', 'suite_ros_wrapper')))
            if args.kind in ('shadow', 'all'):
                suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(
                    module(TOOLS/'test_transport.py', 'suite_transport')))
    elif args.bag:
        p.error('--bag requires --ros')
    else:
        for path, name in ((TOOLS/'test_predictor.py', 'suite_predictor'),
                           (MISSION_TESTS/'test_task_frame_continuity.py', 'suite_task'),
                           (HERE/'tests/test_isolation.py', 'suite_isolation'),
                           (HERE/'tests/test_recorder.py', 'suite_recorder')):
            suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module(path, name)))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(offline=True, master_started=False, nodes_started=False, board_connected=False,
                  tests=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  messages=schemas, success=result.wasSuccessful())
    if args.bag:
        script = TOOLS/('bag_timing.py' if args.timing_only else 'replay_bag.py')
        target = args.out.parent/('bag_timing.json' if args.timing_only else 'bag_replay')
        sys.argv = [str(script), str(args.bag), '--out', str(target)]
        runpy.run_path(str(script), run_name='__main__')
        report['bag_result'] = str(target)
    if args.demo:
        if args.ros:
            p.error('--demo uses conda/non-ROS mode')
        for name, destination in (('offline_demo.py', args.out.parent/'synthetic_demo'),
                                  ('task_boundary_case.py', args.out.parent/'fc_reset_case.json')):
            script = TOOLS/name
            sys.argv = [str(script), '--out', str(destination)]
            runpy.run_path(str(script), run_name='__main__')
        report['cases'] = 'synthetic only; not board flight evidence'
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ImportError, RuntimeError) as exc:
        print('BLOCKED prerequisites; no master/node started: '+str(exc), file=sys.stderr)
        sys.exit(3)
