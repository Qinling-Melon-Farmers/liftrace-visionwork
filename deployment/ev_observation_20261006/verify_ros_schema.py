#!/usr/bin/env python3
"""Offline schema-only verification when local devel is stale.

Generates disposable Python serialization fixtures inside this suite using ROS
genpy and the actual message definitions. This is NOT a Catkin/ARM build or an
online producer, and production message_checks deliberately rejects fixtures.
"""
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from ev_suite.bootstrap import HERE, ROOT, SOURCE, TOOLS, MISSION_TESTS, components
from offline import module


def main():
    scratch = HERE/'verification/.tmp'
    scratch.mkdir(parents=True, exist_ok=True)
    tempfile.tempdir = str(scratch)
    os.environ['TMPDIR'] = str(scratch)
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    sys.dont_write_bytecode = True
    generated = scratch/'generated_schema'
    definitions = {
        'uav_mission': ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/msg/ReleasePermission.msg',
        'fast_lio': SOURCE/'patrol_uav_ws-patrol_planner/src/FAST_LIO/msg/PredictionState.msg'}
    command = '/opt/ros/noetic/lib/genpy/genmsg_py.py'
    for package, source in definitions.items():
        directory = generated/package/'msg'
        directory.mkdir(parents=True, exist_ok=True)
        subprocess.run([sys.executable, command, '-p', package, '-o', str(directory),
            '-Istd_msgs:/opt/ros/noetic/share/std_msgs/msg',
            '-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/msg', str(source)], check=True)
        subprocess.run([sys.executable, command, '--initpy', '-o', str(directory)], check=True)
        (generated/package/'__init__.py').write_text('', encoding='utf-8')
    # Test-only process. The runtime wrapper has no option to select this path.
    sys.path.insert(0, str(generated))
    os.environ['PYTHONPATH'] = str(generated)+':'+str(HERE)+':'+os.environ.get('PYTHONPATH', '')
    components(ros=True)
    suite = unittest.TestSuite()
    for path, name in ((MISSION_TESTS/'test_ev_task_boundary.py', 'schema_ros_reset'),
                       (TOOLS/'test_transport.py', 'schema_transport'),
                       (HERE/'tests/test_ros_wrapper.py', 'schema_ros_wrapper')):
        suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module(path, name)))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(scope='offline ROS serialization fixtures generated from actual schemas',
        fixture_sources={key: str(value) for key, value in definitions.items()},
        tests=result.testsRun, failures=len(result.failures), errors=len(result.errors),
        success=result.wasSuccessful(), catkin_build_verified=False, arm_build_verified=False,
        runtime_message_authority_verified=False, master_started=False, nodes_started=False,
        source_generation='genpy; runtime preflight rejects this fixture location')
    (HERE/'verification/schema_fixture_result.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__': sys.exit(main())
