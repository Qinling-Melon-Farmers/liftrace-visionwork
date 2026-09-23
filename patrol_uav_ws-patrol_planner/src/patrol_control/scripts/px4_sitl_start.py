#!/usr/bin/env python3
"""Load a SITL profile through PX4's native pre-estimator parameter hook."""
import math
import os
from pathlib import Path
import re
import shlex
import sys


def parameter_script(original, parameters):
    lines = ['#!/bin/sh', '. ' + shlex.quote(str(original))]
    for key, value in sorted(parameters.items()):
        if not re.fullmatch(r'[A-Z][A-Z0-9_]{0,15}', key):
            raise ValueError('invalid PX4 parameter name')
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('PX4 SITL parameters must be finite numbers')
        lines.append('param set %s %s' % (key, value))
    return '\n'.join(lines)+'\n'


def main():
    import rospy
    args = sys.argv[1:]
    parameters = rospy.get_param('/simulation/px4_parameters', {})
    if parameters:
        if not rospy.get_param('/use_sim_time', False) or not os.environ.get('SIM_RUN_DIR'):
            raise RuntimeError('PX4 SITL profile requires the authorized simulation run directory')
        rootfs = Path(args[1]).resolve()
        original = rootfs/'init.d-posix/px4-rc.params'
        if not original.is_file():
            raise RuntimeError('PX4 native parameter hook is missing: '+str(original))
        directory = Path(os.environ['SIM_RUN_DIR'])/'px4'/'profile'
        directory.mkdir(parents=True, exist_ok=True)
        hook = directory/'px4-rc.params'
        hook.write_text(parameter_script(original, parameters), encoding='utf-8')
        os.environ['PATH'] = str(directory)+os.pathsep+os.environ['PATH']
        print('PX4 SITL parameters will load before sensors and EKF: '+str(hook), flush=True)
    os.execv(args[0], args)


if __name__ == '__main__':
    main()
