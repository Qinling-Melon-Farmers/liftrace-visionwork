#!/usr/bin/env python3
"""Hardware adapter for the exact same full strategy used by research."""
import importlib.util,os
from pathlib import Path
import rospy
spec=importlib.util.spec_from_file_location('competition_full_shell',str(Path(__file__).with_name('navigation_high_view_full.py')))
full=importlib.util.module_from_spec(spec);spec.loader.exec_module(full)

if __name__=='__main__':
    rospy.init_node('mission_manager')
    if rospy.get_param('/use_sim_time',False) or os.environ.get('SIM_RUN_DIR'):
        raise RuntimeError('Competition hardware entry refuses simulation')
    if not rospy.get_param('~hardware_reference_ready',False):
        raise RuntimeError('Use the competition supervisor to establish the ground reference first')
    full.FullManager(hardware=True)
    rospy.spin()
