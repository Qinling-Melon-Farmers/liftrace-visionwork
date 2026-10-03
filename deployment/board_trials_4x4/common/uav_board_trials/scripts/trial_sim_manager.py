#!/usr/bin/env python3
"""SITL-only adapter using the exact board mission runtimes."""
import sys
from pathlib import Path
import rospkg,rospy
sys.path.insert(0,str(Path(rospkg.RosPack().get_path('uav_board_trials'))/'scripts'))
from trial_manager import BoardManager
if __name__=='__main__':
    rospy.init_node('mission_manager')
    BoardManager(simulation=True)
    rospy.spin()
