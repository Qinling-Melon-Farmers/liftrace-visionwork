#!/usr/bin/env python3
"""Hardware adapter for the exact same full strategy used by research."""
import importlib.util,os
from pathlib import Path
import rospy
from uav_mission.high_view_full import HighViewFull
from uav_mission.mission_runtime import MissionRuntime
from uav_high_view.core import Epoch
spec=importlib.util.spec_from_file_location('competition_full_shell',str(Path(__file__).with_name('navigation_high_view_full.py')))
full=importlib.util.module_from_spec(spec);spec.loader.exec_module(full)

class HardwareFullRuntime(HighViewFull):
    "复用已验收 08 的固定板端 session 启动；其余策略均为共有实现。"
    def start(self,mission_id,now,current_xy):
        self.catalog.reset(Epoch(mission_id,'fixed-board-session',self.probe_config.source_key))
        self.survey_until=now+self.core.config.mission_timeout
        return MissionRuntime.start(self,mission_id,now,current_xy)


class CompetitionManager(full.FullManager):
    runtime_type=HardwareFullRuntime


if __name__=='__main__':
    rospy.init_node('mission_manager')
    if rospy.get_param('/use_sim_time',False) or os.environ.get('SIM_RUN_DIR'):
        raise RuntimeError('Competition hardware entry refuses simulation')
    if not rospy.get_param('~hardware_reference_ready',False):
        raise RuntimeError('Use the competition supervisor to establish the ground reference first')
    CompetitionManager(hardware=True)
    rospy.spin()
