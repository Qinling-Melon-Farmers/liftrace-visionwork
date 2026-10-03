#!/usr/bin/env python3
"""Standalone full-mission startup; no hardware side effects in --check-config."""
import argparse
from pathlib import Path
import yaml
from uav_mission.competition_config import validate,generate
from uav_mission.hardware_session import run_session

def main():
    p=argparse.ArgumentParser()
    p.add_argument('mode',choices=('preview','flight'))
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--site-config',type=Path,required=True)
    p.add_argument('--model',type=Path)
    p.add_argument('--metadata',type=Path)
    p.add_argument('--check-config',action='store_true')
    # roslaunch adds __name/__log remaps; accept only ROS remaps as extra arguments.
    a,extra=p.parse_known_args()
    if any(':=' not in value for value in extra):p.error('unknown arguments: '+str(extra))
    a.trial='competition';a.real_release=a.mode=='flight'
    settings=yaml.safe_load(a.site_config.read_text())
    validate(settings,flight=a.mode=='flight' or not a.check_config)
    cfg=a.root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition'
    rig=yaml.safe_load((cfg/'known_rig.yaml').read_text())
    startup=yaml.safe_load((cfg/'mapping_startup.yaml').read_text())
    if a.check_config:
        print('CONFIG_VALID; no ROS nodes started; site_confirmed='+str(settings['site_confirmed']))
        return
    run_session(a,settings,rig,startup,generate,'uav_mission','competition_')
if __name__=='__main__':main()
