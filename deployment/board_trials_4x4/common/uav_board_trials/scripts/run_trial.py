#!/usr/bin/env python3
"""Board supervisor: never arms; optionally starts the mission after manual arm/hover."""
import argparse,json,math,os,signal,subprocess,threading,time,shutil
from pathlib import Path
from collections import deque
import numpy as np,yaml
from trial_config import generate,validate_settings,apply_site_profile,TRIAL_FOLDERS,NO_DROP_MODES,mapping_profile
from uav_mission.hardware_session import run_session
from mapping_startup import PoseAgreement,MapWarmup,VisionReadiness

def main():
    p=argparse.ArgumentParser();p.add_argument('trial',choices=sorted(TRIAL_FOLDERS));p.add_argument('mode',choices=['preview','flight']);p.add_argument('--root',type=Path,required=True);p.add_argument('--model',type=Path);p.add_argument('--metadata',type=Path);p.add_argument('--check-config',action='store_true');p.add_argument('--site-config',type=Path);p.add_argument('--real-release',action='store_true');p.add_argument('--mapping-startup-config',type=Path);p.add_argument('--capture-speed',type=float,choices=(.5,1.));p.add_argument('--capture-lighting',choices=('normal','dim','unspecified'));a=p.parse_args()
    folder=TRIAL_FOLDERS[a.trial]
    base=a.root/'deployment/board_trials_4x4';settings=yaml.safe_load((base/folder/'settings.yaml').read_text());rig=yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
    startup=yaml.safe_load((a.mapping_startup_config or base/'common/uav_board_trials/config/mapping_startup.yaml').read_text())
    agreement=PoseAgreement(startup)
    vision_ready=VisionReadiness(startup['vision_detection_topics']+[startup['vision_targets_topic']],startup['vision_max_age'])
    MapWarmup(0.,startup)  # Validate before starting ROS children.
    for key in ('map_timeout','state_max_age'):
        if not math.isfinite(float(startup[key])) or float(startup[key])<=0:raise ValueError('invalid mapping startup '+key)
    if a.site_config:
        profile=yaml.safe_load(a.site_config.read_text()) or {}
        try:apply_site_profile(settings,profile)
        except ValueError as error:p.error(str(error))
    if a.capture_speed is not None or a.capture_lighting is not None:
        if a.trial!='high_speed_capture':p.error('Capture options are only valid for high_speed_capture')
        if a.capture_speed is not None:settings['cruise_speed']=a.capture_speed
        if a.capture_lighting is not None:settings['capture_lighting']=a.capture_lighting
    if settings.get('actuator_mode','mock')!='mock':p.error('settings must default to mock; use --real-release explicitly')
    if a.real_release and (a.mode!='flight' or settings['mode'] in NO_DROP_MODES):p.error('--real-release requires a delivery flight module')
    settings['actuator_mode']='real' if a.real_release else ('none' if settings['mode'] in NO_DROP_MODES else 'mock')
    try:validate_settings(settings)
    except ValueError as error:p.error(str(error))
    if a.check_config:
        print('CONFIG_VALID; no ROS nodes started');return
    settings['mapping_profile']=mapping_profile(settings)
    run_session(a,settings,rig,startup,generate,'uav_board_trials',finish_script=Path(__file__).with_name('finish_trial.py'))
if __name__=='__main__':main()
