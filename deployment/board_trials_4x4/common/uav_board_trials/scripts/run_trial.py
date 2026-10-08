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
    p=argparse.ArgumentParser();p.add_argument('trial',choices=sorted(TRIAL_FOLDERS));p.add_argument('mode',choices=['preview','flight']);p.add_argument('--root',type=Path,required=True);p.add_argument('--model',type=Path);p.add_argument('--metadata',type=Path);p.add_argument('--check-config',action='store_true');p.add_argument('--site-config',type=Path);p.add_argument('--real-release',action='store_true');p.add_argument('--mapping-startup-config',type=Path);p.add_argument('--capture-speed',type=float,choices=(.5,1.,1.2));p.add_argument('--capture-lighting',choices=('normal','dim','unspecified'));p.add_argument('--motion-optimized',action='store_true');p.add_argument('--survey-pattern',choices=('rectangle','snake2','snake3'))
    p.add_argument('--resume-survey',choices=('on','off'),help='Override interrupted survey resume for priority/full mission; omit to inherit settings')
    p.add_argument('--speed-profile',choices=('limited','competition'),help='08 only: same measured field, explicit planning/following speeds')
    p.add_argument('--generate-config',type=Path,help='With --check-config: write configuration only, never start ROS')
    p.add_argument('--reference-fc',type=float,nargs=3,metavar=('X','Y','Z'),help='Stationary reference for offline generation')
    a=p.parse_args()
    if a.generate_config and (not a.check_config or a.reference_fc is None):
        p.error('--generate-config requires --check-config and --reference-fc X Y Z')
    if a.reference_fc is not None and not a.generate_config:
        p.error('--reference-fc requires --generate-config')
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
    if a.speed_profile is not None:
        from trial_speed_profiles import apply_speed_profile
        try:apply_speed_profile(a.root,settings,a.trial,a.speed_profile)
        except ValueError as error:p.error(str(error))
    if a.capture_speed is not None or a.capture_lighting is not None:
        if a.trial!='high_speed_capture':p.error('Capture options are only valid for high_speed_capture')
        if a.capture_speed is not None:settings['cruise_speed']=a.capture_speed
        if a.capture_lighting is not None:settings['capture_lighting']=a.capture_lighting
    if a.motion_optimized:
        motion=settings.get('motion_optimization',{})
        if not isinstance(motion,dict):p.error('motion_optimization must be an object')
        settings['motion_optimization']={**motion,'enabled':True}
    if a.survey_pattern:settings['survey_pattern']=a.survey_pattern
    if a.resume_survey is not None:
        if a.trial not in ('high_priority','full_mission'):
            p.error('--resume-survey is only valid for high_priority/full_mission')
        settings['resume_survey_enabled']=a.resume_survey=='on'
    resume=settings.get('resume_survey_enabled',False)
    if type(resume) is not bool:p.error('resume_survey_enabled must be boolean')
    if resume and a.trial not in ('high_priority','full_mission'):
        p.error('survey resume is only available for priority/full mission trials')
    if settings.get('actuator_mode','mock')!='mock':p.error('settings must default to mock; use --real-release explicitly')
    if a.real_release and (a.mode!='flight' or settings['mode'] in NO_DROP_MODES):p.error('--real-release requires a delivery flight module')
    settings['actuator_mode']='real' if a.real_release else ('none' if settings['mode'] in NO_DROP_MODES else 'mock')
    try:validate_settings(settings)
    except ValueError as error:p.error(str(error))
    if a.check_config:
        following=dict(settings.get('following_speed_profile') or
            (dict(cruise_lead_m=min(settings['cruise_speed'],1.),precision_lead_m=.4,corridor_lead_m=.15)
             if settings['mode']=='high_speed_capture' else dict(cruise_lead_m=.5,precision_lead_m=.25,boundary_lead_m=.2,corridor_lead_m=.25)))
        if settings.get('trial_kind') in ('corridor_landing','full_mission'):
            following.update(precision_lead_m=max(.4,following['precision_lead_m']),corridor_lead_m=.4)
        result=dict(status='CONFIG_VALID',source='validated_settings',
            motion_optimization=settings.get('motion_optimization',{}).get('enabled',False),
            resume_survey=settings.get('resume_survey_enabled',False),
            generation_ready=True, speed_profile=settings.get('speed_profile','limited'),
            planning=dict(max_vel=settings['cruise_speed'],max_acc=settings['cruise_acceleration']),
            initial_distances=(dict(controller_limit_m=.4,traj_target_dist_m=.4,planner_start_max_distance_m=1.2)
                if settings.get('speed_profile')=='competition' else dict(controller_limit_m=.25,traj_target_dist_m=.25,planner_start_max_distance_m=(max(.75,min(settings['cruise_speed'],1.)+.25) if settings['mode']=='high_speed_capture' else .75))),
            following=following,
            corridor=dict(open_lead_m=.6,door_lead_m=.4) if settings.get('corridor_geometry') else None,
            drop_agl=settings['drop_agl'],high_agl=settings['high_agl'],
            landing_capture_agl=settings['landing_capture_agl'],
            landing_posctl=settings.get('landing_posctl'), settings=settings)
        if a.generate_config:
            generate(a.root,a.generate_config,settings,a.reference_fc,rig)
            preview_runtime=yaml.safe_load((a.generate_config/'runtime.yaml').read_text())
            preview_control=yaml.safe_load((a.generate_config/'control.yaml').read_text())
            preview_overrides=yaml.safe_load((a.generate_config/'overrides.yaml').read_text())
            result['initial_distances']=dict(controller_limit_m=preview_control['px4_max_distance'],traj_target_dist_m=preview_overrides['/traj_server/traj_server/target_dist'],planner_start_max_distance_m=preview_overrides['/external_planner_start_max_distance'])
            result.update(source='generated_runtime',following=preview_runtime['following_speed_profile'],corridor=preview_runtime.get('corridor_speed_schedule'))
            result['preview_reference_fc']=a.reference_fc
            result['runtime_path']=str(a.generate_config/'runtime.yaml')
        print('CONFIG_VALID; no ROS nodes started')
        print(json.dumps(result,ensure_ascii=False))
        return
    settings['mapping_profile']=mapping_profile(settings)
    run_session(a,settings,rig,startup,generate,'uav_board_trials',finish_script=Path(__file__).with_name('finish_trial.py'))
if __name__=='__main__':main()
