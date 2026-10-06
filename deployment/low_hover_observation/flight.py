#!/usr/bin/env python3
"""Low hover observations. Manual arm + fresh OFFBOARD switch; no mode services."""
import argparse
from collections import deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import yaml
from observation_core import Observation, route, validate

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def xyz_yaw(message):
    p,q=message.pose.position,message.pose.orientation
    norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if not .8<norm<1.2 or not all(math.isfinite(v) for v in (p.x,p.y,p.z,norm)):
        raise ValueError('invalid pose')
    x,y,z,w=(q.x/norm,q.y/norm,q.z/norm,q.w/norm)
    return (p.x,p.y,p.z,math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def valid_pose(message, frame, now, age, received_age):
    if message is None:return False
    try:
        xyz_yaw(message)
        stamp=message.header.stamp.to_sec()
        return (message.header.frame_id==frame and stamp>0 and
                0<=now-stamp<=age and 0<=received_age<=age)
    except (ValueError,AttributeError):return False


def valid_speed(message, frame, now, age, received_age):
    if message is None:return math.inf
    if (message.header.frame_id!=frame or message.header.stamp.to_sec()<=0
            or not 0<=now-message.header.stamp.to_sec()<=age or not 0<=received_age<=age):return math.inf
    v=message.twist.twist.linear
    speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
    return speed if math.isfinite(speed) else math.inf


def conflicts(master, caller):
    pubs,subs,services=master.getSystemState()
    blocked=[]
    for topic,nodes in pubs:
        if (topic.startswith('/mavros/setpoint_') and '/target_' not in topic
                or topic.startswith('/uav_vision/') or '/image' in topic
                or topic.startswith('/camera/') or topic.startswith('/mission/')):
            blocked += [(topic,n) for n in nodes if n!=caller]
    return blocked


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['preview','flight','localization-check'])
    parser.add_argument('--profile',choices=['hover','forward','square'],default='hover')
    parser.add_argument('--config',type=Path,default=HERE/'profiles.yaml')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    c=yaml.safe_load(args.config.read_text());validate(c,args.profile)
    if args.mode=='preview':
        print(json.dumps(dict(profile=args.profile,fc_agl=c['height_agl'],
            relative_local_z_target=c['height_agl']-c['fc_ground_clearance'],
            route_at_origin_yaw_zero=route(c,args.profile,(0,0,0,0)),
            end='hover_until_manual_landing',camera=False,yolo=False,planner=False,servo=False,
            auto_arm=False,auto_mode=False,config=c),indent=2));return
    import rospy,rosgraph,rosnode
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    sys.path.insert(0,str(ROOT/'deployment/board_trials_4x4/common/uav_board_trials/scripts'))
    from mapping_startup import PoseAgreement
    rospy.init_node('low_hover_observation',disable_signals=True)
    if rospy.get_param('/use_sim_time',False):raise RuntimeError('hardware-only entry; simulation time refused')
    topics=dict(state='/mavros/state',pose='/mavros/local_position/pose',
                ev='/mavros/vision_pose/pose',odom='/mavros/local_position/odom',
                setpoint='/mavros/setpoint_position/local',status='/low_hover_observation/status')
    topics.update(c.get('topics',{}))
    lock=threading.RLock();data={};samples=deque(maxlen=200);ev_samples=deque(maxlen=200)
    def remember(name,msg):
        with lock:
            data[name]=(msg,time.monotonic())
            if name in ('pose','ev'):
                try:
                    point=xyz_yaw(msg)
                    expected=c['frame'] if name=='pose' else c['mission_frame']
                    if msg.header.frame_id!=expected:return
                    row=point+(msg.header.stamp.to_sec(),)
                    (samples if name=='pose' else ev_samples).append(row)
                except ValueError:pass
    subs=[rospy.Subscriber(topics[name],kind,lambda m,n=name:remember(n,m),queue_size=1)
          for name,kind in [('state',State),('pose',PoseStamped),('ev',PoseStamped),('odom',Odometry)]]
    master=rosgraph.Master(rospy.get_name())
    until=time.monotonic()+8
    while 'state' not in data and time.monotonic()<until:time.sleep(.05)
    state,rx=data.get('state',(None,0))
    if state is None or not state.connected or state.armed or time.monotonic()-rx>c['state_max_age']:
        raise RuntimeError('start on ground, connected and disarmed')
    busy=conflicts(master,rospy.get_name())
    if busy:raise RuntimeError('stop previous flight/camera/vision publishers before observation: '+str(busy))
    if args.mode=='localization-check':
        names=rosnode.get_node_names()
        if any(n in names for n in ('/laserMapping','/lio_external_pose')):
            raise RuntimeError('existing LIO/EV publisher; reuse it or stop on ground, never duplicate')
        print('LOCALIZATION_START_ALLOWED: ground/disarmed, no competing app');return
    # The inherited comparison assumes the existing static identity map<->camera_init.
    # No estimated transform is learned merely to hide FC/LIO disagreement.
    agree=PoseAgreement(dict(pose_max_age=.3,pair_max_skew=.1,position_tolerance=.2,
                             yaw_tolerance_deg=5.,stable_seconds=2.))
    output=args.output or ROOT/'logs'/('low_hover_'+args.profile+'_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True,exist_ok=False)
    (output/'profile.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
    stopping=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *unused:stopping.set())
    recording_log=(output/'recorder.log').open('w')
    recorder=subprocess.Popen([sys.executable,str(HERE/'record_diagnostics.py'),'--output',str(output/'recording'),
                               '--config',str(HERE/'recording.yaml')],stdout=recording_log,stderr=subprocess.STDOUT,start_new_session=True)
    controller=None
    try:
        until=time.monotonic()+90;last_print=0.
        while not stopping.is_set() and time.monotonic()<until and not rospy.is_shutdown():
            now=rospy.Time.now().to_sec();wall=time.monotonic()
            with lock:
                state,rx=data.get('state',(None,0))
                disarmed=state is not None and state.connected and not state.armed and wall-rx<=c['state_max_age']
                ready,detail=agree.update(list(samples),list(ev_samples),now,disarmed)
                span=list(samples)
                odom,odom_rx=data.get('odom',(None,0))
                speed=valid_speed(odom,c['frame'],now,c['pose_max_age'],wall-odom_rx)
            if recorder.poll() is not None:raise RuntimeError('diagnostic recorder failed before flight; see recorder.log')
            if not disarmed:raise RuntimeError('must remain connected and disarmed through initialization')
            if wall-last_print>1:
                print('WAIT_GROUND_REFERENCE',json.dumps(detail),flush=True);last_print=wall
            recent=[v for v in span if 0<=now-v[4]<=2.]
            stable=(len(recent)>=15 and recent[-1][4]-recent[0][4]>=1.5
                    and all(max(v[i] for v in recent)-min(v[i] for v in recent)<.025 for i in range(3)))
            ready_file=output/'recording/recording_ready.json'
            recorder_ready=(ready_file.exists() and json.loads(ready_file.read_text()).get('ready') is True)
            if ready and stable and math.isfinite(speed) and recorder_ready:break
            time.sleep(.05)
        else:
            if stopping.is_set():return
            raise RuntimeError('ground FC/LIO agreement or recorder not ready; no control commands published')
        with lock:origin=xyz_yaw(data['pose'][0])
        controller=Observation(c,args.profile,origin,time.monotonic())
        metadata=dict(profile=args.profile,origin_fc_local=list(origin),ground_local_z=origin[2]-c['fc_ground_clearance'],
                      fc_ground_clearance=c['fc_ground_clearance'],planned_goals=controller.goals,
                      wall_time=time.time(),monotonic=time.monotonic(),ros_time=rospy.Time.now().to_sec(),
                      purpose='motor_low_hover_observation_only',alignment='inherited_static_identity',
                      altitude_is_estimate_not_range_measurement=True)
        (output/'reference.json').write_text(json.dumps(metadata,indent=2))
        pub=rospy.Publisher(topics['setpoint'],PoseStamped,queue_size=1)
        status_pub=rospy.Publisher(topics['status'],String,queue_size=1,latch=True)
        next_tick=time.monotonic();last_status=0.;announced=False;previous_stage=None
        print('PRESTREAM: keep disarmed for two seconds, then manually arm and switch OFFBOARD',flush=True)
        while not rospy.is_shutdown():
            wall=time.monotonic();now=rospy.Time.now().to_sec()
            with lock:snapshot=dict(data)
            state,state_rx=snapshot.get('state',(None,0));msg,pose_rx=snapshot.get('pose',(None,0))
            state_fresh=state is not None and wall-state_rx<=c['state_max_age']
            connected=bool(state_fresh and state.connected)
            armed=bool(state.armed) if state is not None else True
            mode=state.mode if state is not None else 'UNKNOWN'
            pose=None;stamp=0.;fresh=False
            if msg is not None:
                try:
                    pose=xyz_yaw(msg);stamp=msg.header.stamp.to_sec()
                    ev,ev_rx=snapshot.get('ev',(None,0))
                    fresh=(valid_pose(msg,c['frame'],now,c['pose_max_age'],wall-pose_rx)
                           and valid_pose(ev,c['mission_frame'],now,c['pose_max_age'],wall-ev_rx))
                except ValueError:pass
            odom,odom_rx=snapshot.get('odom',(None,0))
            speed=valid_speed(odom,c['frame'],now,c['pose_max_age'],wall-odom_rx)
            recorder_failed=recorder.poll() is not None
            if recorder_failed and controller.stage not in ('HOLD_FOR_PILOT','TAKEN_OVER'):
                controller.hold('recording_failed_take_over',pose if fresh else None)
            target=controller.step(wall,pose,stamp,fresh,armed,mode,connected,speed,stopping.is_set())
            if target is not None:
                command=PoseStamped();command.header.stamp=rospy.Time.now();command.header.frame_id=c['frame']
                command.pose.position.x,command.pose.position.y,command.pose.position.z=target[:3]
                command.pose.orientation.z=math.sin(target[3]/2);command.pose.orientation.w=math.cos(target[3]/2)
                pub.publish(command)
            if wall-last_status>=1 or controller.stage!=previous_stage:
                status=dict(stage=controller.stage,reason=controller.reason,profile=args.profile,index=controller.index,
                            time=now,monotonic=wall,pose=pose,pose_stamp=stamp,pose_age=now-stamp if stamp else None,
                            target=target,mode=mode,armed=armed,recorder_alive=not recorder_failed)
                status_pub.publish(String(data=json.dumps(status)))
                print(json.dumps(status),flush=True);last_status=wall;previous_stage=controller.stage
            if not announced and controller.stage=='READY' and connected and fresh and wall-controller.ready_at>=c['warmup_seconds']:
                print('READY_FOR_MANUAL_ARM_AND_OFFBOARD (pilot checks still apply)',flush=True);announced=True
            if (stopping.is_set() or controller.ever_started) and state_fresh and not armed:break
            next_tick+=1/30
            delay=next_tick-time.monotonic()
            if delay>0:time.sleep(delay)
            else:next_tick=time.monotonic()
    finally:
        # Normal exit follows fresh disarm. SIGINT while airborne only requests
        # HOLD_FOR_PILOT above; it does not tear down LIO or MAVROS.
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:recorder.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print('Recorder still finalizing; inspect its PID '+str(recorder.pid),flush=True)
        recording_log.close()
        if controller:
            (output/'ending.json').write_text(json.dumps(dict(stage=controller.stage,reason=controller.reason,
                                              index=controller.index),indent=2))
    print('OBSERVATION_CLOSED',output,flush=True)


if __name__=='__main__':main()
