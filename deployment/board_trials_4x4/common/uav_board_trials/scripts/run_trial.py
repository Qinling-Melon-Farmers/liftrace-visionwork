#!/usr/bin/env python3
"""Board supervisor: never arms; optionally starts the mission after manual arm/hover."""
import argparse,json,math,os,signal,subprocess,threading,time,shutil
from pathlib import Path
from collections import deque
import numpy as np,yaml
from trial_config import generate,validate_settings,apply_site_profile,TRIAL_FOLDERS,NO_DROP_MODES,mapping_profile
from trial_bag import TrialBag
from mapping_startup import PoseAgreement,MapWarmup,VisionReadiness,startup_transport_pending

def main():
    p=argparse.ArgumentParser();p.add_argument('trial',choices=sorted(TRIAL_FOLDERS));p.add_argument('mode',choices=['preview','flight']);p.add_argument('--root',type=Path,required=True);p.add_argument('--model',type=Path);p.add_argument('--metadata',type=Path);p.add_argument('--check-config',action='store_true');p.add_argument('--site-config',type=Path);p.add_argument('--real-release',action='store_true');p.add_argument('--mapping-startup-config',type=Path);p.add_argument('--capture-speed',type=float,choices=(.5,1.,1.2));p.add_argument('--capture-lighting',choices=('normal','dim','unspecified'));p.add_argument('--motion-optimized',action='store_true');p.add_argument('--survey-pattern',choices=('rectangle','snake2','snake3'));a=p.parse_args()
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
    if a.motion_optimized:
        motion=settings.get('motion_optimization',{})
        if not isinstance(motion,dict):p.error('motion_optimization must be an object')
        settings['motion_optimization']={**motion,'enabled':True}
    if a.survey_pattern:settings['survey_pattern']=a.survey_pattern
    if settings.get('actuator_mode','mock')!='mock':p.error('settings must default to mock; use --real-release explicitly')
    if a.real_release and (a.mode!='flight' or settings['mode'] in NO_DROP_MODES):p.error('--real-release requires a delivery flight module')
    settings['actuator_mode']='real' if a.real_release else ('none' if settings['mode'] in NO_DROP_MODES else 'mock')
    try:validate_settings(settings)
    except ValueError as error:p.error(str(error))
    if a.check_config:
        print('CONFIG_VALID; no ROS nodes started');return
    import rospy,rosnode,tf2_ros
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo,Image,CompressedImage,PointCloud2
    from mavros_msgs.msg import State,ExtendedState
    from std_msgs.msg import String
    from uav_vision.msg import TargetDetectionArray,TargetCandidateArray
    rospy.init_node('board_trial_supervisor',disable_signals=True)
    if rospy.get_param('/use_sim_time',False):raise RuntimeError('Board trials refuse /use_sim_time=true; do not run against laptop SITL')
    existing=set(rosnode.get_node_names())
    conflicts=existing.intersection({'/laserMapping','/freedom','/patrol_control','/fast_planner_node','/navigation_frame_adapter','/navigation/mission_manager','/target_detector_rknn','/target_detector','/detection_fusion','/target_refiner','/target_map_projector','/target_memory','/cross_detector','/circle_detector','/landing_detector','/drop_aligner','/navigation/planner_bridge','/release_permission_arbiter','/guarded_servo_proxy','/trial_auto_land','/board_mock_servo','/trial_recorder','/trial_journal','/map_camera_alignment','/board_trial_bag'})
    if conflicts:raise RuntimeError('Stop the old application first: '+','.join(sorted(conflicts)))
    if '/mavros' not in existing:raise RuntimeError('Start device MAVROS and driver2 first')
    model=a.model or Path(os.environ.get('UAV_VISION_RKNN_MODEL_PATH',str(a.root/'runtime_models/flight_5cls_20260928_fp16.rknn')))
    if not model.is_file():raise RuntimeError('RKNN model not found; set UAV_VISION_RKNN_MODEL_PATH once or use --model')
    metadata=a.metadata or a.root/'vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml'
    if not metadata.is_file():raise RuntimeError('Model metadata not found; use --metadata with matching weights')
    out=a.root/'logs'/('board_'+a.trial+'_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(out).free<2*1024**3:raise RuntimeError('Less than 2GB recording space available')
    lock=threading.RLock();samples=deque(maxlen=400);lio_samples=deque(maxlen=400);state_rx=[-1e9];state=[None];camera=[None];extended=[None];ever_armed=[False];ever_airborne=[False];image_ref=[None];compressed_rx=[-1e9];end_reason='interrupted_or_error'
    def pose(msg, destination=samples):
        with lock:
            now=rospy.Time.now().to_sec();stamp=msg.header.stamp.to_sec()
            if msg.header.frame_id!=startup['frame'] or not 0<=now-stamp<=float(startup['pose_max_age']):return
            q=msg.pose.orientation;norm=sum(v*v for v in (q.x,q.y,q.z,q.w))
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            values=(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,yaw,stamp)
            if not all(math.isfinite(v) for v in values) or abs(norm-1)>.05:return
            destination.append(values)
    def vehicle(msg):
        with lock:state_rx[0]=time.monotonic();state[0]=msg;ever_armed[0]=ever_armed[0] or msg.armed
    def extended_state(msg):
        with lock:
            extended[0]=msg
            if msg.landed_state==ExtendedState.LANDED_STATE_IN_AIR and state[0] is not None and state[0].armed:ever_airborne[0]=True
    subs=[rospy.Subscriber(startup['pose_topic'],PoseStamped,pose,queue_size=1),rospy.Subscriber('/mavros/state',State,vehicle,queue_size=1),rospy.Subscriber(settings.get('camera_info_topic','/camera/camera_info'),CameraInfo,lambda m:camera.__setitem__(0,m),queue_size=1),rospy.Subscriber('/mavros/extended_state',ExtendedState,extended_state,queue_size=1)]
    subs.append(rospy.Subscriber(settings.get('compressed_image_topic',settings.get('image_topic','/camera/image_raw')+'/compressed'),CompressedImage,lambda m:compressed_rx.__setitem__(0,time.monotonic()),queue_size=1))
    subs.append(rospy.Subscriber(startup['lio_pose_topic'],PoseStamped,lambda m:pose(m,lio_samples),queue_size=1))
    subs.append(rospy.Subscriber(settings.get('image_topic','/camera/image_raw'),Image,lambda m:image_ref.__setitem__(0,(m.header.stamp.to_sec(),m.header.frame_id)),queue_size=1,buff_size=8*1024**2))
    manifest_path=a.root/'DEPLOYMENT_MANIFEST.json'
    run_metadata=dict(trial=a.trial,mode=a.mode,settings=settings,model_path=str(model),
        model_bytes=model.stat().st_size,metadata_path=str(metadata),
        model_metadata=yaml.safe_load(metadata.read_text()),known_rig=rig,mapping_startup=startup,
        deployment=json.loads(manifest_path.read_text()) if manifest_path.exists() else {})
    metadata_pub=rospy.Publisher('/board_trials/run_metadata',String,queue_size=1,latch=True)
    def record_metadata(reference=None):
        if reference is not None:run_metadata['ground_reference']=reference
        encoded=json.dumps(run_metadata)
        (out/'run_metadata.json').write_text(encoded)
        metadata_pub.publish(String(data=encoded))
    children=[];files=[];bag=None
    env=os.environ.copy();env['ROS_LOG_DIR']=str(out/'roslog')
    def launch(name,args):
        stream=(out/(name+'.log')).open('w');files.append(stream)
        child=subprocess.Popen(['roslaunch','uav_board_trials',name+'.launch',*args],env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child
    def interrupted(*unused):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        bag=TrialBag(out,settings,env);bag.start();record_metadata()
        local=launch('localization',['start_mapping:=false','map_frame:='+startup['frame'],'mapping_profile:='+mapping_profile(settings),'alignment_mode:='+settings.get('alignment_mode','measured'),'enable_control_output:='+str(a.mode=='flight').lower(),'body_to_imu_xyz:='+' '.join(map(str,rig['body_to_imu_xyz'])),'imu_to_camera_z:='+str(rig['imu_to_camera_xyz'][2]),'camera_quat_xyzw:='+' '.join(map(str,rig['camera_quat_xyzw']))])
        until=time.monotonic()+float(settings.get("initialization_timeout",90));reference=None;last_wait_log=0.
        while time.monotonic()<until:
            if local.poll() is not None:raise RuntimeError('Localization launch exited; inspect localization.log')
            with lock:
                s=state[0];c=camera[0];now=rospy.Time.now().to_sec()
                v=np.array([v for v in samples if 0<=now-v[4]<=2.0])
                im=image_ref[0];valid=(s is not None and s.connected and not s.armed and c is not None and c.width>0 and c.K[0]>0 and c.K[4]>0 and len(v)>=30 and im is not None and 0<=rospy.Time.now().to_sec()-im[0]<=1.)
                disarmed=s is not None and s.connected and not s.armed and time.monotonic()-state_rx[0]<=float(startup['state_max_age'])
                aligned,alignment=agreement.update(list(samples),list(lio_samples),now,disarmed)
                valid=valid and aligned and time.monotonic()-compressed_rx[0]<=1.
                if time.monotonic()-last_wait_log>5:
                    last_wait_log=time.monotonic();print('INITIALIZING',dict(pose_samples=len(v),camera_info=c is not None,image_seen=im is not None,compressed_fresh=time.monotonic()-compressed_rx[0]<=1.,mapping_alignment=alignment,armed=s.armed if s else None,yaw_deg=float(np.degrees(np.mean(v[:,3]))) if len(v) else None,xyz_span=np.ptp(v[:,:3],axis=0).tolist() if len(v) else None),flush=True)
                if valid:
                    if np.max(np.ptp(v[:,:3],axis=0))>.025 or np.ptp(v[:,3])>.04 or abs(float(np.mean(v[:,3])))>.10:valid=False
                    if v[-1,4]-v[0,4]<1.5 or rospy.Time.now().to_sec()-v[-1,4]>.3:valid=False
                    if c.header.frame_id!='downward_camera_optical_frame':raise RuntimeError('CameraInfo frame does not match inherited camera optical TF')
                    if im[1]!=c.header.frame_id:raise RuntimeError('Image and CameraInfo frames differ')
                if valid:reference=generate(a.root,out,settings,np.median(v[:,:3],axis=0),rig);break
            time.sleep(.1)
        if reference is None:raise RuntimeError('No stationary disarmed camera_init reference. Inspect map<->camera_init conversion, initial heading, camera and compressed stream; no manual Z guess was applied')
        # No FreeDOM process existed during localization convergence. Start a
        # clean map and require live observations before any application/planner.
        mapping_start=rospy.Time.now().to_sec();warmup=MapWarmup(mapping_start,startup)
        def cloud(msg):
            with lock:
                warmup.observe(msg.header.stamp.to_sec(),rospy.Time.now().to_sec(),
                    msg.header.frame_id==startup['frame'] and msg.width*msg.height>0 and bool(msg.data))
        cloud_sub=rospy.Subscriber(startup['cloud_topic'],PointCloud2,cloud,queue_size=1)
        mapper=launch('mapping',['mapping_profile:='+mapping_profile(settings),'map_frame:='+startup['frame']])
        def check_mapping_alignment(allow_pending=False):
            with lock:
                s=state[0]
                ready,detail=agreement.update(list(samples),list(lio_samples),rospy.Time.now().to_sec(),
                    s is not None and s.connected and not s.armed and time.monotonic()-state_rx[0]<=float(startup['state_max_age']))
            if not ready:
                if allow_pending and startup_transport_pending(detail):
                    rospy.logwarn_throttle(2.,'Startup waiting for fresh stable pose: %s',json.dumps(detail))
                    return False
                raise RuntimeError('Mapping initialization lost agreement; stop and rerun from empty map: '+json.dumps(detail))
            return True
        until=time.monotonic()+float(startup['map_timeout'])
        while time.monotonic()<until:
            if any(child.poll() is not None for child in children):raise RuntimeError('Localization/mapping launch exited before map readiness')
            aligned=check_mapping_alignment(allow_pending=True)
            if aligned and warmup.ready(rospy.Time.now().to_sec()):break
            time.sleep(.1)
        else:raise RuntimeError('No fresh nonempty FreeDOM map after localization convergence; inspect mapping.log')
        run_metadata['mapping_startup_result']=dict(mapping_started=mapping_start,
            map_ready=rospy.Time.now().to_sec(),first_cloud=warmup.first,last_cloud=warmup.last,
            distinct_clouds=warmup.count,alignment=alignment)
        print('MAPPING_READY',run_metadata['mapping_startup_result'],flush=True)
        record_metadata(reference)
        subs[-1].unregister()
        (out/'camera_info.json').write_text(json.dumps(dict(width=c.width,height=c.height,K=list(c.K),D=list(c.D),frame=c.header.frame_id),indent=2))
        args=['enable_control_output:='+str(a.mode=='flight').lower(),f'mode:={settings["mode"]}',f'model_path:={model}',f'metadata_path:={metadata}',f'generated_dir:={out}',f'ground_z:={reference["ground_z"]}',f'low_z:={reference["low_z"]}']
        args+=['cruise_speed:='+str(settings['cruise_speed']),'cruise_acceleration:='+str(settings['cruise_acceleration'])]
        args+=['image_topic:='+settings.get('image_topic','/camera/image_raw'),'camera_info_topic:='+settings.get('camera_info_topic','/camera/camera_info')]
        args+=['actuator_mode:='+settings['actuator_mode'],'raw_servo_service:='+settings.get('raw_servo_service','/legacy/Servo_raw')]
        if a.real_release:
            rospy.wait_for_service(settings['raw_servo_service'],timeout=10.)
            import rosservice
            if rosservice.get_service_type(settings['raw_servo_service'])!='patrol_control/Servo':raise RuntimeError('Unexpected hardware Servo service type')
        args+=['terminal_hover_enabled:='+str('terminal_hover_agl' in settings).lower(),'max_command_z:='+str(reference['ground_z']+settings.get('max_agl',2.9))]
        check_mapping_alignment()
        # Subscribe before launch; do not mistake a latched empty array for a live chain.
        def visual(topic,msg):
            with lock:vision_ready.observe(topic,msg.header.stamp.to_sec(),rospy.Time.now().to_sec())
        vision_subs=[rospy.Subscriber(topic,TargetDetectionArray,lambda msg,t=topic:visual(t,msg),queue_size=1) for topic in startup['vision_detection_topics']]
        vision_subs.append(rospy.Subscriber(startup['vision_targets_topic'],TargetCandidateArray,lambda msg:visual(startup['vision_targets_topic'],msg),queue_size=1))
        app=launch('application',args)
        control_rx=[-1e9]
        control_ready_sub=rospy.Subscriber('/navigation/setpoint_mission',PoseStamped,lambda msg:control_rx.__setitem__(0,time.monotonic()),queue_size=1)
        ready_until=time.monotonic()+60
        while time.monotonic()<ready_until:
            if any(child.poll() is not None for child in children):raise RuntimeError('Application failed before model readiness')
            aligned=check_mapping_alignment(allow_pending=True)
            if (aligned and not vision_ready.missing(rospy.Time.now().to_sec())
                    and (a.mode=='preview' or time.monotonic()-control_rx[0]<.5)
                    and warmup.ready(rospy.Time.now().to_sec())):break
            time.sleep(.1)
        # Keep readiness subscriptions alive: unregister may block long enough
        # to make the already-live control timestamp stale before its check.
        missing=vision_ready.missing(rospy.Time.now().to_sec())
        if missing:raise RuntimeError('Vision chain not live; inspect application.log: '+','.join(missing))
        if a.mode=='flight' and time.monotonic()-control_rx[0]>=.5:raise RuntimeError('Controller setpoints did not become live; inspect patrol_control startup errors')
        check_mapping_alignment()
        if not warmup.ready(rospy.Time.now().to_sec()):raise RuntimeError('FreeDOM map stale before READY')
        print('READY:',out,flush=True)
        if a.mode=='flight' and settings.get('auto_start_after_arm',False):print('AUTO_SEQUENCE: operator arms AND selects OFFBOARD -> stable takeoff hover -> mission. No automatic arming or mode request.',flush=True)
        print('Ground/reference and all local-Z limits generated automatically. No arming or mission start was sent.',flush=True)
        if a.mode=='flight' and not settings.get('auto_start_after_arm',False):print('After local inspection, operator chooses flight mode/arming and calls: rosservice call /navigation/start_mission "{}"',flush=True)
        mission_rx=[{}]
        def mission_status(msg):
            try:mission_rx[0]=json.loads(msg.data)
            except (ValueError,TypeError):pass
        mission_sub=rospy.Subscriber('/navigation/mission_status',String,mission_status,queue_size=1)
        on_ground_since=None;last_status_log=0.
        auto_enabled=a.mode=='flight' and settings.get('auto_start_after_arm',False)
        start_attempted=False;auto_cancelled=False;offboard_seen=False;hover_since=None
        while not rospy.is_shutdown():
            if any(c.poll() is not None for c in children):raise RuntimeError('A launch exited; inspect logs')
            bag.check()
            s=state[0];e=extended[0]
            if (auto_enabled and s is not None and s.connected and s.armed and not auto_cancelled
                    and 0<=time.monotonic()-state_rx[0]<float(startup['state_max_age'])):
                if offboard_seen and s.mode!='OFFBOARD':
                    auto_cancelled=True
                    print('AUTO_SEQUENCE_CANCELLED_PILOT_MODE_CHANGE',flush=True)
                elif s.mode=='OFFBOARD':
                    offboard_seen=True
                    with lock:
                        now=rospy.Time.now().to_sec()
                        recent=[v for v in samples if 0<=now-v[4]<=1.]
                    ready=(len(recent)>=10 and now-recent[-1][4]<.3
                           and abs(recent[-1][2]-reference['takeoff_z'])<.15
                           and np.max(np.ptp(np.array(recent)[:,:3],axis=0))<.12)
                    if ready:
                        hover_since=hover_since or time.monotonic()
                    else:hover_since=None
                    if not start_attempted and hover_since is not None and time.monotonic()-hover_since>=1.:
                        start_attempted=True
                        from std_srvs.srv import Trigger
                        try:
                            response=rospy.ServiceProxy('/navigation/start_mission',Trigger)()
                            print('AUTO_MISSION_START',response.success,response.message,flush=True)
                        except rospy.ServiceException as error:
                            print('AUTO_MISSION_START_FAILED_NO_RETRY',str(error),flush=True)
                else:hover_since=None
            else:hover_since=None
            if time.monotonic()-last_status_log>=5.:
                last_status_log=time.monotonic();status=mission_rx[0]
                print('FLIGHT_STATUS',dict(mode=s.mode if s else None,armed=s.armed if s else None,
                    phase=status.get('phase','UNKNOWN'),reason=status.get('reason',status.get('last_reason',''))),flush=True)
                if a.mode=='flight' and not auto_enabled and status.get('phase')=='IDLE':
                    print('WAITING_FOR_MANUAL_MISSION_START: flight READY is not route START.',flush=True)
            done=ever_airborne[0] and s is not None and not s.armed and e is not None and e.landed_state==ExtendedState.LANDED_STATE_ON_GROUND
            if done:
                on_ground_since=on_ground_since or time.monotonic()
                if time.monotonic()-on_ground_since>=3:end_reason='landed_after_flight';break
            else:on_ground_since=None
            time.sleep(.2)
    except KeyboardInterrupt:pass
    finally:
        try:
            for child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=25)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        finally:
            if bag is not None:bag.close()
            for stream in files:stream.close()
        s=state[0];e=extended[0]
        (out/'supervisor_result.json').write_text(json.dumps(dict(end_reason=end_reason,ever_armed=ever_armed[0],ever_airborne=ever_airborne[0],armed=s.armed if s else None,mode=s.mode if s else None,landed_state=e.landed_state if e else None,trial=a.trial,actuator_mode=settings['actuator_mode']),indent=2))
        print('Trial application stopped; device MAVROS/driver2 left running. Logs:',out,flush=True)
        subprocess.run([os.environ.get('BOARD_PYTHON','/usr/bin/python3'),str(Path(__file__).with_name('finish_trial.py')),str(out)],check=False)
if __name__=='__main__':main()
