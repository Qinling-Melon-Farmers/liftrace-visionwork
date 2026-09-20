#!/usr/bin/env python3
"""Local board supervisor: measures a stationary reference; never arms or starts mission."""
import argparse,json,math,os,signal,subprocess,threading,time,shutil
from pathlib import Path
from collections import deque
import numpy as np,yaml,rospy,rosnode,tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo,Image
from mavros_msgs.msg import State,ExtendedState
from trial_config import generate,validate_settings

def main():
    p=argparse.ArgumentParser();p.add_argument('trial',choices=['visual_interrupt','high_view','landing','corridor_landing']);p.add_argument('mode',choices=['preview','flight']);p.add_argument('--root',type=Path,required=True);p.add_argument('--model',type=Path);p.add_argument('--check-config',action='store_true');a=p.parse_args()
    folder={'visual_interrupt':'01_visual_interrupt','high_view':'02_high_view_revisit','landing':'03_h_landing','corridor_landing':'04_corridor_landing'}[a.trial]
    base=a.root/'deployment/board_trials_4x4';settings=yaml.safe_load((base/folder/'settings.yaml').read_text());rig=yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
    try:validate_settings(settings)
    except ValueError as error:p.error(str(error))
    if a.check_config:
        print('CONFIG_VALID; no ROS nodes started');return
    rospy.init_node('board_trial_supervisor',disable_signals=True)
    if rospy.get_param('/use_sim_time',False):raise RuntimeError('Board trials refuse /use_sim_time=true; do not run against laptop SITL')
    existing=set(rosnode.get_node_names())
    conflicts=existing.intersection({'/laserMapping','/freedom','/patrol_control','/fast_planner_node','/navigation_frame_adapter','/navigation/mission_manager','/target_detector_rknn','/navigation/planner_bridge','/release_permission_arbiter','/guarded_servo_proxy','/trial_auto_land','/board_mock_servo','/trial_recorder','/map_camera_alignment'})
    if conflicts:raise RuntimeError('Stop the old application first: '+','.join(sorted(conflicts)))
    if '/mavros' not in existing:raise RuntimeError('Start device MAVROS and driver2 first')
    model=a.model or Path(os.environ.get('UAV_VISION_RKNN_MODEL_PATH',str(a.root/'runtime_models/merged_standard_fp32.rknn')))
    if not model.is_file():raise RuntimeError('RKNN model not found; set UAV_VISION_RKNN_MODEL_PATH once or use --model')
    out=a.root/'logs'/('board_'+a.trial+'_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(out).free<2*1024**3:raise RuntimeError('Less than 2GB recording space available')
    lock=threading.RLock();samples=deque(maxlen=400);state=[None];camera=[None];extended=[None];ever_armed=[False];ever_airborne=[False];image_ref=[None];lio_ref=[None];end_reason='interrupted_or_error'
    def pose(msg):
        with lock:
            now=rospy.Time.now().to_sec();stamp=msg.header.stamp.to_sec()
            if msg.header.frame_id!='camera_init' or not 0<=now-stamp<=.3:return
            q=msg.pose.orientation;norm=sum(v*v for v in (q.x,q.y,q.z,q.w))
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            values=(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,yaw,stamp)
            if not all(math.isfinite(v) for v in values) or abs(norm-1)>.05:return
            samples.append(values)
    def vehicle(msg):
        with lock:state[0]=msg;ever_armed[0]=ever_armed[0] or msg.armed
    def extended_state(msg):
        with lock:
            extended[0]=msg
            if msg.landed_state==ExtendedState.LANDED_STATE_IN_AIR and state[0] is not None and state[0].armed:ever_airborne[0]=True
    subs=[rospy.Subscriber('/navigation/local_pose',PoseStamped,pose,queue_size=1),rospy.Subscriber('/mavros/state',State,vehicle,queue_size=1),rospy.Subscriber(settings.get('camera_info_topic','/camera/camera_info'),CameraInfo,lambda m:camera.__setitem__(0,m),queue_size=1),rospy.Subscriber('/mavros/extended_state',ExtendedState,extended_state,queue_size=1)]
    subs.append(rospy.Subscriber(settings.get('lio_pose_topic','/mavros/vision_pose/pose'),PoseStamped,lambda m:lio_ref.__setitem__(0,(m.header.stamp.to_sec(),m.header.frame_id)),queue_size=1))
    subs.append(rospy.Subscriber(settings.get('image_topic','/camera/image_raw'),Image,lambda m:image_ref.__setitem__(0,(m.header.stamp.to_sec(),m.header.frame_id)),queue_size=1,buff_size=8*1024**2))
    children=[];files=[]
    env=os.environ.copy();env['ROS_LOG_DIR']=str(out/'roslog')
    def launch(name,args):
        stream=(out/(name+'.log')).open('w');files.append(stream)
        child=subprocess.Popen(['roslaunch','uav_board_trials',name+'.launch',*args],env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child
    def interrupted(*unused):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        local=launch('localization',['alignment_mode:='+settings.get('alignment_mode','measured'),'enable_control_output:='+str(a.mode=='flight').lower(),'body_to_imu_xyz:='+' '.join(map(str,rig['body_to_imu_xyz'])),'imu_to_camera_z:='+str(rig['imu_to_camera_xyz'][2]),'camera_quat_xyzw:='+' '.join(map(str,rig['camera_quat_xyzw']))])
        until=time.monotonic()+90;reference=None;last_wait_log=0.
        while time.monotonic()<until:
            if local.poll() is not None:raise RuntimeError('Localization launch exited; inspect localization.log')
            with lock:
                s=state[0];c=camera[0];v=np.array(samples)
                im=image_ref[0];valid=(s is not None and s.connected and not s.armed and c is not None and c.width>0 and c.K[0]>0 and c.K[4]>0 and len(v)>=30 and im is not None and 0<=rospy.Time.now().to_sec()-im[0]<=1.)
                lio=lio_ref[0];lio_fresh=lio is not None and lio[1]=='camera_init' and 0<=rospy.Time.now().to_sec()-lio[0]<=.3
                valid=valid and lio_fresh
                if time.monotonic()-last_wait_log>5:
                    last_wait_log=time.monotonic();print('INITIALIZING',dict(pose_samples=len(v),camera_info=c is not None,image_seen=im is not None,lio_fresh=lio_fresh,armed=s.armed if s else None),flush=True)
                if valid:
                    if np.max(np.ptp(v[:,:3],axis=0))>.025 or np.ptp(v[:,3])>.04 or abs(float(np.mean(v[:,3])))>.10:valid=False
                    if v[-1,4]-v[0,4]<1.5 or rospy.Time.now().to_sec()-v[-1,4]>.3:valid=False
                    if c.header.frame_id!='downward_camera_optical_frame':raise RuntimeError('CameraInfo frame does not match inherited camera optical TF')
                    if im[1]!=c.header.frame_id:raise RuntimeError('Image and CameraInfo frames differ')
                if valid:reference=generate(a.root,out,settings,np.median(v[:,:3],axis=0),rig);break
            time.sleep(.1)
        if reference is None:raise RuntimeError('No stationary disarmed camera_init reference. Inspect map<->camera_init conversion, initial heading and camera; no manual Z guess was applied')
        subs[-1].unregister()
        (out/'camera_info.json').write_text(json.dumps(dict(width=c.width,height=c.height,K=list(c.K),D=list(c.D),frame=c.header.frame_id),indent=2))
        args=['enable_control_output:='+str(a.mode=='flight').lower(),f'mode:={settings["mode"]}',f'model_path:={model}',f'generated_dir:={out}',f'ground_z:={reference["ground_z"]}',f'low_z:={reference["low_z"]}']
        args+=['cruise_speed:='+str(settings['cruise_speed']),'cruise_acceleration:='+str(settings['cruise_acceleration'])]
        args+=['image_topic:='+settings.get('image_topic','/camera/image_raw'),'camera_info_topic:='+settings.get('camera_info_topic','/camera/camera_info')]
        app=launch('application',args)
        detections_seen=[False];control_rx=[-1e9]
        control_ready_sub=rospy.Subscriber('/navigation/setpoint_mission',PoseStamped,lambda msg:control_rx.__setitem__(0,time.monotonic()),queue_size=1)
        model_ready_sub=rospy.Subscriber('/uav_vision/detections',rospy.AnyMsg,lambda msg:detections_seen.__setitem__(0,True),queue_size=1)
        ready_until=time.monotonic()+60
        while not (detections_seen[0] and (a.mode=='preview' or time.monotonic()-control_rx[0]<.5)) and time.monotonic()<ready_until:
            if any(child.poll() is not None for child in children):raise RuntimeError('Application failed before model readiness')
            time.sleep(.1)
        model_ready_sub.unregister();control_ready_sub.unregister()
        if not detections_seen[0]:raise RuntimeError('RKNN output did not become ready; inspect application.log')
        if a.mode=='flight' and time.monotonic()-control_rx[0]>=.5:raise RuntimeError('Controller setpoints did not become live; inspect patrol_control startup errors')
        print('READY:',out,flush=True)
        print('Ground/reference and all local-Z limits generated automatically. No arming or mission start was sent.',flush=True)
        if a.mode=='flight':print('After local inspection, operator chooses flight mode/arming and calls: rosservice call /navigation/start_mission "{}"',flush=True)
        on_ground_since=None
        while not rospy.is_shutdown():
            if any(c.poll() is not None for c in children):raise RuntimeError('A launch exited; inspect logs')
            s=state[0];e=extended[0]
            done=ever_airborne[0] and s is not None and not s.armed and e is not None and e.landed_state==ExtendedState.LANDED_STATE_ON_GROUND
            if done:
                on_ground_since=on_ground_since or time.monotonic()
                if time.monotonic()-on_ground_since>=3:end_reason='landed_after_flight';break
            else:on_ground_since=None
            time.sleep(.2)
    except KeyboardInterrupt:pass
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=25)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        for stream in files:stream.close()
        s=state[0];e=extended[0]
        (out/'supervisor_result.json').write_text(json.dumps(dict(end_reason=end_reason,ever_armed=ever_armed[0],ever_airborne=ever_airborne[0],armed=s.armed if s else None,mode=s.mode if s else None,landed_state=e.landed_state if e else None,trial=a.trial),indent=2))
        print('Trial application stopped; device MAVROS/driver2 left running. Logs:',out,flush=True)
        subprocess.run([os.environ.get('BOARD_PYTHON','/usr/bin/python3'),str(Path(__file__).with_name('finish_recording.py')),str(out)],check=False)
if __name__=='__main__':main()
