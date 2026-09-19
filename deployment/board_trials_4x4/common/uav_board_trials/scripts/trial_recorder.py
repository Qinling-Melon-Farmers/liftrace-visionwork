#!/usr/bin/env python3
"""Bounded low-rate raw + annotated camera recording and full-chain state journal."""
from pathlib import Path
from collections import deque
import csv,json,threading,time
import shutil
import cv2,numpy as np,rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Bool
from uav_vision.msg import TargetDetectionArray,TargetCandidateArray,DropOffset,ReleaseEvidence

def image_to_bgr(msg):
    channels={'bgr8':3,'rgb8':3,'mono8':1,'bgra8':4,'rgba8':4}.get(msg.encoding)
    if channels is None or msg.width<=0 or msg.height<=0 or msg.step<msg.width*channels or len(msg.data)<msg.height*msg.step:raise ValueError('Unsupported or malformed RGB camera image')
    array=np.ndarray((msg.height,msg.width,channels),dtype=np.uint8,buffer=bytes(msg.data),strides=(msg.step,channels,1))
    if msg.encoding=='bgr8':return array
    code={'rgb8':cv2.COLOR_RGB2BGR,'mono8':cv2.COLOR_GRAY2BGR,'bgra8':cv2.COLOR_BGRA2BGR,'rgba8':cv2.COLOR_RGBA2BGR}[msg.encoding]
    return cv2.cvtColor(array,code)

class Recorder:
    def __init__(self):
        self.out=Path(rospy.get_param('~directory'));self.out.mkdir(parents=True,exist_ok=True)
        self.fps=float(rospy.get_param('~fps',5));self.width=int(rospy.get_param('~width',640));self.started=time.monotonic();self.max_seconds=float(rospy.get_param('~max_seconds',900));self.lock=threading.RLock()
        if not 1<=self.fps<=10 or not 160<=self.width<=1280 or self.width%2:raise ValueError('Recording rate/size outside board budget')
        self.images=deque(maxlen=16);self.yolo=deque(maxlen=20);self.mapped=deque(maxlen=20);self.state={};self.last={};self.writers=[];self.frames=0;self.matched=0
        self.closed=False;self.events=(self.out/'vision_events.jsonl').open('w');self.csvfile=(self.out/'camera_frames.csv').open('w');self.csv=csv.writer(self.csvfile);self.csv.writerow(['frame','record_ros_sec','image_ros_sec','image_age_s','yolo_dt_s','mapped_dt_s'])
        self.posefile=(self.out/'navigation_pose.csv').open('w');self.posecsv=csv.writer(self.posefile);self.posecsv.writerow(['t','x','y','z','qx','qy','qz','qw','frame'])
        cv2.setNumThreads(1)
        self.subs=[rospy.Subscriber(rospy.get_param('~image_topic','/camera/image_raw'),Image,self.image,queue_size=1,buff_size=8*1024**2),
            rospy.Subscriber(rospy.get_param('~yolo_topic','/uav_vision/detections'),TargetDetectionArray,lambda m:self.detections('yolo',m),queue_size=1),
            rospy.Subscriber(rospy.get_param('~mapped_topic','/uav_vision/detections_mapped'),TargetDetectionArray,lambda m:self.detections('mapped',m),queue_size=1),
            rospy.Subscriber('/uav_vision/targets',TargetCandidateArray,self.targets,queue_size=1),
            rospy.Subscriber('/uav_vision/drop_offset',DropOffset,self.offset,queue_size=1),
            rospy.Subscriber('/uav_vision/release_evidence',ReleaseEvidence,self.evidence,queue_size=1),
            rospy.Subscriber('/mission/release_permission_active',Bool,self.permission,queue_size=1),
            rospy.Subscriber('/navigation/local_pose',PoseStamped,self.pose,queue_size=1)]
        for key,topic in [('mission','/navigation/mission_status'),('high','/uav_high_view/probe_status'),('mock','/board_trials/mock_release'),('align','/uav_vision/align_mode'),('land_handoff','/board_trials/auto_land_status')]:
            self.subs.append(rospy.Subscriber(topic,String,lambda m,k=key:self.text(k,m),queue_size=1))
        self.timer=rospy.Timer(rospy.Duration(1/self.fps),self.frame);rospy.on_shutdown(self.close)
    def log(self,key,data,period=.2):
        with self.lock:
            if self.closed:return
            now=rospy.Time.now().to_sec()
            if now-self.last.get(key,-1e9)<period:return
            self.last[key]=now;self.events.write(json.dumps(dict(t=now,kind=key,data=data))+'\n');self.events.flush()
    def image(self,msg):
        with self.lock:
            if not self.closed:self.images.append(msg)
    def text(self,key,msg):
        try:value=json.loads(msg.data)
        except ValueError:value=msg.data
        with self.lock:self.state[key]=value
        self.log(key,value,0 if key=='mock' else .2)
    def detections(self,key,msg):
        with self.lock:getattr(self,key).append(msg)
        self.log(key,[dict(class_name=d.class_name,confidence=d.class_confidence,geometry=d.geometry_confidence,refined=d.center_refined,association=d.association_valid,reject=d.reject_reason,map_valid=d.map_valid,xy=[d.map_point.x,d.map_point.y],stamp=d.header.stamp.to_sec()) for d in msg.detections])
    def targets(self,msg):
        rows=[dict(id=t.id,class_name=t.class_name,map_valid=t.map_valid,xy=[t.map_point.x,t.map_point.y]) for t in msg.targets]
        with self.lock:self.state['targets']=rows
        self.log('memory',rows)
    def offset(self,msg):
        value=dict(dx=float(msg.dx_px),dy=float(msg.dy_px))
        with self.lock:self.state['offset']=value
        self.log('offset',value)
    def evidence(self,msg):
        value={k:getattr(msg,k,None) for k in ('aligned','evidence_valid','stable_frames','target_class','observation_fresh')}
        with self.lock:self.state['evidence']=value
        self.log('evidence',value)
    def permission(self,msg):
        with self.lock:
            changed=self.state.get('permit')!=msg.data;self.state['permit']=msg.data
        if changed:self.log('permission_active',bool(msg.data),0)
    def pose(self,msg):
        now=rospy.Time.now().to_sec()
        with self.lock:
            if self.closed:return
            if now-self.last.get('pose',-1e9)<.2:return
            self.last['pose']=now;p=msg.pose.position;q=msg.pose.orientation
            self.posecsv.writerow([msg.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w,msg.header.frame_id]);self.posefile.flush()
    def frame(self,event):
        with self.lock:
            if self.closed or not self.images:return
            if time.monotonic()-self.started>self.max_seconds or shutil.disk_usage(self.out).free<512*1024**2:
                rospy.logerr('Camera recording budget reached; closing video cleanly');self.close();return
            now=rospy.Time.now().to_sec();msg=min(self.images,key=lambda m:abs(m.header.stamp.to_sec()-(now-.25)));stamp=msg.header.stamp.to_sec()
            states=dict(self.state);det=[]
            for name in ('yolo','mapped'):
                buf=getattr(self,name);nearest=min(buf,key=lambda m:abs(m.header.stamp.to_sec()-stamp)) if buf else None
                dt=nearest.header.stamp.to_sec()-stamp if nearest else None
                det.append((nearest if dt is not None and abs(dt)<=.03 else None,dt))
            try:image=image_to_bgr(msg)
            except Exception as e:rospy.logwarn_throttle(5,'Camera recording rejected: %s',e);return
            ratio=self.width/image.shape[1];height=int(round(image.shape[0]*ratio/2)*2);raw=cv2.resize(image,(self.width,height));annotated=raw.copy()
            for (array,dt),color in zip(det,[(0,170,255),(70,230,70)]):
                if array is None:continue
                for item in array.detections:
                    b=item.roi;x1=int(b.x_offset*ratio);y1=int(b.y_offset*ratio);x2=int((b.x_offset+b.width)*ratio);y2=int((b.y_offset+b.height)*ratio)
                    cv2.rectangle(annotated,(x1,y1),(x2,y2),color,1)
                    label=f'{item.class_name} {item.class_confidence:.2f}'
                    if color[1]==230:label+=f' G{item.geometry_confidence:.2f} map={int(item.map_valid)}'
                    cv2.putText(annotated,label,(x1,max(12,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.38,color,1,cv2.LINE_AA)
                    if item.center_refined and np.isfinite([item.center_px.x,item.center_px.y]).all():cv2.drawMarker(annotated,(int(item.center_px.x*ratio),int(item.center_px.y*ratio)),color,cv2.MARKER_CROSS,9,1)
            mission=states.get('mission',{});high=states.get('high',{});offset=states.get('offset',{});evidence=states.get('evidence',{})
            banner=np.zeros((84,self.width,3),np.uint8)
            lines=[f'MOCK DROP ONLY | ROS {now:.2f} | image age {now-stamp:.2f}s',f'phase {mission.get("phase","WAIT")} / {high.get("stage","")} | memory {high.get("trial_memory_count",len(states.get("targets",[])))} | drops {mission.get("committed_slots",0)}',f'align {states.get("align","disabled")} | aligned={evidence.get("aligned","?")} valid={evidence.get("evidence_valid","?")} permit={states.get("permit","?")}',f'dx={offset.get("dx","?")} dy={offset.get("dy","?")} | {"STALE IMAGE" if now-stamp>.6 else "boxes matched <=30ms"}']
            for i,line in enumerate(lines):cv2.putText(banner,line[:105],(7,17+20*i),cv2.FONT_HERSHEY_SIMPLEX,.38,(220,220,220),1,cv2.LINE_AA)
            full=np.vstack((annotated,banner))
            if not self.writers:
                for name,size in [('camera_raw',(self.width,height)),('camera_annotated',(self.width,height+84))]:
                    writer=cv2.VideoWriter(str(self.out/(name+'.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),self.fps,size)
                    if not writer.isOpened():raise RuntimeError('Cannot open camera recorder')
                    self.writers.append(writer)
            self.writers[0].write(raw);self.writers[1].write(full);self.csv.writerow([self.frames,now,stamp,now-stamp,det[0][1],det[1][1]]);self.csvfile.flush();self.frames+=1;self.matched+=int(det[0][0] is not None)
    def close(self):
        with self.lock:
            if self.closed:return
            self.closed=True
            for writer in self.writers:writer.release()
            for f in (self.events,self.csvfile,self.posefile):
                if not f.closed:f.close()
            (self.out/'recording.json').write_text(json.dumps(dict(frames=self.frames,fps=self.fps,yolo_matched_frames=self.matched,boxed_frame_tolerance_s=.03,images_buffered_max=16,width=self.width,clock='hardware ROS time; see camera_frames.csv'),indent=2))
if __name__=='__main__':rospy.init_node('trial_recorder');Recorder();rospy.spin()
