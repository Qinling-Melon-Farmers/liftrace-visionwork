#!/usr/bin/env python3
"""Lightweight state journal; camera pixels are recorded only by rosbag."""
from pathlib import Path
import csv,json,threading
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Bool
from uav_vision.msg import TargetDetectionArray,TargetCandidateArray,DropOffset,ReleaseEvidence

class Journal:
    def __init__(self):
        self.out=Path(rospy.get_param('~directory'));self.out.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock();self.state={};self.last={};self.closed=False
        self.events=(self.out/'vision_events.jsonl').open('w')
        self.posefile=(self.out/'navigation_pose.csv').open('w');self.posecsv=csv.writer(self.posefile)
        self.posecsv.writerow(['t','x','y','z','qx','qy','qz','qw','frame'])
        self.subs=[rospy.Subscriber(rospy.get_param('~yolo_topic','/uav_vision/detections'),TargetDetectionArray,lambda m:self.detections('yolo',m),queue_size=1),
            rospy.Subscriber(rospy.get_param('~mapped_topic','/uav_vision/detections_mapped'),TargetDetectionArray,lambda m:self.detections('mapped',m),queue_size=1),
            rospy.Subscriber('/uav_vision/navigation_hints',TargetDetectionArray,lambda m:self.detections('coarse',m),queue_size=1),
            rospy.Subscriber('/uav_vision/targets',TargetCandidateArray,self.targets,queue_size=1),
            rospy.Subscriber('/uav_vision/drop_offset',DropOffset,self.offset,queue_size=1),
            rospy.Subscriber('/uav_vision/release_evidence',ReleaseEvidence,self.evidence,queue_size=1),
            rospy.Subscriber('/mission/release_permission_active',Bool,self.permission,queue_size=1),
            rospy.Subscriber('/navigation/local_pose',PoseStamped,self.pose,queue_size=1)]
        for key,topic in [('mission','/navigation/mission_status'),('high','/uav_high_view/probe_status'),('mock','/board_trials/mock_release'),('align','/uav_vision/align_mode'),('land_handoff','/board_trials/auto_land_status'),('terminal_hover','/board_trials/terminal_hover_status')]:
            self.subs.append(rospy.Subscriber(topic,String,lambda m,k=key:self.text(k,m),queue_size=1))
        rospy.on_shutdown(self.close)
    def log(self,key,data,period=.2):
        with self.lock:
            if self.closed:return
            now=rospy.Time.now().to_sec()
            if now-self.last.get(key,-1e9)<period:return
            self.last[key]=now;self.events.write(json.dumps(dict(t=now,kind=key,data=data))+'\n');self.events.flush()
    def text(self,key,msg):
        try:value=json.loads(msg.data)
        except ValueError:value=msg.data
        with self.lock:self.state[key]=value
        self.log(key,value,0 if key=='mock' else .2)
    def detections(self,key,msg):
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
    def close(self):
        with self.lock:
            if self.closed:return
            self.closed=True
            self.events.close();self.posefile.close()
            (self.out/'recording.json').write_text(json.dumps(dict(camera_storage='rosbag_only',video_enabled=False,journal='vision_events.jsonl',pose='navigation_pose.csv'),indent=2))
if __name__=='__main__':rospy.init_node('trial_journal');Journal();rospy.spin()
