"""Recompute fusion/refinement/projection/memory and shadow admission; no ROS IO."""
import argparse
import bisect
import copy
import json
import sys
from pathlib import Path
from unittest.mock import patch,Mock
import rosbag
import rospy
import tf2_ros
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/"vision_ws/src/uav_vision/scripts"),
              str(ROOT/"patrol_uav_ws-patrol_planner/src/uav_mission/src")]
from uav_vision.detection_fusion import DetectionFusion
from target_refiner import TargetRefiner
from target_map_projector import TargetMapProjector
from target_memory import TargetMemory
from uav_mission.mission_core import (MissionCore,MissionConfig,CandidateSnapshot,
                                    validate_candidate,MissionPhase,SlotStatus)
from uav_mission.profile_policy import CompetitionProfile
from uav_vision.msg import TargetDetectionArray

def plain(m):
    if hasattr(m,"__slots__"):return {k:plain(getattr(m,k)) for k in m.__slots__}
    if isinstance(m,(tuple,list)):return [plain(x) for x in m]
    return m

def snapshot(m):
    return CandidateSnapshot(target_id=m.id,class_name=m.class_name,
        class_confidence=m.class_confidence,geometry_confidence=m.geometry_confidence,
        map_quality=m.map_quality,x=m.map_point.x,y=m.map_point.y,z=m.map_point.z,
        map_frame=m.map_frame,state=m.state,consecutive_observe_count=m.consecutive_observe_count,
        map_valid=m.map_valid,association_valid=m.association_valid,reject_reason=m.reject_reason,
        transform_age_sec=m.transform_age_sec,first_seen_ns=m.first_seen.to_nsec(),
        last_seen_ns=m.last_seen.to_nsec())

class Replay:
    def __init__(self,bag,rawbag=None):
        self.rows=[];self.mapped=[];self.coarse=[];self.meta={}
        self.clock=rospy.Time.from_sec(1.)
        self.params={}
        self.patches=[patch.object(rospy,"init_node"),patch.object(rospy,"Publisher"),
                      patch.object(rospy,"Subscriber"),patch.object(rospy,"Service"),
                      patch.object(rospy,"Timer"),patch.object(tf2_ros,"TransformListener"),
                      patch.object(rospy.Time,"now",side_effect=lambda:self.clock),
                      patch.object(rospy,"get_param",side_effect=lambda name,default=None:self.params.get(name.lstrip("~"),default))]
        for p in self.patches:p.start()
        self.buffer=tf2_ros.Buffer(cache_time=rospy.Duration(1000.),debug=False)
        info=None;self.events=[];self.poses=[];self.statuses=[]
        self.yolo=[];self.begin=None
        with rosbag.Bag(str(bag)) as b:
            self.begin=b.get_start_time();self.end=b.get_end_time()
            for topic,m,t in b.read_messages(topics=["/tf","/tf_static","/camera/camera_info",
                     "/uav_vision/detections","/uav_vision/align_mode","/navigation/mission_status",
                     "/mavros/local_position/pose"]):
                if topic=="/tf_static":
                    for tr in m.transforms:self.buffer.set_transform_static(tr,"bag")
                elif topic=="/tf":
                    for tr in m.transforms:self.buffer.set_transform(tr,"bag")
                elif topic=="/camera/camera_info" and info is None:info=m
                elif topic=="/uav_vision/detections" and rawbag is None:self.events.append((t.to_sec(),0,m))
                elif topic=="/uav_vision/align_mode":self.events.append((t.to_sec(),1,m))
                elif topic=="/navigation/mission_status":self.statuses.append((t.to_sec(),json.loads(m.data)))
                elif topic=="/mavros/local_position/pose":self.poses.append((t.to_sec(),m))
        if rawbag:
            with rosbag.Bag(str(rawbag)) as b:
                for _,m,t in b.read_messages():self.events.append((t.to_sec(),0,m))
        self.events.sort(key=lambda row:row[0])
        self.status_times=[s[0] for s in self.statuses]
        self.pose_times=[s[0] for s in self.poses]
        self.clock=rospy.Time.from_sec(self.begin)
        self.setparams("detection_fusion");self.fusion=DetectionFusion()
        self.setparams("target_refiner",class_profile="r2026");self.refiner=TargetRefiner()
        with patch.object(tf2_ros,"Buffer",return_value=self.buffer):
            self.setparams("target_map_projector",ground_z=-.22,map_frame="camera_init",
                           coarse_navigation_enabled=True)
            self.projector=TargetMapProjector()
        self.projector._on_camera_info(info)
        self.memories={}
        for name,gap in (("strict",0.),("gap1s",1.)):
            self.setparams("target_memory",class_profile="r2026",
                           require_map_for_candidates=True,require_complete_detection_sources=True,
                           search_confirmation_max_gap_sec=gap)
            memory=TargetMemory()
            memory._targets_pub=Mock(publish=lambda m,n=name:self.collect(n,m))
            self.memories[name]=memory
        self.fusion._publisher=Mock(publish=self.refiner._on_detections)
        self.refiner._pub=Mock(publish=self.projector._on_detections)
        self.projector._pub=Mock(publish=self.on_mapped)
        self.projector._coarse_pub=Mock(publish=lambda m:self.coarse.append(
            dict(t=self.clock.to_sec()-self.begin,stamp=m.header.stamp.to_sec()-self.begin,m=plain(m))))
        self.profile=CompetitionProfile(name="r2026",weights=dict(tent=1.,pillbox=1.5,bridge=2.,panzer=2.5,red_cross=10.),
                                        interrupt_top_k=3,required_deliveries=3)
        self.cfg=MissionConfig(min_streak=2,approach_altitude=1.,return_altitude=1.,motion_action_timeout=60.)

    def setparams(self,name,**kwargs):
        path=ROOT/("vision_ws/src/uav_vision/config/"+name+".yaml")
        self.params=yaml.safe_load(path.read_text()) or {}
        self.params.update(kwargs)

    def on_mapped(self,msg):
        self.mapped.append(dict(t=self.clock.to_sec()-self.begin,
                               stamp=msg.header.stamp.to_sec()-self.begin,m=plain(msg)))
        for memory in self.memories.values():memory._on_detections(copy.deepcopy(msg))

    def collect(self,name,msg):
        now=self.clock.to_sec()
        status=self.statuses[max(0,bisect.bisect_right(self.status_times,now)-1)][1]
        pose=self.poses[max(0,bisect.bisect_right(self.pose_times,now)-1)][1].pose.position
        candidates=[snapshot(m) for m in msg.targets if m.class_name in self.profile.weights]
        vals=[validate_candidate(c,now,self.profile,self.cfg) for c in candidates]
        # Isolated one-decision fork at the recorded mission phase, never a simulated flight.
        core=MissionCore(self.profile,self.cfg)
        core.start("bag-shadow",self.begin)
        core.phase=MissionPhase(status["phase"])
        for slot,s in zip(core.slots,status.get("slot_status",[])):slot.status=SlotStatus(s)
        if status.get("committed_slots",0):core.queue.delivered_classes.add("red_cross")
        core.ingest(candidates,now)
        action=core.choose(now,(pose.x,pose.y),False)
        self.rows.append(dict(t=now-self.begin,source_stamp=msg.header.stamp.to_sec()-self.begin,
            variant=name,recorded_phase=status["phase"],
            suggested_action=action.command if action else None,
            suggested_class=action.target_class if action else None,
            targets=[dict(class_name=c.class_name,id=c.target_id,state=c.state,
                          streak=c.consecutive_observe_count,map_valid=c.map_valid,
                          xy=[c.x,c.y],last_seen=c.last_seen_ns/1e9-self.begin,
                          admitted=v.accepted,rejection=v.reason) for c,v in zip(candidates,vals)]))

    def run(self,out):
        for t,kind,m in self.events:
            self.clock=rospy.Time.from_sec(t)
            if kind==1:
                self.fusion._on_align_mode(m)
                for memory in self.memories.values():memory._on_align_mode(m)
            else:
                if m.source=="target_detector":
                    self.yolo.append(dict(t=t-self.begin,stamp=m.header.stamp.to_sec()-self.begin,m=plain(m)))
                    self.projector._on_coarse(m)
                self.fusion._on_detections(m)
            self.fusion._flush_ready(None)
        self.clock=rospy.Time.from_sec(self.end+1.)
        self.fusion._flush_ready(None)
        summary={}
        for variant in self.memories:
            records=[r for r in self.rows if r["variant"]==variant]
            summary[variant]={}
            for cls in ("panzer","red_cross"):
                valid=[r for r in records if any(c["class_name"]==cls and c["admitted"] for c in r["targets"])]
                conf=[r for r in records if any(c["class_name"]==cls and c["state"]==2 and c["map_valid"] for c in r["targets"])]
                approach=[r for r in records if r["suggested_action"]=="APPROACH" and r["suggested_class"]==cls]
                summary[variant][cls]=dict(first_confirmed=conf[0]["t"] if conf else None,
                    first_admitted=valid[0]["t"] if valid else None,
                    first_approach=approach[0]["t"] if approach else None,
                    confirmed_rows=len(conf),admitted_rows=len(valid),
                    outbound_approach_rows=sum(r["t"]<19.027 for r in approach))
        summary["yolo_counts"]={c:sum(d["class_name"]==c for r in self.yolo for d in r["m"]["detections"]) for c in ("panzer","red_cross")}
        summary["mapped_panzer"]=[dict(t=r["t"],stamp=r["stamp"],map_valid=d["map_valid"],reason=d["reject_reason"],conf=d["class_confidence"])
                   for r in self.mapped for d in r["m"]["detections"] if d["class_name"]=="panzer"]
        result=dict(summary=summary,rows=self.rows,mapped=self.mapped,coarse=self.coarse,yolo=self.yolo,
                    begin=self.begin,duration=self.end-self.begin)
        out.write_text(json.dumps(result,ensure_ascii=False))
        print(json.dumps({k:v for k,v in summary.items() if k!="mapped_panzer"},ensure_ascii=False,indent=2))
        for p in reversed(self.patches):p.stop()

if __name__=="__main__":
    a=argparse.ArgumentParser();a.add_argument("--bag",type=Path,required=True)
    a.add_argument("--rawbag",type=Path);a.add_argument("--out",type=Path,required=True)
    args=a.parse_args();Replay(args.bag,args.rawbag).run(args.out)
