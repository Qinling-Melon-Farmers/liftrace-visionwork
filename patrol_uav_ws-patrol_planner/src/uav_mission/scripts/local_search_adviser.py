#!/usr/bin/env python3
"""Opt-in research adviser. Publishes JSON hints only; never changes flight goals."""
import json
import threading
import time
from collections import deque
from dataclasses import fields, MISSING
from pathlib import Path
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from plan_manage.srv import CheckLocalSegments, CheckLocalSegmentsRequest
from uav_mission.msg import NavigationDecision
from uav_coverage_memory.memory import Config
from uav_coverage_memory.queries import Snapshot, MissionLedger
from uav_coverage_memory.local_proposals import Context, ProposalConfig, LocalProposer


class LocalSearchAdviser:
    def __init__(self):
        self.lock=threading.RLock()
        values={}
        for f in fields(ProposalConfig):
            values[f.name]=(rospy.get_param('~proposal/'+f.name) if f.default is MISSING
                            else rospy.get_param('~proposal/'+f.name,f.default))
        self.proposer=LocalProposer(ProposalConfig(**values))
        self.grids=deque(maxlen=4);self.statuses=deque(maxlen=4)
        self.pose=self.command=self.mission=None
        self.ledger=MissionLedger();self.snapshot=None
        self.pending=None;self.last_wall=-float('inf');self.last_ros=0.
        self.planner_session=None;self.planner_revision=0;self.blocked_memory=None
        self.service_name=rospy.get_param('~probe_service')
        self.record=None;self.record_count=0
        self.record_limit=int(rospy.get_param('~record_limit',1500))
        if not 0<self.record_limit<=3000:raise ValueError('invalid record limit')
        record_path=rospy.get_param('~record_path','')
        if record_path:
            path=Path(record_path)
            if not path.is_absolute():raise ValueError('record_path must be absolute')
            self.record=path.open('x',encoding='utf-8')
            rospy.on_shutdown(self.close)
        self.rate=float(rospy.get_param('~query_hz',1.))
        if not 0<self.rate<=2:raise ValueError('query_hz must be in (0,2]')
        self.pub=rospy.Publisher('~proposal',String,queue_size=1,latch=True)
        rospy.Subscriber(rospy.get_param('~grid_topic'),OccupancyGrid,self.on_grid,queue_size=1)
        rospy.Subscriber(rospy.get_param('~coverage_status_topic'),String,self.on_status,queue_size=1)
        rospy.Subscriber(rospy.get_param('~pose_topic'),PoseStamped,self.on_pose,queue_size=1)
        rospy.Subscriber(rospy.get_param('~decision_topic'),NavigationDecision,self.on_command,queue_size=1)
        rospy.Subscriber(rospy.get_param('~mission_status_topic'),String,self.on_mission,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(1./self.rate),self.tick,reset=True)

    def on_grid(self,message):
        with self.lock:
            if len(message.data)>20000 or len(message.data)!=message.info.width*message.info.height:
                self.grids.clear();return
            self.grids.append(message)

    @staticmethod
    def json_message(message):
        if len(message.data)>65536:raise ValueError('oversized diagnostic JSON')
        data=json.loads(message.data)
        if not isinstance(data,dict):raise ValueError('invalid diagnostic JSON')
        return data

    def on_status(self,message):
        with self.lock:
            try:self.statuses.append(self.json_message(message))
            except (ValueError,TypeError):self.statuses.clear()

    def on_mission(self,message):
        with self.lock:
            try:self.mission=self.json_message(message)
            except (ValueError,TypeError):self.mission=None

    def on_pose(self,message):
        with self.lock:self.pose=message

    def on_command(self,message):
        with self.lock:self.command=message

    def context(self,now):
        p,m,s=self.pose,self.command,self.mission
        if p is None or m is None or s is None:raise ValueError('motion_context_missing')
        local=s.get('local_motion',{})
        if m.reason.startswith('research_local_entry:') or (local.get('enabled') and
                (local.get('active_seq') or local.get('remaining_for_waypoint')==0)):
            raise ValueError('local_entry_active_or_budget_used')
        if (m.schema_version!=NavigationDecision.SCHEMA_VERSION or not m.has_goal or m.has_target or
                m.command not in (NavigationDecision.SEARCH,NavigationDecision.RESUME) or
                s.get('phase')!='SEARCH' or s.get('mission_id')!=m.mission_id or
                s.get('active_decision_seq')!=m.decision_seq or
                s.get('active_command')!=('SEARCH' if m.command==NavigationDecision.SEARCH else 'RESUME') or
                p.header.frame_id!=m.goal.header.frame_id):
            raise ValueError('inactive_or_mismatched_search_context')
        a,b=p.pose.position,m.goal.pose.position
        context=Context(m.mission_id,m.decision_seq,'SEARCH' if m.command==NavigationDecision.SEARCH else 'RESUME',
            p.header.frame_id,(a.x,a.y,a.z),p.header.stamp.to_sec(),(b.x,b.y,b.z),m.deadline.to_sec())
        context.validate(now,self.proposer.config)
        return context

    def observation(self,context,now):
        pairs=[(s,g) for s in self.statuses for g in self.grids
               if isinstance(s.get('receipt_ros'),(int,float)) and
               abs(g.header.stamp.to_sec()-s['receipt_ros'])<1e-6 and
               s.get('epoch')==context.mission_id and s.get('frame_id')==g.header.frame_id]
        if not pairs:raise ValueError('coverage_pair_missing')
        s,g=max(pairs,key=lambda pair:pair[0]['receipt_ros'])
        c=Config(**s['grid_geometry'])
        origin=g.info.origin
        if (not np.isfinite([g.info.resolution,origin.position.x,origin.position.y,origin.position.z,
                             origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w]).all() or
                c.frame_id!=context.frame_id or abs(g.info.resolution-c.resolution)>1e-7 or
                abs(g.info.origin.position.x-c.min_x)>1e-8 or abs(g.info.origin.position.y-c.min_y)>1e-8 or
                abs(g.info.origin.position.z-c.ground_z)>1e-8 or
                abs(g.info.origin.orientation.w-1)>1e-8 or
                any(abs(x)>1e-8 for x in [g.info.origin.orientation.x,g.info.origin.orientation.y,g.info.origin.orientation.z])):
            raise ValueError('coverage_geometry_mismatch')
        raw=Snapshot(c,np.asarray(g.data).reshape(g.info.height,g.info.width),s['receipt_ros'],s['epoch'],s['generation'])
        reason=raw.check(now,context.frame_id,context.mission_id,raw.generation,self.proposer.config.max_snapshot_age)
        if reason:raise ValueError(reason)
        if self.blocked_memory==(raw.epoch,raw.generation):
            raise ValueError('planner_restart_requires_memory_reset')
        if self.snapshot is None or (raw.epoch,raw.generation,raw.stamp,raw.config)!=(self.snapshot.epoch,self.snapshot.generation,self.snapshot.stamp,self.snapshot.config):
            self.snapshot=self.ledger.update(raw,now,max_age=self.proposer.config.max_snapshot_age)
        return self.snapshot

    def query(self,batch):
        rospy.wait_for_service(self.service_name,timeout=.05)
        req=CheckLocalSegmentsRequest()
        req.header.stamp=rospy.Time.from_sec(batch.created);req.header.frame_id=batch.context.frame_id
        req.request_id=batch.request_id
        req.starts=[Point(*a) for a,b in batch.segments];req.ends=[Point(*b) for a,b in batch.segments]
        response=rospy.ServiceProxy(self.service_name,CheckLocalSegments)(req)
        return dict(request_id=response.request_id,accepted=response.accepted,reason=response.reason,
            frame_id=response.header.frame_id,stamp=response.header.stamp.to_sec(),
            map_stamp=response.map_stamp.to_sec(),map_revision=response.map_revision,
            planner_session=response.planner_session,status=list(response.status),
            known_free_proven=response.known_free_proven,requires_trajectory_validation=response.requires_trajectory_validation,
            checked_voxels=response.checked_voxels,work_ms=response.work_ms)

    def tick(self,_event):
        with self.lock:
            now=rospy.Time.now().to_sec()
            if now<self.last_ros:
                self.ledger=MissionLedger();self.snapshot=None;self.grids.clear();self.statuses.clear()
            self.last_ros=now
            wall=time.monotonic()
            if wall-self.last_wall<1./self.rate:return
            self.last_wall=wall
            # A slow/hung RPC owns the single worker. Never queue another or
            # create an unbounded number of timeout replacement threads.
            if self.pending is not None and self.pending.is_alive():
                self.publish(dict(accepted=False,reason='probe_pending_no_second_request'),now);return
            try:
                context=self.context(now);snapshot=self.observation(context,now)
                batch=self.proposer.prepare(snapshot,context,now)
                self.pending=threading.Thread(target=self.query_job,args=(batch,),daemon=True)
                self.pending.start()
            except (ValueError,TypeError,KeyError,AttributeError) as e:
                self.publish(dict(accepted=False,reason=str(e)),now)

    def query_job(self,batch):
        try:self.complete(batch,reply=self.query(batch))
        except Exception as e:self.complete(batch,error=e)

    def complete(self,batch,reply=None,error=None):
        with self.lock:
            if rospy.is_shutdown():return
            now=rospy.Time.now().to_sec()
            try:
                if error is not None:raise error
                context=self.context(now);snapshot=self.observation(context,now)
                if self.planner_session is not None and (reply['planner_session']!=self.planner_session or reply['map_revision']<self.planner_revision):
                    self.blocked_memory=(snapshot.epoch,snapshot.generation)
                    self.ledger=MissionLedger();self.snapshot=None
                    self.planner_session=reply['planner_session'];self.planner_revision=reply['map_revision']
                    raise ValueError('planner_restart_requires_memory_reset')
                result=self.proposer.finish(batch,reply,context,snapshot.generation,now)
                if result['accepted']:
                    self.planner_session=reply['planner_session'];self.planner_revision=reply['map_revision']
                    result['probe_work_ms']=reply['work_ms'];result['checked_voxels']=reply['checked_voxels']
                self.publish(result,now)
            except Exception as e:
                self.publish(dict(accepted=False,reason='probe_or_context_rejected:'+str(e)),now)

    def publish(self,result,now):
        result.update(receipt_ros=now,flight_authorized=False,requires_trajectory_validation=True)
        result['record_limit_reached']=self.record is not None and self.record_count>=self.record_limit
        encoded=json.dumps(result,sort_keys=True,allow_nan=False)
        if self.record and self.record_count<self.record_limit:
            try:
                self.record.write(encoded+'\n');self.record.flush();self.record_count+=1
            except OSError as e:
                rospy.logwarn_throttle(5,'local advice recorder: %s',e)
                stream=self.record;self.record=None
                try:stream.close()
                except OSError:pass
        self.pub.publish(String(data=encoded))

    def close(self):
        with self.lock:
            if self.record:self.record.close();self.record=None


if __name__=='__main__':
    rospy.init_node('local_search_adviser')
    LocalSearchAdviser()
    rospy.spin()
