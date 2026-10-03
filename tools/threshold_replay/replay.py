#!/usr/bin/env python3
"""Offline threshold comparison; no ROS node, publisher, services or aircraft."""
import argparse,bisect,collections,contextlib,copy,importlib.util,json,math,sys
from pathlib import Path
from types import MethodType,SimpleNamespace
from unittest.mock import patch
import numpy as np,yaml,rospy
from genpy.message import fill_message_args
from std_msgs.msg import String
from uav_vision.msg import TargetDetectionArray


def circle_replay(root,path,threshold):
    spec=importlib.util.spec_from_file_location('offline_target_memory',root/'vision_ws/src/uav_vision/scripts/target_memory.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    data=json.loads(path.read_text());base=data['start'];rows=data['rows'];clock=[base];published=[]
    params=yaml.safe_load((root/'vision_ws/src/uav_vision/config/target_memory.yaml').read_text())
    params.update(require_map_for_candidates=True,require_complete_detection_sources=True,
                  class_profile='r2026',search_confirmation_max_gap_sec=1.)
    def publisher(topic,*args,**kwargs):
        return SimpleNamespace(publish=lambda msg:published.append(msg) if topic=='/uav_vision/targets' else None)
    events=sorted([(row['t'],0,row) for row in rows['mode']]+[(row['t'],1,row) for row in rows['fc']]+[(row['t'],2,row) for row in rows['mapped']],key=lambda v:(v[0],v[1]))
    frames=[];fresh=[];raw=[];seen=set();published=[];fc={};windows=[];active=None
    with contextlib.ExitStack() as stack:
        for name in ('init_node','Subscriber','Service','loginfo','logwarn','logwarn_throttle','logerr'):
            stack.enter_context(patch.object(rospy,name))
        stack.enter_context(patch.object(rospy,'get_param',side_effect=lambda name,default=None:params.get(name.lstrip('~'),default)))
        stack.enter_context(patch.object(rospy,'Publisher',side_effect=publisher))
        stack.enter_context(patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(clock[0])))
        memory=mod.TargetMemory();original=memory._pass_threshold
        def only_circle(self,det):
            saved=self._aux_geom
            try:
                if det.class_name=='circle':self._aux_geom=threshold
                return original(det)
            finally:self._aux_geom=saved
        memory._pass_threshold=MethodType(only_circle,memory)
        for t,kind,row in events:
            clock[0]=base+t
            if kind==0:memory._on_align_mode(String(data=row['m']['data']))
            if kind==1:fc=row['m']
            include=(memory._align_mode=='drop_circle' and fc.get('armed') is True and fc.get('mode')=='OFFBOARD')
            if include and active is None:active=t
            elif not include and active is not None:windows.append((active,t));active=None
            if kind!=2:continue
            msg=TargetDetectionArray();fill_message_args(msg,[row['m']])
            stamp=msg.header.stamp.to_nsec()
            mode=memory._align_mode
            for det in msg.detections:
                if not include or det.class_name!='circle':continue
                key=(stamp,det.center_px.x,det.center_px.y)
                if key in seen:continue
                seen.add(key)
                if det.map_valid and det.center_refined and det.geometry_verified:
                    raw.append(dict(t=t,stamp=row['stamp'],q=det.geometry_confidence,
                                    lag=t-row['stamp'],xy=[det.map_point.x,det.map_point.y]))
            published.clear();memory._on_detections(msg)
            if not include or not published:continue
            for c in published[-1].targets:
                if (c.class_name=='circle' and c.state==2 and c.consecutive_observe_count>=3
                        and c.last_seen.to_nsec()==stamp and c.map_valid):
                    frames.append(dict(t=t,stamp=row['stamp'],xy=[c.map_point.x,c.map_point.y]))
                    if 0<=t-row['stamp']<=.5:fresh.append(frames[-1])
    # Compare low-score centre to a nearby-in-time high-score map estimate.
    # This is agreement, NOT ground-truth target-centre error.
    hi=[v for v in raw if v['q']>=.8];extra=[v for v in raw if threshold<=v['q']<.8]
    distances=[]
    for v in extra:
        nearby=[h for h in hi if abs(h['stamp']-v['stamp'])<=.5]
        if nearby:
            h=min(nearby,key=lambda h:abs(h['stamp']-v['stamp']))
            distances.append(math.dist(v['xy'],h['xy']))
    if active is not None:windows.append((active,data['duration']))
    def first(seq,a,b):return next((v['t'] for v in seq if a<=v['t']<b),None)
    return dict(replay=str(path),threshold=threshold,mapped_circle_samples=len(raw),
        score_pass=sum(v['q']>=threshold for v in raw),
        new_score_pass=len(extra),new_score_pass_fresh=sum(0<=v['lag']<=.5 for v in extra),
        newly_observed_confirmed_frames=len(frames),fresh_confirmed_frames=len(fresh),
        windows=[dict(start=a,end=b,first_confirmed=first(frames,a,b),first_fresh_confirmed=first(fresh,a,b),
                      fresh_frames=sum(a<=v['t']<b for v in fresh)) for a,b in windows],
        extra_centre_agreement=dict(paired=len(distances),unpaired=len(extra)-len(distances),
              median_m=float(np.median(distances)) if distances else None,p95_m=float(np.percentile(distances,95)) if distances else None),
        scope='Production TargetMemory replay of recorded mapped frames with arrival-time mode switches; metrics only for armed OFFBOARD drop_circle windows; only circle score changed. No detector rerun, flight, alignment-context or release simulation; H remains 0.80.')


def coarse_replay(root,path,end):
    from uav_mission.profile_policy import load_profile
    from uav_mission.mission_core import MissionCore,MissionConfig,GoalSnapshot
    from uav_mission.high_view_probe import ProbeConfig
    from uav_mission.high_view_full import HighViewFull
    from uav_high_view.survey_policy import SurveyPolicy
    source=json.loads(path.read_text());ts=[float(p['t']) for p in source['poses']];output=[]
    prof=load_profile(root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition_profiles.yaml','r2026')
    for threshold in (.60,.65,.70,.75,.80):
        cfg=MissionConfig(mission_timeout=600.,forced_return_at=510.,early_return_enabled=False,landing_xy=(.6,0.),return_altitude=1.18,
             post_delivery_route=(GoalSnapshot('camera_init',.6,0.,1.18),))
        r=HighViewFull(MissionCore(prof,cfg),ProbeConfig(source['reference']['ground_z'],((1.,0.),),high_agl=2.),
              SurveyPolicy(coarse_enabled=True,coarse_min_confidence=threshold,high_min_agl=1.8,high_max_agl=2.2))
        r.start('offline-threshold',100.,(0.,0.));counts=collections.Counter();observations=[];max_interrupt=0
        for event in source['events']:
            if end is not None and event['t']>end:break
            index=bisect.bisect_right(ts,event['t'])-1
            if index<0:continue
            p=source['poses'][index];now=101.+event['t']-source['events'][0]['t'];r.pose=tuple(float(p[k]) for k in ('x','y','z'));r.pose_stamp=now-(event['t']-float(p['t']))
            for det in event['data']:
                reason=r.ingest_coarse(class_name=det['class_name'],xy=tuple(det['xy']),stamp_ns=int((now-event['t']+det['stamp'])*1e9),
                    frame='camera_init',confidence=det['confidence'],transform_age_sec=0.,map_valid=det['map_valid'],now=now)
                counts[reason]+=1;observations.append(dict(time=event['t'],class_name=det['class_name'],confidence=det['confidence'],reason=reason))
            hints=r._all_hints(now);interrupt=r._interrupt_top(now);max_interrupt=max(max_interrupt,len(interrupt))
        output.append(dict(threshold=threshold,counts=dict(counts),revisit={c:list(h.xy) for c,h in hints.items()},
           conflicts=sorted(r.memory.suspended),interrupt=list(interrupt),max_interrupt=max_interrupt,
           observations=observations,committed=r.core.committed_slots))
    return dict(scope='Recorded sampled projected boxes with paired recorded poses; TF age assumed zero. Production coarse ingest/memory only; no detector rerun or truth score.',results=output)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--replay',type=Path,action='append',default=[])
    ap.add_argument('--coarse',type=Path);ap.add_argument('--coarse-end',type=float);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    results={'circle':[]}
    for path in a.replay:
        for threshold in (.80,.75):
            row=circle_replay(a.root,path,threshold);results['circle'].append(row);print(json.dumps(row),flush=True)
    if a.coarse:
        results['coarse']=coarse_replay(a.root,a.coarse,a.coarse_end)
        for row in results['coarse']['results']:print(json.dumps({k:v for k,v in row.items() if k!='observations'}),flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(results,indent=2))
if __name__=='__main__':main()
