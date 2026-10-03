"""Read recorded bag exports; reproduce identity bookkeeping in current pure classes.
No ROS publishers, model inference, flight commands, or online code changes.
"""
import argparse,bisect,collections,csv,json,sys,subprocess
from pathlib import Path
import numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def ns(t):return int(t['secs'])*1000000000+int(t['nsecs'])
def pos(q):return [q[x] for x in ('x','y','z')]
def snapshot(q,typ):
 return typ(target_id=q['id'],class_name=q['class_name'],class_confidence=q['class_confidence'],geometry_confidence=q['geometry_confidence'],map_quality=q['map_quality'],x=q['map_point']['x'],y=q['map_point']['y'],z=q['map_point']['z'],map_frame=q['map_frame'],state=q['state'],consecutive_observe_count=q['consecutive_observe_count'],map_valid=q['map_valid'],association_valid=q['association_valid'],reject_reason=q['reject_reason'],transform_age_sec=q['transform_age_sec'],first_seen_ns=ns(q['first_seen']),last_seen_ns=ns(q['last_seen']))

def main():
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--report',type=Path,required=True);p.add_argument('--research',type=Path,required=True);a=p.parse_args();a.report.mkdir(exist_ok=True,parents=True)
 sys.path[:0]=[str(a.research/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),str(a.research/'vision_ws/src/uav_high_view/src')]
 from uav_mission.mission_core import CandidateSnapshot,CandidateQueue,MissionConfig,R2026_WEIGHTS,validate_candidate
 from uav_mission.profile_policy import CompetitionProfile
 from uav_high_view.core import Hint,Epoch,Key
 from uav_high_view.navigation_memory import NavigationMemory
 profile=CompetitionProfile('r2026',R2026_WEIGHTS,3);cfg=MissionConfig();summary={};probes={};cv2.setNumThreads(2)
 for tag in ['20-23-54','20-33-08']:
  run=a.base/tag;out=a.report/tag;out.mkdir(exist_ok=True)
  d=json.loads((run/'replay/data.json').read_text());r=d['rows'];sup=json.loads((run/'supplement.json').read_text())
  motion=np.array([[x['t'],*pos(x['m']['pose']['pose']['position'])] for x in r['odom']]);mt=motion[:,0]
  vel=np.column_stack([(np.interp(mt+.25,mt,motion[:,i])-np.interp(mt-.25,mt,motion[:,i]))/.5 for i in (1,2,3)]);speed=np.linalg.norm(vel[:,:2],axis=1)
  rows=[dict(t=x['t'],stamp=x['stamp'],q=q) for x in r['targets'] for q in x['m']['targets']]
  first_cmd=next(x for x in r['command'] if x['m']['command']==1)
  early=next(x for x in rows if x['q']['id']==1 and x['q']['class_name']=='bridge' and validate_candidate(snapshot(x['q'],CandidateSnapshot),d['start']+x['t'],profile,cfg).accepted)
  low=next(x for x in rows if x['q']['id']==1 and x['q']['class_name']=='panzer' and x['t']>first_cmd['t'] and validate_candidate(snapshot(x['q'],CandidateSnapshot),d['start']+x['t'],profile,cfg).accepted)
  late=next(x for x in rows if x['q']['id']==5 and x['q']['class_name']=='bridge' and validate_candidate(snapshot(x['q'],CandidateSnapshot),d['start']+x['t'],profile,cfg).accepted)
  # A recorded valid target ingested directly into the latest queue: both class changes and frozen reservation are preserved.
  queue=CandidateQueue(profile,cfg);s1=snapshot(early['q'],CandidateSnapshot);s2=snapshot(low['q'],CandidateSnapshot);s3=snapshot(late['q'],CandidateSnapshot)
  v1=queue.ingest(s1,d['start']+early['t']);assert v1.accepted;queue.reserve(s1.key,1)
  v2=queue.ingest(s2,d['start']+low['t']);assert v2.accepted
  before=dict(current=queue.entries[s1.key].snapshot.class_name,reserved=queue.entries[s1.key].reserved_snapshot.class_name)
  queue.commit(s1.key,1,'bridge','recorded_raw_ack');v3=queue.ingest(s3,d['start']+late['t']);assert v3.accepted
  ranked=queue.ranked(d['start']+late['t'],(s3.x,s3.y));assert not any(x.snapshot.class_name=='bridge' for x in ranked)
  control=CandidateQueue(profile,cfg);assert control.ingest(s3,d['start']+late['t']).accepted;assert any(x.snapshot.class_name=='bridge' for x in control.ranked(d['start']+late['t'],(s3.x,s3.y)))
  # Component probe only: insert both recorded places as coarse hypotheses, then apply recorded low confirmation.
  epoch=Epoch(tag,'recorded_frame','recorded_camera');memory=NavigationMemory(R2026_WEIGHTS,600000000000)
  def hint(s,source):return Hint(epoch,Key(s.target_id,s.first_seen_ns,source),s.class_name,(s.x,s.y),.45,s.last_seen_ns,1.,s.consecutive_observe_count)
  memory.update([hint(s1,'bbox')],epoch,s1.last_seen_ns)
  resolved=memory.resolve_low(hint(s2,'vision'),s2.last_seen_ns)
  after_low={c:list(h.xy) for c,h in memory.saved.items()}
  visible=memory.update([hint(s3,'bbox')],epoch,s3.last_seen_ns)
  assert resolved and 'panzer' in visible and 'bridge' in visible and not memory.suspended
  high_only=NavigationMemory(R2026_WEIGHTS,600000000000)
  high_only.update([hint(s1,'bbox')],epoch,s1.last_seen_ns)
  high_only.update([hint(s3,'bbox')],epoch,s3.last_seen_ns)
  assert 'bridge' in high_only.suspended
  probes[tag]=dict(kind='RECORDED_SNAPSHOTS_COMPONENT_PROBE_NOT_FLIGHT_REPLAY',queue_before_commit=before,delivered_classes=sorted(queue.delivered_classes),late_bridge_validation=v3.reason,late_bridge_selected_after_wrong_commit=any(x.snapshot.class_name=='bridge' for x in ranked),late_bridge_selected_without_wrong_commit=True,high_only_coarse_conflicts=sorted(high_only.suspended),memory_after_low=after_low,memory_conflict_after=sorted(memory.suspended),memory_resolved=resolved,memory_locations={c:list(h.xy) for c,h in visible.items()},limitations='Chronological component inputs from recorded valid snapshots. Memory input promotes recorded map locations to Hint objects, not an end-to-end coarse projector/Catalog replay; no live height/stage/admissibility/route/actuator acceptance is asserted. Queue uses original source and receipt times.')
  targets={}
  for x in rows:
   q=x['q'];k=str(q['id'])+':'+q['class_name'];v=targets.setdefault(k,dict(first=x['t'],last=x['t'],max_count=0,max_streak=0,first_confirmed=None,first_actionable=None))
   v['last']=x['t'];v['max_count']=max(v['max_count'],q['observe_count']);v['max_streak']=max(v['max_streak'],q['consecutive_observe_count'])
   if q['state']==2 and v['first_confirmed'] is None:v['first_confirmed']=x['t']
   if validate_candidate(snapshot(q,CandidateSnapshot),d['start']+x['t'],profile,cfg).accepted and v['first_actionable'] is None:v['first_actionable']=x['t']
  det=[x for x in r['raw'] if x['m']['source']=='target_detector'];lags=[x['t']-x['stamp'] for x in det]
  mapped=[dict(t=x['t'],stamp=x['stamp'],**q) for x in r['mapped'] for q in x['m']['detections'] if q['class_name'] in ('bridge','panzer','red_cross')]
  phases=[];names={0:'SEARCH',1:'APPROACH',2:'DROP',3:'RESUME',4:'RETURN_HOME',5:'LAND'}
  for i,x in enumerate(r['command']):
   start=max(0,x['t']);end=r['command'][i+1]['t'] if i+1<len(r['command']) else d['duration'];mask=(mt>=start)&(mt<end)
   if mask.any():phases.append(dict(seq=x['m']['decision_seq'],command=names[x['m']['command']],target=x['m']['target_class'],start=start,end=end,duration=end-start,z_p50=float(np.median(motion[mask,3])),speed_p50=float(np.median(speed[mask])),speed_p95=float(np.percentile(speed[mask],95))))
  with (out/'phases.csv').open('w',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(phases[0]),lineterminator='\n');w.writeheader();w.writerows(phases)
  first_armed=next((x['t'] for x in r['fc'] if x['m']['armed']),0.)
  prearmed=motion[(mt<first_armed)&(mt>=0),3]
  ground_fc=float(np.median(prearmed)) if len(prearmed)>=3 else None
  landstates=[];state=None
  for x in sup['/mavros/extended_state']:
   if state!=x['m']['landed_state']:landstates.append(dict(t=x['t'],state=x['m']['landed_state']));state=x['m']['landed_state']
  fc=[];last=None
  for x in r['fc']:
   state=(x['m']['mode'],x['m']['armed'])
   if state!=last:fc.append(dict(t=x['t'],mode=state[0],armed=state[1]));last=state
  events=[dict(t=x['t'],seq=x['m']['decision_seq'],reason=x['m']['reason']) for x in r['result']]
  ar=next(x['t'] for x in r['result'] if x['m']['decision_seq']==first_cmd['m']['decision_seq'] and x['m']['reason']=='approach_arrival_confirmed')
  stable={}
  for cls in ('panzer','red_cross','bridge'):
   valid=[q for q in mapped if q['class_name']==cls and q['map_valid'] and (cls!='bridge' or q['map_point']['y']>0)]
   stable[cls]=np.median(np.array([pos(q['map_point']) for q in valid]),axis=0).tolist()
  late_valid=[x for x in rows if x['q']['id']==5 and validate_candidate(snapshot(x['q'],CandidateSnapshot),d['start']+x['t'],profile,cfg).accepted]
  first_ack=r['release'][0]['t'];ground_z=ground_fc-.22 if ground_fc is not None else None
  summary[tag]=dict(duration=d['duration'],camera_frames=len(d['frames']),source_start=d['start'],search_local_z=sorted({x['m']['goal']['pose']['position']['z'] for x in r['command'] if x['m']['command']==0}),search_x_max=max(x['m']['goal']['pose']['position']['x'] for x in r['command'] if x['m']['command']==0),estimated_ground_z=ground_z,estimated_search_agl=[z-ground_z for z in sorted({x['m']['goal']['pose']['position']['z'] for x in r['command'] if x['m']['command']==0})] if ground_z is not None else None,max_local_z=float(motion[:,3].max()),max_agl=float(motion[:,3].max()-ground_z) if ground_z is not None else None,detector_messages=len(det),detector_counts=dict(collections.Counter(q['class_name'] for x in det for q in x['m']['detections'])),lag_p50=float(np.median(lags)),lag_p95=float(np.percentile(lags,95)),targets=targets,commands=[dict(t=x['t'],**x['m']) for x in r['command']],results=events,release=[dict(t=x['t'],**x['m'],fc_local_z=float(np.interp(x['t'],mt,motion[:,3])),estimated_fc_agl=float(np.interp(x['t'],mt,motion[:,3])-ground_z) if ground_z is not None else None) for x in r['release']],commitments=sup['/mission/release_commitment_evidence'],landed_changes=landstates,fc_changes=fc,physical_target_estimates=stable,first_bridge_action=first_cmd['t'],first_panzer_valid=low['t'],first_panzer_valid_after_arrival=low['t']-ar,early_bridge_valid=early['t'],late_bridge_valid=late['t'],late_bridge_valid_count=len(late_valid),late_bridge_last_valid=late_valid[-1]['t'],reserved_class_mismatch_before_ack=first_ack-low['t'],final=json.loads(r['mission'][-1]['m']['data']))
  # Export the small exact snapshots needed to reproduce class/queue results.
  (out/'identity_snapshots.json').write_text(json.dumps(dict(early=early,low=low,late=late),indent=2)+'\n')
  fig,axs=plt.subplots(3,1,figsize=(13,9),sharex=True)
  axs[0].plot(mt,motion[:,3],label='FC local Z');axs[1].plot(mt,speed,color='tab:green',label='XY speed (0.5s pose difference)')
  for cls,color in [('bridge','tab:blue'),('panzer','tab:orange'),('red_cross','tab:red')]:
   xy=[(x['stamp'],q['class_confidence']) for x in det for q in x['m']['detections'] if q['class_name']==cls]
   if xy:axs[2].scatter(*np.array(xy).T,s=12,label=cls,color=color)
  for ax in axs:
   for x in r['release']:ax.axvline(x['t'],c='k',ls=':',alpha=.65)
   ax.axvline(low['t'],c='tab:orange',ls='--',alpha=.6);ax.axvline(late['t'],c='tab:blue',ls='--',alpha=.6);ax.grid(alpha=.2);ax.legend(loc='upper right')
  axs[0].set_ylabel('FC local Z / m');axs[1].set_ylabel('XY speed / m/s');axs[2].set_ylabel('YOLO confidence');axs[2].set_xlabel('Bag seconds: image source time for detections; receipt time for events');fig.suptitle(tag+' | physical panzer released as bridge; real bridge later excluded');fig.tight_layout();fig.savefig(out/'motion_vision.png',dpi=145);plt.close(fig)
  fig,ax=plt.subplots(figsize=(10,6));ax.plot(motion[:,1],motion[:,2],label='recorded FC pose',lw=1.4)
  goals=np.array([pos(x['m']['goal']['pose']['position']) for x in r['command'] if x['m']['command']==0]);ax.plot(goals[:,0],goals[:,1],'--o',label='issued SEARCH goals',alpha=.6)
  for cls,xy in stable.items():ax.scatter(*xy[:2],marker='x',s=110,label='physical '+cls+' (vision map)')
  ax.annotate('initial heading +X',xy=(.8,0),xytext=(0,0),arrowprops=dict(arrowstyle='->',color='red'),color='red');ax.set(xlabel='X forward / m',ylabel='Y left / m');ax.axis('equal');ax.legend(loc='upper center',bbox_to_anchor=(.5,1.23),ncol=2);ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'trajectory.png',dpi=145);plt.close(fig)
  # Select actual detector source frames, one set per physical identity transition.
  early_det=[x for x in det for q in x['m']['detections'] if q['class_name']=='bridge' and 16<x['stamp']<first_cmd['t']]
  low_det=[x for x in det for q in x['m']['detections'] if q['class_name']=='panzer' and first_cmd['t']<x['stamp']<first_ack]
  late_det=[x for x in det for q in x['m']['detections'] if q['class_name']=='bridge' and x['stamp']>120]
  selections=[('True panzer / predicted bridge',early_det[-1]),('True panzer / now predicted panzer',min(low_det,key=lambda x:abs(x['stamp']-(low['t']-.3)))),('True bridge / recorded bridge',min(late_det,key=lambda x:abs(x['stamp']-(late['t']-.3))))]
  cells=[]
  for label,x in selections:
   fr=min(d['frames'],key=lambda f:abs(f['stamp']-x['stamp']));assert abs(fr['stamp']-x['stamp'])<.001
   im=cv2.imread(str(run/'replay'/fr['file']))
   for q in x['m']['detections']:
    bb=q['roi'];cv2.rectangle(im,(bb['x_offset'],bb['y_offset']),(bb['x_offset']+bb['width'],bb['y_offset']+bb['height']),(0,240,0),2);cv2.putText(im,q['class_name']+' %.3f'%q['class_confidence'],(bb['x_offset'],max(22,bb['y_offset']-8)),0,.7,(0,240,0),2)
   im=cv2.resize(im,(640,360));bar=np.zeros((66,640,3),np.uint8);cv2.putText(bar,label,(8,23),0,.53,(255,255,255),1);cv2.putText(bar,tag+' source t=%.3fs'%fr['stamp'],(8,49),0,.52,(255,255,255),1);cells.append(np.vstack([im,bar]))
  cv2.imwrite(str(out/'identity_triptych.jpg'),np.hstack(cells))
  print(tag,json.dumps({k:summary[tag][k] for k in ['detector_counts','search_local_z','estimated_search_agl','first_bridge_action','first_panzer_valid','first_panzer_valid_after_arrival','late_bridge_valid','late_bridge_valid_count','reserved_class_mismatch_before_ack']},ensure_ascii=False),flush=True)
 (a.report/'metrics.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n');(a.report/'component_probes.json').write_text(json.dumps(probes,indent=2,ensure_ascii=False)+'\n');print(json.dumps(probes,indent=2))
if __name__=='__main__':main()