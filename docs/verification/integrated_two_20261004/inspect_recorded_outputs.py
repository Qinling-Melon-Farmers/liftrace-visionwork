#!/usr/bin/env python3
"""Offline final coverage, landing publications, release association and tail frames."""
import argparse,bisect,collections,csv,json,math
from pathlib import Path
import cv2
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seed',type=int,required=True)
p.add_argument('--data',type=Path,required=True)
a=p.parse_args()
D=Path(__file__).resolve().parent
folder=D/f'{a.seed}_centers'
base=json.loads((folder/'center_summary.json').read_text())
R=Path(base['run']);truth=base['truth']
dest=folder/'inspection'
if dest.exists() and any(dest.iterdir()):raise ValueError('Inspection outputs already exist')
dest.mkdir(exist_ok=True)
events=[json.loads(x) for x in a.data.read_text().splitlines()]
modes=sorted((e['t'],e['m']['data']) for e in events if e['kind']=='mode');mt=[x[0] for x in modes]
selected=sorted(((e['t'],e['m']) for e in events if e['kind']=='selected'),key=lambda x:x[0]);sel_t=[x[0] for x in selected]
def stamp(m):
    s=m.get('header',{}).get('stamp',{})
    return s.get('secs',0)+s.get('nsecs',0)/1e9
def at(t,ts,vs,default):
    i=bisect.bisect_right(ts,t)-1
    return vs[i][1] if i>=0 else default
def assoc(cls,xy):
    ds=sorted([(math.hypot(xy[0]-r['eval_xy'][0],xy[1]-r['eval_xy'][1]),r) for r in truth],key=lambda x:x[0])
    own=next((v for v in ds if v[1]['class_name']==cls),None)
    return dict(nearest_class=ds[0][1]['class_name'],nearest_instance=ds[0][1]['name'],nearest_distance_m=ds[0][0],
                same_class_distance_m=own[0] if own else None)
land=collections.Counter();completed=collections.Counter();detector_marks=[]
mapped={}
for e in events:
    if e['kind']!='mapped':continue
    m=e['m'];t=stamp(m);mapped[round(t,9)]=m
    if at(t,mt,modes,'UNKNOWN')!='landing':continue
    land['arrays']+=1;completed.update(m.get('completed_sources',[]))
    for d in m['detections']:land[d['class_name']]+=1
    if 'target_detector' in m.get('completed_sources',[]):
        detector_marks.append(dict(source_ros_s=t,receipt_ros_s=e['t'],classes=[d['class_name'] for d in m['detections']]))
gate=json.loads((R/'gate_status.json').read_text())
fences={r['decision_seq']:r for r in gate['decision_fences']}
commits=[];seen=set()
for line in (R/'key_events.jsonl').open():
    e=json.loads(line);d=e.get('data',{})
    if e.get('kind')!='result' or not d.get('payload_committed'):continue
    k=(d['decision_seq'],d['payload_slot'])
    if k in seen:continue
    seen.add(k);f=fences[d['decision_seq']]
    commits.append(dict(ack_ros_s=e['ros_sec'],decision_seq=d['decision_seq'],payload_slot=d['payload_slot'],
        target_id=d['target_id'],target_class=d['target_class'],reason=d['reason'],
        decision_xy=[f['goal_x'],f['goal_y']],decision_target_id=f['target_id'],
        identity_matches_decision=d['target_id']==f['target_id'] and d['target_class']==f['target_class'],
        **assoc(d['target_class'],[f['goal_x'],f['goal_y']])))
raw=list(csv.DictReader((R/'downward.csv').open()));rt=[float(x['image_stamp_ros_sec']) for x in raw]
mapping=list(csv.DictReader((D/f'{a.seed}_mapped_overlay.frames.csv').open()))
last10=[rt[i]-rt[i-1] for i in range(1,len(rt)) if rt[i]>=rt[-1]-10]
coverage=dict(source_frames=len(rt),first_source_ros_s=rt[0],last_source_ros_s=rt[-1],
    mission_end_ros_s=base['mission_end_ros_s'],source_after_mission_end_s=rt[-1]-base['mission_end_ros_s'],
    last_output_playback_ros_s=float(mapping[-1]['playback_ros_s']),
    last_output_source_ros_s=float(mapping[-1]['image_stamp_ros_s']),
    last_output_source_frame=int(mapping[-1]['source_frame']),
    final_source_frames_not_selected_by_output_grid=len(rt)-int(mapping[-1]['source_frame'])-1,
    final_source_to_last_output_tick_s=rt[-1]-float(mapping[-1]['playback_ros_s']),
    last10s_source_max_gap_s=max(last10),last10s_source_gaps_over_150ms=sum(x>.15 for x in last10),
    last10s_source_gaps_over_300ms=sum(x>.3 for x in last10))
rows=list(csv.DictReader((folder/'stages/center_samples_by_phase.csv').open()))
valid=[r for r in rows if r['stream']=='mapped' and r['nearest_any_class']==r['class_name'] and
       not r['exclusion'] and r['duplicate']=='False' and r['fresh']=='True' and r['in_mission']=='True']
tails=[];cases=[]
cv2.setNumThreads(1)
cap=cv2.VideoCapture(str(D/f'{a.seed}_mapped_overlay.mp4'),cv2.CAP_FFMPEG,[cv2.CAP_PROP_N_THREADS,1])
for cls in ('bridge','panzer'):
    cr=[r for r in valid if r['class_name']==cls]
    if not cr:continue
    values=[float(r['xy_error_m']) for r in cr];q=float(np.percentile(values,95))
    tails.append(dict(class_name=cls,n=len(cr),p95_m=q,max_m=max(values),
        above_30cm_by_phase=dict(collections.Counter(r['source_phase'] for r in cr if float(r['xy_error_m'])>.3)),
        p95_tail_by_phase=dict(collections.Counter(r['source_phase'] for r in cr if float(r['xy_error_m'])>=q))))
    for phase in sorted(set(r['source_phase'] for r in cr if float(r['xy_error_m'])>.3)):
        ranked=sorted((r for r in cr if r['source_phase']==phase),key=lambda r:float(r['xy_error_m']),reverse=True)
        chosen=None
        for r in ranked:
            t=float(r['source_ros_s']);i=bisect.bisect_left(rt,t)
            nearest=min((j for j in (i-1,i) if 0<=j<len(rt)),key=lambda j:abs(rt[j]-t))
            if abs(rt[nearest]-t)>.03:continue
            frame=next((v for v in mapping if int(v['source_frame'])==nearest and v['mapped_source_stamp_ros_s'] and abs(float(v['mapped_source_stamp_ros_s'])-t)<1e-6),None)
            if frame:chosen=(r,frame);break
        if chosen is None:continue
        r,frame=chosen;t=float(r['source_ros_s']);m=mapped.get(round(t,9),{})
        ds=[d for d in m.get('detections',[]) if d['class_name']==cls]
        cap.set(cv2.CAP_PROP_POS_FRAMES,int(frame['output_frame']));ok,img=cap.read()
        if not ok:raise ValueError('Case frame decode failed')
        name=f'{cls}_{phase}_{t:.3f}.png';cv2.imwrite(str(dest/name),img)
        sel=at(t,sel_t,selected,{})
        cases.append(dict(class_name=cls,phase=phase,sample=r,frame_mapping=frame,preview=name,
            recorded_detections=ds,selected_at_source=dict(class_name=sel.get('class_name'),id=sel.get('id'),
            map_point=sel.get('map_point'),map_valid=sel.get('map_valid'))))
cap.release()
result=dict(seed=a.seed,run=str(R),coverage=coverage,landing_source_mode_counts=land,
    landing_completed_sources=completed,target_detector_completion_marks_in_landing=detector_marks,
    landing_note='Mode as of source image stamp. Completion does not prove inference ran or emitted boxes. No raw detector topic was recorded.',
    release_commits=commits,tails=tails,cases=cases)
(dest/'inspection.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(seed=a.seed,coverage=coverage,land=land,completed=completed,target_detector_marks=detector_marks,
                     release_commits=commits,tails=tails,previews=[x['preview'] for x in cases]),indent=2))
