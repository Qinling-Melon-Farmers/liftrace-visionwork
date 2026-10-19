import rosbag,json,math,csv
from pathlib import Path
from collections import deque,Counter
import numpy as np
out=Path('/home/xhj/liftrace-deliverables/takeoff_failure_20261011')
meta=json.load(open(out/'run_metadata.json')); start=meta['mapping_startup']; target=meta['ground_reference']['takeoff_z']
p=next(Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests').glob('*/flight_debug_0 (1).bag'))
with rosbag.Bag(str(p)) as b:
 t0=b.get_start_time(); poses=[]; states=[]
 for topic,m,t in b.read_messages(topics=[start['pose_topic'],'/mavros/state']):
  rx=t.to_sec()-t0
  if topic=='/mavros/state':states.append((rx,m.connected,m.armed,m.mode));continue
  st=m.header.stamp.to_sec()-t0;q=m.pose.orientation
  norm=sum(x*x for x in [q.x,q.y,q.z,q.w]);pos=m.pose.position
  ok=m.header.frame_id==start['frame'] and 0<=rx-st<=start['pose_max_age'] and abs(norm-1)<=.05 and all(math.isfinite(x) for x in [pos.x,pos.y,pos.z,st])
  poses.append((rx,st,pos.x,pos.y,pos.z,ok))
rows=[];buf=deque(maxlen=400);i=0;si=0;state=None;ready_since=None
for now in np.arange(136.8,162.51,.02):
 while i<len(poses) and poses[i][0]<=now:
  v=poses[i]
  if v[-1]:buf.append(v)
  i+=1
 while si<len(states) and states[si][0]<=now:state=states[si];si+=1
 recent=[v for v in buf if 0<=now-v[1]<=1]
 z=recent[-1][4] if recent else float('nan');age=now-recent[-1][1] if recent else 99
 span=np.ptp(np.array(recent)[:,2:5],axis=0) if recent else [99]*3
 reasons=[]
 if state is None or not(state[1] and state[2] and state[3]=='OFFBOARD' and 0<=now-state[0]<start['state_max_age']):reasons.append('state')
 if len(recent)<10:reasons.append('count')
 if age>=.3:reasons.append('age')
 if not(abs(z-target)<.15):reasons.append('height')
 if max(span)>=.12:reasons.append('span')
 if reasons:ready_since=None
 elif ready_since is None:ready_since=now
 rows.append(dict(t=round(float(now),3),z=z,height_error=z-target,n=len(recent),age=age,span_x=float(span[0]),span_y=float(span[1]),span_z=float(span[2]),reasons=','.join(reasons) or 'ready',stable_elapsed=0 if ready_since is None else float(now-ready_since)))
with open(out/'startup_gate_20ms.csv','w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
segments=[]
for r in rows:
 if not segments or segments[-1]['reasons']!=r['reasons']:segments.append(dict(start=r['t'],end=r['t']+.02,reasons=r['reasons']))
 else:segments[-1]['end']=r['t']+.02
print('TARGET',target,'accepted',sum(v[-1] for v in poses),'of',len(poses))
print('SEGMENTS',json.dumps(segments,indent=2))
print('ONE_SECOND_FIRST',next((r for r in rows if r['stable_elapsed']>=1),None))
print('AT_SECONDS')
for sec in range(143,163):
 r=min(rows,key=lambda r:abs(r['t']-sec)); print(json.dumps(r))
json.dump(dict(target_z=target,segments=segments,first_one_second=next((r for r in rows if r['stable_elapsed']>=1),None)),open(out/'startup_gate_summary.json','w'),indent=2)