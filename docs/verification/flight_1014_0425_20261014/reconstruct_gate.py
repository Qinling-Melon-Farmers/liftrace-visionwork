from pathlib import Path
import json,numpy as np,re,csv,subprocess
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--run-dir',type=Path,required=True);args=ap.parse_args();run=args.run_dir;n=run/'numeric'
def read(k):return json.loads((n/(k+'.json')).read_text())
sp=[r for r in read('navigation__setpoint_mission') if 271.2<=r['t']<296.17]
pose=read('navigation__local_pose');odom=read('navigation__local_odom');pt=np.array([r['stamp'] for r in pose]);ot=np.array([r['stamp'] for r in odom])
marks=[]
for r in read('uav_vision__detections_mapped'):
 for d in r['m']['detections']:
  if d['class_name']=='landing_pad' and d['map_valid'] and 271.2<=r['t']<296.17:marks.append((r,d))
mp=np.array([[d['map_point'][k] for k in 'xy'] for r,d in marks]);mt=np.array([r['stamp'] for r,d in marks])
def xy(p):return np.array([p[k] for k in 'xy'])
def normvel(m):
 v=m['twist']['twist']['linear'];q=m['pose']['pose']['orientation'];v=np.array([v[k] for k in 'xyz']);qv=np.array([q[k] for k in 'xyz']);v=v+2*np.cross(qv,np.cross(qv,v)+q['w']*v);return float(np.linalg.norm(v[:2]))
rows=[]
for s in sp:
 now=s['m']['header']['stamp'];tr=now-1791923122.167234;goal=xy(s['m']['pose']['position']);dist=np.linalg.norm(mp-goal,axis=1);score=dist.copy();score[mt>s['stamp']+.02]=np.inf;j=score.argmin()
 if dist[j]>1e-4:continue
 mr,md=marks[j];pi=max(0,np.searchsorted(pt,s['stamp'],side='right')-1);oi=max(0,np.searchsorted(ot,s['stamp'],side='right')-1);pr=pose[pi];od=odom[oi];m=od['m'];err=max(np.linalg.norm(xy(pr['m']['pose']['position'])-goal),np.linalg.norm(xy(m['pose']['pose']['position'])-goal));v=normvel(m)
 stamps=[m['header']['stamp'],od['stamp'],md['header']['stamp'],mr['stamp']];ages=[now-x for x in stamps]
 valid=min(ages)>=-.015 and max(ages)<.5
 flags=[]
 if err>.06:flags.append('xy')
 if v>=.1:flags.append('speed')
 if ages[2]>.2:flags.append('image_age')
 if ages[0]>.2:flags.append('odom_age')
 if ages[1]>.2 or ages[3]>.2:flags.append('receipt_age')
 if min(ages)<0:flags.append('future_receipt_or_stamp')
 row={'t':tr,'now':now,'odom':stamps[0],'receipt':stamps[1],'mark':stamps[2],'mark_receipt':stamps[3],'xy':float(err),'speed':v,'valid':int(valid),'image_age':ages[2],'odom_age':ages[0],'failures':'+'.join(flags),'goal_x':goal[0],'goal_y':goal[1],'pose_x':pr['m']['pose']['position']['x'],'pose_y':pr['m']['pose']['position']['y'],'pose_agl':pr['m']['pose']['position']['z']+.20592633428052062}
 rows.append(row)
with (n/'h_gate_reconstruction.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
with (n/'gate_input.txt').open('w') as f:
 for r in rows:f.write(' '.join(str(r[k]) for k in ['now','odom','receipt','mark','mark_receipt','xy','speed','valid'])+'\n')
from collections import Counter
print('Matched control samples',len(rows),'failures',Counter(r['failures'] for r in rows))
def longest(pred):
 groups=[];start=None;last=None
 for r in rows:
  if pred(r):
   if start is None:start=r['t']
   last=r['t']
  elif start is not None:groups.append((last-start,start,last));start=None
 if start is not None:groups.append((last-start,start,last))
 return sorted(groups,reverse=True)[:8]
print('Geometry and speed only spans',longest(lambda r:r['xy']<=.06 and r['speed']<.1))
print('All gate basic criteria spans',longest(lambda r:not r['failures']))
logs=read('rosout_agg');pat=re.compile(r'high alignment waiting: xy=([\d.]+) speed=([\d.]+) valid=(\d)')
print('ROSLOG vs BAG RECONSTRUCTION')
errors=[]
for r in logs:
 m=pat.search(r['m']['msg'])
 if not m or r['t']<276:continue
 rr=min(rows,key=lambda a:abs(a['t']-r['t']));errors.append(abs(rr['xy']-float(m[1])))
 print(f"{r['t']:.3f} log_xy={m[1]} log_v={m[2]} approx_xy={rr['xy']:.4f} approx_v={rr['speed']:.4f} image_age={rr['image_age']:.3f} fail={rr['failures']}")
print('xy log fit abs median/p95',np.percentile(errors,[50,95]))