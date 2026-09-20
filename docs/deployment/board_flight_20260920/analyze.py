from pathlib import Path
import csv,json,collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
B=Path(__file__).resolve().parents[3];run=B/'logs/visual_flight_review_20260920/board_visual_interrupt_20260920_120748';D=B/'docs/deployment/board_flight_20260920';D.mkdir(parents=True,exist_ok=True)
ref=json.loads((run/'ground_reference.json').read_text());rows=list(csv.DictReader((run/'navigation_pose.csv').open()));a=np.array([[float(r[k]) for k in ('t','x','y','z')] for r in rows]);agl=a[:,3]-ref['ground_z'];events=[json.loads(line) for line in (run/'vision_events.jsonl').read_text().splitlines()];missions=[e['data'] for e in events if e['kind']=='mission'];result=json.loads((run/'result.json').read_text())
summary=dict(run=str(run),samples=len(a),recording_duration_s=float(a[-1,0]-a[0,0]),maximum_fc_agl_m=float(agl.max()),xy_span_m=np.ptp(a[:,1:3],axis=0).tolist(),mission_phases=sorted(set(m.get('phase','') for m in missions)),mission_ids=sorted(set(m.get('mission_id','') for m in missions)),mock_calls=result['mock_service_calls'],ground_z=ref['ground_z'])
for lo,hi in [(.35,.8),(.4,1.),(.4,1.2),(.4,1.35)]:
 idx=np.flatnonzero(agl>=lo);jdx=np.flatnonzero(agl>=hi)
 if len(idx) and len(jdx) and jdx[0]>idx[0]:
  t1,t2=a[idx[0],0],a[jdx[0],0];summary[f'climb_{lo}_{hi}']=dict(start=float(t1),end=float(t2),duration=float(t2-t1),average_mps=float((hi-lo)/(t2-t1)))
elevated=agl>.4
summary['max_xy_distance_from_initial_m']=float(np.linalg.norm(a[:,1:3]-np.array(ref['fc_xyz'][:2]),axis=1).max())
fig,axs=plt.subplots(2,1,figsize=(10,8));axs[0].plot(a[:,0]-a[0,0],agl,label='Measured FC AGL');axs[0].axhline(1.4,color='orange',ls='--',label='Requested 1.4 m');axs[0].set(xlabel='Recorded time (s)',ylabel='FC AGL (m)',title='Actual board flight; mission stayed IDLE');axs[0].legend();axs[0].grid(alpha=.3)
axs[1].plot(a[:,1],a[:,2],label='Recorded XY');axs[1].scatter(*ref['fc_xyz'][:2],color='black',label='Ground reference');axs[1].set(xlabel='Mission X (m)',ylabel='Mission Y (m)');axs[1].axis('equal');axs[1].grid(alpha=.3);axs[1].legend();fig.tight_layout();fig.savefig(D/'flight_height_xy.png',dpi=150);plt.close(fig)
(D/'metrics.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
