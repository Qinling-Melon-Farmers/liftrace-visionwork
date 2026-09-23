from pathlib import Path
import csv,json,numpy as np
r=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search')
d=r/'docs/verification/failed_three_20260923'
state=json.loads((d/'selected_matrix.json').read_text())
out=[]
for it in state['results']:
 run=Path(it['run']); contact=json.loads((run/'gazebo_contact_status.json').read_text()).get('events',[])
 times=[contact[0]['ros_stamp']] if contact else [34.45,99.3,111.41]
 samples={}
 for name in ['truth_pose','mavros_pose','lio_pose','mavros_setpoint','planner_setpoint']:
  rows=list(csv.DictReader((run/(name+'.csv')).open()))
  samples[name]=[{**min(rows,key=lambda p:abs(float(p['t'])-t)),'requested_t':t} for t in times] if rows else []
 ev=[json.loads(s) for s in (run/'key_events.jsonl').read_text().splitlines()]
 dec=[e for e in ev if e['kind']=='decision']
 out.append(dict(seed=it['seed'],samples=samples,decisions=dec[-5:]))

(d/'terminal_samples.json').write_text(json.dumps(out,indent=2))
