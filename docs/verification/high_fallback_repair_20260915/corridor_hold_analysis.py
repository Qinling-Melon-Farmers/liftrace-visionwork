import json,csv
from pathlib import Path
import numpy as np
d=Path(__file__).resolve().parent;r=d.parents[2]
matrix=json.loads((d/'matrix.json').read_text());out=[]
for row in matrix['results']:
    if row['seed'] not in (31,34):continue
    p=Path(row['run']);events=[json.loads(l) for l in (p/'key_events.jsonl').read_text().splitlines()]
    e=next(e for e in events if e['kind']=='decision' and e['data'].get('reason','').startswith('post_delivery_route:8/9:'))
    start=e['ros_sec'];end=e['data']['deadline']['stamp_ns']/1e9
    def data(name):
        with (p/name).open() as f:
            reader=csv.reader(f);next(reader);return np.array([[float(v) for v in row] for row in reader])
    a=data('planner_setpoint.csv');a=a[(a[:,0]>=start)&(a[:,0]<end-.05)]
    last=a[-1,1:4];changes=np.flatnonzero(np.linalg.norm(a[:,1:4]-last,axis=1)>.01)
    i=int(changes[-1]+1) if len(changes) else 0
    odom=data('lio_pose.csv');near=odom[(odom[:,0]>=a[i,0]-.3)&(odom[:,0]<=a[i,0]+.3)]
    errors=np.linalg.norm(near[:,1:4]-last,axis=1);j=errors.argmin()
    item=dict(seed=row['seed'],goal_issued_s=start,deadline_s=end,command_within_1cm_of_terminal_from_s=float(a[i,0]),within_1cm_duration_s=float(a[-1,0]-a[i,0]),held_xyz=last.tolist(),nearest_lio_stamp=float(near[j,0]),nearest_lio_error_m=float(errors[j]),goal_xyz=e['data']['goal']['pose']['position'])
    late=a[a[:,0]>start+15]
    item['late_command_xyz_min']=late[:,1:4].min(axis=0).tolist();item['late_command_xyz_max']=late[:,1:4].max(axis=0).tolist()
    item['late_fraction_within_1cm']=float((np.linalg.norm(late[:,1:4]-last,axis=1)<.01).mean())
    out.append(item)
(d/'corridor_hold_evidence.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
