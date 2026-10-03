from pathlib import Path
import json,csv,collections,numpy as np
R=Path(__file__).resolve().parents[3];state=json.loads((R/'logs/failed_six_20260921_batch/matrix.json').read_text())
for row in state['results']:
 if row['seed'] not in [34,38,40]:continue
 r=Path(row['run']);ev=[json.loads(v) for v in (r/'key_events.jsonl').read_text().splitlines() if v]
 hi=json.loads((r/'high_view_full_events.jsonl').read_text().splitlines()[-1])['status']
 print('SEED',row['seed'],'low_limits',hi.get('low_limits_applied'),'ascent_verified',hi.get('ascent_verified'))
 for p in sorted(r.glob('local_map_failure_*.json')):
  v=json.loads(p.read_text());print(p.name,'keys',list(v),'t',v.get('ros_sec'),'goal',v.get('goal'),'clouds',[(k,len(c.get('points',[])),c.get('stamp')) for k,c in v.get('clouds',{}).items()])
 if row['seed']==34:
  for e in ev:
   if e['kind']=='planner' and (e['ros_sec']<36 or e['data'].get('reason') in ['server_hold_budget_exhausted','liveness_budget_exhausted']):
    d=e['data'];print('PLAN',e['ros_sec'],d['goal_seq'],d['status'],d['reason'],d['requested_goal']['pose']['position'],d['effective_goal']['pose']['position'])
 else:
  contact=json.loads((r/'gazebo_contact_status.json').read_text())['events'][0]['ros_stamp']
  for e in ev:
   if e['kind']=='decision' or (e['kind']=='result' and e['data'].get('terminal')):
    if e['ros_sec']>=contact-25:print('NEAR',e['ros_sec'],e['kind'],e['data'].get('command'),e['data'].get('reason'),e['data'].get('target_class'),e['data'].get('goal_x'),e['data'].get('goal_y'))
  for name in ['truth_pose.csv','lio_pose.csv','mavros_pose.csv','mavros_setpoint.csv']:
   with (r/name).open() as f:records=list(csv.DictReader(f))
   for t in [contact-1,contact]:
    v=min(records,key=lambda v:abs(float(v['t'])-t));print(name,t,[v[k] for k in ['t','x','y','z']])
