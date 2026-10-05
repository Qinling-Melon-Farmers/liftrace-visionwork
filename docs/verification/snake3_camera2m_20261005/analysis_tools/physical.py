from pathlib import Path
import json,csv
h=Path(__file__).resolve().parents[4];d=h/'docs/verification/snake3_camera2m_20261005';out=[]
for x in json.loads((d/'metrics.json').read_text()):
 r=Path(x['run']);g=json.loads((r/'gazebo_contact_status.json').read_text());ev=[json.loads(t) for t in (r/'key_events.jsonl').read_text().splitlines()];rows=list(csv.DictReader((r/'truth_pose.csv').open()))
 land=min((e['ros_sec'] for e in ev if e['kind']=='decision' and e['data']['command']==5),default=None)
 support=next((e for e in g['support_events'] if land is not None and e['ros_stamp']>=land),None)
 abort=next((e for e in ev if e['kind']=='decision' and e['data']['command']==7),None)
 v=dict(seed=x['seed'],variant=x['label'],raw_gate=x['status'],collision_count=g['actual_collision_count'])
 if support:
  t=support['ros_stamp'];vals=[tuple(float(p[k]) for k in ['x','y','z']) for p in rows if float(p['t'])>=t+.5]
  spread=[max(z[i] for z in vals)-min(z[i] for z in vals) for i in range(3)] if vals else []
  v.update(support_ros_s=t,touchdown_mission_s=t-x['start_ros_s'],abort_after_support_s=None if not abort else abort['ros_sec']-t,terminal_reason=None if not abort else abort['data']['reason'],tail_xyz_range_m=spread,final_truth_xy=vals[-1][:2] if vals else None,physical_landing_accepted=g['actual_collision_count']==0 and bool(vals) and all(s<.02 for s in spread))
 out.append(v)
(d/'physical_completion.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
