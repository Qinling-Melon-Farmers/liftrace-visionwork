from pathlib import Path
import json,csv,bisect
h=Path(__file__).resolve().parents[4];d=h/'docs/verification/latest_six_20261005';out=[]
for m in json.loads((d/'metrics.json').read_text()):
 if not m['corridor_geometry']['samples']:continue
 r=Path(m['run']);events=[json.loads(l) for l in (r/'key_events.jsonl').read_text().splitlines()];ret=min(e['ros_sec'] for e in events if e['kind']=='decision' and e['data']['command']==4)
 poses=[{k:float(v) for k,v in x.items()} for x in csv.DictReader((r/'truth_pose.csv').open())];rows=[x for x in poses if 8<=x['x']<=9.5 and -5<=x['y']<=5 and x['t']>ret];peak=max(rows,key=lambda x:x['z']);t=peak['t'];v=dict(seed=m['seed'],variant=m['label'],peak_ros_s=t,truth_agl_m=peak['z']+.22,streams={})
 for stream in ('mavros_pose','lio_pose','mavros_setpoint','planner_setpoint'):
  a=list(csv.DictReader((r/(stream+'.csv')).open()));p=min(a,key=lambda x:abs(float(x['t'])-t));v['streams'][stream]=dict(agl_m=float(p['z'])+.22,sample_offset_s=float(p['t'])-t)
 v['decision']=next(e['data'] for e in reversed(events) if e['kind']=='decision' and e['ros_sec']<=t)
 out.append(v)
(d/'corridor_peak_context.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))