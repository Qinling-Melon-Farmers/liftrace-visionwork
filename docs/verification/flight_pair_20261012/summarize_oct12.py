import json,re
from pathlib import Path
out=Path('/home/xhj/liftrace-deliverables/flight_review_20261012')
for key in ['bag6','full']:
 j=json.load(open(out/key/'analysis.json'));meta=next(m['data'] for m in reversed(j['metadata']) if m['data'].get('ground_reference'));g=meta['ground_reference']['ground_z']
 events=[];active=None
 for r in j['logs']:
  s=r['text'];t=r['t']
  if 'External ALIGN target=' in s:
   active=dict(class_name=re.search(r'class=(\w+)',s)[1],align=t);events.append(active)
  if not active:continue
  for needle,name in [('frozen slot=','freeze'),('descent_started slot=','descent'),('async Servo submitted','request'),('received positive Servo ACK','ack'),('External ALIGN completed','recover')]:
   if needle in s and name not in active:active[name]=t
  if '[DropGeometry] waiting:' in s:
   m=re.search(r'xy=([\d.-]+).*fc_z=([\d.-]+).*floor_z=([\d.-]+).*ceiling_z=([\d.-]+).*xy_tolerance=([\d.-]+)',s)
   if m:
    xy,z,lo,hi,tol=map(float,m.groups());active.setdefault('waits',[]).append(dict(t=t,xy=xy,agl=z-g,height_ok=lo<=z<=hi,xy_ok=xy<tol))
 for e in events:
  e['align_to_ack']=e.get('ack',0)-e['align']
  e['segments']={f'{a}_to_{b}':e[b]-e[a] for a,b in [('align','freeze'),('freeze','descent'),('descent','request'),('request','ack')] if a in e and b in e}
  w=[v for v in e.get('waits',[]) if e.get('descent',1e9)<=v['t']<e.get('request',0)]
  e['waiting_samples_after_descent']=dict(total=len(w),height_only=sum(not v['height_ok'] and v['xy_ok'] for v in w),xy_only=sum(v['height_ok'] and not v['xy_ok'] for v in w),both_fail=sum(not v['height_ok'] and not v['xy_ok'] for v in w))
 print(key,'ground',g,'DEPLOYMENT',meta.get('deployment'),'MODEL',meta.get('model_path'))
 for e in events:print({k:v for k,v in e.items() if k!='waits'})
 json.dump(dict(source=j['source'],ground_z=g,metadata=meta,events=events),open(out/key/'timing.json','w'),ensure_ascii=False,indent=2)