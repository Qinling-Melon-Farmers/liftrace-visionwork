from pathlib import Path
import json, rosbag
import argparse
p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('out',type=Path);args=p.parse_args();bag=args.bag;out=args.out;out.mkdir(parents=True,exist_ok=True)
def conv(v):
    if hasattr(v,'to_sec'): return v.to_sec()
    if hasattr(v,'__slots__'): return {k:conv(getattr(v,k)) for k in v.__slots__}
    if isinstance(v,(list,tuple)): return [conv(x) for x in v]
    if isinstance(v,bytes): return '<bytes>'
    return v

with rosbag.Bag(str(bag)) as b:
 start=b.get_start_time(); ts=b.get_type_and_topic_info().topics
 chosen=[k for k,v in ts.items() if any(s in v.msg_type for s in ['TargetDetectionArray','TargetCandidateArray','NavigationHint'])]
 print([(k,ts[k].msg_type,ts[k].message_count) for k in chosen])
 with (out/'detections.jsonl').open('w') as f:
  first=set()
  for topic,m,t in b.read_messages(topics=chosen):
   d=conv(m);r={'t':t.to_sec()-start,'topic':topic,'msg':d};f.write(json.dumps(r)+'\n')
   if topic not in first:print(topic,json.dumps(d)[:6000]);first.add(topic)
print(out/'detections.jsonl')