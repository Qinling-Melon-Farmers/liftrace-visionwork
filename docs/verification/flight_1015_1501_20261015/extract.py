from pathlib import Path
import rosbag, json, collections
import argparse
p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('out',type=Path);args=p.parse_args()
bag=args.bag;out=args.out
out.mkdir(exist_ok=True)
def conv(v):
    if hasattr(v,'to_sec'): return v.to_sec()
    if hasattr(v,'__slots__'): return {k:conv(getattr(v,k)) for k in v.__slots__}
    if isinstance(v,(list,tuple)): return [conv(x) for x in v]
    if isinstance(v,bytes): return '<bytes>'
    return v
skip={'sensor_msgs/CompressedImage','sensor_msgs/PointCloud2','tf2_msgs/TFMessage','sensor_msgs/CameraInfo','uav_vision/TargetDetectionArray','uav_vision/TargetCandidateArray','uav_vision/TargetCandidate'}
with rosbag.Bag(str(bag)) as b:
    start=b.get_start_time()
    info={'bag':str(bag),'start':start,'end':b.get_end_time(),'topics':{k:{'type':v.msg_type,'count':v.message_count} for k,v in b.get_type_and_topic_info().topics.items()}}
    (out/'bag_info.json').write_text(json.dumps(info,indent=2))
    f=(out/'records.jsonl').open('w'); logs=(out/'rosout.txt').open('w'); meta=[]
    for topic,m,t in b.read_messages():
        sec=t.to_sec()-start
        if m._type not in skip:
            d=conv(m)
            f.write(json.dumps({'t':sec,'topic':topic,'msg':d},ensure_ascii=False)+'\n')
        if topic=='/rosout_agg': logs.write(f'{sec:9.3f} [{m.level}] {m.name}: {m.msg}\n')
        if topic=='/board_trials/run_metadata':
            try: meta.append({'t':sec,'data':json.loads(m.data)})
            except Exception: meta.append({'t':sec,'data':m.data})
    f.close(); logs.close()
    (out/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
print(out)
