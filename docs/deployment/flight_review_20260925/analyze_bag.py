"""Offline bag extraction; never publishes ROS messages or starts flight nodes."""
import json
import math
from pathlib import Path
from collections import defaultdict

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent

def plain(m):
    if hasattr(m, '__slots__'):
        return {k: plain(getattr(m, k)) for k in m.__slots__}
    if isinstance(m, (tuple, list)):
        return [plain(v) for v in m]
    return m

def extract():
    import rosbag
    bag = next((ROOT/'试飞产物').glob('flight_debug_2026-09-25*.bag'))
    events=[]; motion=[]; candidates=[]; detections=[]; states=[]; status=[]
    selected=[]; tf=[]; ready=[]; modes=[]
    with rosbag.Bag(str(bag)) as b:
        start=b.get_start_time()
        for topic,m,t in b.read_messages():
            dt=t.to_sec()-start
            if topic == '/mavros/local_position/odom':
                p=m.pose.pose.position; v=m.twist.twist.linear
                motion.append(dict(t=dt,stamp=m.header.stamp.to_sec(),x=p.x,y=p.y,z=p.z,vx=v.x,vy=v.y,vz=v.z,vxy=math.hypot(v.x,v.y),orientation=plain(m.pose.pose.orientation),frame=m.header.frame_id,child=m.child_frame_id))
            elif topic in ['/navigation/mission_command_raw','/navigation/mission_result','/mission/release_result','/mission/command','/mission/release_commitment_evidence','/fastplanner/goal']:
                events.append(dict(t=dt,topic=topic,msg=plain(m)))
            elif topic == '/uav_vision/targets':
                for q in m.targets:
                    candidates.append(dict(t=dt,**plain(q)))
            elif topic.startswith('/uav_vision/detections'):
                for q in m.detections:
                    detections.append(dict(t=dt,topic=topic,source=m.source,**plain(q)))
            elif topic == '/uav_vision/selected_target':
                selected.append(dict(t=dt,**plain(m)))
            elif topic == '/uav_vision/align_mode':
                if not modes or modes[-1]['mode']!=m.data:
                    modes.append(dict(t=dt,mode=m.data))
            elif topic == '/mavros/state':
                states.append(dict(t=dt,**plain(m)))
            elif topic == '/navigation/mission_status':
                status.append(dict(t=dt,**json.loads(m.data)))
            elif topic == '/tf_static':
                tf.append(plain(m))
            elif topic == '/uav_vision/drop_ready' and getattr(m,'ready',False):
                ready.append(dict(t=dt,**plain(m)))
    data=dict(start=start,bag=str(bag),motion=motion,events=events,candidates=candidates,detections=detections,selected=selected,states=states,status=status,tf_static=tf,ready=ready,modes=modes)
    # Detailed observations remain a local log artifact, not a repository asset.
    local=ROOT/'logs'/'flight_review_20260925'; local.mkdir(parents=True,exist_ok=True)
    (local/'extracted.json').write_text(json.dumps(data,ensure_ascii=False))
    first_panzer=next(r for r in detections if r['topic']=='/uav_vision/detections' and r['class_name']=='panzer')
    image_stamp=first_panzer['header']['stamp']
    wanted=image_stamp['secs']+image_stamp['nsecs']*1e-9
    closest=None
    with rosbag.Bag(str(bag)) as b:
        for _,msg,_ in b.read_messages(topics=['/camera/image_raw/compressed']):
            delta=abs(msg.header.stamp.to_sec()-wanted)
            if closest is None or delta<closest[0]: closest=(delta,msg)
    (OUT/'panzer_first.jpg').write_bytes(bytes(closest[1].data))
    print('PANZER IMAGE MATCH delta',closest[0], 'format',closest[1].format)
    groups=defaultdict(list)
    for q in detections:
        groups[(q['topic'],q['class_name'])].append(q)
    print('DETECTIONS')
    for (topic,cl),rows in sorted(groups.items()):
        good=[r for r in rows if r['map_valid']]
        rejects=defaultdict(int)
        for r in rows: rejects[r['reject_reason']]+=1
        print(topic,cl,len(rows),'first/last',round(rows[0]['t'],3),round(rows[-1]['t'],3),'map_valid',len(good),'first_valid',round(good[0]['t'],3) if good else None,'rejects',dict(rejects))
    groups=defaultdict(list)
    for q in candidates: groups[(q['class_name'],q['id'])].append(q)
    print('CANDIDATES')
    for key,rows in sorted(groups.items()):
        good=[r for r in rows if r['map_valid']]
        transitions=[]; prev=None
        for r in rows:
            val=(r['state'],r['map_valid'],r['reject_reason'])
            if val!=prev: transitions.append((round(r['t'],3),val,r['consecutive_observe_count'])); prev=val
        print(key,'n',len(rows),'first/last',rows[0]['t'],rows[-1]['t'],'max_count',max(r['observe_count'] for r in rows),'max_consecutive',max(r['consecutive_observe_count'] for r in rows),'transitions',transitions)
    print('STATUS TRANSITIONS')
    prev=None
    for r in status:
        val=(r.get('phase'),r.get('active_command'),r.get('committed_slots'),r.get('slot_status'))
        if val!=prev: print(round(r['t'],3),val,r.get('last_reason')); prev=val
    print('FC STATES')
    prev=None
    for r in states:
        val=(r['armed'],r['mode'])
        if val!=prev: print(round(r['t'],3),val); prev=val
    print('TF STATIC',json.dumps(tf))

if __name__=='__main__':
    extract()
