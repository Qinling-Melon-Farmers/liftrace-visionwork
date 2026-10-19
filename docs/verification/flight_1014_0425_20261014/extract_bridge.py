"""Extract extra bridge/FOV topics and selected JPEGs without starting ROS."""
from pathlib import Path
import argparse, json, collections
import rosbag
ap=argparse.ArgumentParser();ap.add_argument('--bag',type=Path,required=True);ap.add_argument('--run-dir',type=Path,required=True);args=ap.parse_args()
out=args.run_dir/'numeric';out.mkdir(parents=True,exist_ok=True)
frames=args.run_dir/'bridge_review';frames.mkdir(parents=True,exist_ok=True)
def plain(m):
 if hasattr(m,'to_sec'):return m.to_sec()
 if hasattr(m,'__slots__'):return {k:plain(getattr(m,k)) for k in m.__slots__}
 if isinstance(m,(tuple,list)):return [plain(x) for x in m]
 if isinstance(m,bytes):return '<bytes>'
 return m
extra=['/uav_vision/detections','/uav_vision/detections_resolved','/uav_vision/navigation_hints','/uav_vision/targets','/tf']
cam='/camera/camera_info';image='/board_trials/recording/image/compressed'
rows=collections.defaultdict(list);first=None;images=[];ci_versions=set()
with rosbag.Bag(str(args.bag)) as bag:
 begin=bag.get_start_time()
 for topic,m,t in bag.read_messages(topics=extra+[cam,image]):
  rel=t.to_sec()-begin
  if topic==image:
   if 163<rel<220:images.append((rel,m.header.stamp.to_sec()-begin,bytes(m.data)))
   continue
  r={'t':rel,'stamp':t.to_sec(),'m':plain(m)}
  if topic==cam:
   if first is None:first=r
   ci_versions.add((m.width,m.height,tuple(m.K),tuple(m.D)));continue
  rows[topic].append(r)
for topic,data in rows.items():(out/(topic.strip('/').replace('/','__')+'.json')).write_text(json.dumps(data,ensure_ascii=False),encoding='utf8')
if first:(out/'camera__camera_info_first.json').write_text(json.dumps(first,indent=2),encoding='utf8')
meta=[]
for wanted in [165.75,184.0,217.3]:
 rel,src,data=min(images,key=lambda v:abs(v[0]-wanted));name='raw_'+str(wanted).replace('.','_')+'.jpg';(frames/name).write_bytes(data);meta.append({'requested':wanted,'receipt':rel,'source':src,'path':name})
(frames/'selected_frames.json').write_text(json.dumps(meta,indent=2),encoding='utf8')
print('Camera K/D/size versions:',len(ci_versions),'extra topics:',len(rows),'selected frames:',len(meta))