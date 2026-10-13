from pathlib import Path
import rosbag,json,collections
import argparse
p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True);args=p.parse_args()
f=args.bag;out=args.out_dir;out.mkdir(parents=True,exist_ok=True)
def obj(m):
 if hasattr(m,'to_sec'): return m.to_sec()
 if hasattr(m,'__slots__'): return {k:obj(getattr(m,k)) for k in m.__slots__}
 if isinstance(m,(tuple,list)): return [obj(x) for x in m]
 if isinstance(m,bytes): return '<bytes>'
 return m
wanted=['/board_trials/run_metadata','/fastplanner/goal','/mission/command','/mission/release_result','/navigation/mission_command_raw','/navigation/mission_result','/navigation/mission_status','/navigation/planner_bridge_status','/uav_high_view/probe_status','/planning/goal_status','/planning/progress','/planning/bspline','/planning/pos_cmd','/mavros/local_position/pose','/navigation/local_pose','/mavros/local_position/velocity_local','/navigation/setpoint_mission','/mavros/setpoint_position/local','/mavros/state','/mavros/extended_state','/tf_static','/rosout_agg','/laserMapping/realtime']
data=collections.defaultdict(list)
with rosbag.Bag(str(f)) as b:
 start=b.get_start_time()
 for topic,m,t in b.read_messages(topics=wanted):
  row={'t':t.to_sec()-start,'stamp':t.to_sec(),'m':obj(m)}
  if m._type=='std_msgs/String':
   try:row['m']=json.loads(m.data)
   except ValueError: pass
  data[topic].append(row)
for topic,rows in data.items():
 (out/(topic.strip('/').replace('/','__')+'.json')).write_text(json.dumps(rows,ensure_ascii=False),encoding='utf-8')

import numpy as np
times=[179.75,182.54,190.24,194.63,197.65]; picked={}
with rosbag.Bag(str(f)) as bag:
 start=bag.get_start_time()
 for topic,m,t in bag.read_messages(topics=['/sdf_map/occupancy_inflate']):
  tr=t.to_sec()-start
  if not 178<tr<199:continue
  for target in times:
   if target not in picked or abs(tr-target)<abs(picked[target][0]-target):picked[target]=(tr,m)
arrays={}; metadata=[]
for i,(target,(tr,m)) in enumerate(picked.items()):
 fields={x.name:x for x in m.fields}; dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k].offset for k in ['x','y','z']],'itemsize':m.point_step})
 a=np.frombuffer(m.data,dtype=dtype); xyz=np.column_stack([a[k] for k in ['x','y','z']]); valid=np.isfinite(xyz).all(axis=1)&(xyz[:,0]>6)&(xyz[:,0]<9)&(xyz[:,1]>-4)&(xyz[:,1]<2.5)&(xyz[:,2]>-.2)&(xyz[:,2]<1.4)
 arrays['cloud_'+str(i)]=xyz[valid]; metadata.append({'key':'cloud_'+str(i),'t':tr,'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id,'points_total':len(xyz),'points_roi':int(valid.sum())})
np.savez_compressed(str(out/'corridor_clouds.npz'),**arrays);(out/'corridor_clouds.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata,indent=2))
print("Extracted",len(data),"topics to",out)
