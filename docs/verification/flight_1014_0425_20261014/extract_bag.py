from pathlib import Path
import rosbag,json,collections
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--bag',type=Path,required=True);ap.add_argument('--out-dir',type=Path,required=True);args=ap.parse_args()
f=args.bag;out=args.out_dir;out.mkdir(parents=True,exist_ok=True)
def obj(m):
 if hasattr(m,'to_sec'): return m.to_sec()
 if hasattr(m,'__slots__'): return {k:obj(getattr(m,k)) for k in m.__slots__}
 if isinstance(m,(tuple,list)): return [obj(x) for x in m]
 if isinstance(m,bytes): return '<bytes>'
 return m
wanted=['/board_trials/run_metadata','/fastplanner/goal','/mission/command','/mission/release_result','/navigation/mission_command_raw','/navigation/mission_result','/navigation/mission_status','/navigation/planner_bridge_status','/uav_high_view/probe_status','/planning/goal_status','/planning/progress','/planning/bspline','/planning/pos_cmd','/mavros/local_position/pose','/navigation/local_pose','/mavros/local_position/velocity_local','/navigation/setpoint_mission','/mavros/setpoint_position/local','/mavros/state','/mavros/extended_state','/tf_static','/rosout_agg','/laserMapping/realtime']
wanted += ['/Odometry','/mavros/local_position/odom','/navigation/local_odom','/mavros/setpoint_raw/target_local','/patrol_control/external_landing_handoff','/uav_vision/detections_mapped','/uav_vision/detections_refined','/uav_vision/align_mode']
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

print("Extracted",len(data),"topics to",out)
