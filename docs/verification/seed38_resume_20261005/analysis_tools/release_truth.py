from pathlib import Path
import json,csv,math
H=Path(__file__).resolve().parents[4];D=H/'docs/verification/seed38_resume_20261005'
results=[]
for m in json.loads((D/'metrics.json').read_text()):
 run=Path(m['run']);truth=json.loads((D/(str(m['seed'])+'_'+m['label']+'_centers')/'center_summary.json').read_text())['truth'];poses=list(csv.DictReader((run/'truth_pose.csv').open()));drops=[]
 for event in m['releases']:
  t=event['ros_sec'];p=min(poses,key=lambda r:abs(float(r['t'])-t));xy=[float(p['x']),float(p['y'])];tar=min(truth,key=lambda q:math.dist(xy,q['world_xy']))
  drops.append(dict(class_name=event['data']['target_class'],slot=event['data']['payload_slot'],ros_s=t,body_xy=xy,nearest_class=tar['class_name'],nearest_distance_m=math.dist(xy,tar['world_xy']),label_matches=event['data']['target_class']==tar['class_name'],pose_age_s=abs(float(p['t'])-t)))
 c={}
 for fname in ['planner_setpoint','mavros_setpoint','mavros_pose','truth_pose']:
  rows=[r for r in csv.DictReader((run/(fname+'.csv')).open()) if float(r['x'])>=8 and 120<float(r['t'])<(m.get('land_command_mission_s') or 9999)+m['start_ros_s']]
  c[fname]=dict(n=len(rows),max_recorded_z=max((float(r['z']) for r in rows),default=None))
 results.append(dict(seed=m['seed'],variant=m['label'],drops=drops,corridor_raw_coordinate_diagnostic=c))
(D/'release_truth.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))