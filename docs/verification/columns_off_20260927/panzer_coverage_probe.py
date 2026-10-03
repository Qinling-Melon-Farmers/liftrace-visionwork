from pathlib import Path
import csv,json,numpy as np,cv2,yaml
from scipy.spatial.transform import Rotation
R=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');r=R/'logs/columnsoff_seed31_20260927_120609'
cam=json.loads((r/'actual_camera_info.json').read_text());K=np.array(cam['K']).reshape(3,3)
target=next(t for t in yaml.safe_load((r/'random_field_truth.yaml').read_text())['targets'] if t['class']=='panzer')
rot=Rotation.from_euler('z',target['yaw']).as_matrix()
points=np.vstack([np.zeros(3),np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])@rot.T])+[target['world_x'],target['world_y'],.01]
rows=list(csv.DictReader((r/'truth_pose.csv').open()));samples=[]
for row in rows:
 t=float(row['t']);pos=np.array([float(row[k]) for k in ['x','y','z']]);pos[2]+=.22
 if t>61.84 or pos[2]<2.4:continue
 rb=Rotation.from_quat([float(row[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
 center=pos+rb@np.array([0,0,-.16])
 rc=rb@Rotation.from_quat([0,1,0,0]).as_matrix()
 pc=(points-center)@rc
 uv=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),K,np.array(cam['D']))[0].reshape(-1,2)
 inside=(pc[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<cam['width'])&(uv[:,1]>=0)&(uv[:,1]<cam['height'])
 samples.append([t,bool(inside[0]),bool(inside.all()),uv[0].tolist()])
result=dict(scope='truth attitude pinhole+reported distortion, FC camera offset -0.16m; no occlusion or detector guarantee',target=target,high_samples=len(samples),center_inside_samples=sum(s[1] for s in samples),full_board_inside_samples=sum(s[2] for s in samples),center_inside_ros_times=[s[0] for s in samples if s[1]])
(R/'logs/columns_off_20260927_batch/panzer_coverage.json').write_text(json.dumps(result,indent=2))
print({k:v for k,v in result.items() if k!='center_inside_ros_times'})
print('center time bounds',min(result['center_inside_ros_times'],default=None),max(result['center_inside_ros_times'],default=None))