from pathlib import Path
import rosbag,numpy as np,json
R=Path(__file__).resolve().parents[3];D=R/'logs/flight_delivery_20260921'
for hour in ('22','23'):
 p=next(D.glob('corridor_diag_20260920_'+hour+'*/corridor*/navigation_0.bag'))
 with rosbag.Bag(str(p)) as b:
  t0=b.get_start_time();splines=[];states=[];cmd=[]
  for topic,m,t in b.read_messages(topics=['/planning/bspline','/mavros/state','/planning/pos_cmd']):
   if topic=='/planning/bspline':splines.append(dict(t=t.to_sec()-t0,knots=list(m.knots),pts=[[v.x,v.y,v.z] for v in m.pos_pts],id=m.traj_id))
   if topic=='/mavros/state' and (not states or states[-1][1:]!=[m.mode,m.armed]):states.append([t.to_sec()-t0,m.mode,m.armed])
   if topic=='/planning/pos_cmd':cmd.append([t.to_sec()-t0,m.position.x,m.position.y,m.position.z,m.velocity.x,m.velocity.y,m.velocity.z,m.trajectory_id])
 (D/('extra'+hour+'.json')).write_text(json.dumps(dict(splines=splines,states=states,cmd=cmd)))
 print(hour,states)
 mapbag=next((R/'试飞产物').glob('2026-09-20-'+hour+'*.bag'));snap={}
 with rosbag.Bag(str(mapbag)) as b:
  for topic,m,t in b.read_messages(topics=['/freedom/static_pointcloud','/sdf_map/occupancy_inflate']):
   if t.to_sec()>t0+52:continue
   dtype=np.dtype(dict(names=['x','y','z'],formats=['<f4']*3,offsets=[next(f.offset for f in m.fields if f.name==k) for k in ('x','y','z')],itemsize=m.point_step))
   ar=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
   xyz=np.column_stack([ar[k].ravel() for k in ('x','y','z')]);snap[topic]=(xyz,t.to_sec(),m.header.frame_id)
 for topic,(xyz,t,frame) in snap.items():
  label='raw' if 'freedom' in topic else 'inflate';np.save(D/(hour+'_'+label+'.npy'),xyz)
  print(hour,label,t-t0,frame,len(xyz))
