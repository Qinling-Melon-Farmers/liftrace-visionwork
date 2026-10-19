import os,sys,json,math
from pathlib import Path
sys.dont_write_bytecode=True
import rosbag,genpy,numpy as np
root=Path('/home/xhj/liftrace-deliverables/flight_review_20261012')
source=root/'full/source_reindexed.bag'
out=root/'corridor_geometry_data'
out.mkdir(exist_ok=True)
def cv(o):
    if isinstance(o,float) and not math.isfinite(o):return None
    if isinstance(o,(genpy.Time,genpy.Duration)):return o.to_sec()
    if hasattr(o,'__slots__'):return {s:cv(getattr(o,s)) for s in o.__slots__}
    if isinstance(o,(list,tuple)):return [cv(x) for x in o]
    if isinstance(o,bytes):return None
    return o
records=[];clouds=[];arrays={}
with rosbag.Bag(str(source),'r') as bag:
    start=bag.get_start_time()
    topics=['/sdf_map/occupancy_inflate','/fastplanner/goal','/planning_vis/trajectory','/mission/command','/Odometry']
    for topic,msg,t in bag.read_messages(topics=topics,start_time=genpy.Time.from_sec(start+235),end_time=genpy.Time.from_sec(start+272.3)):
        rel=t.to_sec()-start
        if topic=='/sdf_map/occupancy_inflate':
            names={f.name:f for f in msg.fields}
            assert all(names[v].datatype==7 for v in ['x','y','z']),names
            dtype=np.dtype({'names':['x','y','z'],'formats':['>f4' if msg.is_bigendian else '<f4']*3,'offsets':[names[v].offset for v in ['x','y','z']],'itemsize':msg.point_step})
            a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
            xyz=np.stack([a[v] for v in ['x','y','z']],axis=1);xyz=xyz[np.isfinite(xyz).all(axis=1)]
            # Keep the corridor and walls nearby, in the message's own frame.
            xyz=xyz[(xyz[:,0]>=5.5)&(xyz[:,0]<=9.5)&(xyz[:,1]>=-2.5)&(xyz[:,1]<=3.5)]
            key='cloud_%03d'%len(clouds);arrays[key]=xyz
            clouds.append(dict(t=rel,header_stamp_sec=msg.header.stamp.to_sec(),stamp=(msg.header.stamp.to_sec()-start if not msg.header.stamp.is_zero() else None),frame=msg.header.frame_id,key=key,total=msg.width*msg.height,kept=len(xyz)))
        else:records.append(dict(t=rel,topic=topic,data=cv(msg)))
np.savez_compressed(out/'inflated_clouds.npz',**arrays)
(out/'bag_extract.json').write_text(json.dumps(dict(source=str(source),start=start,clouds=clouds,records=records),ensure_ascii=False),encoding='utf8')
print(json.dumps(dict(clouds=len(clouds),records=len(records),cloud_frames=sorted(set(c['frame'] for c in clouds)),cloud_first=clouds[:2],cloud_last=clouds[-2:]),indent=2))
