from pathlib import Path
import rosbag,numpy as np,json
R=Path(__file__).resolve().parents[3]
D=R/'logs/flight_delivery_20260921'
out=D/'cloud_sequence';out.mkdir(exist_ok=True)
p=next((R/'试飞产物').glob('2026-09-20-23*.bag'))
t0=1789917118.88
times=[35,40,45,48,51,54,56,60,65]
snap={};counts=[]
with rosbag.Bag(str(p)) as b:
 print({k:(v.msg_type,v.message_count) for k,v in b.get_type_and_topic_info().topics.items()})
 for topic,m,t in b.read_messages(topics=['/freedom/static_pointcloud','/sdf_map/occupancy_inflate']):
  sec=t.to_sec()-t0
  dtype=np.dtype(dict(names=['x','y','z'],formats=['<f4']*3,offsets=[next(f.offset for f in m.fields if f.name==k) for k in ('x','y','z')],itemsize=m.point_step))
  ar=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
  xyz=np.column_stack([ar[k].ravel() for k in ('x','y','z')]);xyz=xyz[np.isfinite(xyz).all(axis=1)]
  label='raw' if 'freedom' in topic else 'inflate'
  mask=(xyz[:,0]>2.5)&(xyz[:,0]<3.7)&(np.abs(xyz[:,1])<.5)&(xyz[:,2]>.15)&(xyz[:,2]<.65)
  counts.append([sec,label,int(mask.sum()),len(xyz),m.header.frame_id])
  for s in times:
   if sec<=s:snap[(label,s)]=(xyz,sec)
for (label,s),(xyz,sec) in snap.items():np.save(out/(label+str(s)+'.npy'),xyz)
(out/'counts.json').write_text(json.dumps(counts))
print('snapshots',[(k,round(v[1],3)) for k,v in snap.items()])
for s in times:
 for label in ['raw','inflate']:
  a=[c for c in counts if c[1]==label and c[0]<=s]
  if a:print(s,a[-1])
