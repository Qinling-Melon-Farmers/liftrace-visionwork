"""Extract only camera calibration, odometry, and five original images; no ROS publication."""
import argparse,json
from pathlib import Path
import rosbag
p=argparse.ArgumentParser()
p.add_argument('bag',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--image-topic',default='/camera/image_raw/compressed')
p.add_argument('--odom-topic',default='/mavros/local_position/odom')
p.add_argument('--info-topic',default='/camera/camera_info')
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
times=[25.,53.,54.,55.,56.];near={};motion=[];cameras=[]
with rosbag.Bag(str(a.bag)) as bag:
    begin=bag.get_start_time()
    for topic,m,t in bag.read_messages(topics=[a.image_topic,a.odom_topic,a.info_topic]):
        if topic==a.image_topic:
            ts=m.header.stamp.to_sec()-begin
            for sec in times:
                delta=abs(ts-sec)
                if sec not in near or delta<near[sec][0]:near[sec]=(delta,ts,bytes(m.data))
        elif topic==a.odom_topic:
            pose=m.pose.pose
            motion.append(dict(t=t.to_sec()-begin,stamp=m.header.stamp.to_sec(),
                **{k:getattr(pose.position,k) for k in ('x','y','z')},
                orientation={k:getattr(pose.orientation,k) for k in ('x','y','z','w')}))
        else:
            value=dict(topic=topic,stamp=m.header.stamp.to_sec(),w=m.width,h=m.height,K=list(m.K),D=list(m.D),P=list(m.P),model=m.distortion_model,binning=[m.binning_x,m.binning_y],roi=dict(x=m.roi.x_offset,y=m.roi.y_offset,w=m.roi.width,h=m.roi.height))
            if not cameras or {k:v for k,v in value.items() if k!='stamp'}!={k:v for k,v in cameras[-1].items() if k!='stamp'}:cameras.append(value)
if len(cameras)!=1 or not motion or len(near)!=5:raise ValueError('expected one unchanged calibration and five available frames')
meta=[]
for sec,(delta,stamp,raw) in near.items():
    if delta>.05:raise ValueError('requested image missing')
    name=f'frame_{sec:.0f}.jpg';(a.out/name).write_bytes(raw)
    meta.append(dict(file=name,requested=sec,stamp=stamp))
(a.out/'frames.json').write_text(json.dumps(meta,indent=2))
(a.out/'camera_info.json').write_text(json.dumps(cameras,indent=2))
(a.out/'motion.json').write_text(json.dumps(dict(start=begin,motion=motion,bag=str(a.bag))))
print('Extracted five original frames, stamped odom and CameraInfo')
