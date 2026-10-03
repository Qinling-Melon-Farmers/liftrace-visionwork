#!/usr/bin/env python3
"""Replay recorded camera samples through detectors only; no flight interfaces."""
import bisect
import json
import os
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import rosbag
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
from uav_vision.msg import TargetDetectionArray

def main():
    rospy.init_node("bag_detector_replay")
    bag_path=Path(rospy.get_param("~bag"))
    output=Path(os.environ["SIM_RUN_DIR"])
    camera_topic=rospy.get_param("~source_camera_topic","/camera/image_raw/compressed")
    raw_topic=rospy.get_param("~source_raw_topic","/uav_vision/detections")
    source_info=rospy.get_param("~source_camera_info_topic","/camera/camera_info")
    output_topic=rospy.get_param("~raw_topic","/replay/detections")
    frames=[];samples=[];info=None
    with rosbag.Bag(str(bag_path)) as bag:
        start=bag.get_start_time()
        for topic,msg,receipt in bag.read_messages(topics=[camera_topic,raw_topic,source_info]):
            if topic==camera_topic:frames.append(msg)
            elif topic==raw_topic and msg.source=="target_detector":
                samples.append((msg.header.stamp,receipt))
            elif topic==source_info and info is None:info=msg
    if info is None or not frames or not samples:raise RuntimeError("missing camera/info/detector samples")
    frames.sort(key=lambda f:f.header.stamp.to_sec());times=[f.header.stamp.to_sec() for f in frames]
    clock=rospy.Publisher("/clock",Clock,queue_size=1,latch=True)
    images=rospy.Publisher(rospy.get_param("~image_topic","/replay/image"),Image,queue_size=1)
    infos=rospy.Publisher(rospy.get_param("~camera_info_topic","/replay/camera_info"),CameraInfo,queue_size=1,latch=True)
    condition=threading.Condition();results={};active=[None]
    def callback(msg):
        with condition:
            if msg.header.stamp==active[0]:
                results[msg.source]=msg;condition.notify_all()
    sub=rospy.Subscriber(output_topic,TargetDetectionArray,callback,queue_size=16)
    clock.publish(Clock(samples[0][0]));infos.publish(info)
    deadline=time.monotonic()+120.
    while images.get_num_connections()<3 or infos.get_num_connections()<2:
        if rospy.is_shutdown() or time.monotonic()>deadline:raise RuntimeError("detectors not ready")
        time.sleep(.1)
    metadata=dict(kind="OFFLINE_IMAGE_REINFERENCE_NOT_FLIGHT",
                  bag=str(bag_path),model=rospy.get_param("~model"),
                  requested_samples=len(samples),source_images=len(frames),
                  cadence="nearest recorded YOLO source image; synchronous detector completion",
                  frames=[],bag_start=start)
    used=set()
    with rosbag.Bag(str(output/"reinferred_raw.bag"),"w") as target:
        for i,(source_stamp,receipt) in enumerate(samples):
            s=source_stamp.to_sec();index=bisect.bisect_left(times,s)
            ids=[j for j in (index-1,index) if 0<=j<len(frames)]
            j=min(ids,key=lambda k:abs(times[k]-s))
            if abs(times[j]-s)>.025:raise RuntimeError("missing synchronized image")
            if j in used:continue
            used.add(j);image=frames[j]
            bgr=cv2.imdecode(np.frombuffer(image.data,dtype=np.uint8),cv2.IMREAD_COLOR)
            if bgr is None:raise RuntimeError("invalid recorded image")
            msg=Image();msg.header=image.header
            msg.height,msg.width=bgr.shape[:2];msg.encoding="bgr8"
            msg.step=msg.width*3;msg.data=bgr.tobytes()
            with condition:results.clear();active[0]=msg.header.stamp
            clock.publish(Clock(receipt))
            images.publish(msg)
            deadline=time.monotonic()+60.
            with condition:
                while not {"target_detector","circle_detector","cross_detector"}<=set(results):
                    if rospy.is_shutdown() or time.monotonic()>deadline:
                        raise RuntimeError("frame timeout: "+str(results.keys()))
                    condition.wait(.05)
                result=dict(results)
            for name in sorted(result):
                target.write("/uav_vision/detections",result[name],receipt)
            metadata["frames"].append(dict(stamp=msg.header.stamp.to_sec(),receipt=receipt.to_sec(),
                                           delta_sec=times[j]-s,
                                           counts={k:len(v.detections) for k,v in result.items()}))
            if i%100==0:print("reinference %d/%d"%(i+1,len(samples)),flush=True)
    metadata["completed_samples"]=len(used)
    (output/"reinference.json").write_text(json.dumps(metadata,indent=2))
    print("reinference complete:",len(used),flush=True)

if __name__=="__main__":main()
