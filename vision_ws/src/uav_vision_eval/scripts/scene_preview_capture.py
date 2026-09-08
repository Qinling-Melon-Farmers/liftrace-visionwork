#!/usr/bin/env python3
"""Capture requested Gazebo camera views after all seeded targets are present."""
import json,os,time
from pathlib import Path
import cv2
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates

def main():
    rospy.init_node('scene_preview_capture')
    output=Path(rospy.get_param('~output_dir',os.environ.get('SIM_RUN_DIR')))
    output.mkdir(parents=True,exist_ok=True);timeout=float(rospy.get_param('~timeout_sec',180))
    deadline=time.monotonic()+timeout;status_topic=rospy.get_param('~field_status_topic')
    field=None
    while time.monotonic()<deadline and not rospy.is_shutdown():
        try:
            msg=rospy.wait_for_message(status_topic,String,timeout=min(10,max(.1,deadline-time.monotonic())))
        except rospy.ROSException:
            continue
        candidate=json.loads(msg.data)
        if candidate.get('ready'):field=candidate;break
    if field is None:raise RuntimeError('Seeded field was not ready')
    ready_stamp=rospy.Time.now();bridge=CvBridge();images=[]
    for name,topic in rospy.get_param('~views').items():
        assert Path(name).name==name
        while time.monotonic()<deadline and not rospy.is_shutdown():
            try:
                msg=rospy.wait_for_message(topic,Image,timeout=min(15,max(.1,deadline-time.monotonic())))
            except rospy.ROSException:
                continue
            if msg.header.stamp>ready_stamp:break
        else:raise RuntimeError('No fresh preview frame: '+topic)
        frame=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        if float(frame.std())<3.0:raise RuntimeError('Uniform/empty preview frame: '+topic)
        path=output/(name+'.png');assert cv2.imwrite(str(path),frame)
        images.append({'file':path.name,'topic':topic,'stamp':msg.header.stamp.to_sec(),'width':msg.width,'height':msg.height,'std':float(frame.std())})
    states=rospy.wait_for_message('/gazebo/model_states',ModelStates,timeout=10)
    poses={n:{'x':p.position.x,'y':p.position.y,'z':p.position.z} for n,p in zip(states.name,states.pose)}
    result={'status':'CAPTURED','scope':'static_scene_no_flight','flight_validation':False,'field_status':field,'images':images,'model_positions':poses}
    (output/'preview_status.json').write_text(json.dumps(result,indent=2))
    rospy.loginfo('Static scene captured; no PX4/mission/arming was started')
    rospy.signal_shutdown('screenshots complete')

if __name__=='__main__':main()
