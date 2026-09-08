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
from gazebo_msgs.srv import GetLinkState
from tf.transformations import quaternion_matrix
import numpy as np

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
    if rospy.has_param('~link_names'):
        links={};get_link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        for name,scoped in rospy.get_param('~link_names').items():
            state=get_link(link_name=scoped,reference_frame='world')
            if not state.success:raise RuntimeError(state.status_message)
            p=state.link_state.pose;links[name]={'xyz':[p.position.x,p.position.y,p.position.z],
                                              'quat':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}
        lidar=links['radar_link'];imu=np.array(lidar['xyz'])+quaternion_matrix(lidar['quat'])[:3,:3].dot(np.array(rospy.get_param('~imu_in_lidar_link')))
        fc=np.array(links['fc']['xyz']);camera=np.array(links['camera']['xyz'])
        assert np.allclose(fc-camera,[0,0,rospy.get_param('~camera_below_fc')],atol=1e-6)
        assert np.allclose(imu-camera,[0,0,rospy.get_param('~camera_below_imu')],atol=1e-6)
        clearance=float(camera[2]-rospy.get_param('~support_surface_z'))
        assert abs(clearance-rospy.get_param('~camera_ground_clearance'))<1e-6
        result['live_assembly']={'links':links,'imu_world_xyz':imu.tolist(),'fc_to_camera_xyz':(camera-fc).tolist(),
                                 'imu_to_camera_xyz':(camera-imu).tolist(),'camera_height_above_support':clearance,'passed':True}
    (output/'preview_status.json').write_text(json.dumps(result,indent=2))
    rospy.loginfo('Static scene captured; no PX4/mission/arming was started')
    rospy.signal_shutdown('screenshots complete')

if __name__=='__main__':main()
