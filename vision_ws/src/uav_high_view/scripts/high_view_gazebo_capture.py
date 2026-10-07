#!/usr/bin/env python3
"""Gazebo-only stationary sensor sweep. Does not control a vehicle or use target positions."""
import json
import os
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


class Capture:
    def __init__(self):
        if not rospy.get_param('/use_sim_time', False) or not os.environ.get('SIM_RUN_DIR'):
            raise RuntimeError('requires simulated clock and sim_run directory')
        self.root = Path(os.environ['SIM_RUN_DIR'])
        self.cfg = json.loads(Path(rospy.get_param('~config')).read_text())
        self.model_source = Path(rospy.get_param('~vehicle_sdf'))
        self.name = rospy.get_param('~model_name','high_view_camera')
        self.image_topic = rospy.get_param('~image_topic','/high_view_eval/image_raw')
        self.info_topic = rospy.get_param('~camera_info_topic','/high_view_eval/camera_info')
        self.frame = rospy.get_param('~optical_frame','high_view_eval_optical')
        self.gazebo = rospy.get_param('~gazebo_namespace','/gazebo').rstrip('/')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image = self.info = None
        self.ready = False
        self.bytes = 0
        self.image_sub = rospy.Subscriber(self.image_topic,Image,self.on_image,queue_size=1)
        self.info_sub = rospy.Subscriber(self.info_topic,CameraInfo,self.on_info,queue_size=1)
        self.status_sub = rospy.Subscriber(rospy.get_param('~field_status_topic','/mission/random_field_status'),String,self.on_status,queue_size=1)

    def on_image(self,msg):
        with self.lock:self.image=msg

    def on_info(self,msg):
        with self.lock:self.info=msg

    def on_status(self,msg):
        self.ready = json.loads(msg.data).get('ready',False)

    def wait(self,predicate,seconds):
        deadline=time.monotonic()+seconds
        while not rospy.is_shutdown() and time.monotonic()<deadline:
            if predicate():return
            time.sleep(.02)
        raise RuntimeError('capture wait timed out')

    def execute(self):
        self.wait(lambda:self.ready,100.)
        rospy.wait_for_service(self.gazebo+'/spawn_sdf_model',timeout=20)
        source=ET.parse(self.model_source).getroot()
        link=source.find("model/link[@name='downward_camera_link']")
        if link is None:raise RuntimeError('installed camera link missing')
        # Keep the installed camera link, optical mounting, intrinsics and distortion.
        # The rig has no motors, vehicle dynamics, LIO or navigation authority.
        for item in list(link):
            if item.tag not in ('pose','sensor'):link.remove(item)
        mount=[float(v) for v in link.findtext('pose').split()]
        if max(abs(mount[i]) for i in [0,1,3,5])>1e-8 or abs(mount[4]-np.pi/2)>1e-8:
            raise RuntimeError('capture optical transform requires the verified nadir mounting')
        sensor=link.find('sensor')
        sensor.find('update_rate').text='10'
        plugin=sensor.find('plugin')
        plugin.find('cameraName').text='high_view_eval'
        plugin.find('frameName').text=self.frame
        plugin.find('imageTopicName').text=self.image_topic
        plugin.find('cameraInfoTopicName').text=self.info_topic
        plugin.find('updateRate').text='10'
        sdf=ET.Element('sdf',version='1.6');model=ET.SubElement(sdf,'model',name=self.name)
        ET.SubElement(model,'static').text='true';model.append(link)
        model_xml=ET.tostring(sdf,encoding='unicode')
        (self.root/'camera_rig.sdf').write_text(model_xml)
        pose=Pose();pose.position.z=1.4;pose.orientation.w=1.
        spawn=rospy.ServiceProxy(self.gazebo+'/spawn_sdf_model',SpawnModel)
        if not spawn(self.name,model_xml,'',pose,'world').success:raise RuntimeError('spawn failed')
        move=rospy.ServiceProxy(self.gazebo+'/set_model_state',SetModelState)
        read=rospy.ServiceProxy(self.gazebo+'/get_model_state',GetModelState)
        self.wait(lambda:self.image is not None and self.info is not None,30.)
        cfg=self.cfg
        directory=self.root/'frames';directory.mkdir()
        records=[]
        for height in cfg['fc_heights']:
            for view,(x,y) in enumerate(cfg['view_xy']):
                state=ModelState();state.model_name=self.name;state.reference_frame='world'
                state.pose.position.x=x;state.pose.position.y=y;state.pose.position.z=height
                state.pose.orientation.w=1.
                if not move(state).success:raise RuntimeError('camera move failed')
                moved_at=rospy.Time.now().to_sec()
                self.wait(lambda:rospy.Time.now().to_sec()>moved_at+cfg['settle_seconds'],10.)
                actual=read(self.name,'world')
                if not actual.success:raise RuntimeError('pose readback failed')
                p=actual.pose.position
                if max(abs(p.x-x),abs(p.y-y),abs(p.z-height))>.002:raise RuntimeError('pose mismatch')
                previous=rospy.Time.now().to_sec()
                for frame in range(cfg['frames_per_view']):
                    self.wait(lambda:self.image.header.stamp.to_sec()>=previous+cfg['frame_interval'],10.)
                    with self.lock:im,info=self.image,self.info
                    previous=im.header.stamp.to_sec()
                    if abs(info.header.stamp.to_sec()-previous)>.15:raise RuntimeError('CameraInfo not synchronized')
                    image=self.bridge.imgmsg_to_cv2(im,'bgr8')
                    ok,encoded=cv2.imencode('.png',image,[cv2.IMWRITE_PNG_COMPRESSION,6])
                    if not ok:raise RuntimeError('PNG encode failed')
                    self.bytes+=len(encoded)
                    if self.bytes>cfg['max_image_mib']*1024*1024:raise RuntimeError('image budget exceeded')
                    filename='h%02d_v%02d_f%d.png'%(round(height*10),view,frame)
                    (directory/filename).write_bytes(encoded.tobytes())
                    row=dict(image='frames/'+filename,stamp=previous,fc_agl=height,
                        view=view,frame=frame,fc_world=[p.x,p.y,p.z],camera_world=[p.x,p.y,p.z+mount[2]],
                        optical_to_world=[[0,-1,0],[-1,0,0],[0,0,-1]],
                        width=info.width,height=info.height,k=list(info.K),d=list(info.D),
                        info_stamp=info.header.stamp.to_sec(),pose_source='gazebo_static_rig_readback',
                        scope='STATIONARY_RENDER_NOT_A_FLIGHT_OR_LIO_TEST')
                    records.append(row)
                    with (self.root/'captures.jsonl').open('a') as out:out.write(json.dumps(row)+'\n')
                rospy.loginfo('high-view capture H=%.1f view=%d (%d frames, %.1fMiB)',height,view,len(records),self.bytes/1048576.)
        (self.root/'capture_config.json').write_text(json.dumps(cfg,indent=2))
        return dict(status='PASS',scope='RENDER_CAPTURE_COMPLETENESS_ONLY',
                    images=len(records),image_bytes=self.bytes,fc_heights=cfg['fc_heights'],
                    views_per_height=len(cfg['view_xy']),not_flight_acceptance=True)


if __name__=='__main__':
    rospy.init_node('high_view_gazebo_capture')
    root=Path(os.environ.get('SIM_RUN_DIR','/tmp'))
    try:
        result=Capture().execute()
    except Exception as error:
        result=dict(status='FAIL',scope='RENDER_CAPTURE_COMPLETENESS_ONLY',reason=str(error))
        rospy.logerr(str(error))
    (root/'gate_status.json').write_text(json.dumps(result,indent=2))
    rospy.signal_shutdown(result['status'])
