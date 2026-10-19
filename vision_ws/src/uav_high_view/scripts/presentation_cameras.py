#!/usr/bin/env python3
"""Optional simulation observer. Only moves its own rendering camera model."""
import json,math,os,threading
from pathlib import Path
import xml.etree.ElementTree as ET
import rospy
from gazebo_msgs.msg import ModelStates,ModelState
from gazebo_msgs.srv import SpawnModel,SetModelState
from geometry_msgs.msg import Pose
from uav_high_view.presentation import FollowView,look_at


class Cameras:
    def __init__(self):
        if not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR'):
            raise RuntimeError('simulation run wrapper required')
        self.model=rospy.get_param('~vehicle_model','iris_mid360')
        self.camera_name='presentation_follow_camera'
        if self.model in (self.camera_name,'presentation_overview_camera'):raise ValueError('camera cannot follow itself')
        self.root=Path(os.environ['SIM_RUN_DIR']);self.lock=threading.Lock();self.latest=None
        self.previous=None;self.last=None;self.written=0
        self.follow=FollowView(**rospy.get_param('~follow',{}))
        self.walls=[]
        for link in ET.parse(rospy.get_param('~world')).getroot().iter('link'):
            if not link.get('name','').startswith('Wall'):continue
            pose=list(map(float,link.findtext('pose','0 0 0 0 0 0').split()))
            size=link.findtext('collision/geometry/box/size')
            if size is None:continue
            if any(abs(v)>1e-8 for v in pose[3:]):raise ValueError('camera LOS requires axis-aligned wall fixture')
            size=list(map(float,size.split()));self.walls.append(tuple(value for i in range(3) for value in (pose[i]-size[i]/2,pose[i]+size[i]/2)))
        self.move=rospy.ServiceProxy(rospy.get_param('~move_service','/gazebo/set_model_state'),SetModelState)
        spawn_name=rospy.get_param('~spawn_service','/gazebo/spawn_sdf_model');rospy.wait_for_service(spawn_name,timeout=60)
        spawn=rospy.ServiceProxy(spawn_name,SpawnModel)
        template=rospy.get_param('~camera_sdf')
        views=rospy.get_param('~views')
        for kind in ('overview','follow'):
            config=views[kind];xyz=config['xyz'];near=config['near'];width=config['width'];height=config['height'];fov=config['hfov']
            if (len(xyz)!=3 or not all(math.isfinite(v) for v in (*xyz,near,fov)) or not 0<near<20
                    or not .3<fov<2.6 or not 64<=width<=1920 or not 64<=height<=1080):
                raise ValueError('invalid presentation camera')
            tree=ET.parse(template);model=tree.getroot().find('model');model.set('name','presentation_'+kind+'_camera')
            camera=model.find('.//camera');camera.find('clip/near').text=str(near)
            camera.find('horizontal_fov').text=str(fov);camera.find('image/width').text=str(width);camera.find('image/height').text=str(height)
            plugin=model.find('.//plugin');plugin.find('cameraName').text='competition_'+kind
            plugin.find('frameName').text='presentation_'+kind+'_optical'
            pose=Pose();pose.position.x,pose.position.y,pose.position.z=xyz
            q=look_at(xyz,(xyz[0],xyz[1],0.),config.get('vertical_yaw',0.)) if kind=='overview' else look_at(xyz,(0.,1.,0.))
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=q
            result=spawn(model.get('name'),ET.tostring(tree.getroot(),encoding='unicode'),'',pose,'world')
            if not result.success:raise RuntimeError(result.status_message)
        self.log=(self.root/'presentation_camera_poses.jsonl').open('w',buffering=1);self.closed=False
        self.sub=rospy.Subscriber(rospy.get_param('~models_topic','/gazebo/model_states'),ModelStates,self.on_models,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(.1),self.tick)
        rospy.on_shutdown(self.close)

    def close(self):
        self.timer.shutdown()
        with self.lock:
            self.closed=True;self.log.close()

    def on_models(self,msg):
        try:i=msg.name.index(self.model)
        except ValueError:return
        with self.lock:self.latest=(msg.pose[i],rospy.get_time())

    def tick(self,event):
        with self.lock:sample=self.latest
        now=rospy.get_time()
        if sample is None or not 0<=now-sample[1]<=.5:return
        pose,stamp=sample;p,q=pose.position,pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        dt=0.1 if self.last is None else now-self.last
        if dt<0:self.previous=None;dt=.1
        result=self.follow.propose((p.x,p.y,p.z),yaw,self.previous,dt,self.walls)
        cmd=ModelState();cmd.model_name=self.camera_name;cmd.reference_frame='world'
        cmd.pose.position.x,cmd.pose.position.y,cmd.pose.position.z=result['xyz']
        cmd.pose.orientation.x,cmd.pose.orientation.y,cmd.pose.orientation.z,cmd.pose.orientation.w=result['xyzw']
        # No caller-supplied name can change the model this node moves.
        response=self.move(cmd)
        if response.success:self.previous=result['xyz'];self.last=now
        row=json.dumps(dict(t=now,body=[p.x,p.y,p.z],camera=result,moved=response.success))+'\n'
        with self.lock:
            if not self.closed and self.written+len(row)<=3*1024*1024:self.log.write(row);self.written+=len(row)


if __name__=='__main__':
    rospy.init_node('presentation_cameras');node=Cameras();rospy.spin()
