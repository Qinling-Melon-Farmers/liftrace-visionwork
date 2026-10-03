#!/usr/bin/env python3
"""Re-render recorded true poses. This cannot run a flight or score a Gate."""
import copy,csv,json,math,os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel,SetModelState
from sensor_msgs.msg import Image


def resolve_model(name, roots):
    for root in roots:
        folder=root/name
        if (folder/'model.sdf').exists():return folder/'model.sdf'
        if (folder/'model.config').exists():
            source=folder/ET.parse(folder/'model.config').getroot().findtext('sdf')
            if source.is_file():return source
    raise ValueError('unresolved replay visual model://'+name)


def visual_only(path, roots):
    model=ET.parse(path).getroot().find('model')
    for parent in list(model.iter()):
        for item in list(parent):
            if item.tag=='include':
                uri=item.findtext('uri');assert uri.startswith('model://'),uri
                name=uri[len('model://'):]
                source=resolve_model(name,roots)
                nested=visual_only(source,roots)
                nested.set('name',item.findtext('name',name))
                old=nested.find('pose')
                if old is not None:nested.remove(old)
                ET.SubElement(nested,'pose').text=item.findtext('pose','0 0 0 0 0 0')
                parent.remove(item);parent.append(nested)
            elif item.tag in ('collision','sensor','plugin','joint','inertial','velocity_decay'):
                parent.remove(item)
    old=model.find('static')
    if old is None:old=ET.SubElement(model,'static')
    old.text='true'
    return model


def main():
    rospy.init_node('recorded_flight_replay')
    if not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR'):
        raise RuntimeError('run wrapper and simulation clock required')
    source=Path(rospy.get_param('~source_run')).resolve()
    output=Path(os.environ['SIM_RUN_DIR'])
    roots=[Path(p) for p in rospy.get_param('~model_roots').split(':') if p]
    with (source/'truth_pose.csv').open() as f:
        rows=np.array([[float(row[k]) for k in ('t','x','y','z','qx','qy','qz','qw')] for row in csv.DictReader(f)])
    assert len(rows)>10 and np.all(np.isfinite(rows)) and np.all(np.diff(rows[:,0])>0)
    params=yaml.safe_load((source/'rosparams.yaml').read_text())
    offset=np.asarray(params['competition_key_recorder']['truth_world_offset'],dtype=float)
    truth=yaml.safe_load((source/'random_field_truth.yaml').read_text())
    for svc in ('/gazebo/spawn_sdf_model','/gazebo/set_model_state'):rospy.wait_for_service(svc,timeout=90)
    spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
    move=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
    def add(name, model, pose):
        sdf=ET.Element('sdf',version='1.6');sdf.append(model)
        result=spawn(name,ET.tostring(sdf,encoding='unicode'),'',pose,'world')
        if not result.success:raise RuntimeError(result.status_message)
    for target in truth['targets']:
        path=resolve_model(target['source'],roots)
        pose=Pose();pose.position.x=target['world_x'];pose.position.y=target['world_y'];pose.position.z=.02
        pose.orientation.z=math.sin(target['yaw']/2);pose.orientation.w=math.cos(target['yaw']/2)
        add(target['model'],visual_only(path,roots),pose)
    vehicle=visual_only(Path(rospy.get_param('~vehicle_sdf')),roots)
    assert not any(vehicle.iter('plugin')) and not any(vehicle.iter('collision'))
    pose=Pose();pose.orientation.w=1.;pose.position.z=offset[2]
    add('recorded_vehicle',vehicle,pose)
    external_cameras=rospy.get_param('~external_cameras',False)
    if not external_cameras:
        camera=ET.parse(rospy.get_param('~overview_sdf')).getroot().find('model')
        camera.find('.//camera/clip/near').text=str(rospy.get_param('~camera_near'))
        pose=Pose();pose.position.y=4.3;pose.position.z=float(rospy.get_param('~camera_z'))
        # Rz(-pi/2)*Ry(pi/2)
        pose.orientation.x=.5;pose.orientation.y=.5;pose.orientation.z=-.5;pose.orientation.w=.5
        add('competition_overview_camera',camera,pose)
    # R64 recorder receives the same camera topic. Wait for rendering before movement.
    frame=rospy.wait_for_message('/competition_overview/image_raw',Image,timeout=30)
    assert frame.encoding in ('rgb8','bgr8')
    image=np.ndarray((frame.height,frame.width,3),dtype=np.uint8,buffer=frame.data,strides=(frame.step,3,1))
    if np.std(image)<4:raise RuntimeError('overview still occluded/blank; do not generate misleading replay')
    import cv2
    cv2.imwrite(str(output/'overview_preview.jpg'),image[:,:,::-1] if frame.encoding=='rgb8' else image)
    start=rospy.get_time();origin=rows[0,0];duration=rows[-1,0]-origin
    metadata=dict(kind='RECORDED_TRUTH_RENDER_REPLAY_NOT_A_NEW_FLIGHT',source_run=str(source),
                  source_start=origin,source_end=float(rows[-1,0]),replay_start_ros=start,
                  camera_z=None if external_cameras else pose.position.z,camera_near=None if external_cameras else float(rospy.get_param('~camera_near')),
                  presentation_recording=external_cameras,
                  vehicles_dynamic=False,controller_nodes=False,source_gate=json.loads((source/'gate_status.json').read_text())['status'])
    (output/'replay_metadata.json').write_text(json.dumps(metadata,indent=2))
    rate=rospy.Rate(20);count=0
    while not rospy.is_shutdown():
        elapsed=rospy.get_time()-start;time=min(rows[-1,0],origin+elapsed)
        i=max(0,min(len(rows)-2,int(np.searchsorted(rows[:,0],time)-1)))
        first,last=rows[i:i+2];u=(time-first[0])/(last[0]-first[0])
        xyz=first[1:4]+u*(last[1:4]-first[1:4])+offset
        q0=first[4:];q1=last[4:]
        if np.dot(q0,q1)<0:q1=-q1
        q=q0*(1-u)+q1*u;q/=np.linalg.norm(q)
        state=ModelState();state.model_name='recorded_vehicle';state.reference_frame='world'
        state.pose.position.x,state.pose.position.y,state.pose.position.z=xyz
        state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w=q
        response=move(state)
        if not response.success:raise RuntimeError(response.status_message)
        count+=1
        if elapsed>=duration+2:break
        rate.sleep()
    metadata.update(status='RENDER_COMPLETE',updates=count,replay_end_ros=rospy.get_time())
    (output/'replay_metadata.json').write_text(json.dumps(metadata,indent=2))
    rospy.loginfo('Recorded flight rendering complete: %.1fs source span',duration)


if __name__=='__main__':main()
