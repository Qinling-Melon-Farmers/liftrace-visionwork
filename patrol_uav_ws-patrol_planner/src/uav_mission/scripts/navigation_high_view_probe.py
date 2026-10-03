#!/usr/bin/env python3
"""Opt-in simulation research manager; reuses the original ROS transaction shell."""
import json
import os
import importlib.util
from pathlib import Path
import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from uav_mission.high_view_probe import HighViewProbe, ProbeConfig

# Catkin devel relay modules execute in an isolated dictionary and cannot be
# imported for their classes. Load the sibling source/install script directly.
_spec=importlib.util.spec_from_file_location('high_view_base_manager',str(Path(__file__).with_name('navigation_mission_manager.py')))
_base=importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_base)
NavigationMissionManager=_base.NavigationMissionManager


class ProbeManager(NavigationMissionManager):
    def __init__(self):
        if not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR'):
            raise RuntimeError('high-view probe is simulation-only')
        self._probe_pub=rospy.Publisher(rospy.get_param('~high_view_probe/status_topic','/uav_high_view/probe_status'),String,queue_size=1,latch=True)
        self._camera_signature=None
        self._low_limits_applied=False
        super().__init__()
        self._camera_sub=rospy.Subscriber(rospy.get_param('~high_view_probe/camera_info_topic','/downward_camera/camera_info'),CameraInfo,self._camera_info,queue_size=1)

    def _new_runtime(self):
        ordinary=super()._new_runtime()
        cfg=dict(rospy.get_param('~high_view_probe/config'))
        cfg['survey_xy']=tuple(tuple(p) for p in cfg['survey_xy'])
        if cfg.get('staging_xy'):
            cfg['staging_xy']=tuple(cfg['staging_xy'])
        return HighViewProbe(ordinary.core,ProbeConfig(**cfg))

    def _camera_info(self,message):
        signature=(message.width,message.height,message.header.frame_id,tuple(message.K),tuple(message.D))
        with self._lock:
            if self._camera_signature is not None and signature!=self._camera_signature and self._runtime is not None:
                self._handle_callback_exception('camera_info',ValueError('calibration_changed'))
            self._camera_signature=signature

    def _on_pose(self,message):
        super()._on_pose(message)
        with self._lock:
            if self._runtime is not None:
                try:
                    p=message.pose.position
                    self._runtime.update_pose((p.x,p.y,p.z),message.header.stamp.to_sec(),message.header.frame_id)
                except Exception as error:
                    self._handle_callback_exception('probe_pose',error)

    def _publish_action(self,action):
        if action is not None and self._runtime is not None and self._runtime.stage in ('REVISIT','LOW_COVERAGE') and not self._low_limits_applied:
            limits=rospy.get_param('~high_view_probe/low_stage_parameters')
            for item in limits:rospy.set_param(item["name"],item["value"])
            if any(rospy.get_param(item["name"])!=item["value"] for item in limits):
                raise RuntimeError('low-stage parameter readback failed')
            self._low_limits_applied=True
        super()._publish_action(action)

    def _publish_status(self,force=False):
        super()._publish_status(force)
        payload=self._runtime.probe_status() if self._runtime is not None else dict(scope='HIGH_VIEW_SINGLE_REVISIT_NO_DELIVERY',stage='IDLE')
        payload['low_limits_applied']=self._low_limits_applied
        self._probe_pub.publish(String(data=json.dumps(payload,sort_keys=True)))


if __name__=='__main__':
    rospy.init_node('mission_manager')
    ProbeManager()
    rospy.spin()
