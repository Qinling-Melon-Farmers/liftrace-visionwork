#!/usr/bin/env python3
"""Hardware-only ROS adapter; simulation-only guards remain untouched."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np,rospy,rospkg
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint
from uav_mission.high_view_probe import ProbeConfig
from uav_mission.boundary_revisit import BoundaryRevisit
from uav_high_view.survey_policy import SurveyPolicy
helper_dir=Path(__file__).resolve().parent
if not (helper_dir/'trial_runtime.py').exists():helper_dir=Path(rospkg.RosPack().get_path('uav_board_trials'))/'scripts'
sys.path.insert(0,str(helper_dir))
from trial_runtime import SingleDeliveryRuntime,FullCircleRuntime,OpenTourGrid
path=Path(rospkg.RosPack().get_path('uav_mission'))/'scripts/navigation_mission_manager.py'
spec=importlib.util.spec_from_file_location('board_existing_mission_shell',path);base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)

class BoardManager(base.NavigationMissionManager):
    def __init__(self):
        if rospy.get_param('/use_sim_time',False):raise RuntimeError('Board entry refuses simulated clock')
        self.mode=rospy.get_param('~trial/mode');self._grid_last=-1e9;self._camera_signature=None;self._low_limits_applied=False
        self._probe_pub=rospy.Publisher('/uav_high_view/probe_status',String,queue_size=1,latch=True)
        self._landing_pub=rospy.Publisher('/board_trials/landing_context',String,queue_size=1,latch=True)
        super().__init__()
        self._camera_sub=rospy.Subscriber(rospy.get_param('~trial/camera_info_topic','/camera/camera_info'),CameraInfo,self._camera_info,queue_size=1)
    def _new_runtime(self):
        ordinary=super()._new_runtime()
        points=tuple(Waypoint(*p) for p in rospy.get_param('~trial/waypoints'))
        route=CoverageRoute(points,'board-trial-search',1)
        if self.mode!='high_view':return SingleDeliveryRuntime(ordinary.core,route)
        cfg=dict(rospy.get_param('~high_view_probe/config'));cfg['survey_xy']=tuple(tuple(v) for v in cfg['survey_xy'])
        if cfg.get('staging_xy'):cfg['staging_xy']=tuple(cfg['staging_xy'])
        runtime=FullCircleRuntime(ordinary.core,ProbeConfig(**cfg),SurveyPolicy(**dict(rospy.get_param('~high_view_full/policy'))),
            fallback_route=None,boundary_policy=BoundaryRevisit(**dict(rospy.get_param('~high_view_full/boundary_policy'))))
        runtime.grid=OpenTourGrid(**dict(rospy.get_param('~high_view_full/grid')));return runtime
    def _camera_info(self,msg):
        signature=(msg.width,msg.height,msg.header.frame_id,tuple(msg.K),tuple(msg.D))
        with self._lock:
            if self._camera_signature is not None and signature!=self._camera_signature and self._runtime is not None:self._handle_callback_exception('camera_info',ValueError('calibration_changed'))
            self._camera_signature=signature
    def _on_pose(self,msg):
        super()._on_pose(msg)
        with self._lock:
            if self.mode=='high_view' and self._runtime is not None:
                p=msg.pose.position
                try:self._runtime.update_pose((p.x,p.y,p.z),msg.header.stamp.to_sec(),msg.header.frame_id)
                except Exception as e:self._handle_callback_exception('board_pose',e)
    def _on_map(self,msg):
        super()._on_map(msg)
        with self._lock:
            now=rospy.Time.now().to_sec()
            if self.mode!='high_view' or self._runtime is None or now-self._grid_last<1.:return
            self._grid_last=now
            try:
                if msg.header.frame_id!=self._runtime.core.config.mission_frame or msg.width*msg.height>250000:raise ValueError('cost map frame/size')
                fields={v.name:v for v in msg.fields}
                if any(k not in fields or fields[k].datatype!=7 for k in ('x','y','z')):raise ValueError('XYZ float32 required')
                dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
                array=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step));xyz=np.column_stack([array[k].ravel() for k in ('x','y','z')])
                ground=self._runtime.probe_config.ground_z;self._runtime.grid.update(xyz,msg.header.stamp.to_sec(),ground+.4,ground+3.)
            except Exception as e:self._handle_callback_exception('cost_map',e)
    def _publish_status(self,force=False):
        super()._publish_status(force)
        if self.mode=='high_view':self._probe_pub.publish(String(data=json.dumps(self._runtime.probe_status() if self._runtime else {'stage':'IDLE'},sort_keys=True)))
    def _publish_action(self,action):
        if action is not None and self.mode=='high_view' and self._runtime is not None and self._runtime.stage=='REVISIT' and not self._low_limits_applied:
            limits=rospy.get_param('~high_view_probe/low_stage_parameters')
            for item in limits:rospy.set_param(item['name'],item['value'])
            if any(rospy.get_param(item['name'])!=item['value'] for item in limits):raise RuntimeError('Low-stage parameter readback failed')
            self._low_limits_applied=True
        if action is not None and action.command=='LAND' and self.mode!='landing':
            core=self._runtime.core;expected=len(self._runtime.trial_manifest or {}) if self.mode=='high_view' else 1
            self._landing_pub.publish(String(data=json.dumps(dict(scope='board_trial_landing_after_mock',mode=self.mode,mission_id=core.mission_id,decision_seq=action.decision_seq,frame=core.config.mission_frame,xy=list(core.config.landing_xy),z=core.config.return_altitude,expected=expected,committed=core.committed_slots,time=rospy.Time.now().to_sec()))))
        super()._publish_action(action)

if __name__=='__main__':rospy.init_node('mission_manager');BoardManager();rospy.spin()
