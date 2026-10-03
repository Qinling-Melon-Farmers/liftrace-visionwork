#!/usr/bin/env python3
"""Hardware-only ROS adapter; simulation-only guards remain untouched."""
import importlib.util,json,sys,os
from pathlib import Path
import numpy as np,rospy,rospkg
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from uav_vision.msg import TargetDetectionArray
from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint
from uav_mission.high_view_probe import ProbeConfig
from uav_mission.boundary_revisit import BoundaryRevisit
from uav_high_view.survey_policy import SurveyPolicy
from uav_high_view.grid_cost import GridCost
helper_dir=Path(__file__).resolve().parent
if not (helper_dir/'trial_runtime.py').exists():helper_dir=Path(rospkg.RosPack().get_path('uav_board_trials'))/'scripts'
sys.path.insert(0,str(helper_dir))
from trial_runtime import SingleDeliveryRuntime,FullCircleRuntime,OpenTourGrid,MultiDeliveryRuntime,PriorityRevisitRuntime,MemoryOnlyRuntime,FullMissionTrialRuntime,HighSpeedCaptureRuntime
from trial_config import HIGH_MODES,H_MODES
path=Path(rospkg.RosPack().get_path('uav_mission'))/'scripts/navigation_mission_manager.py'
spec=importlib.util.spec_from_file_location('board_existing_mission_shell',path);base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)

class BoardManager(base.NavigationMissionManager):
    def __init__(self, simulation=False):
        simulated_clock=rospy.get_param('/use_sim_time',False)
        if simulation:
            if not simulated_clock or not os.environ.get('SIM_RUN_DIR') or rospy.get_param('/board_trials/actuator_mode','real') not in ('mock','none'):
                raise RuntimeError('Simulation adapter requires sim clock, run directory and mock/none actuator')
        elif simulated_clock:raise RuntimeError('Board entry refuses simulated clock')
        self.mode=rospy.get_param('~trial/mode');self._grid_last=-1e9;self._camera_signature=None;self._low_limits_applied=False
        self._probe_pub=rospy.Publisher('/uav_high_view/probe_status',String,queue_size=1,latch=True)
        self._landing_pub=rospy.Publisher('/board_trials/landing_context',String,queue_size=1,latch=True)
        super().__init__()
        if self.mode in HIGH_MODES:
            self._hint_sub=rospy.Subscriber(rospy.get_param('~high_view_full/navigation_hints_topic','/uav_vision/navigation_hints'),TargetDetectionArray,self._on_navigation_hints,queue_size=1)
        self._camera_sub=rospy.Subscriber(rospy.get_param('~trial/camera_info_topic','/camera/camera_info'),CameraInfo,self._camera_info,queue_size=1)
    def _new_runtime(self):
        ordinary=super()._new_runtime()
        points=tuple(Waypoint(*p) for p in rospy.get_param('~trial/waypoints'))
        route=CoverageRoute(points,'board-trial-search',1)
        self._low_limits_applied=False
        if self.mode=='low_multi':return MultiDeliveryRuntime(ordinary.core,route,delivery_count=rospy.get_param('~trial/delivery_count',2))
        if self.mode not in HIGH_MODES:return SingleDeliveryRuntime(ordinary.core,route)
        cfg=dict(rospy.get_param('~high_view_probe/config'));cfg['survey_xy']=tuple(tuple(v) for v in cfg['survey_xy'])
        if cfg.get('staging_xy'):cfg['staging_xy']=tuple(cfg['staging_xy'])
        runtime_class={'high_view':FullCircleRuntime,'high_priority':PriorityRevisitRuntime,'memory_only':MemoryOnlyRuntime,'high_view_full':FullMissionTrialRuntime,'high_speed_capture':HighSpeedCaptureRuntime}[self.mode]
        runtime=runtime_class(ordinary.core,ProbeConfig(**cfg),SurveyPolicy(**dict(rospy.get_param('~high_view_full/policy'))),
            fallback_route=ordinary.route if self.mode=='high_view_full' else None,boundary_policy=BoundaryRevisit(**dict(rospy.get_param('~high_view_full/boundary_policy'))))
        runtime.grid=(GridCost if self.mode=='high_view_full' else OpenTourGrid)(**dict(rospy.get_param('~high_view_full/grid')))
        runtime.descent_grid=GridCost(**dict(rospy.get_param("~high_view_full/grid")))
        return runtime
    def _on_navigation_hints(self,message):
        if message.source!='coarse_navigation_projector':return
        with self._lock:
            if self._runtime is None or self._runtime.catalog.epoch is None:return
            try:
                now=rospy.Time.now().to_sec()
                for det in message.detections:
                    if det.center_source!='bbox_navigation_only' or det.center_refined or det.association_valid or det.geometry_verified:continue
                    self._runtime.ingest_coarse(class_name=det.class_name,xy=(det.map_point.x,det.map_point.y),stamp_ns=message.header.stamp.to_nsec(),frame=det.map_frame,confidence=det.class_confidence,transform_age_sec=det.transform_age_sec,map_valid=det.map_valid,now=now)
            except Exception as error:self._handle_callback_exception('navigation_hints',error)
    def _camera_info(self,msg):
        signature=(msg.width,msg.height,msg.header.frame_id,tuple(msg.K),tuple(msg.D))
        with self._lock:
            if self._camera_signature is not None and signature!=self._camera_signature and self._runtime is not None:self._handle_callback_exception('camera_info',ValueError('calibration_changed'))
            self._camera_signature=signature
    def _on_pose(self,msg):
        super()._on_pose(msg)
        with self._lock:
            if self.mode in HIGH_MODES and self._runtime is not None:
                p=msg.pose.position
                try:self._runtime.update_pose((p.x,p.y,p.z),msg.header.stamp.to_sec(),msg.header.frame_id)
                except Exception as e:self._handle_callback_exception('board_pose',e)
    def _on_map(self,msg):
        super()._on_map(msg)
        with self._lock:
            now=rospy.Time.now().to_sec()
            if self.mode not in HIGH_MODES or self._runtime is None or now-self._grid_last<1.:return
            self._grid_last=now
            try:
                if msg.header.frame_id!=self._runtime.core.config.mission_frame or msg.width*msg.height>250000:raise ValueError('cost map frame/size')
                fields={v.name:v for v in msg.fields}
                if any(k not in fields or fields[k].datatype!=7 for k in ('x','y','z')):raise ValueError('XYZ float32 required')
                dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
                array=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step));xyz=np.column_stack([array[k].ravel() for k in ('x','y','z')])
                ground=self._runtime.probe_config.ground_z;self._runtime.grid.update(xyz,msg.header.stamp.to_sec(),ground+.4,ground+3.)
                # Separate swept vertical volume from the broad 2-D tour cost map.
                if self._runtime.pose is None:
                    self._runtime.descent_grid.stamp=None
                    return
                cfg=self._runtime.probe_config
                low=ground+cfg.low_agl
                high=max(ground+cfg.high_agl,self._runtime.pose[2])
                below=float(rospy.get_param('~high_view_full/descent_margin_below_m',0.10))
                above=float(rospy.get_param('~high_view_full/descent_margin_above_m',0.20))
                if not np.isfinite([below,above]).all() or min(below,above)<0:raise ValueError('descent margins')
                self._runtime.descent_grid.update(xyz,msg.header.stamp.to_sec(),min(low,self._runtime.pose[2])-below,high+above)

            except Exception as e:self._handle_callback_exception('cost_map',e)
    def _publish_status(self,force=False):
        super()._publish_status(force)
        if self.mode in HIGH_MODES:self._probe_pub.publish(String(data=json.dumps(self._runtime.probe_status() if self._runtime else {'stage':'IDLE'},sort_keys=True)))
    def _publish_action(self,action):
        if action is not None and self.mode in HIGH_MODES and self._runtime is not None and self._runtime.stage in ('REVISIT','LOW_COVERAGE') and not self._low_limits_applied:
            limits=rospy.get_param('~high_view_probe/low_stage_parameters')
            for item in limits:rospy.set_param(item['name'],item['value'])
            if any(rospy.get_param(item['name'])!=item['value'] for item in limits):raise RuntimeError('Low-stage parameter readback failed')
            self._low_limits_applied=True
        if action is not None and action.command=='LAND' and self.mode not in H_MODES:
            core=self._runtime.core;expected=0 if self.mode in ('memory_only','high_speed_capture') else (len(self._runtime.trial_manifest or {}) if self.mode in HIGH_MODES else getattr(self._runtime,'delivery_count',1))
            self._landing_pub.publish(String(data=json.dumps(dict(scope='board_trial_landing_after_mock',mode=self.mode,actuator_mode=rospy.get_param('~trial/actuator_mode','mock'),memory_count=len(getattr(self._runtime,'trial_manifest',None) or {}),memory_complete=self.mode=='memory_only' and action.reason=='board_memory_only_complete',capture_complete=self.mode=='high_speed_capture' and action.reason=='board_high_speed_capture_complete',mission_id=core.mission_id,decision_seq=action.decision_seq,frame=core.config.mission_frame,xy=list(core.config.landing_xy),z=core.config.return_altitude,expected=expected,committed=core.committed_slots,time=rospy.Time.now().to_sec()))))
        super()._publish_action(action)

if __name__=='__main__':rospy.init_node('mission_manager');BoardManager();rospy.spin()
