#!/usr/bin/env python3
"""Full high-view strategy on the original simulation mission/executor chain."""
import importlib.util
from pathlib import Path
import numpy as np
import rospy
from uav_mission.high_view_full import HighViewFull
from uav_mission.boundary_revisit import BoundaryRevisit
from uav_mission.high_view_probe import ProbeConfig
from uav_high_view.survey_policy import SurveyPolicy
from uav_high_view.grid_cost import GridCost

spec=importlib.util.spec_from_file_location('high_view_probe_shell',str(Path(__file__).with_name('navigation_high_view_probe.py')))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)


class FullManager(base.ProbeManager):
    def __init__(self):
        self._grid_last=-1e9
        super().__init__()

    def _new_runtime(self):
        ordinary=base.NavigationMissionManager._new_runtime(self)
        config=dict(rospy.get_param('~high_view_probe/config'))
        config['survey_xy']=tuple(tuple(p) for p in config['survey_xy'])
        policy=SurveyPolicy(**dict(rospy.get_param('~high_view_full/policy',{})))
        runtime=HighViewFull(ordinary.core,ProbeConfig(**config),policy,fallback_route=ordinary.route,
                            boundary_policy=BoundaryRevisit(**dict(rospy.get_param('~high_view_full/boundary_policy',{}))))
        runtime.grid=GridCost(**dict(rospy.get_param('~high_view_full/grid',{})))
        return runtime

    def _on_map(self,message):
        super()._on_map(message)
        with self._lock:
            now=rospy.Time.now().to_sec()
            if self._runtime is None or now-self._grid_last<1.:return
            self._grid_last=now
            try:
                if message.header.frame_id!=self._runtime.core.config.mission_frame:raise ValueError('cost map frame')
                if message.width*message.height>250000:raise ValueError('cost map point limit')
                fields={v.name:v for v in message.fields}
                if any(k not in fields or fields[k].datatype!=7 for k in ['x','y','z']):raise ValueError('XYZ float32 required')
                dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if message.is_bigendian else '<')+'f4']*3,
                                    offsets=[fields[k].offset for k in ['x','y','z']],itemsize=message.point_step))
                array=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,strides=(message.row_step,message.point_step))
                xyz=np.column_stack([array[k].ravel() for k in ['x','y','z']])
                ground=self._runtime.probe_config.ground_z
                self._runtime.grid.update(xyz,message.header.stamp.to_sec(),ground+.4,ground+3.)
            except Exception as error:
                self._handle_callback_exception('cost_map',error)


if __name__=='__main__':
    rospy.init_node('mission_manager');FullManager();rospy.spin()
