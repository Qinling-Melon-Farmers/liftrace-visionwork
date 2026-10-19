"""Exercise actual dependency classes with a fake ROS transport; no ROS init."""
import importlib.abc, importlib.util, os, sys, types, unittest
import numpy as np
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[5]
SCRIPTS=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts'
sys.path[:0]=[str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),
             str(ROOT/'vision_ws/src/uav_high_view/src'),
             str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/test')]
from uav_mission.coverage_route import CoverageRoute
from uav_mission.mission_core import MissionCore, MissionConfig
from uav_mission.search_types import Waypoint
from test_mission_runtime import profile

class BaseShell:
    def __init__(self): self._runtime=None
    def _new_runtime(self):
        return types.SimpleNamespace(core=MissionCore(profile(),MissionConfig()),
            route=CoverageRoute([Waypoint(1.,0.,1.4)],'offline-hardware-entry'))

class BaseLoader(importlib.abc.Loader):
    def create_module(self,spec): return None
    def exec_module(self,module): module.NavigationMissionManager=BaseShell

class HardwareFullEntry(unittest.TestCase):
    def setUp(self):
        params={'/use_sim_time':False,'~high_view_probe/config':dict(ground_z=0.,
            survey_xy=[[1.,-1.],[3.,-1.],[3.,1.],[1.,1.]],high_agl=2.,low_agl=1.4),
            '~high_view_full/policy':{}}
        ros=types.ModuleType('rospy')
        ros.get_param=lambda name,default=None:params.get(name,default)
        ros.Publisher=lambda *a,**k:types.SimpleNamespace(publish=lambda *a,**k:None)
        ros.Subscriber=lambda *a,**k:types.SimpleNamespace(unregister=lambda:None)
        mocks={'rospy':ros}
        for pkg,name in [('sensor_msgs','CameraInfo'),('std_msgs','String'),('uav_vision','TargetDetectionArray')]:
            module=types.ModuleType(pkg);msg=types.ModuleType(pkg+'.msg')
            setattr(msg,name,type(name,(),{}));module.msg=msg;mocks[pkg]=module;mocks[pkg+'.msg']=msg
        self.modules=patch.dict(sys.modules,mocks);self.modules.start();self.addCleanup(self.modules.stop)
        self.env=patch.dict(os.environ);self.env.start();self.addCleanup(self.env.stop)
        os.environ.pop('SIM_RUN_DIR',None)
        actual_spec=importlib.util.spec_from_file_location
        def source_spec(name,path,*args,**kwargs):
            if Path(path).name=='navigation_mission_manager.py':
                return importlib.util.spec_from_loader(name,BaseLoader())
            return actual_spec(name,path,*args,**kwargs)
        with patch.object(importlib.util,'spec_from_file_location',side_effect=source_spec):
            spec=actual_spec('offline_actual_competition_entry',SCRIPTS/'navigation_competition_manager.py')
            self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)

    def test_hardware_constructor_bypasses_only_probe_sim_guard(self):
        manager=self.module.CompetitionManager(hardware=True)
        self.assertIsNone(manager._runtime)
        with self.assertRaisesRegex(RuntimeError,'simulation-only'):
            self.module.full.FullManager()

    def test_new_runtime_really_uses_hardware_subclass(self):
        manager=self.module.CompetitionManager(hardware=True)
        runtime=manager._new_runtime()
        self.assertIsInstance(runtime,self.module.HardwareFullRuntime)
        output=runtime.start('offline-entry',100.,(0.,0.))
        self.assertIsNotNone(output)
        self.assertEqual(runtime.catalog.epoch.localization,'fixed-board-session')

if __name__=='__main__':unittest.main()
