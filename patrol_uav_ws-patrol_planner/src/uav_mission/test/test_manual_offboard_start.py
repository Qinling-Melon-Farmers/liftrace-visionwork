"""Execute the real supervisor start block without ROS nodes or mode services."""
import ast
from pathlib import Path
import threading
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1]/"src/uav_mission/hardware_session.py"
TREE = ast.parse(SCRIPT.read_text())
BLOCK = next(node for node in ast.walk(TREE) if isinstance(node, ast.If)
             and isinstance(node.test, ast.BoolOp)
             and isinstance(node.test.values[0], ast.Name)
             and node.test.values[0].id == "auto_enabled")
CODE = compile(ast.fix_missing_locations(ast.Module(body=[BLOCK], type_ignores=[])), str(SCRIPT), "exec")


class ManualOffboardStart(unittest.TestCase):
    def setUp(self):
        self.wall=10.
        self.calls=[]
        def service(name, _type):
            self.calls.append(name)
            return lambda: SimpleNamespace(success=True, message="started")
        self.env=dict(auto_enabled=True, s=SimpleNamespace(connected=True,armed=True,mode="POSCTL"),
            auto_cancelled=False,offboard_seen=False,hover_since=None,start_attempted=False,
            time=SimpleNamespace(monotonic=lambda:self.wall),state_rx=[self.wall],
            startup={"state_max_age":2.5}, lock=threading.RLock(),np=np,
            rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:100.)),
                ServiceProxy=service,ServiceException=RuntimeError),
            samples=[(0.,0.,1.,0.,99.5+i*.025) for i in range(20)],reference={"takeoff_z":1.},print=Mock())
        self.srv=SimpleNamespace(Trigger=object)

    def step(self, mode=None, seconds=0., fresh=True):
        self.wall+=seconds
        if mode is not None:self.env["s"].mode=mode
        if fresh:self.env["state_rx"][0]=self.wall
        with patch.dict(sys.modules,{"std_srvs.srv":self.srv}):exec(CODE,self.env)

    def test_arm_alone_never_requests_mode_or_mission(self):
        for _ in range(10):self.step(seconds=1.)
        self.assertEqual(self.calls,[])
        self.assertFalse(self.env["start_attempted"])

    def test_manual_offboard_stable_hover_starts_once(self):
        self.step("OFFBOARD")
        self.step(seconds=1.1)
        self.step(seconds=2.)
        self.assertEqual(self.calls,["/navigation/start_mission"])

    def test_pilot_exit_latches_cancellation(self):
        self.step("OFFBOARD")
        self.step("POSCTL",.2)
        self.step("OFFBOARD",3.)
        self.assertTrue(self.env["auto_cancelled"])
        self.assertEqual(self.calls,[])

    def test_no_ground_or_stale_state_start(self):
        self.env["s"].armed=False
        self.step("OFFBOARD",2.)
        self.env["s"].armed=True
        self.env["s"].connected=False
        self.step(seconds=2.)
        self.env["s"].connected=True
        self.step(seconds=4.,fresh=False)
        self.assertEqual(self.calls,[])

    def test_wrong_hover_height_does_not_start(self):
        self.env["reference"]["takeoff_z"]=2.
        self.step("OFFBOARD")
        self.step(seconds=3.)
        self.assertEqual(self.calls,[])

    def test_supervisor_cannot_call_arming_or_mode_service(self):
        literal=[node.value for node in ast.walk(TREE) if isinstance(node,ast.Constant) and isinstance(node.value,str)]
        self.assertNotIn("/mavros/set_mode",literal)
        self.assertNotIn("/mavros/cmd/arming",literal)


if __name__=="__main__":unittest.main()
