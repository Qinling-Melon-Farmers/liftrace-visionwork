import unittest
import subprocess,sys,tempfile
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace as N
import threading
from geometry_msgs.msg import PoseStamped
from trial_terminal_hover import TerminalHover,descend

class TerminalTests(unittest.TestCase):
    def node(self):
        n=TerminalHover.__new__(TerminalHover)
        n.lock=threading.RLock();n.frame='camera_init';n.cap=1.78;n.target=.08;n.speed=.15
        n.raw=PoseStamped();n.raw.header.frame_id=n.frame;n.raw.pose.position.z=2.5
        n.raw_at=10.;n.odom=None;n.state=N(armed=True,mode='OFFBOARD',connected=True)
        n.state_at=10.;n.status_at=10.;n.status={};n.context={}
        n.active=False;n.cancelled=False;n.since=None;n.setpoint=None;n.previous=10.
        n.outputs=[];n.pub=N(publish=n.outputs.append);n.status_pub=N(publish=lambda m:None)
        return n
    def test_catkin_relay_does_not_import_sibling_relay(self):
        source=Path(__file__).resolve().parents[1]/'scripts/trial_terminal_hover.py'
        with tempfile.TemporaryDirectory() as directory:
            shadow=Path(directory)
            (shadow/'trial_auto_land.py').write_text('# catkin relay does not export helpers\n')
            code="import sys;sys.path.insert(0,"+repr(directory)+");p="+repr(str(source))+";exec(compile(open(p).read(),p,'exec'),{'__file__':p,'__name__':'relay_smoke'})"
            subprocess.run([sys.executable,'-c',code],cwd=directory,check=True,capture_output=True,text=True)
    def test_ramp_cannot_overshoot_or_jump_after_pause(self):
        self.assertEqual(descend(.081,.08,.15,.1),.08)
        self.assertAlmostEqual(descend(1.,.08,.15,20.),.9775)
    @patch('trial_terminal_hover.rospy.Time.now')
    def test_cap_and_no_terminal_from_unverified_land(self,clock):
        clock.return_value.to_sec.return_value=10.
        clock.return_value=__import__('rospy').Time.from_sec(10.)
        n=self.node();n.tick(None)
        self.assertFalse(n.active);self.assertAlmostEqual(n.outputs[-1].pose.position.z,1.78)
    @patch('trial_terminal_hover.rospy.Time.now')
    def test_takeover_latches_last_hold_not_old_cruise(self,clock):
        clock.return_value=__import__('rospy').Time.from_sec(10.)
        n=self.node();n.active=True;n.setpoint=PoseStamped();n.setpoint.pose.position.z=.12
        n.state.mode='POSCTL';n.tick(None)
        self.assertTrue(n.cancelled);self.assertFalse(n.active)
        self.assertAlmostEqual(n.outputs[-1].pose.position.z,.12)
        n.state.mode='OFFBOARD';n.tick(None)
        self.assertFalse(n.active);self.assertAlmostEqual(n.outputs[-1].pose.position.z,.12)
if __name__=='__main__':unittest.main()