import importlib.util
from pathlib import Path
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

spec=importlib.util.spec_from_file_location('alignment',str(Path(__file__).resolve().parents[1]/'scripts/map_camera_alignment.py'))
alignment=importlib.util.module_from_spec(spec);spec.loader.exec_module(alignment)

class AlignmentTests(unittest.TestCase):
    def setUp(self):
        n=alignment.MapCameraAlignment.__new__(alignment.MapCameraAlignment)
        n.lio_frame='camera_init';n.mavros_frame='map';n.sync_slop=.04;n.max_age=.3
        n.sample_count=3;n.max_spread=.1;n.max_angle_spread=.1;n.require_disarmed=True
        n.samples=deque(maxlen=3);n.jump_samples=deque(maxlen=3);n.alignment=None
        n.jump_translation=.5;n.jump_angle=.2;n.update_alpha=.1
        n.state=SimpleNamespace(connected=True,armed=False)
        n.lio=PoseStamped();n.lio.header.frame_id='camera_init';n.lio.pose.position.x=3;n.lio.pose.position.y=-1
        n.lio.pose.orientation.z=np.sqrt(.5);n.lio.pose.orientation.w=np.sqrt(.5)
        n.mavros=PoseStamped();n.mavros.header.frame_id='map';n.mavros.pose.position.x=1;n.mavros.pose.orientation.w=1
        self.n=n
    def update(self,count=1):
        for i in range(count):
            stamp=rospy.Time.from_sec(10+i*.01);self.n.lio.header.stamp=stamp;self.n.mavros.header.stamp=stamp
            with patch.object(rospy.Time,'now',return_value=stamp),patch.object(rospy,'logwarn_throttle'),patch.object(rospy,'loginfo'):
                self.n._update()
    def test_nonidentity_camera_to_map_initialization(self):
        self.update(3);t,q=self.n.alignment
        np.testing.assert_allclose(t,[3,-2,0],atol=1e-12)
        np.testing.assert_allclose(q,[0,0,np.sqrt(.5),np.sqrt(.5)])
    def test_initialization_requires_disarmed(self):
        self.n.state.armed=True;self.update(3);self.assertIsNone(self.n.alignment)
    def test_mislabelled_frame_rejected(self):
        self.n.mavros.header.frame_id='camera_init';self.update(3);self.assertIsNone(self.n.alignment)
    def test_single_jump_does_not_realign_but_consistent_reset_does(self):
        self.update(3);self.n.lio.pose.position.x=13
        self.update();np.testing.assert_allclose(self.n.alignment[0],[3,-2,0],atol=1e-12)
        self.update(3);np.testing.assert_allclose(self.n.alignment[0],[13,-2,0],atol=1e-12)

if __name__=='__main__':unittest.main()
