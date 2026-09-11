"""Offline adapter tests; never instantiate ROS subscriptions or a ROS master."""
import ast
import importlib.util
import threading
import os
from collections import deque
import unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
import rospy
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from uav_coverage_memory.memory import Config, Memory

PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('coverage_shadow_test',PACKAGE/'scripts/coverage_memory_shadow.py')
adapter=importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class ShadowTests(unittest.TestCase):
    def setUp(self):
        self.node=adapter.Shadow.__new__(adapter.Shadow)
        self.node.lock=threading.RLock()
        self.node.memory=Memory(Config(-1,1,-1,1,.1,0,'map'))
        self.node.mission_id=None
        self.node.image=self.node.cloud=object()
        self.node.infos=deque(maxlen=8)
        self.node.clouds=deque(maxlen=4)

    def test_default_disabled_and_no_flight_include(self):
        launch=ET.parse(PACKAGE/'launch/shadow.launch').getroot()
        self.assertEqual(launch.find("arg[@name='enabled']").get('default'),'false')
        self.assertEqual(len(list(launch.iter('include'))),0)
        self.assertEqual([n.get('pkg') for n in launch.iter('node')],['uav_coverage_memory'])

    def test_only_private_diagnostic_publishers_and_reset_service(self):
        tree=ast.parse((PACKAGE/'scripts/coverage_memory_shadow.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        pubs=[n.args[0].value for n in calls if n.func.attr=='Publisher']
        self.assertEqual(set(pubs),{'~state_grid','~status'})
        self.assertFalse(any(n.func.attr=='ServiceProxy' for n in calls))
        self.assertNotIn('/fastplanner/goal',(PACKAGE/'scripts/coverage_memory_shadow.py').read_text())

    def test_new_mission_resets_but_repeated_status_does_not(self):
        with patch.object(rospy.Time,'now',return_value=rospy.Time(10)):
            self.node.on_mission(String(data='{"mission_id":"round1"}'))
            generation=self.node.memory.generation
            self.assertIsNone(self.node.image);self.assertIsNone(self.node.cloud)
            self.node.on_mission(String(data='{"mission_id":"round1"}'))
            self.assertEqual(self.node.memory.generation,generation)
            self.node.on_mission(String(data='{"mission_id":"round2"}'))
            self.assertEqual(self.node.memory.generation,generation+1)

    def test_invalid_mission_json_has_no_reset(self):
        generation=self.node.memory.generation
        for text in ['{bad','[]','{"mission_id":5}']:
            self.node.on_mission(String(data=text))
        self.assertEqual(generation,self.node.memory.generation)

    def test_explicit_reset_is_only_local(self):
        with patch.object(rospy.Time,'now',return_value=rospy.Time(10)):
            reply=self.node.on_reset(None)
        self.assertTrue(reply.success)
        self.assertIn('external map not reset',reply.message)

    def test_tf_rotation_and_translation(self):
        tf=TransformStamped();tf.transform.rotation.w=1;tf.transform.translation.x=2
        matrix=adapter.transform_matrix(tf)
        np.testing.assert_allclose(matrix[:3,:3],np.eye(3))
        self.assertEqual(matrix[0,3],2)
        tf.transform.rotation.w=0
        with self.assertRaises(ValueError):adapter.transform_matrix(tf)

    def test_queued_image_before_reset_epoch_rejected(self):
        self.node.image=Image();self.node.info=CameraInfo()
        self.node.image.header.stamp=rospy.Time(9)
        self.node.epoch_start=10
        self.assertEqual(self.node.process(10)['reason'],'image_before_epoch')

    def test_no_new_image_does_not_count_timer_ticks(self):
        self.node.image=Image();self.node.info=CameraInfo()
        self.node.image.header.stamp=rospy.Time(10)
        self.node.last_processed=10;self.node.epoch_start=9
        self.node.memory.count.fill(1)
        self.assertEqual(self.node.process(10.1)['reason'],'no_new_image')
        self.assertTrue((self.node.memory.count==1).all())

    def test_oversized_cloud_remains_unknown_without_truncation(self):
        self.node.cloud=PointCloud2();self.node.cloud.header.stamp=rospy.Time(10)
        self.node.cloud.width=self.node.memory.config.max_cloud_points+1;self.node.cloud.height=1
        self.node.epoch_start=9
        self.node.on_cloud(self.node.cloud)
        self.assertEqual(self.node.map_snapshot(10.1),(None,None))

    def test_exact_tf_lookup_has_no_latest_fallback(self):
        class FakeTF:
            def __init__(self):self.requests=[]
            def lookup_transform(self,target,source,stamp,timeout):
                self.requests.append(stamp.to_sec())
                raise ValueError('no transform at source time')
        self.node.tf=FakeTF();self.node.tf_timeout=.03
        with self.assertRaises(ValueError):self.node.lookup('map','camera',rospy.Time(10))
        self.assertEqual(self.node.tf.requests,[10.])

    def test_source_time_chooses_past_snapshot_over_latest_future(self):
        self.node.epoch_start=9
        past=CameraInfo();past.header.stamp=rospy.Time.from_sec(10.)
        future=CameraInfo();future.header.stamp=rospy.Time.from_sec(10.2)
        self.node.on_info(past);self.node.on_info(future)
        self.assertIs(self.node.as_of(self.node.infos,10.05,2),past)
        self.assertIsNone(self.node.as_of(self.node.infos,9.5,2))

    def test_snapshot_buffers_are_bounded(self):
        for i in range(20):
            self.node.on_info(CameraInfo())
            self.node.on_cloud(PointCloud2())
        self.assertEqual(len(self.node.infos),8)
        self.assertEqual(len(self.node.clouds),4)

    def test_research_trial_requires_wrapper_run_directory(self):
        args=['target_model_path:=/tmp/model.pt','field_seed:=32','world:=/tmp/world',
              'field_config:=/tmp/field','runtime_config:=/tmp/runtime','gate_geometry_config:=/tmp/gate']
        with patch.dict(os.environ):
            os.environ.pop('SIM_RUN_DIR',None)
            with self.assertRaisesRegex(roslaunch.RLException,'SIM_RUN_DIR'):
                roslaunch.config.load_config_default([(str(PACKAGE/'launch/research_trial.launch'),args)],None)


if __name__=='__main__':unittest.main()
