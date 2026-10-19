#!/usr/bin/env python3
"""ROS message/bag round trip; no ROS master or published flight topics."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Imu
from fast_lio.msg import PredictionState

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/
    'patrol_uav_ws-patrol_planner/src/FAST_LIO/scripts'))
from ev_shadow import Shadow


class TransportTests(unittest.TestCase):
    def test_generated_snapshot_round_trip_and_replay(self):
        with tempfile.TemporaryDirectory(prefix='ev_transport_') as tmp:
            root=Path(tmp); events=[]
            for i in range(201):
                t=10+i*.005
                imu=Imu(); imu.header.stamp=rospy.Time.from_sec(t)
                imu.header.frame_id='livox_frame'; imu.linear_acceleration.z=9.81
                events.append((t,'/livox/imu',imu))
                if i % 20 == 4:
                    s=PredictionState(); s.header.stamp=rospy.Time.from_sec(t)
                    s.header.frame_id='camera_init'; s.imu_frame='livox_frame'
                    s.valid=True; s.epoch=5; s.pose.orientation.w=1.
                    s.pose.position.x=t-10; s.velocity.x=1.; s.gravity.z=-9.81
                    s.accel_scale=1.; s.covariance=(np.eye(18)*1e-5).ravel().tolist()
                    s.process_noise=[1e-4]*12
                    events.append((t+.07,'/laserMapping/prediction_state',s))
            with rosbag.Bag(str(root/'input.bag'),'w') as bag:
                for t,topic,msg in sorted(events,key=lambda row:row[0]):
                    bag.write(topic,msg,rospy.Time.from_sec(t))
            result=subprocess.run([sys.executable,str(Path(__file__).with_name('replay_bag.py')),
                                   str(root/'input.bag'),'--out',str(root/'result')],
                                  capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            summary=json.loads((root/'result/summary.json').read_text())
            self.assertIsNone(summary['final_fault'])
            with (root/'result/shadow_replay.csv').open() as f:
                rows=list(csv.DictReader(f))
            self.assertGreater(len(rows),30)
            prev=0.
            for row in rows:
                stamp=float(row['stamp'])
                self.assertGreater(stamp,prev); prev=stamp
                self.assertLessEqual(stamp,float(row['receipt'])+1e-6)
                self.assertAlmostEqual(float(row['smooth_x']),stamp-10,places=5)

    def test_live_topic_remap_rejected_before_subscription(self):
        def param(name, default=None):
            return default
        with patch('ev_shadow.rospy.get_param',side_effect=param), \
             patch('ev_shadow.rospy.resolve_name',return_value='/mavros/vision_pose/pose'), \
             patch('ev_shadow.rospy.Publisher') as publish, \
             patch('ev_shadow.rospy.Subscriber') as subscribe:
            with self.assertRaisesRegex(ValueError,'cannot drive flight'):
                Shadow()
            publish.assert_not_called(); subscribe.assert_not_called()


if __name__=='__main__':
    unittest.main()
