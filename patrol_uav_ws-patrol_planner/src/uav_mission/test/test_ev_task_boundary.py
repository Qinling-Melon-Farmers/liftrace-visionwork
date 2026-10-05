import importlib.util
import io
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from uav_mission.msg import ReleasePermission
from uav_mission.task_frame_continuity import BoundaryRejected, FcState, Pose, Transform
fixture_spec=importlib.util.spec_from_file_location('ev_boundary_fixtures',str(
    Path(__file__).with_name('test_task_frame_continuity.py')))
fixtures=importlib.util.module_from_spec(fixture_spec); fixture_spec.loader.exec_module(fixtures)
boundary, ready, pair, reset=fixtures.boundary, fixtures.ready, fixtures.pair, fixtures.reset

path=Path(__file__).resolve().parents[1]/'scripts/ev_task_boundary.py'
spec=importlib.util.spec_from_file_location('ev_task_boundary_node',str(path))
node=importlib.util.module_from_spec(spec); spec.loader.exec_module(node)


def health(t,epoch='lio-boot-1',healthy=True):
    return String(data=json.dumps(dict(version=1,stamp=t,lio_epoch=epoch,frame_id='lio',
                                       healthy=healthy,body_calibrated=True)))


def reset_msg(t=1.2):
    return String(data=json.dumps(dict(version=1,stamp=t,fc_epoch='fc-boot-1',
        previous_counter=0,counter=1,frame_id='map',authoritative=True,
        translation_new_from_previous=[0.,0.,.524],
        quaternion_new_from_previous_xyzw=[0.,0.,0.,1.])))


def observation(n,t,z=.6,fc_z=None):
    n.core.update_state(FcState(t,True,True,'OFFBOARD'))
    n.lio_msg=node.to_pose(Pose(t,'lio',(0.,0.,z),(0.,0.,0.,1.)))
    n.health=node.parse_health(health(t).data)
    out=Odometry(); out.header.stamp=rospy.Time.from_sec(t); out.header.frame_id='map'
    out.child_frame_id='base_link'; out.pose.pose.orientation.w=1.
    out.pose.pose.position.z=z if fc_z is None else fc_z
    out.pose.covariance=np.eye(6).ravel().tolist()
    out.twist.twist.linear.x=.1
    n.on_fc(out)


class RosBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.n=node.EvTaskBoundary.__new__(node.EvTaskBoundary)
        self.n.lock=threading.RLock(); self.n.core=ready()
        self.n.fc_msg=self.n.lio_msg=self.n.health=self.n.permission=None
        self.n.pubs={name:Mock() for name in ('task_pose','task_odom','camera_pose',
             'fc_setpoint','fc_hold_request','release_permission','status')}

    def test_ros_reset_feedback_projection_and_fc_command_agree(self):
        n=self.n
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.2)):
            n.on_reset(reset_msg())
        for t in (1.21,1.27,1.33):
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(t)):
                observation(n,t,fc_z=1.124)
        body=n.pubs['task_pose'].publish.call_args[0][0]
        cam=n.pubs['camera_pose'].publish.call_args[0][0]
        odom=n.pubs['task_odom'].publish.call_args[0][0]
        self.assertAlmostEqual(body.pose.position.z,.6)
        self.assertAlmostEqual(cam.pose.position.z,.44)
        self.assertEqual(body.header.stamp,rospy.Time.from_sec(1.33))
        self.assertEqual(odom.child_frame_id,'base_link')
        self.assertEqual(odom.twist.twist.linear.x,.1)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.33)):
            n.on_setpoint(node.to_pose(Pose(1.33,'task',(0.,0.,.6),(0.,0.,0.,1.))))
            n.on_timer(None)
        command=n.pubs['fc_setpoint'].publish.call_args[0][0]
        self.assertEqual(command.header.frame_id,'map')
        self.assertAlmostEqual(command.pose.position.z,1.124)
        status=json.loads(n.pubs['status'].publish.call_args[0][0].data)
        self.assertTrue(status['observe_only']); self.assertTrue(status['ready'])
        self.assertEqual(status['generation'],1)
        self.assertAlmostEqual(status['camera_agl'],.44)
        stream=io.BytesIO(); body.serialize(stream)
        decoded=PoseStamped(); decoded.deserialize(stream.getvalue())
        self.assertEqual(decoded,body)

    def test_reset_immediately_revokes_old_permission_and_descent(self):
        n=self.n
        permission=ReleasePermission()
        permission.header.stamp=rospy.Time.from_sec(1.18)
        permission.valid_until=rospy.Time.from_sec(2.)
        permission.permitted=True
        n.on_permission(permission)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.2)):
            n.on_reset(reset_msg()); n.on_timer(None)
            n.on_setpoint(node.to_pose(Pose(1.2,'task',(0.,0.,.3),(0.,0.,0.,1.))))
        result=n.pubs['release_permission'].publish.call_args[0][0]
        self.assertFalse(result.permitted)
        self.assertEqual(result.header.stamp,permission.header.stamp)
        self.assertEqual(result.valid_until,permission.valid_until)
        n.pubs['fc_setpoint'].publish.assert_not_called()
        n.pubs['fc_hold_request'].publish.assert_not_called() # no verified new FC pair yet
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.21)):
            observation(n,1.21,fc_z=1.124); n.on_timer(None)
        hold=n.pubs['fc_hold_request'].publish.call_args[0][0]
        self.assertAlmostEqual(hold.pose.position.z,1.124)
        # Timer must never turn an old permission into a fresh one.
        self.assertEqual(n.pubs['release_permission'].publish.call_args[0][0].header.stamp,
                         permission.header.stamp)

    def test_malformed_or_untrusted_reset_cannot_remap(self):
        n=self.n
        data=json.loads(reset_msg().data); data['authoritative']='true'
        with patch.object(rospy,'logwarn_throttle'):
            n.on_reset(String(data=json.dumps(data)))
        self.assertEqual(n.core.generation,0)
        self.assertTrue(n.core.fault)

    def test_malformed_lio_health_inhibits_outputs(self):
        with patch.object(rospy,'logwarn_throttle'):
            self.n.on_health(String(data='{}'))
        self.assertFalse(self.n.core.ready)
        self.assertTrue(self.n.core.fault)

    def test_stale_lio_no_projection_or_command(self):
        n=self.n
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.5)):
            n.on_timer(None)
            n.on_setpoint(node.to_pose(Pose(1.5,'task',(0.,0.,.3),(0.,0.,0.,1.))))
        n.pubs['fc_setpoint'].publish.assert_not_called()
        self.assertFalse(json.loads(n.pubs['status'].publish.call_args[0][0].data)['ready'])

    def test_source_epoch_change_never_treated_as_fc_reset(self):
        n=self.n
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.2)):
            observation(n,1.2)
            n.on_health(health(1.2,epoch='lio-restarted'))
        self.assertTrue(n.core.fault)
        self.assertEqual(n.core.generation,0)

    def test_covariance_and_child_velocity_preserved_with_rotation(self):
        n=self.n
        tf=Transform((1.,-2.,0.),(0.,0.,np.sqrt(.5),np.sqrt(.5)))
        n.core=boundary(task_from_fc=tf)
        for t in (1.,1.06,1.12):
            fc=Pose(t,'map',(0.,0.,.6),(0.,0.,0.,1.))
            n.lio_msg=node.to_pose(fc.transformed(tf,'lio'))
            n.health=node.parse_health(health(t).data)
            n.core.update_state(FcState(t,True,False,'POSCTL'))
            out=Odometry(); out.header.stamp=rospy.Time.from_sec(t); out.header.frame_id='map'
            out.child_frame_id='base_link'; out.pose.pose=node.to_pose(fc).pose
            out.pose.covariance=np.diag([1,4,9,16,25,36]).ravel().tolist()
            out.twist.twist.linear.x=.25
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(t)): n.on_fc(out)
        published=n.pubs['task_odom'].publish.call_args[0][0]
        np.testing.assert_allclose(np.diag(np.array(published.pose.covariance).reshape(6,6)),
                                   [4,1,9,25,16,36])
        self.assertEqual(published.twist,out.twist)

    def test_observer_cannot_publish_live_outputs(self):
        config=dict(task_frame='task',fc_frame='map',lio_frame='lio',
                    fc_epoch='fc-boot-1',lio_epoch='lio-boot-1',initial_reset_counter=0,
                    calibration_verified=False,ground_z=0.,
                    task_from_fc=dict(xyz=[0.,0.,0.],xyzw=[0.,0.,0.,1.]),
                    task_from_lio=dict(xyz=[0.,0.,0.],xyzw=[0.,0.,0.,1.]),
                    body_to_camera=dict(xyz=[0.,0.,-.16],xyzw=[0.,1.,0.,0.]))
        for settings in ({'~observer_namespace':'/mavros/setpoint_position'},
                         {'~fc_setpoint_output':'/mavros/setpoint_position/local'}):
            def get_param(key,default=None):
                if key=='~reference': return config
                return settings.get(key,default)
            with patch.object(rospy,'get_param',side_effect=get_param), \
                 patch.object(rospy,'resolve_name',side_effect=lambda x:x), \
                 patch.object(rospy,'Publisher') as publisher:
                with self.assertRaises(BoundaryRejected): node.EvTaskBoundary()
                self.assertNotIn('/mavros/setpoint_position/local',
                    [call.args[0] for call in publisher.call_args_list])

    def test_new_permission_requires_post_reset_visual_evidence(self):
        n=self.n
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.2)):
            n.on_reset(reset_msg())
        for t in (1.21,1.27,1.33):
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(t)):
                observation(n,t,fc_z=1.124)
        permission=ReleasePermission(); permission.permitted=True
        permission.header.stamp=rospy.Time.from_sec(1.33)
        permission.valid_until=rospy.Time.from_sec(1.5)
        permission.evidence_stamp=rospy.Time.from_sec(1.18)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.33)):
            n.on_permission(permission); n.on_timer(None)
        self.assertFalse(n.pubs['release_permission'].publish.call_args[0][0].permitted)
        permission.evidence_stamp=rospy.Time.from_sec(1.32)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(1.33)):
            n.on_permission(permission); n.on_timer(None)
        self.assertTrue(n.pubs['release_permission'].publish.call_args[0][0].permitted)

    def test_contract_requires_true_boolean_and_sequence(self):
        data=json.loads(reset_msg().data)
        data['counter']=True
        with self.assertRaises(BoundaryRejected): node.parse_reset(json.dumps(data))
        data['counter']=1; data['version']=2
        with self.assertRaises(BoundaryRejected): node.parse_reset(json.dumps(data))


if __name__=='__main__': unittest.main()
