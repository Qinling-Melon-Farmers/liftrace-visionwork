"""Exercise production aligner/projector methods offline, with actual ROS messages."""
import copy
import importlib.util
import math
from collections import deque
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo
from scipy.spatial.transform import Rotation
from uav_vision.msg import TargetDetection, TargetDetectionArray
from uav_vision.drop_geometry_policy import slot_goal, pixel_equivalent_limit
from uav_vision.ground_projection import intersect_ground
import test_drop_observation_stamp as stamp_test
ALIGNER = stamp_test.ALIGNER

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("drop_shared_projector_test", PACKAGE/"scripts/target_map_projector.py")
PROJECTOR = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PROJECTOR)


def transform(parent, child, stamp, xyz, quaternion):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp = stamp if isinstance(stamp, rospy.Time) else rospy.Time.from_sec(stamp)
    t, q = msg.transform.translation, msg.transform.rotation
    t.x, t.y, t.z = xyz
    q.x, q.y, q.z, q.w = quaternion
    return msg


def camera(stamp=0.):
    msg = CameraInfo()
    msg.header.frame_id = "downward_camera_optical_frame"
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.width, msg.height = 1280, 720
    msg.K = [800.,0.,640.,0.,800.,360.,0.,0.,1.]
    msg.P = [800.,0.,640.,0.,0.,800.,360.,0.,0.,0.,1.,0.]
    msg.R = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
    msg.D = [0.]*5
    msg.distortion_model = "plumb_bob"
    return msg


class DropGeometryTest(stamp_test.DropObservationStampTest):
    def fixture(self, mode="drop_circle", last_seen=99.8):
        node, target = super().fixture(mode, last_seen)
        node._state_lock = __import__('threading').RLock()
        node._exact_projection = True
        node._map_frame, node._body_frame = "camera_init", "vision_body"
        node._ground_z, node._tf_timeout = 0., .05
        node._max_error_m, node._body_max_age = 0., .1
        node._slot_mode = "zero"
        node._slot_positions_body = []
        node._camera_info_max_skew = .1
        node._camera_info_mode = "fixed"
        node._allow_static_camera_info = True
        node._rectify_input_pixels = True
        node._camera_model = ALIGNER.PinholeCameraModel()
        node._camera_ready = False
        node._calibrations, node._mapped_frames = deque(maxlen=64), deque(maxlen=64)
        node._projection_jacobians = {}
        node._projection_principal_points = {}
        node._on_camera_info(camera())
        node._alignment_context.target_pose.pose.position = Point(0.,0.,0.)
        target.association_valid = True
        target.map_valid = True
        target.map_point = Point(20.,30.,0.)  # deliberately wrong fused history
        self.source_frame(node, target)
        node._tf_buffer = Mock()
        node._tf_buffer.lookup_transform.side_effect = lambda parent, child, stamp, timeout: (
            transform(parent, child, stamp, (0.,0.,1.), (0.,1.,0.,0.))
            if child != node._body_frame else
            transform(parent, child, 100., (-.01,.0075,1.), (0.,0.,0.,1.)))
        return node, target

    @staticmethod
    def source_frame(node, target):
        det = TargetDetection()
        det.class_name, det.center_px = target.class_name, copy.deepcopy(target.center_px)
        det.center_refined = det.association_valid = True
        frame = TargetDetectionArray(detections=[det])
        frame.header.stamp = target.last_seen
        frame.header.frame_id = "downward_camera_optical_frame"
        node._on_mapped_frame(frame)

    # Inherited legacy timestamp tests use a legacy fixture, separately run in
    # test_drop_observation_stamp. Here assert the exact interface and evidence.
    def test_drop_modes_use_observation_stamp_without_mutating_candidate(self):
        for mode in ("drop_circle", "drop_cross"):
            node, target = self.fixture(mode)
            self.publish(node,target)
            msg = node._offset_pub.publish.call_args.args[0]
            self.assertEqual(msg.header.stamp,target.last_seen)
            self.assertEqual(msg.header.frame_id,"downward_camera_optical_frame")
            self.assertTrue(msg.map_valid)
            self.assertAlmostEqual(msg.map_point.x,-.01)
            self.assertAlmostEqual(msg.map_point.y,.0075)
            self.assertEqual((target.map_point.x,target.map_point.y),(20.,30.))
            self.assertEqual(msg.target_id,target.id)

    def test_stability_evidence_and_ready_timestamps_are_unchanged(self):
        node,target=self.fixture()
        self.publish(node,target)
        for _ in range(5): self.publish(node,target)
        self.assertEqual(node._consecutive_ok,1)
        self.assertEqual(node._evidence_pub.publish.call_count,1)
        target.last_seen=rospy.Time.from_sec(99.9)
        self.source_frame(node,target)
        self.publish(node,target)
        self.assertTrue(node._ready_pub.publish.call_args.args[0].ready)
        original=node._evidence_pub.publish.call_args.args[0].header.stamp
        for _ in range(12): self.publish(node,target)
        self.assertEqual(node._consecutive_ok,2)
        self.assertEqual(node._evidence_pub.publish.call_count,2)
        self.assertEqual(node._evidence_pub.publish.call_args.args[0].header.stamp,original)
        self.assertEqual(node._offset_pub.publish.call_count,2)
        older=copy.deepcopy(target);older.last_seen=rospy.Time.from_sec(99.8)
        self.publish(node,older)
        self.assertEqual(node._evidence_pub.publish.call_count,2)
        self.assertEqual(node._active_geometry_last_seen,target.last_seen)
        self.clock.return_value=rospy.Time.from_sec(100.5)
        node._last_context_watchdog_reason=None
        node._on_alignment_context_watchdog(None)
        self.assertFalse(node._ready_pub.publish.call_args.args[0].ready)

    def test_republished_red_cross_keeps_the_same_pixel_observation_time(self):
        node,target=self.fixture("drop_cross")
        self.publish(node,target)
        self.publish(node,target)
        self.assertEqual(node._offset_pub.publish.call_count,1)

    def test_landing_keeps_its_existing_header_and_timestamp(self):
        # Exact drop mode never changes H's offset or its pixel stability.
        node,target=super().fixture("landing")
        node._exact_projection=True
        self.publish(node,target)
        self.assertIs(node._offset_pub.publish.call_args.args[0].header,target.header)

    def test_zero_and_expired_observations_keep_existing_rejection(self):
        for stamp in (0.,99.49,100.01):
            node,target=self.fixture(last_seen=stamp)
            self.publish(node,target)
            self.assertEqual(node._consecutive_ok,0)
            node._offset_pub.publish.assert_not_called()

    def test_roll_pitch_yaw_exposure_projection_shared_with_h(self):
        for angles in ((0,0,0),(.2,0,0),(0,-.25,0),(0,0,1.1),(.15,-.2,.7)):
            with self.subTest(angles=angles):
                node,target=self.fixture()
                optical=(Rotation.from_euler("xyz",angles)*Rotation.from_quat((0.,1.,0.,0.))).as_quat()
                camera_tf=transform(node._map_frame,"downward_camera_optical_frame",99.8,(.04,-.03,1.),optical)
                node._tf_buffer.lookup_transform.return_value=camera_tf
                node._tf_buffer.lookup_transform.side_effect=None
                projected=node._project_current_target(target)
                # Independent SciPy matrix reference (conda has no ROS PyKDL),
                # including original mounting translation.
                ray=node._calibrations[0][1].projectPixelTo3dRay((648,366))
                direction=Rotation.from_quat(optical).apply(ray)
                scale=-1./direction[2]
                self.assertAlmostEqual(projected.map_point.x,.04+scale*direction[0])
                self.assertAlmostEqual(projected.map_point.y,-.03+scale*direction[1])
                h=PROJECTOR.TargetMapProjector.__new__(PROJECTOR.TargetMapProjector)
                h._camera_model=node._calibrations[0][1]
                h._camera_ready=True;h._camera_has_distortion=False
                h._rectify_input_pixels=True;h._map_frame=node._map_frame
                h._ground_z=0.;h._ray_epsilon=1e-5;h._tf_timeout=.05
                h._allow_latest_tf_fallback=False;h._tf_buffer=node._tf_buffer
                det=TargetDetection(center_px=copy.deepcopy(target.center_px),center_refined=True,association_valid=True)
                ok,reason=h._project(det,target.last_seen,"downward_camera_optical_frame")
                self.assertTrue(ok,reason)
                self.assertEqual(det.map_point,projected.map_point)
                self.assertEqual(node._tf_buffer.lookup_transform.call_args.args[2],target.last_seen)

    def test_missing_mismatched_and_wrong_time_tf_fail_closed(self):
        for failure in ("missing","time","frame","horizon","behind","nan"):
            node,target=self.fixture()
            tf=transform(node._map_frame,"downward_camera_optical_frame",99.8,(0.,0.,1.),(0.,1.,0.,0.))
            if failure=="time":tf.header.stamp=rospy.Time(100)
            if failure=="frame":tf.child_frame_id="different_camera"
            if failure=="horizon":
                tf.transform.rotation.y=math.sqrt(.5);tf.transform.rotation.w=math.sqrt(.5)
                target.center_px=Point(640.,360.,40.);self.source_frame(node,target)
            if failure=="behind":tf.transform.rotation.y=0.;tf.transform.rotation.w=1.
            if failure=="nan":tf.transform.translation.x=float("nan")
            node._tf_buffer.lookup_transform.side_effect=None
            node._tf_buffer.lookup_transform.return_value=tf
            if failure=="missing":node._tf_buffer.lookup_transform.side_effect=tf2_ros.ExtrapolationException("missing")
            self.publish(node,target)
            self.assertFalse(node._ready_pub.publish.call_args.args[0].ready)
            node._offset_pub.publish.assert_not_called()

    def test_exposure_calibration_distortion_and_version(self):
        node,target=self.fixture()
        info=camera(99.)
        info.D=[.2,-.05,.001,-.002,0.]
        node._on_camera_info(info)
        target.center_px=Point(800.,420.,40.)
        self.source_frame(node,target)
        projected=node._project_current_target(target)
        pixel=node._calibrations[-1][1].rectifyPoint((800.,420.))
        expected=intersect_ground(node._calibrations[-1][1].projectPixelTo3dRay(pixel),(0,0,1),(0,1,0,0),0)
        self.assertAlmostEqual(projected.map_point.x,expected[0])
        self.assertNotAlmostEqual(projected.map_point.x,-.2)
        future=camera(99.9);future.K[0]=1600.;future.P[0]=1600.
        node._on_camera_info(future)
        self.assertAlmostEqual(node._project_current_target(target).map_point.x,expected[0])
        target.last_seen=rospy.Time.from_sec(99.95)
        self.source_frame(node,target)
        self.assertAlmostEqual(node._project_current_target(target).map_point.x,-.1)
        # Fixed intrinsics remain valid for low-frequency and zero-stamped input.
        node._camera_info_mode="per_frame"
        target.last_seen=rospy.Time.from_sec(100.2);self.source_frame(node,target)
        with self.assertRaisesRegex(ValueError,"camera_info_unavailable"):
            node._project_current_target(target)

    def test_invalid_camera_info_and_ambiguous_source(self):
        for invalid in ("focal","frame","roi","distortion","finite"):
            node,target=self.fixture()
            info=camera(99.7)
            if invalid=="focal":info.K[0]=0.
            if invalid=="frame":info.header.frame_id="other_camera";node._calibrations.clear()
            if invalid=="roi":info.width=0
            if invalid=="distortion":info.distortion_model="equidistant"
            if invalid=="finite":info.D=[float("nan")]*5
            node._on_camera_info(info)
            with self.assertRaises(ValueError):node._project_current_target(target)
        node,target=self.fixture()
        node._mapped_frames[-1].detections.append(copy.deepcopy(node._mapped_frames[-1].detections[0]))
        with self.assertRaisesRegex(ValueError,"ambiguous"):node._project_current_target(target)

    def test_compensated_goal_controls_all_evidence_even_far_from_principal_pixel(self):
        node,target=self.fixture()
        node._slot_mode="physical_body_position"
        node._slot_positions_body=[[-.12,0.],[0.,-.12],[0.,.12]]
        target.center_px.x=560.  # 80px from principal, outside old 20px condition
        self.source_frame(node,target)
        self.publish(node,target)
        msg=node._offset_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.map_point.x,.22)
        self.assertGreater(msg.alignment_error_m,msg.alignment_tolerance_m)
        # Current body now at the compensated goal, unlike exposure camera pose.
        old=node._tf_buffer.lookup_transform.side_effect
        node._tf_buffer.lookup_transform.side_effect=lambda p,c,s,d: (
            transform(p,c,100.,(.22,.0075,1.),(0,0,0,1)) if c==node._body_frame else old(p,c,s,d))
        for stamp in (99.9,99.95):
            target.last_seen=rospy.Time.from_sec(stamp);self.source_frame(node,target);self.publish(node,target)
        msg=node._offset_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.alignment_error_m,0.)
        self.assertTrue(node._evidence_pub.publish.call_args.args[0].aligned)
        self.assertTrue(node._evidence_pub.publish.call_args.args[0].evidence_valid)
        self.assertTrue(node._ready_pub.publish.call_args.args[0].ready)
        self.assertGreater(abs(msg.dx_px),node._max_offset_px)

    def test_measured_three_slots_yaw_and_no_double_compensation(self):
        positions=[[-.12,0.],[0.,-.12],[0.,.12]]
        for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
            for slot in (1,2,3):
                goal,error=slot_goal((1.,2.,0.),(0.,0.,1.),yaw,slot,positions,"physical_body_position")
                dx,dy=positions[slot-1]
                self.assertAlmostEqual(goal[0]+math.cos(yaw)*dx-math.sin(yaw)*dy,1.)
                self.assertAlmostEqual(goal[1]+math.sin(yaw)*dx+math.cos(yaw)*dy,2.)
                zero,_=slot_goal((1.,2.,0.),(0.,0.,1.),yaw,slot,positions,"zero")
                self.assertEqual(zero,(1.,2.,0.))

    def test_pixel_tolerance_equivalent_height_and_anisotropy(self):
        for height in (.16,.5,1.24):
            for direction in ((1.,0.),(0.,1.),(.03,-.02)):
                limit=pixel_equivalent_limit((height/800,0.,0.,height/600),direction,30.)
                eq=math.hypot(direction[0]*800/height,direction[1]*600/height)
                self.assertAlmostEqual(limit,30*math.hypot(*direction)/eq)
        self.assertAlmostEqual(pixel_equivalent_limit((.16/800,0,0,.16/800),(1,0),30),.006)

    def test_geometry_id_change_resets_stability_then_recovers(self):
        node,target=self.fixture();self.publish(node,target)
        target.id+=1;target.first_seen=rospy.Time.from_sec(99.85)
        for stamp,expected in ((99.9,False),(99.95,True)):
            target.last_seen=rospy.Time.from_sec(stamp);self.source_frame(node,target);self.publish(node,target)
            self.assertEqual(node._ready_pub.publish.call_args.args[0].ready,expected)

    def test_targets_before_mapped_then_duplicate_does_not_reset_five_frame_streak(self):
        node,target=self.fixture()
        node._stable_frames=5
        node._mapped_frames.clear()
        for count,stamp in enumerate((99.6,99.7,99.8,99.9,99.95),start=1):
            target.last_seen=rospy.Time.from_sec(stamp)
            self.publish(node,target)  # target N arrives before source metadata N
            self.assertEqual(node._consecutive_ok,count-1)
            self.source_frame(node,target)  # mapped N triggers exactly one retry
            self.assertEqual(node._consecutive_ok,count)
            for _ in range(3):self.publish(node,target)
            self.assertEqual(node._consecutive_ok,count)
        self.assertEqual(node._offset_pub.publish.call_count,5)
        self.assertTrue(node._ready_pub.publish.call_args.args[0].ready)

    def test_pending_metadata_expires_and_invalid_arrived_metadata_blocks(self):
        node,target=self.fixture();node._mapped_frames.clear()
        self.publish(node,target)
        node._offset_pub.publish.assert_not_called()
        self.clock.return_value=rospy.Time.from_sec(100.31)
        node._on_alignment_context_watchdog(None)
        self.assertIsNone(node._pending_targets)
        self.assertFalse(node._ready_pub.publish.call_args.args[0].ready)
        self.assertEqual(node._ready_pub.publish.call_args.args[0].reason,"source_observation_timeout")
        self.clock.return_value=rospy.Time(100)
        node,target=self.fixture()
        node._mapped_frames[-1].detections[0].reject_reason="tf_unavailable"
        self.publish(node,target)
        node._offset_pub.publish.assert_not_called()
        self.assertFalse(node._ready_pub.publish.call_args.args[0].ready)


if __name__=="__main__":unittest.main()
