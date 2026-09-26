#!/usr/bin/env python3
"""Offline ROS-message regression; no ROS master, camera or flight process."""
import unittest
from unittest.mock import Mock, patch
import rospy
from std_msgs.msg import String
from target_memory import TargetMemory, ST_CONFIRMED
from target_map_projector import TargetMapProjector
from target_memory_physical_assertion import PhysicalMemoryAssertion as Fixtures
from target_map_projector_rectification_assertion import _CameraModel, _TfBuffer
from uav_vision.msg import TargetDetectionArray

def stamp(value):return rospy.Time.from_sec(value)

class MemoryTests(unittest.TestCase):
    def setUp(self):
        patches=[
            patch.object(rospy,'init_node'),patch.object(rospy,'Publisher'),
            patch.object(rospy,'Subscriber'),patch.object(rospy,'Service'),
            patch.object(rospy.Time,'now',return_value=stamp(200.)),
            patch.object(rospy,'get_param',side_effect=lambda name,default=None:
                         1.0 if name=='~search_confirmation_max_gap_sec' else default)]
        for p in patches:p.start();self.addCleanup(p.stop)
        self.m=TargetMemory()
        self.m._require_map_for_candidates=True
        self.m._require_complete_detection_sources=True

    def frame(self,t,hit=True):
        ds=[Fixtures._detection('panzer',.93,1.)] if hit else []
        self.m._process_detections_locked(Fixtures._message(ds,stamp=stamp(t)))
        return next(iter(self.m._candidates.values()))

    def test_three_spaced_hits_confirm_but_a_miss_is_not_current(self):
        c=self.frame(201.)
        self.frame(201.3,False)
        self.assertEqual(c.consecutive_observe_count,1)
        self.assertFalse(c.to_msg(stamp(201.3)).map_valid)
        self.frame(201.8);self.frame(202.,False);self.frame(202.6)
        self.assertEqual(c.state,ST_CONFIRMED)
        self.assertEqual(c.observe_count,3)
        self.assertTrue(c.current_map_valid)

    def test_one_hit_and_duplicates_do_not_confirm(self):
        c=self.frame(201.);self.frame(201.);self.frame(200.9)
        self.assertEqual(c.observe_count,1)
        self.assertNotEqual(c.state,ST_CONFIRMED)

    def test_expired_gap_restarts_even_without_empty_frames(self):
        c=self.frame(201.);self.frame(201.8);self.frame(202.81)
        self.assertEqual(c.consecutive_observe_count,1)
        self.assertNotEqual(c.state,ST_CONFIRMED)

    def test_alignment_mode_drops_search_streak_and_queued_frames(self):
        c=self.frame(201.);self.frame(201.5);self.frame(202.)
        self.assertEqual(c.state,ST_CONFIRMED)
        with patch.object(rospy.Time,'now',return_value=stamp(202.1)):
            self.m._on_align_mode(String(data='drop_circle'))
        self.assertFalse(c.current_map_valid)
        self.assertEqual(c.consecutive_observe_count,0)
        self.frame(202.05)  # source image captured before the mode change
        self.assertEqual(c.consecutive_observe_count,0)
        self.frame(202.2);self.frame(202.3,False);self.frame(202.4)
        self.assertEqual(c.consecutive_observe_count,1)
        self.frame(202.5);self.frame(202.6)
        self.assertEqual(c.state,ST_CONFIRMED)
        self.assertEqual(c.id,0)

    def test_default_keeps_consecutive_requirement(self):
        self.m._search_confirmation_max_gap=0.
        c=self.frame(201.);self.frame(201.1);self.frame(201.2,False);self.frame(201.3)
        self.assertEqual(c.consecutive_observe_count,1)
        self.assertNotEqual(c.state,ST_CONFIRMED)

    def test_no_geometry_does_not_join_low_confirmation(self):
        det=Fixtures._detection('panzer',.99,1.)
        det.center_refined=det.geometry_verified=det.association_valid=False
        self.m._process_detections_locked(Fixtures._message([det],stamp=stamp(201.)))
        self.assertFalse(self.m._candidates)

class ProjectionTests(unittest.TestCase):
    def setUp(self):
        p=self.p=TargetMapProjector.__new__(TargetMapProjector)
        p._camera_ready=True;p._camera_model=_CameraModel();p._tf_buffer=_TfBuffer()
        p._map_frame='map';p._ground_z=0.;p._ray_epsilon=1e-5;p._tf_timeout=.05
        p._allow_latest_tf_fallback=False;p._max_latest_tf_age=.1
        p._rectify_input_pixels=True;p._camera_has_distortion=False
        p._coarse_min_confidence=.60;p._coarse_classes={'panzer','red_cross'}
        p._coarse_pub=Mock()
        self.msg=Fixtures._message([Fixtures._detection('panzer',.61,1.)],stamp=stamp(10.))
        self.msg.source='target_detector'
        det=self.msg.detections[0]
        det.center_refined=det.association_valid=det.geometry_verified=False
        clock=patch.object(rospy.Time,'now',return_value=stamp(10.1))
        clock.start();self.addCleanup(clock.stop)

    def test_bbox_projects_separately_and_remains_invalid_for_precision(self):
        self.p._on_coarse(self.msg)
        out=self.p._coarse_pub.publish.call_args[0][0]
        d=out.detections[0]
        self.assertEqual(out.source,'coarse_navigation_projector')
        self.assertTrue(d.map_valid)
        self.assertFalse(d.association_valid or d.geometry_verified or d.center_refined)
        self.assertEqual(d.map_quality,0.)
        self.assertEqual((d.center_px.x,d.center_px.y),(640.,480.))
        self.assertEqual(out.header.stamp,self.msg.header.stamp)
        ok,_=self.p._project(d,self.msg.header.stamp,'camera')
        self.assertFalse(ok)

    def test_old_frame_wrong_source_and_low_confidence(self):
        self.msg.header.stamp=stamp(9.)
        self.p._on_coarse(self.msg);self.p._coarse_pub.publish.assert_not_called()
        self.msg.header.stamp=stamp(10.);self.msg.source='circle_detector'
        self.p._on_coarse(self.msg);self.p._coarse_pub.publish.assert_not_called()
        self.msg.source='target_detector';self.msg.detections[0].class_confidence=.59
        self.p._on_coarse(self.msg)
        self.assertFalse(self.p._coarse_pub.publish.call_args[0][0].detections)

    def test_camera_and_geometry_remain_required(self):
        det=self.msg.detections[0]
        self.p._camera_ready=False
        self.assertEqual(self.p._project(det,stamp(10.),'camera',False)[1],'camera_info_unavailable')
        self.p._camera_ready=True
        self.p._camera_model.projectPixelTo3dRay=lambda pixel:(1.,0.,0.)
        self.assertEqual(self.p._project(det,stamp(10.),'camera',False)[1],'ray_parallel_ground')
        self.p._camera_model.projectPixelTo3dRay=lambda pixel:(0.,0.,1.)
        self.assertEqual(self.p._project(det,stamp(10.),'camera',False)[1],'intersection_behind_camera')
        self.p._camera_model.projectPixelTo3dRay=lambda pixel:(float('nan'),0.,-1.)
        self.assertEqual(self.p._project(det,stamp(10.),'camera',False)[1],'projection_nonfinite')

if __name__=='__main__':unittest.main()
