"""Check production DropOffset timestamps without starting a ROS node."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import rospy
from uav_vision.msg import AlignmentTargetContext, TargetCandidate, TargetCandidateArray


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "drop_aligner_observation_test", PACKAGE / "scripts/drop_aligner.py")
ALIGNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ALIGNER)


class DropObservationStampTest(unittest.TestCase):
    def setUp(self):
        clock = patch.object(ALIGNER.rospy.Time, "now", return_value=rospy.Time(100))
        self.clock = clock.start()
        self.addCleanup(clock.stop)

    def fixture(self, mode, last_seen=99.8):
        target = TargetCandidate()
        target.header.seq = 9
        target.header.stamp = rospy.Time(100)
        target.header.frame_id = "downward_camera_optical_frame"
        target.id = 7
        target.class_name = {
            "drop_circle": "circle", "drop_cross": "red_cross", "landing": "landing_pad",
        }[mode]
        target.class_confidence = .95
        target.geometry_confidence = .9
        target.center_refined = True
        target.center_px.x = 648
        target.center_px.y = 366
        target.center_px.z = 40
        target.map_valid = mode != "drop_cross"
        target.map_frame = "camera_init"
        target.map_point.x = 1
        target.map_point.y = 2
        target.state = 2
        target.observe_count = 10
        target.first_seen = rospy.Time(99)
        target.last_seen = rospy.Time.from_sec(last_seen)

        context = AlignmentTargetContext()
        context.header.stamp = rospy.Time(100)
        context.source = "navigation_coordinator"
        context.schema_version = 1
        context.active = True
        context.mission_id = "stamp-regression"
        context.decision_seq = 1
        context.deadline = rospy.Time(110)
        context.command = ALIGNER.COMMAND_ALIGN
        context.class_profile = "r2026"
        context.align_mode = mode
        context.has_target = True
        context.semantic_target_id = target.id
        context.semantic_target_first_seen = target.first_seen
        context.target_observation_stamp = rospy.Time.from_sec(99.8)
        context.semantic_target_class = "tent" if mode == "drop_circle" else target.class_name
        context.attempt = 1
        context.payload_slot = 1
        context.target_pose.header.frame_id = "camera_init"
        context.target_pose.pose.position = target.map_point
        context.max_association_distance_m = .5

        # Avoid __init__, which registers ROS publishers/subscribers and timers.
        node = ALIGNER.DropAligner.__new__(ALIGNER.DropAligner)
        node._align_mode = mode
        node._require_alignment_context = True
        node._alignment_context = context
        node._alignment_context_max_age = .5
        node._class_profile = "r2026"
        node._allowed_alignment_commands = {ALIGNER.COMMAND_ALIGN}
        node._allowed_semantic_classes = {"tent", "pillbox", "bridge", "panzer", "red_cross"}
        node._selected_target = None
        node._active_geometry_key = None
        node._active_geometry_last_seen = None
        node._target_max_age = .5
        node._target_cx = 640
        node._target_cy = 360
        node._min_confidence = .7
        node._max_offset_px = 20
        node._consecutive_ok = 0
        node._stable_frames = 2
        node._offset_pub = Mock()
        node._evidence_pub = Mock()
        node._ready_pub = Mock()
        # Evidence wrapping is unchanged and separate from timestamp selection.
        node._publish_evidence_context = Mock()
        return node, target

    def publish(self, node, target):
        node._on_targets_locked(TargetCandidateArray(targets=[target]))

    def test_drop_modes_use_observation_stamp_without_mutating_candidate(self):
        for mode in ("drop_circle", "drop_cross"):
            with self.subTest(mode=mode):
                node, target = self.fixture(mode)
                original_header = target.header
                self.publish(node, target)
                offset = node._offset_pub.publish.call_args[0][0]
                self.assertEqual(offset.header.stamp, target.last_seen)
                self.assertIsNot(offset.header, original_header)
                self.assertIs(target.header, original_header)
                self.assertEqual(target.header.stamp, rospy.Time(100))
                self.assertEqual(offset.header.seq, original_header.seq)
                self.assertEqual(offset.header.frame_id, original_header.frame_id)
                self.assertEqual((offset.dx_px, offset.dy_px, offset.radius_px), (8, 6, 40))
                self.assertEqual(offset.quality, target.geometry_confidence)

    def test_republished_red_cross_keeps_the_same_pixel_observation_time(self):
        node, target = self.fixture("drop_cross")
        self.assertFalse(target.map_valid)
        self.publish(node, target)
        target.header.stamp = rospy.Time.from_sec(100.1)
        self.clock.return_value = rospy.Time.from_sec(100.1)
        self.publish(node, target)
        offsets = [call.args[0] for call in node._offset_pub.publish.call_args_list]
        self.assertEqual(len(offsets), 1)
        self.assertTrue(all(offset.header.stamp == target.last_seen for offset in offsets))
        self.assertEqual(target.header.stamp, rospy.Time.from_sec(100.1))

    def test_landing_keeps_its_existing_header_and_timestamp(self):
        node, target = self.fixture("landing")
        self.publish(node, target)
        offset = node._offset_pub.publish.call_args[0][0]
        self.assertIs(offset.header, target.header)
        self.assertEqual(offset.header.stamp, rospy.Time(100))
        self.assertNotEqual(offset.header.stamp, target.last_seen)

    def test_zero_and_expired_observations_keep_existing_rejection(self):
        for mode in ("drop_circle", "drop_cross", "landing"):
            for last_seen in (0., 99.49):
                with self.subTest(mode=mode, last_seen=last_seen):
                    node, target = self.fixture(mode, last_seen)
                    node._consecutive_ok = 1
                    self.publish(node, target)
                    node._offset_pub.publish.assert_not_called()
                    self.assertEqual(node._consecutive_ok, 0)
                    ready = node._ready_pub.publish.call_args[0][0]
                    self.assertFalse(ready.ready)
                    self.assertEqual(ready.reason, "stale_observation")

    def test_stability_evidence_and_ready_timestamps_are_unchanged(self):
        for mode in ("drop_circle", "drop_cross", "landing"):
            with self.subTest(mode=mode):
                node, target = self.fixture(mode)
                self.publish(node, target)
                self.assertFalse(node._ready_pub.publish.call_args[0][0].ready)
                target.last_seen = rospy.Time.from_sec(99.9)
                self.publish(node, target)
                evidence = node._evidence_pub.publish.call_args[0][0]
                ready = node._ready_pub.publish.call_args[0][0]
                self.assertTrue(evidence.evidence_valid)
                self.assertEqual(evidence.stable_frames, 2)
                self.assertAlmostEqual(evidence.observation_age_sec, .1)
                self.assertEqual(evidence.header.stamp, rospy.Time(100))
                self.assertTrue(ready.ready)
                self.assertEqual(ready.header.stamp, rospy.Time(100))


if __name__ == "__main__":
    unittest.main()
