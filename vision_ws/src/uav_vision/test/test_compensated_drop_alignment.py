"""Exercise production callbacks with generated messages, without ROS nodes."""

import copy
import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import rospy
from std_msgs.msg import String
from uav_vision.msg import (
    AlignmentTargetContext, DropAlignmentFeedback, TargetCandidate,
    TargetCandidateArray,
)


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "compensated_drop_aligner_test", PACKAGE / "scripts/drop_aligner.py")
ALIGNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ALIGNER)


class CompensatedDropAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.parameters = {
            "~use_camera_info": False,
            "~require_alignment_context": True,
            "~require_compensated_alignment": True,
            "~class_profile": "r2026",
            "~target_center_x": 640.0,
            "~target_center_y": 360.0,
            "~max_offset_px": 20.0,
            "~stable_frames": 2,
            "~compensated_observation_cache_size": 4,
        }
        self.publishers = {}

        def publisher(topic, *_args, **_kwargs):
            return self.publishers.setdefault(topic, Mock())

        for name, replacement in (
                ("init_node", Mock()), ("loginfo", Mock()),
                ("get_param", lambda name, default=None: self.parameters.get(name, default)),
                ("Publisher", publisher), ("Subscriber", Mock()), ("Timer", Mock())):
            patched = patch.object(ALIGNER.rospy, name, replacement)
            patched.start()
            self.addCleanup(patched.stop)
        patched = patch.object(ALIGNER.rospy.Time, "now", return_value=rospy.Time(100))
        self.clock = patched.start()
        self.addCleanup(patched.stop)
        patched = patch.object(ALIGNER.time, "monotonic", return_value=0.0)
        self.wall_clock = patched.start()
        self.addCleanup(patched.stop)
        self.node = ALIGNER.DropAligner()
        self.node._on_align_mode(String(data="drop_circle"))
        self.context = self.make_context()
        self.node._on_alignment_context(self.context)

    def make_context(self, mode="drop_circle"):
        context = AlignmentTargetContext()
        context.header.stamp = rospy.Time(100)
        context.header.frame_id = "camera_init"
        context.source = "navigation_coordinator"
        context.schema_version = 1
        context.active = True
        context.mission_id = "slot-compensation"
        context.decision_seq = 7
        context.deadline = rospy.Time(110)
        context.command = ALIGNER.COMMAND_ALIGN
        context.class_profile = "r2026"
        context.align_mode = mode
        context.has_target = True
        context.semantic_target_id = 0  # Zero is a valid target identity.
        context.semantic_target_first_seen = rospy.Time(90)
        context.target_observation_stamp = rospy.Time(99)
        context.semantic_target_class = "tent" if mode == "drop_circle" else "red_cross"
        context.attempt = 1
        context.payload_slot = 2
        context.target_pose.header.frame_id = "camera_init"
        context.target_pose.pose.position.x = 1.0
        context.target_pose.pose.position.y = 2.0
        context.max_association_distance_m = .5
        return context

    def target(self, stamp=99.8):
        target = TargetCandidate()
        target.header.stamp = rospy.Time(100)  # Memory publication time.
        target.header.frame_id = "downward_camera_optical_frame"
        target.id = 42
        target.first_seen = rospy.Time(95)
        target.last_seen = rospy.Time.from_sec(stamp)
        target.class_name = "circle"
        target.class_confidence = .95
        target.geometry_confidence = .9
        target.state = 2
        target.center_refined = True
        # At fx=1000 and 1m camera height, 120px represents intentional 12cm.
        target.center_px.x = 760
        target.center_px.y = 360
        target.center_px.z = 40
        target.observe_count = 10
        target.map_valid = True
        target.map_frame = "camera_init"
        target.map_point = copy.deepcopy(self.context.target_pose.pose.position)
        if self.context.align_mode == "drop_cross":
            target.class_name = "red_cross"
            target.id = self.context.semantic_target_id
            target.first_seen = self.context.semantic_target_first_seen
            target.map_valid = False  # Exact cross identity is the existing gate.
        return target

    def feedback(self, target):
        msg = DropAlignmentFeedback()
        msg.header.stamp = rospy.Time(100)
        msg.header.frame_id = "camera_init"
        msg.observation_stamp = target.last_seen
        msg.odom_stamp = rospy.Time.from_sec(99.99)
        for name in ("mission_id", "decision_seq", "attempt", "payload_slot",
                     "semantic_target_id", "semantic_target_first_seen",
                     "semantic_target_class", "align_mode"):
            setattr(msg, name, getattr(self.context, name))
        msg.valid = True
        msg.aligned = True
        msg.frozen = True
        msg.target_fc.x = 1.12
        msg.target_fc.y = 2.0
        msg.target_fc.z = .8
        msg.horizontal_error_m = .005
        msg.horizontal_speed_mps = .01
        msg.reason = "compensated_fc_settled"
        return msg

    def observe(self, target):
        self.node._on_targets(TargetCandidateArray(targets=[target]))

    def ready(self):
        return self.publishers["/uav_vision/drop_ready"].publish.call_args[0][0]

    def evidence(self):
        return self.publishers["/uav_vision/release_evidence"].publish.call_args[0][0]

    def align(self, stamp=99.8):
        target = self.target(stamp)
        self.observe(target)
        feedback = self.feedback(target)
        self.node._on_drop_alignment_feedback(feedback)
        return target, feedback

    def capture_clock(self, now):
        self.clock.return_value = rospy.Time.from_sec(now)
        self.context.header.stamp = self.clock.return_value
        self.node._on_alignment_context(copy.deepcopy(self.context))

    def capture_image(self, source, receipt, offset=10., target_id=42):
        self.capture_clock(receipt)
        target = self.target(source)
        target.id = target_id
        target.center_px.x = self.node._target_cx + offset
        self.observe(target)
        feedback = self.feedback(target)
        feedback.frozen = False
        feedback.header.stamp = self.clock.return_value
        feedback.odom_stamp = rospy.Time.from_sec(receipt - .001)
        self.node._on_drop_alignment_feedback(feedback)
        return target, feedback

    def capture_four_images(self):
        self.node._stable_frames = 5
        self.node._max_offset_px = 30.
        for index in range(4):
            pair = self.capture_image(100. + index * .1, 100.43 + index * .1)
            self.assertEqual(self.node._consecutive_ok, index + 1)
            self.assertFalse(self.ready().ready)
        return pair

    def test_capture_four_images_survive_expired_cache_until_fifth_fresh_image(self):
        target, feedback = self.capture_four_images()
        self.capture_clock(100.81)
        self.node._on_alignment_context_watchdog(None)
        self.assertEqual(self.ready().reason, "compensated_alignment_observation_stale")
        self.assertFalse(self.ready().ready)
        self.assertFalse(self.evidence().evidence_valid)
        self.assertEqual(self.node._consecutive_ok, 4)
        self.observe(target)  # Same memory snapshot is now stale as well.
        self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(self.node._consecutive_ok, 4)
        self.assertFalse(self.ready().ready)
        fifth, _ = self.capture_image(100.4, 100.83)
        self.assertEqual(self.node._consecutive_ok, 5)
        self.assertTrue(self.ready().ready)
        self.assertAlmostEqual(self.evidence().observation_age_sec, .43, places=5)
        self.assertEqual(self.node._compensated_feedback.observation_stamp, fifth.last_seen)
        self.capture_clock(100.91)
        self.node._on_alignment_context_watchdog(None)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.node._consecutive_ok, 5)

    def test_capture_long_dropout_resets_even_with_no_new_feedback(self):
        target, feedback = self.capture_four_images()
        self.capture_clock(101.31)
        for _ in range(3):
            self.node._on_alignment_context_watchdog(None)
            self.observe(target)
            self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertFalse(self.ready().ready)
        self.capture_image(101.1, 101.53)
        self.assertEqual(self.node._consecutive_ok, 1)
        self.assertFalse(self.ready().ready)

    def test_capture_long_dropout_resets_when_watchdog_is_delayed(self):
        self.capture_four_images()
        self.capture_image(101.1, 101.53)
        self.assertEqual(self.node._consecutive_ok, 1)
        self.assertFalse(self.ready().ready)

    def test_capture_new_geometry_and_new_action_cannot_inherit_four_images(self):
        self.capture_four_images()
        self.capture_image(100.4, 100.83, target_id=43)
        self.assertEqual(self.node._consecutive_ok, 1)
        self.assertFalse(self.ready().ready)
        self.context.decision_seq += 1
        self.node._on_alignment_context(copy.deepcopy(self.context))
        self.assertEqual(self.node._consecutive_ok, 0)
        self.capture_image(100.5, 100.93, target_id=43)
        self.assertEqual(self.node._consecutive_ok, 1)

    def test_capture_new_image_over_thirty_pixels_resets_streak(self):
        self.capture_four_images()
        self.capture_image(100.4, 100.83, offset=30.01)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertEqual(self.ready().reason, "offset_exceeds_limit")
        self.capture_image(100.5, 100.93, offset=30.)
        self.assertEqual(self.node._consecutive_ok, 1)
        self.assertFalse(self.ready().ready)

    def test_capture_new_off_center_image_resets_before_controller_feedback(self):
        _, old_feedback = self.capture_four_images()
        self.capture_clock(100.75)  # Previous evidence is still fresh.
        bad = self.target(100.4)
        bad.center_px.x = self.node._target_cx + 30.01
        self.observe(bad)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertEqual(self.ready().reason, "offset_exceeds_limit")
        self.node._on_drop_alignment_feedback(old_feedback)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.node._consecutive_ok, 0)

    def test_capture_duplicate_messages_neither_count_nor_extend_gap(self):
        target, feedback = self.capture_four_images()
        original = copy.deepcopy(feedback)
        for now in (100.75, 100.79, 100.81, 101.0, 101.31):
            self.capture_clock(now)
            self.observe(target)
            self.node._on_drop_alignment_feedback(feedback)
            self.node._on_alignment_context_watchdog(None)
            self.assertEqual(feedback, original)
            self.assertEqual(self.node._consecutive_ok, 4 if now < 101.3 else 0)
            self.assertFalse(self.ready().ready)

    def test_capture_new_expired_image_does_not_preserve_old_streak(self):
        self.capture_four_images()
        self.capture_image(100.31, 100.9)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertFalse(self.ready().ready)

    def test_capture_stale_odom_still_resets_instead_of_extending_history(self):
        _, feedback = self.capture_four_images()
        self.node._compensated_odom_max_age = .01
        self.capture_clock(100.75)
        self.node._on_alignment_context_watchdog(None)
        self.assertEqual(self.ready().reason, "compensated_alignment_odom_stale")
        self.assertEqual(self.node._consecutive_ok, 0)

    def test_intentional_12cm_offset_is_ready_only_from_distinct_feedback_images(self):
        for mode in ("drop_circle", "drop_cross"):
            with self.subTest(mode=mode):
                self.node._on_align_mode(String(data=mode))
                self.context = self.make_context(mode)
                self.node._on_alignment_context(self.context)
                self.node._last_counted_observation = None
                target, feedback = self.align()
                offset = self.publishers["/uav_vision/drop_offset"].publish.call_args[0][0]
                self.assertEqual(offset.header.stamp, target.last_seen)
                self.assertEqual(offset.dx_px, 120)
                self.assertFalse(self.ready().ready)
                for _ in range(5):
                    self.observe(target)
                    self.node._on_drop_alignment_feedback(feedback)
                    self.node._on_alignment_context_watchdog(None)
                self.assertEqual(self.node._consecutive_ok, 1)
                self.assertFalse(self.ready().ready)
                self.align(99.9)
                self.assertTrue(self.ready().ready)
                self.assertTrue(self.evidence().aligned)
                self.assertTrue(self.evidence().geometry_verified)
                self.assertEqual(self.evidence().stable_frames, 2)
                wrapped = self.publishers["/uav_vision/release_evidence_context"].publish.call_args[0][0]
                self.assertTrue(wrapped.context_valid)
                self.assertTrue(wrapped.semantic_geometry_match)
                self.assertEqual(wrapped.payload_slot, 2)

    def test_capture_keeps_original_centering_without_release_level_motion_gate(self):
        for mode in ("drop_circle", "drop_cross"):
            with self.subTest(mode=mode):
                self.node._on_align_mode(String(data=mode))
                self.context = self.make_context(mode)
                self.node._on_alignment_context(self.context)
                self.node._last_counted_observation = None
                for stamp in (99.8, 99.9):
                    target = self.target(stamp)
                    target.center_px.x = 640
                    self.observe(target)
                    feedback = self.feedback(target)
                    feedback.frozen = False
                    feedback.aligned = False  # No high release-level settlement.
                    feedback.horizontal_error_m = .12
                    feedback.horizontal_speed_mps = .20
                    self.node._on_drop_alignment_feedback(feedback)
                self.assertTrue(self.ready().ready)
                self.assertEqual(self.evidence().stable_frames, 2)
                # The same frozen capture permits the intentional nonzero pixel
                # displacement, while final physical release is controller-owned.
                feedback.frozen = True
                feedback.aligned = True
                self.node._on_drop_alignment_feedback(feedback)
                self.assertTrue(self.ready().ready)
                self.assertEqual(self.evidence().stable_frames, 2)

    def test_unfrozen_capture_rejects_uncentered_target_despite_valid_controller(self):
        for stamp in (99.8, 99.9):
            target = self.target(stamp)  # 120px, outside original 20px limit.
            self.observe(target)
            feedback = self.feedback(target)
            feedback.frozen = False
            self.node._on_drop_alignment_feedback(feedback)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "offset_exceeds_limit")
        self.assertEqual(self.node._consecutive_ok, 0)

    def test_source_is_cached_before_offset_publish_can_trigger_feedback(self):
        for stamp in (99.8, 99.9):
            target = self.target(stamp)
            feedback = self.feedback(target)
            # Reentrant callback is stricter than network delivery, which waits
            # for the target callback's RLock. Cache insertion must precede send.
            self.node._offset_pub.publish.side_effect = (
                lambda _offset, msg=feedback: self.node._on_drop_alignment_feedback(msg))
            self.observe(target)
        self.assertTrue(self.ready().ready)
        self.assertEqual(self.node._consecutive_ok, 2)

    def test_fifo_feedback_at_20hz_keeps_capture_streak_with_new_images_at_10_and_30hz(self):
        for image_hz in (10, 30):
            for feedback_delay in (.05, .15, .30):
                with self.subTest(image_hz=image_hz, feedback_delay=feedback_delay):
                    self.parameters.pop("~compensated_observation_cache_size", None)
                    self.parameters["~stable_frames"] = 5
                    self.clock.return_value = rospy.Time(100)
                    self.node = ALIGNER.DropAligner()
                    self.node._on_align_mode(String(data="drop_circle"))
                    self.node._on_alignment_context(self.context)
                    targets = [self.target(100.0 + index / image_hz)
                               for index in range(2 * image_hz)]
                    events = [(100.0 + index / image_hz, 0, target)
                              for index, target in enumerate(targets)]
                    for index in range(40):
                        evaluation_time = 100.0 + index / 20.0
                        source_index = min(len(targets) - 1,
                                           int(math.floor(index * image_hz / 20.0 + 1e-9)))
                        feedback = self.feedback(targets[source_index])
                        feedback.header.stamp = rospy.Time.from_sec(evaluation_time)
                        feedback.odom_stamp = feedback.header.stamp
                        feedback.aligned = index >= 6  # Capture settles after .30s.
                        events.append((evaluation_time + feedback_delay, 1, feedback))
                    reasons = []
                    readiness_seen = False
                    for event_time, kind, msg in sorted(events, key=lambda event: event[:2]):
                        if event_time > 101.4:
                            break
                        self.clock.return_value = rospy.Time.from_sec(event_time)
                        heartbeat = copy.deepcopy(self.context)
                        heartbeat.header.stamp = self.clock.return_value
                        self.node._on_alignment_context(heartbeat)
                        if kind == 0:
                            self.observe(msg)
                        else:
                            self.node._on_drop_alignment_feedback(msg)
                            reasons.append(self.ready().reason)
                        self.node._on_alignment_context_watchdog(None)
                        readiness_seen = readiness_seen or self.ready().ready
                        self.assertLessEqual(len(self.node._compensated_observations),
                                             self.node._compensated_observation_cache_size)
                    self.assertNotIn("compensated_alignment_observation_not_cached", reasons)
                    self.assertTrue(readiness_seen, reasons)
                    self.assertTrue(self.ready().ready)
                    self.assertGreaterEqual(self.node._consecutive_ok, 5)

    def test_centered_pixels_cannot_bypass_controller_alignment(self):
        target = self.target()
        target.center_px.x = 640
        self.observe(target)
        self.observe(target)
        self.assertFalse(self.ready().ready)
        feedback = self.feedback(target)
        feedback.aligned = False
        self.node._on_drop_alignment_feedback(feedback)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "compensated_alignment_not_aligned")

    def test_every_action_identity_field_is_checked(self):
        changes = {
            "mission_id": "other-mission", "decision_seq": 8, "attempt": 2,
            "payload_slot": 3, "semantic_target_id": 1,
            "semantic_target_first_seen": rospy.Time(91),
            "semantic_target_class": "bridge", "align_mode": "drop_cross",
        }
        target = self.target()
        self.observe(target)
        for name, value in changes.items():
            with self.subTest(field=name):
                feedback = self.feedback(target)
                setattr(feedback, name, value)
                self.node._on_drop_alignment_feedback(feedback)
                self.assertFalse(self.ready().ready)
                self.assertEqual(self.ready().reason, "compensated_alignment_context_mismatch")
                self.assertEqual(self.node._consecutive_ok, 0)

    def test_stale_future_zero_and_inconsistent_feedback_stamps_are_rejected(self):
        cases = [
            ("header", 99.49, "feedback_stale"),
            ("observation_stamp", 99.49, "observation_stale"),
            ("odom_stamp", 99.49, "odom_stale"),
            ("header", 100.2, "feedback_future"),
            ("observation_stamp", 100.1, "observation_future"),
            ("odom_stamp", 100.1, "odom_future"),
            ("odom_stamp", 0.0, "odom_unstamped"),
            ("header", 99.85, "evaluation_precedes_source"),
        ]
        target = self.target()
        self.observe(target)
        for field, seconds, reason in cases:
            with self.subTest(field=field, seconds=seconds):
                feedback = self.feedback(target)
                if field == "header":
                    feedback.header.stamp = rospy.Time.from_sec(seconds)
                else:
                    setattr(feedback, field, rospy.Time.from_sec(seconds))
                self.node._on_drop_alignment_feedback(feedback)
                self.assertFalse(self.ready().ready)
                self.assertEqual(self.ready().reason, "compensated_alignment_" + reason)

    def test_feedback_requires_exact_nanosecond_cached_image_and_valid_metrics(self):
        target = self.target()
        self.observe(target)
        cases = []
        feedback = self.feedback(target)
        feedback.observation_stamp = rospy.Time(target.last_seen.secs, target.last_seen.nsecs + 1)
        cases.append((feedback, "observation_not_cached"))
        feedback = self.feedback(target)
        feedback.header.frame_id = "map"
        cases.append((feedback, "frame_mismatch"))
        feedback = self.feedback(target)
        feedback.valid = False
        cases.append((feedback, "feedback_invalid"))
        for field in ("horizontal_error_m", "horizontal_speed_mps"):
            for value in (float("nan"), float("inf"), -1.0):
                feedback = self.feedback(target)
                setattr(feedback, field, value)
                cases.append((feedback, "metrics_invalid"))
        feedback = self.feedback(target)
        feedback.target_fc.x = float("nan")
        cases.append((feedback, "metrics_invalid"))
        for feedback, reason in cases:
            with self.subTest(reason=reason):
                self.node._on_drop_alignment_feedback(feedback)
                self.assertFalse(self.ready().ready)
                self.assertEqual(self.ready().reason, "compensated_alignment_" + reason)

    def test_continuous_20hz_feedback_and_same_action_context_3ms_ahead_do_not_starve(self):
        originals = []
        for index in range(20):
            now = 100.0 + index * .05
            self.clock.return_value = rospy.Time.from_sec(now)
            self.wall_clock.return_value = index * .25  # Gazebo RTF approximately .2.
            heartbeat = copy.deepcopy(self.context)
            heartbeat.header.stamp = rospy.Time.from_sec(now + .003)
            self.node._on_alignment_context(heartbeat)
            target = self.target(now - .001)
            self.observe(target)
            feedback = self.feedback(target)
            feedback.header.stamp = heartbeat.header.stamp
            feedback.odom_stamp = target.last_seen
            original = copy.deepcopy(feedback)
            self.node._on_drop_alignment_feedback(feedback)
            self.node._on_alignment_context_watchdog(None)
            self.assertEqual(feedback, original)
            self.assertEqual(self.node._consecutive_ok, index)
            self.assertEqual(len(self.node._pending_feedback), 1)
            self.assertEqual(self.ready().ready, index >= 2)
            if self.node._compensated_feedback is not None:
                self.assertLessEqual(self.node._compensated_feedback.header.stamp,
                                     self.clock.return_value)
            originals.append(original)
        self.clock.return_value = originals[-1].header.stamp
        self.wall_clock.return_value += .25
        self.node._on_alignment_context_watchdog(None)
        self.assertTrue(self.ready().ready)
        self.assertEqual(self.node._consecutive_ok, 20)
        self.assertEqual(self.node._compensated_feedback, originals[-1])
        self.assertEqual(len(self.node._pending_feedback), 0)

    def test_future_feedback_preserves_streak_then_counts_original_image_only_once(self):
        self.align()
        target = self.target(99.9)
        self.observe(target)
        feedback = self.feedback(target)
        feedback.header.stamp = rospy.Time.from_sec(100.003)
        for _ in range(5):
            self.node._on_drop_alignment_feedback(feedback)
            self.node._on_alignment_context_watchdog(None)
        self.assertEqual(self.node._consecutive_ok, 1)
        self.assertEqual(len(self.node._pending_feedback), 1)
        self.assertFalse(self.ready().ready)
        self.clock.return_value = feedback.header.stamp
        self.wall_clock.return_value = .25
        self.node._on_alignment_context_watchdog(None)
        self.assertTrue(self.ready().ready)
        self.assertEqual(self.node._compensated_feedback, feedback)
        for _ in range(5):
            self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(self.node._consecutive_ok, 2)

    def test_next_feedback_callback_consumes_old_pending_before_new_future_sample(self):
        targets = [self.target(99.8 + index * .05) for index in range(4)]
        for target in targets:
            self.observe(target)
        for index, target in enumerate(targets):
            now = 100.0 + index * .05
            self.clock.return_value = rospy.Time.from_sec(now)
            self.wall_clock.return_value = index * .25
            feedback = self.feedback(target)
            feedback.header.stamp = rospy.Time.from_sec(now + .003)
            feedback.odom_stamp = rospy.Time.from_sec(now - .001)
            # No target/context/watchdog callbacks during this stream: progress
            # must come from draining before the next feedback is enqueued.
            self.node._on_drop_alignment_feedback(feedback)
            self.assertEqual(self.node._consecutive_ok, index)
            self.assertEqual(len(self.node._pending_feedback), 1)
        self.assertTrue(self.ready().ready)
        self.clock.return_value = feedback.header.stamp
        self.node._on_alignment_context_watchdog(None)
        self.assertEqual(self.node._consecutive_ok, 4)

    def test_pending_context_is_consumed_before_old_heartbeat_age_check(self):
        self.align()
        self.align(99.9)
        self.node._alignment_context_max_age = .01
        self.clock.return_value = rospy.Time.from_sec(100.009)
        heartbeat = copy.deepcopy(self.context)
        heartbeat.header.stamp = rospy.Time.from_sec(100.012)
        self.node._on_alignment_context(heartbeat)
        self.assertEqual(self.node._alignment_context.header.stamp, rospy.Time(100))
        self.clock.return_value = rospy.Time.from_sec(100.02)
        self.wall_clock.return_value = .25
        self.node._on_alignment_context_watchdog(None)
        self.assertEqual(self.node._alignment_context, heartbeat)
        self.assertEqual(self.node._consecutive_ok, 2)
        self.assertTrue(self.ready().ready)

    def test_future_feedback_intrinsic_errors_are_not_deferred(self):
        for field, value, reason in (
                ("valid", False, "feedback_invalid"),
                ("horizontal_error_m", float("nan"), "metrics_invalid"),
                ("horizontal_speed_mps", -1.0, "metrics_invalid"),
                ("mission_id", "wrong-action", "context_mismatch"),
                ("odom_stamp", rospy.Time.from_sec(100.004), "evaluation_precedes_source")):
            with self.subTest(field=field):
                self.node._last_counted_observation = None
                target, _ = self.align()
                feedback = self.feedback(target)
                feedback.header.stamp = rospy.Time.from_sec(100.003)
                setattr(feedback, field, value)
                self.node._on_drop_alignment_feedback(feedback)
                self.assertEqual(len(self.node._pending_feedback), 0)
                self.assertEqual(self.node._consecutive_ok, 0)
                self.assertEqual(self.ready().reason, "compensated_alignment_" + reason)

    def test_context_identity_change_immediately_discards_pending_context_and_feedback(self):
        target, _ = self.align()
        self.align(99.9)
        heartbeat = copy.deepcopy(self.context)
        heartbeat.header.stamp = rospy.Time.from_sec(100.003)
        self.node._on_alignment_context(heartbeat)
        feedback = self.feedback(target)
        feedback.header.stamp = heartbeat.header.stamp
        self.node._on_drop_alignment_feedback(feedback)
        changed = copy.deepcopy(heartbeat)
        changed.decision_seq += 1
        self.node._on_alignment_context(changed)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertIsNone(self.node._pending_context)
        self.assertEqual(len(self.node._pending_feedback), 0)
        self.clock.return_value = heartbeat.header.stamp
        self.node._on_alignment_context_watchdog(None)
        self.assertFalse(self.ready().ready)
        self.assertIsNone(self.node._compensated_feedback)

    def test_pending_feedback_keeps_source_age_and_deadline_gates(self):
        target = self.target(99.503)
        self.observe(target)
        feedback = self.feedback(target)
        feedback.header.stamp = rospy.Time.from_sec(100.01)
        self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(len(self.node._pending_feedback), 1)
        self.clock.return_value = rospy.Time.from_sec(100.02)
        self.node._on_alignment_context_watchdog(None)
        self.assertEqual(len(self.node._pending_feedback), 0)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertFalse(self.ready().ready)
        self.assertIsNone(self.node._compensated_feedback)
        self.clock.return_value = rospy.Time(100)
        self.context.deadline = rospy.Time.from_sec(100.002)
        self.node._on_alignment_context(self.context)
        target = self.target(99.9)
        self.observe(target)
        feedback = self.feedback(target)
        feedback.header.stamp = rospy.Time.from_sec(100.003)
        self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(len(self.node._pending_feedback), 1)
        self.clock.return_value = feedback.header.stamp
        self.node._on_alignment_context_watchdog(None)
        self.assertEqual(len(self.node._pending_feedback), 0)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "alignment_context_deadline_expired")

    def test_future_pending_queue_is_bounded_and_wall_timeout_never_admits_future(self):
        target, _ = self.align()
        for index in range(12):
            feedback = self.feedback(target)
            feedback.header.stamp = rospy.Time.from_sec(100.001 + index * .001)
            self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(len(self.node._pending_feedback), 8)
        self.assertEqual(self.node._consecutive_ok, 1)
        self.wall_clock.return_value = .25  # ROS clock deliberately has not caught up.
        self.node._on_alignment_context_watchdog(None)
        self.assertEqual(len(self.node._pending_feedback), 0)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertFalse(self.ready().ready)

    def test_old_feedback_after_context_switch_cannot_reuse_stability(self):
        self.align()
        _, old_feedback = self.align(99.9)
        self.assertTrue(self.ready().ready)
        self.context = copy.deepcopy(self.context)
        self.context.decision_seq += 1
        self.context.payload_slot = 3
        self.node._on_alignment_context(self.context)
        self.assertFalse(self.ready().ready)
        self.assertEqual(len(self.node._compensated_observations), 0)
        target = self.target(99.95)
        self.observe(target)
        self.node._on_drop_alignment_feedback(old_feedback)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "compensated_alignment_context_mismatch")
        self.node._on_drop_alignment_feedback(self.feedback(target))
        self.assertEqual(self.node._consecutive_ok, 1)
        self.assertFalse(self.ready().ready)

    def test_context_heartbeat_preserves_streak_but_expiry_revokes_it(self):
        self.align()
        self.align(99.9)
        self.clock.return_value = rospy.Time.from_sec(100.1)
        heartbeat = copy.deepcopy(self.context)
        heartbeat.header.stamp = self.clock.return_value
        self.node._on_alignment_context(heartbeat)
        self.assertEqual(self.node._consecutive_ok, 2)
        self.assertTrue(self.ready().ready)
        self.clock.return_value = rospy.Time.from_sec(100.61)
        self.node._on_alignment_context_watchdog(None)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "alignment_context_stale")

    def test_watchdog_revokes_stale_odom_even_while_images_continue(self):
        self.node._compensated_odom_max_age = .05
        self.align()
        self.align(99.9)
        self.assertTrue(self.ready().ready)
        self.clock.return_value = rospy.Time.from_sec(100.06)
        self.node._on_alignment_context_watchdog(None)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "compensated_alignment_odom_stale")

    def test_watchdog_revokes_feedback_and_image_expiry_independently(self):
        for kind, expected in (("feedback", "feedback_stale"),
                               ("observation", "observation_stale")):
            with self.subTest(kind=kind):
                self.node._clear_stability()
                self.node._last_counted_observation = None
                self.clock.return_value = rospy.Time(100)
                self.node._on_alignment_context(self.context)
                self.node._compensated_feedback_max_age = .05 if kind == "feedback" else .5
                self.align(99.7)
                self.align(99.8)
                self.assertTrue(self.ready().ready)
                self.clock.return_value = rospy.Time.from_sec(
                    100.06 if kind == "feedback" else 100.31)
                self.node._on_alignment_context_watchdog(None)
                self.assertFalse(self.ready().ready)
                self.assertEqual(self.ready().reason, "compensated_alignment_" + expected)

    def test_new_feedback_cannot_inherit_expired_streak_when_watchdog_is_delayed(self):
        self.align(99.8)
        self.clock.return_value = rospy.Time.from_sec(100.3)
        target = self.target(100.3)
        self.observe(target)
        self.assertEqual(self.node._consecutive_ok, 1)
        # Previous observation expires after target selection, before feedback.
        # Deliberately do not invoke the timer: the callback must fence itself.
        self.clock.return_value = rospy.Time.from_sec(100.4)
        feedback = self.feedback(target)
        feedback.header.stamp = self.clock.return_value
        feedback.odom_stamp = rospy.Time.from_sec(100.39)
        self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(self.node._consecutive_ok, 1)
        self.assertFalse(self.ready().ready)

    def test_geometry_switch_drops_cached_feedback_and_restarts_with_new_images(self):
        self.align()
        _, old_feedback = self.align(99.9)
        self.assertTrue(self.ready().ready)
        changed = self.target(99.95)
        changed.id += 1
        self.observe(changed)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.node._on_drop_alignment_feedback(old_feedback)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "compensated_alignment_observation_not_cached")
        self.node._on_drop_alignment_feedback(self.feedback(changed))
        self.assertEqual(self.node._consecutive_ok, 1)

    def test_unaligned_then_replayed_image_cannot_rebuild_streak(self):
        self.align()
        target, feedback = self.align(99.9)
        self.assertTrue(self.ready().ready)
        rejected = copy.deepcopy(feedback)
        rejected.aligned = False
        self.node._on_drop_alignment_feedback(rejected)
        self.assertEqual(self.node._consecutive_ok, 0)
        for _ in range(3):
            self.observe(target)
            self.node._on_drop_alignment_feedback(feedback)
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.node._consecutive_ok, 0)

    def test_cache_is_bounded_and_accepts_delayed_selected_observation(self):
        images = [self.target(99.6 + index * .05) for index in range(6)]
        for target in images:
            self.observe(target)
        self.assertEqual(len(self.node._compensated_observations), 4)
        self.node._on_drop_alignment_feedback(self.feedback(images[0]))
        self.assertEqual(self.ready().reason, "compensated_alignment_observation_not_cached")
        self.node._on_drop_alignment_feedback(self.feedback(images[-2]))
        self.assertEqual(self.node._consecutive_ok, 1)
        self.node._on_drop_alignment_feedback(self.feedback(images[-1]))
        self.assertTrue(self.ready().ready)
        self.node._on_drop_alignment_feedback(self.feedback(images[-2]))
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "compensated_alignment_observation_out_of_order")

    def test_image_loss_revokes_ready_without_mutating_frozen_controller_feedback(self):
        self.align()
        target, feedback = self.align(99.9)
        original = copy.deepcopy(feedback)
        self.assertTrue(self.ready().ready)
        self.node._on_targets(TargetCandidateArray())
        self.assertFalse(self.ready().ready)
        self.assertEqual(feedback, original)
        self.assertTrue(feedback.frozen)
        self.observe(target)
        self.node._on_drop_alignment_feedback(feedback)
        self.assertEqual(self.node._consecutive_ok, 0)
        self.assertFalse(self.ready().ready)

    def test_visual_geometry_and_semantic_gates_remain_required(self):
        mutations = [
            ("class_confidence", .1), ("geometry_confidence", .1),
            ("geometry_confidence", float("nan")), ("center_refined", False),
            ("state", 1), ("class_name", "landing_pad"),
            ("map_valid", False), ("map_frame", "map"),
            ("last_seen", rospy.Time.from_sec(99.49)),
        ]
        for name, value in mutations:
            with self.subTest(field=name):
                target = self.target()
                setattr(target, name, value)
                self.observe(target)
                self.node._on_drop_alignment_feedback(self.feedback(target))
                self.assertFalse(self.ready().ready)
                self.assertEqual(self.node._consecutive_ok, 0)
        target = self.target()
        target.map_point.x += 1.0
        self.observe(target)
        self.node._on_drop_alignment_feedback(self.feedback(target))
        self.assertFalse(self.ready().ready)

    def test_opt_in_alone_requires_context_and_optional_legacy_remains_pixel_based(self):
        self.node._require_alignment_context = False
        self.node._alignment_context = None
        self.observe(self.target())
        self.assertFalse(self.ready().ready)
        self.assertEqual(self.ready().reason, "alignment_context_missing")
        self.node._require_compensated_alignment = False
        target = self.target()
        target.center_px.x = 640
        self.observe(target)
        self.observe(target)
        self.assertTrue(self.ready().ready)

    def test_landing_still_uses_existing_pixel_gate(self):
        self.node._require_alignment_context = False
        self.node._on_align_mode(String(data="landing"))
        target = self.target()
        target.class_name = "landing_pad"
        target.center_px.x = 640
        self.observe(target)
        self.observe(target)
        self.assertTrue(self.ready().ready)
        self.node._on_drop_alignment_feedback(self.feedback(target))
        self.assertTrue(self.ready().ready)
        self.node._on_alignment_context_watchdog(None)
        self.assertTrue(self.ready().ready)

    def test_launch_default_follows_context_and_can_be_explicitly_overridden(self):
        from roslaunch.config import ROSLaunchConfig
        from roslaunch.xmlloader import XmlLoader
        from roslaunch import substitution_args
        from rospkg import RosPack

        # Resolve only this source package. Loading parameters is entirely offline;
        # no ROS master, nodes, processes or global overlay changes are involved.
        packages = RosPack(ros_paths=[str(PACKAGE.parent)])
        for name in ("phase_d.launch", "phase_d_board.launch"):
            for arguments, expected in (
                    ([], False), (["require_alignment_context:=true"], True),
                    (["require_alignment_context:=true", "require_compensated_alignment:=false"], False)):
                with self.subTest(launch=name, arguments=arguments):
                    config = ROSLaunchConfig()
                    with patch.object(substitution_args, "_rospack", packages):
                        XmlLoader().load(str(PACKAGE / "launch" / name), config,
                                         argv=arguments, verbose=False)
                    self.assertEqual(
                        config.params["/drop_aligner/require_compensated_alignment"].value,
                        expected)


if __name__ == "__main__":
    unittest.main()
