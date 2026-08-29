#!/usr/bin/env python3
"""验证物理目标身份和连续命中计数的确定性回归。"""
import sys
import threading

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest
from std_srvs.srv import Empty

from uav_vision.msg import (
    TargetCandidate,
    TargetCandidateArray,
    TargetDetection,
    TargetDetectionArray,
)
from uav_vision.target_selection_policy import (
    choose_selected_candidate,
    resolve_class_profile,
)


STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}
CONFIRMED_STATE = 2
PRIORITIES = {
    "tent": 1.0,
    "pillbox": 1.5,
    "bridge": 2.0,
    "panzer": 2.5,
    "tank": 5.0,
    "red_cross": 10.0,
}
COMPLETE_SEARCH_SOURCES = [
    "target_detector", "circle_detector", "cross_detector",
]


class PhysicalMemoryAssertion:
    def __init__(self):
        rospy.init_node("target_memory_physical_assertion")
        self._pub = rospy.Publisher(
            "/uav_vision/detections_memory_test",
            TargetDetectionArray,
            queue_size=1,
        )
        self._condition = threading.Condition()
        self._latest = None
        self._selected = []
        rospy.Subscriber(
            "/uav_vision/targets",
            TargetCandidateArray,
            self._on_targets,
            queue_size=8,
        )
        rospy.Subscriber(
            "/uav_vision/selected_target",
            TargetCandidate,
            self._on_selected,
            queue_size=8,
        )
        self._reset = rospy.ServiceProxy("/uav_vision/reset_memory", Empty)

    def _on_targets(self, message):
        with self._condition:
            self._latest = message
            self._condition.notify_all()

    def _on_selected(self, message):
        with self._condition:
            self._selected.append(message)
            self._condition.notify_all()

    @staticmethod
    def _detection(class_name, confidence, map_x):
        det = TargetDetection()
        det.class_name = class_name
        det.class_confidence = confidence
        det.geometry_confidence = 0.90
        det.geometry_verified = True
        det.roi = RegionOfInterest(
            x_offset=500, y_offset=340, width=280, height=280)
        det.center_px = Point(640.0, 480.0, 0.0)
        det.center_refined = True
        det.center_source = "circle_geometry"
        det.association_valid = True
        det.reject_reason = ""
        det.map_valid = True
        det.map_point = Point(map_x, 2.0, 0.0)
        det.map_frame = "map"
        det.map_quality = 0.90
        return det

    @staticmethod
    def _message(detections, completed_sources=None):
        message = TargetDetectionArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "camera"
        message.source = "memory_regression"
        message.completed_sources = list(
            COMPLETE_SEARCH_SOURCES if completed_sources is None
            else completed_sources)
        for detection in detections:
            detection.header = message.header
        message.detections = detections
        return message

    def _publish(self, detections, completed_sources=None,
                 expect_output=True):
        message = self._message(detections, completed_sources)
        with self._condition:
            self._latest = None
        self._pub.publish(message)
        if not expect_output:
            deadline = rospy.Time.now() + rospy.Duration(0.25)
            with self._condition:
                while self._latest is None and not rospy.is_shutdown():
                    remaining = (deadline - rospy.Time.now()).to_sec()
                    if remaining <= 0.0:
                        return None
                    self._condition.wait(min(remaining, 0.05))
                raise AssertionError(
                    "target_memory published for an incomplete fusion frame")
        deadline = rospy.Time.now() + rospy.Duration(1.0)
        with self._condition:
            while self._latest is None and not rospy.is_shutdown():
                remaining = (deadline - rospy.Time.now()).to_sec()
                if remaining <= 0.0:
                    raise AssertionError("target_memory did not publish in 1.0 s")
                self._condition.wait(min(remaining, 0.1))
            return self._latest

    @staticmethod
    def _standard_targets(message):
        return [target for target in message.targets
                if target.class_name in STANDARD_CLASSES]

    def _reset_memory(self):
        self._reset()
        rospy.sleep(0.05)

    def _assert_class_flicker_keeps_physical_id(self):
        self._reset_memory()
        first = self._publish([self._detection("tent", 0.91, 1.00)])
        first_id = self._standard_targets(first)[0].id
        self._publish([self._detection("panzer", 0.65, 1.05)])
        below_threshold = self._publish([
            self._detection("panzer", 0.65, 0.95)])
        assert self._standard_targets(below_threshold)[0].class_name == "tent", \
            "below-threshold class evidence changed the committed class"
        self._publish([self._detection("panzer", 0.93, 1.10)])
        final = self._publish([self._detection("panzer", 0.94, 0.90)])
        targets = self._standard_targets(final)
        assert len(targets) == 1, "class flicker split one physical target: %r" % [
            (target.id, target.class_name) for target in targets]
        target = targets[0]
        assert target.id == first_id, "physical target ID changed"
        assert target.class_name == "panzer", "stable class did not switch"
        assert target.state == CONFIRMED_STATE, "three consecutive hits not confirmed"
        assert target.observe_count == 5, "cumulative observation count is wrong"
        assert abs(target.map_point.x - 1.0) < 0.03, "map fusion is not weighted"

    def _assert_miss_resets_confirmation_streak(self):
        self._reset_memory()
        self._publish([self._detection("panzer", 0.93, 1.0)])
        self._publish([self._detection("panzer", 0.93, 1.0)])
        self._publish([])
        after_gap = self._publish([self._detection("panzer", 0.93, 1.0)])
        target = self._standard_targets(after_gap)[0]
        assert target.state != CONFIRMED_STATE, \
            "non-consecutive observations incorrectly confirmed target"
        assert target.consecutive_observe_count == 1, \
            "hit streak did not reset after a missed frame"
        self._publish([self._detection("panzer", 0.93, 1.0)])
        confirmed = self._publish([self._detection("panzer", 0.93, 1.0)])
        target = self._standard_targets(confirmed)[0]
        assert target.state == CONFIRMED_STATE, \
            "three new consecutive observations did not confirm target"
        assert target.consecutive_observe_count == 3

    def _assert_partial_fusion_does_not_reset_streak(self):
        self._reset_memory()
        self._publish([self._detection("panzer", 0.93, 1.0)])
        observing = self._publish([
            self._detection("panzer", 0.93, 1.0)])
        target = self._standard_targets(observing)[0]
        assert target.consecutive_observe_count == 2
        self._publish(
            [], completed_sources=["circle_detector", "cross_detector"],
            expect_output=False)
        confirmed = self._publish([
            self._detection("panzer", 0.93, 1.0)])
        target = self._standard_targets(confirmed)[0]
        assert target.state == CONFIRMED_STATE, \
            "aux-only partial fusion frame reset a valid hit streak"
        assert target.consecutive_observe_count == 3
        assert target.observe_count == 3, \
            "partial fusion frame advanced the observation count"

    def _assert_concurrent_reset_isolates_queued_frames(self):
        self._reset_memory()
        publisher_errors = []

        def flood_detections():
            try:
                for index in range(120):
                    self._pub.publish(self._message([
                        self._detection(
                            "panzer", 0.93, 1.0 + 0.001 * index)]))
                    rospy.sleep(0.001)
            except Exception as error:  # propagate thread failures
                publisher_errors.append(error)

        publisher = threading.Thread(target=flood_detections)
        publisher.start()
        for _ in range(24):
            self._reset()
            rospy.sleep(0.002)
        publisher.join(timeout=3.0)
        assert not publisher.is_alive(), "detection publisher thread hung"
        assert not publisher_errors, "detection publisher failed: %r" % (
            publisher_errors,)

        # All queued messages now predate this final cutoff.  The service's
        # empty publication must remain the final state after callbacks drain.
        self._reset()
        rospy.sleep(0.35)
        with self._condition:
            latest = self._latest
        assert latest is not None and not latest.targets, \
            "a pre-reset queued frame repopulated target memory"

    def _assert_map_separates_same_pixel_targets(self):
        self._reset_memory()
        self._publish([self._detection("panzer", 0.93, 0.0)])
        final = self._publish([self._detection("panzer", 0.93, 2.0)])
        targets = self._standard_targets(final)
        assert len(targets) == 2, \
            "pixel fallback merged spatially separate mapped targets"

    def _assert_converged_map_duplicates_merge_to_oldest_id(self):
        self._reset_memory()
        first = self._publish([self._detection("pillbox", 0.93, 0.0)])
        first_id = self._standard_targets(first)[0].id
        split = self._publish([self._detection("pillbox", 0.93, 0.8)])
        assert len(self._standard_targets(split)) == 2
        self._publish([self._detection("pillbox", 0.93, 0.45)])
        merged = self._publish([self._detection("pillbox", 0.93, 0.40)])
        targets = self._standard_targets(merged)
        assert len(targets) == 1, "converged physical duplicates were retained"
        assert targets[0].id == first_id, "oldest stable ID was not retained"

    def _assert_current_map_validity_does_not_erase_memory(self):
        self._reset_memory()
        for _ in range(3):
            confirmed = self._publish([
                self._detection("panzer", 0.93, 1.25)])
        target = self._standard_targets(confirmed)[0]
        stable_id = target.id
        historical_x = target.map_point.x
        assert target.state == CONFIRMED_STATE and target.map_valid

        invalid = self._detection("panzer", 0.93, 99.0)
        invalid.map_valid = False
        invalid.map_frame = ""
        after_invalid = self._publish([invalid])
        target = self._standard_targets(after_invalid)[0]
        assert target.id == stable_id, "invalid projection replaced stable ID"
        assert target.state == CONFIRMED_STATE, \
            "sticky CONFIRMED memory was lost after invalid projection"
        assert not target.map_valid, \
            "historical map validity leaked into the current observation"
        assert abs(target.map_point.x - historical_x) < 1e-6, \
            "invalid projection erased the fused historical map point"
        _, allowed_classes = resolve_class_profile("full")
        assert choose_selected_candidate(
            after_invalid.targets, after_invalid.header.stamp, 3, 0.5,
            PRIORITIES, allowed_classes, CONFIRMED_STATE) is None, \
            "current-map-invalid sticky candidate remained selectable"

        after_miss = self._publish([])
        target = self._standard_targets(after_miss)[0]
        assert not target.map_valid, \
            "missed candidate reported a current valid map projection"
        assert abs(target.map_point.x - historical_x) < 1e-6, \
            "miss erased the fused historical map point"

        for _ in range(3):
            recovered = self._publish([
                self._detection("panzer", 0.93, 1.25)])
        target = self._standard_targets(recovered)[0]
        assert target.id == stable_id and target.map_valid, \
            "valid current projection did not recover the stable candidate"
        selected = choose_selected_candidate(
            recovered.targets, recovered.header.stamp, 3, 0.5,
            PRIORITIES, allowed_classes, CONFIRMED_STATE)
        assert selected is not None and selected.id == stable_id, \
            "reconfirmed current-map-valid candidate was not selectable"

    def _assert_raw_selected_rejects_missing_map_frame(self):
        self._reset_memory()
        with self._condition:
            self._selected = []
        for _ in range(3):
            invalid = self._detection("panzer", 0.93, 1.5)
            invalid.map_frame = ""
            targets = self._publish([invalid])
        candidates = self._standard_targets(targets)
        assert len(candidates) == 1 and candidates[0].state == CONFIRMED_STATE
        assert candidates[0].map_valid, \
            "test precondition did not preserve current map_valid"
        rospy.sleep(0.1)
        with self._condition:
            selected = list(self._selected)
        assert not selected, \
            "target_memory published raw selected with an empty map_frame"

    def run(self):
        rospy.wait_for_service("/uav_vision/reset_memory", timeout=5.0)
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self._pub.get_num_connections() == 0:
            if rospy.Time.now() >= deadline:
                raise AssertionError("target_memory subscriber unavailable")
            rospy.sleep(0.05)
        self._assert_class_flicker_keeps_physical_id()
        self._assert_miss_resets_confirmation_streak()
        self._assert_partial_fusion_does_not_reset_streak()
        self._assert_concurrent_reset_isolates_queued_frames()
        self._assert_map_separates_same_pixel_targets()
        self._assert_converged_map_duplicates_merge_to_oldest_id()
        self._assert_current_map_validity_does_not_erase_memory()
        self._assert_raw_selected_rejects_missing_map_frame()
        rospy.loginfo("V-CL physical target memory PASS")


if __name__ == "__main__":
    try:
        PhysicalMemoryAssertion().run()
    except Exception as exc:  # deterministic roslaunch failure report
        rospy.logerr("V-CL physical target memory FAIL: %s", exc)
        sys.exit(7)
    sys.exit(0)
