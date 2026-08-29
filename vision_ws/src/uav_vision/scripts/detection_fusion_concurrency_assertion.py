#!/usr/bin/env python3
"""Deterministic regression for detection_fusion bucket/timer concurrency."""

from collections import OrderedDict
from types import SimpleNamespace
import threading

import rospy

from uav_vision.msg import TargetDetection

from detection_fusion import DetectionFusion


class PausingOrderedDict(OrderedDict):
    """Pause an active items() iterator so another callback can contend."""

    def __init__(self, iteration_started, resume_iteration):
        super().__init__()
        self._iteration_started = iteration_started
        self._resume_iteration = resume_iteration

    def items(self):
        iterator = iter(super().items())
        try:
            first = next(iterator)
        except StopIteration:
            return
        self._iteration_started.set()
        if not self._resume_iteration.wait(timeout=2.0):
            raise AssertionError("timed out waiting to resume OrderedDict iteration")
        yield first
        yield from iterator


class InstrumentedRLock:
    """Expose when the input callback reaches the contended lock boundary."""

    def __init__(self, input_lock_attempted):
        self._lock = threading.RLock()
        self._input_lock_attempted = input_lock_attempted

    def __enter__(self):
        if threading.current_thread().name == "fusion-input":
            self._input_lock_attempted.set()
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._lock.release()

    def owned_by_current_thread(self):
        return self._lock._is_owned()


def _new_fusion(iteration_started, resume_iteration, input_lock_attempted):
    fusion = DetectionFusion.__new__(DetectionFusion)
    fusion._state_lock = InstrumentedRLock(input_lock_attempted)
    fusion._buckets = PausingOrderedDict(
        iteration_started, resume_iteration)
    fusion._flushed_keys = OrderedDict()
    fusion._flush_delay = 0.0
    fusion._source_sync_slop = 0.05
    fusion._early_flush_complete_evidence = False
    fusion._align_mode = "disabled"
    fusion._published = []

    def publish(bucket, align_mode):
        assert not fusion._state_lock.owned_by_current_thread(), (
            "publish ran while holding the fusion state lock")
        fusion._published.append((bucket, align_mode))

    fusion._publish_bucket = publish
    return fusion


def _bucket(stamp, key):
    return {
        "header": SimpleNamespace(stamp=stamp),
        "stamp": stamp,
        "received_at": rospy.Time(0),
        "detections": [],
        "sources": set(),
        "member_keys": {key},
    }


def _message(stamp):
    # Empty source deliberately takes the direct insert path. Without the state
    # lock this mutates the OrderedDict while the timer iterator is paused.
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        source="",
        detections=[],
    )


def main():
    # Exercise callback methods without requiring a roscore for this unit test.
    rospy.rostime.set_rostime_initialized(True)
    iteration_started = threading.Event()
    resume_iteration = threading.Event()
    input_lock_attempted = threading.Event()
    mutation_finished = threading.Event()
    errors = []
    fusion = _new_fusion(
        iteration_started, resume_iteration, input_lock_attempted)

    for index in range(3):
        key = (10, index)
        fusion._buckets[key] = _bucket(rospy.Time(10, index), key)

    def flush():
        try:
            fusion._flush_ready(None)
        except BaseException as exc:  # surface callback-thread failures
            errors.append(exc)

    def mutate():
        try:
            fusion._on_detections(_message(rospy.Time(20, 1)))
        except BaseException as exc:
            errors.append(exc)
        finally:
            mutation_finished.set()

    flush_thread = threading.Thread(target=flush, name="fusion-flush")
    flush_thread.start()
    assert iteration_started.wait(timeout=1.0), (
        "flush did not enter bucket iteration; callback errors={}".format(errors))

    mutation_thread = threading.Thread(target=mutate, name="fusion-input")
    mutation_thread.start()
    assert input_lock_attempted.wait(timeout=1.0), (
        "input callback did not reach the shared state lock")
    assert not mutation_finished.is_set(), (
        "input callback modified buckets while timer held the iteration lock")

    resume_iteration.set()
    flush_thread.join(timeout=1.0)
    mutation_thread.join(timeout=1.0)
    assert not flush_thread.is_alive(), "flush callback did not finish"
    assert not mutation_thread.is_alive(), "input callback did not finish"
    assert not errors, "callback raised: {}".format(errors)
    assert len(fusion._published) == 3, fusion._published
    assert all(mode == "disabled" for _bucket_value, mode in fusion._published)
    assert (20, 1) in fusion._buckets, fusion._buckets

    # A timer flush and an early-completing callback contending for the same
    # stamp must transfer ownership only once. The callback observes flushed_keys
    # after acquiring the lock and cannot publish a duplicate.
    same_iteration_started = threading.Event()
    same_resume_iteration = threading.Event()
    same_input_lock_attempted = threading.Event()
    same_fusion = _new_fusion(
        same_iteration_started,
        same_resume_iteration,
        same_input_lock_attempted,
    )
    same_fusion._early_flush_complete_evidence = True
    same_key = (30, 1)
    same_stamp = rospy.Time(*same_key)
    same_bucket = _bucket(same_stamp, same_key)
    same_bucket["sources"] = {"target_detector", "circle_detector"}
    same_fusion._buckets[same_key] = same_bucket
    same_errors = []

    def same_flush():
        try:
            same_fusion._flush_ready(None)
        except BaseException as exc:
            same_errors.append(exc)

    def same_complete():
        try:
            message = _message(same_stamp)
            message.source = "cross_detector"
            same_fusion._on_detections(message)
        except BaseException as exc:
            same_errors.append(exc)

    same_flush_thread = threading.Thread(
        target=same_flush, name="fusion-flush")
    same_flush_thread.start()
    assert same_iteration_started.wait(timeout=1.0)
    same_complete_thread = threading.Thread(
        target=same_complete, name="fusion-input")
    same_complete_thread.start()
    assert same_input_lock_attempted.wait(timeout=1.0)
    same_resume_iteration.set()
    same_flush_thread.join(timeout=1.0)
    same_complete_thread.join(timeout=1.0)
    assert not same_errors, same_errors
    assert len(same_fusion._published) == 1, same_fusion._published
    assert same_key in same_fusion._flushed_keys

    # Mode filtering must use the snapshot captured when the bucket was taken,
    # not a later mode update that races with ROS serialization.
    mode_fusion = DetectionFusion.__new__(DetectionFusion)
    mode_fusion._state_lock = threading.RLock()
    mode_fusion._align_mode = "landing"
    mode_fusion._require_red_cross_dual_confirmation = False
    mode_fusion._deduplicate_same_class = False
    mode_fusion._suppress_bridge_on_red_cross = False
    mode_fusion._suppress_bridge_on_landing_pad = False
    mode_fusion._aux_geometry_confidence = 1.0
    published_messages = []
    mode_fusion._publisher = SimpleNamespace(publish=published_messages.append)
    detection = TargetDetection()
    detection.class_name = "tent"
    mode_fusion._publish_bucket(
        {
            "header": detection.header,
            "detections": [detection],
            "sources": {"target_detector"},
        },
        align_mode="disabled",
    )
    assert len(published_messages) == 1
    assert [item.class_name for item in published_messages[0].detections] == ["tent"]
    print("detection_fusion OrderedDict concurrency PASS")


if __name__ == "__main__":
    main()
