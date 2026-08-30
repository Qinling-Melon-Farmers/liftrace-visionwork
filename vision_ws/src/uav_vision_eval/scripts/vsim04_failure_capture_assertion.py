#!/usr/bin/env python3
"""Pure mock/schema assertions for bounded V-SIM failure capture."""

import copy
import json
import math
import os
import tempfile
from types import SimpleNamespace

from uav_vision_eval.failure_capture import (
    CAPTURE_DATASET_KIND,
    CAPTURE_SCHEMA_VERSION,
    ExactStampPairBuffer,
    align_sampling_plan,
    allocate_trial_quotas,
    build_capture_status,
    build_frame_record,
    configure_sampling_plan,
    freeze_camera_info_profile,
    resolve_capture_output_dir,
    rgb8_sentinel_to_bgr8,
    sampling_offsets,
    sampling_plan,
    sampling_timing,
    scene_targets,
    select_truth_target,
    validate_capture_config,
    validate_capture_manifest,
    validate_capture_status,
)
from uav_vision_eval.vsim04_metrics import select_trial_matrix


def ns(**values):
    return SimpleNamespace(**values)


def stamp(secs, nsecs):
    return ns(secs=secs, nsecs=nsecs)


def header(secs, nsecs, frame_id="camera"):
    return ns(stamp=stamp(secs, nsecs), frame_id=frame_id)


def roi(x=10, y=20, width=30, height=40):
    return ns(x_offset=x, y_offset=y, width=width, height=height,
              do_rectify=False)


def expect_value_error(action, message):
    try:
        action()
        raise AssertionError(message)
    except ValueError:
        pass


def main():
    validate_capture_config(False, "", "", 0, "")
    for arguments in (
            (True, "", "", 3, "capture"),
            (True, "trial_a", "slice_a", 3, "capture"),
            (True, "trial_a", "", 0, "capture"),
            (True, "trial_a", "", 3, "")):
        expect_value_error(
            lambda values=arguments: validate_capture_config(*values),
            "unsafe capture configuration was accepted")
    validate_capture_config(True, "trial_a", "", 3, "capture")
    validate_capture_config(True, "", "slice_a", 3, "capture")

    with tempfile.TemporaryDirectory() as temporary:
        assert resolve_capture_output_dir(
            temporary, "failure/pillbox") == os.path.join(
                temporary, "failure", "pillbox")
        for unsafe in ("", ".", "..", "../escape", "/tmp/escape"):
            expect_value_error(
                lambda value=unsafe: resolve_capture_output_dir(
                    temporary, value),
                "escaping capture output directory was accepted")
        expect_value_error(
            lambda: resolve_capture_output_dir("", "capture"),
            "empty capture output root was accepted")
        outside = tempfile.mkdtemp()
        try:
            os.symlink(outside, os.path.join(temporary, "redirect"))
            expect_value_error(
                lambda: resolve_capture_output_dir(temporary, "redirect/run"),
                "symlinked capture output escaped its run root")
        finally:
            os.rmdir(outside)

    selection_matrix = {
        "trials": [
            {"trial_id": "a"}, {"trial_id": "b"}, {"trial_id": "c"}],
        "trial_slices": {"pair": ["a", "c"]},
    }
    selected = select_trial_matrix(selection_matrix, "", "pair")
    assert selected["trial_slice"] == "pair"
    assert [trial["trial_id"] for trial in selected["trials"]] == ["a", "c"]
    expect_value_error(
        lambda: select_trial_matrix(selection_matrix, "a", "pair"),
        "selector/slice conflict was accepted")

    assert dict(allocate_trial_quotas(["a", "b"], 5)) == {
        "a": 3, "b": 2}
    assert dict(allocate_trial_quotas(["a", "b", "c"], 3)) == {
        "a": 1, "b": 1, "c": 1}
    for identifiers, total in ((["a", "b"], 1), (["a", "a"], 2)):
        expect_value_error(
            lambda ids=identifiers, count=total: allocate_trial_quotas(
                ids, count),
            "unsafe capture quota was accepted")
    assert sampling_offsets(10.0, 1) == [4.5]
    assert sampling_offsets(10.0, 3) == [0.0, 4.5, 9.0]
    dynamic_plan = sampling_plan(
        {"dynamic": {"path_half_length_m": 3.5}},
        {"kind": "dynamic", "speed_mps": 2.0}, 3)
    assert dynamic_plan["expected_duration_sec"] == 3.5
    assert dynamic_plan["sample_fractions"] == [0.0, 0.45, 0.9]
    assert dynamic_plan["sample_offsets_sec"] == []
    dynamic_plan = align_sampling_plan(dynamic_plan, 1.5)
    assert all(math.isclose(actual, expected) for actual, expected in zip(
        dynamic_plan["sample_offsets_sec"], [1.5, 1.725, 1.95]))
    # Regression for the real h1.2 run: the first eligible frame arrived at
    # 3.0 s. The old full-path plan captured 3.0/3.16 s back-to-back because
    # 45% was anchored at 3.15 s; the visible-window plan stays separated.
    low_plan = align_sampling_plan(sampling_plan(
        {"dynamic": {"path_half_length_m": 3.5}},
        {"kind": "dynamic", "speed_mps": 1.0}, 3), 3.0)
    assert all(math.isclose(actual, expected) for actual, expected in zip(
        low_plan["sample_offsets_sec"], [3.0, 3.45, 3.9]))
    assert low_plan["sample_offsets_sec"][1] - low_plan[
        "sample_offsets_sec"][0] >= 0.4
    static_plan = sampling_plan(
        {"static": {"center_dwell_sec": 2.0}},
        {"kind": "static"}, 2)
    static_plan = align_sampling_plan(static_plan, 0.1)
    assert all(math.isclose(actual, expected) for actual, expected in zip(
        static_plan["sample_offsets_sec"], [0.1, 1.9]))
    clipped_plan = configure_sampling_plan(sampling_plan(
        {"dynamic": {"path_half_length_m": 3.5}},
        {"kind": "dynamic", "speed_mps": 1.0}, 3),
        5.8, 2.3, 100.0)
    clipped_plan = align_sampling_plan(clipped_plan, 1.0)
    assert all(math.isclose(actual, expected) for actual, expected in zip(
        clipped_plan["sample_offsets_sec"], [1.0, 2.17, 3.34]))
    expect_value_error(
        lambda: align_sampling_plan(sampling_plan(
            {"dynamic": {"path_half_length_m": 3.5}},
            {"kind": "dynamic", "speed_mps": 1.0}, 3), 3.5),
        "late first-visible frame produced an invalid sampling window")
    timing = sampling_timing(3.46, 3.45, 0.25)
    assert abs(timing["sampling_lateness_sec"] - 0.01) < 1.0e-9
    assert timing["lateness_limit_applies"] is True
    assert timing["lateness_within_limit"] is True
    expect_value_error(
        lambda: sampling_timing(3.8, 3.45, 0.25),
        "over-late non-fallback sample was accepted")
    fallback_timing = sampling_timing(3.8, 3.9, 0.25, True)
    assert fallback_timing["lateness_limit_applies"] is False
    assert fallback_timing["lateness_within_limit"] is None

    capture_status = build_capture_status(
        ["trial_a", "trial_b"], "READY", ready=True)
    assert validate_capture_status(
        capture_status, ["trial_a", "trial_b"])["ready"] is True
    running_status = build_capture_status(
        ["trial_a", "trial_b"], "RUNNING", ready=True,
        active_trial="trial_a", active_event="sampling_start",
        active_event_seq=7)
    assert validate_capture_status(
        running_status, ["trial_a", "trial_b"])[
            "active_event_seq"] == 7
    expect_value_error(
        lambda: validate_capture_status(
            capture_status, ["trial_b", "trial_a"]),
        "capture status with a different trial order was accepted")
    broken_status = copy.deepcopy(capture_status)
    broken_status["schema_version"] += 1
    expect_value_error(
        lambda: validate_capture_status(
            broken_status, ["trial_a", "trial_b"]),
        "capture status with an unknown schema was accepted")
    incoherent_status = copy.deepcopy(capture_status)
    incoherent_status["active_trial"] = "trial_a"
    expect_value_error(
        lambda: validate_capture_status(
            incoherent_status, ["trial_a", "trial_b"]),
        "incoherent READY capture status was accepted")

    image = ns(header=header(12, 34), width=640, height=480,
               encoding="rgb8", step=1920)
    later_image = ns(header=header(12, 35), width=640, height=480,
                     encoding="rgb8", step=1920)
    target = ns(
        target_id="pillbox_1", class_name="pillbox", pose_valid=True,
        projection_valid=True, fully_in_frame=True, distance_m=3.6,
        roi=roi())
    co_visible = ns(
        target_id="bridge_0", class_name="bridge", pose_valid=True,
        projection_valid=True, fully_in_frame=True, distance_m=4.0,
        roi=roi(100, 120, 20, 25))
    truth = ns(header=header(12, 34), scenario_id="vsim04",
               targets=[target, co_visible])
    later_truth = ns(header=header(12, 36), scenario_id="vsim04",
                     targets=[target])
    pairs = ExactStampPairBuffer(max_pending=2)
    assert pairs.add_image(image) is None
    assert pairs.add_truth(later_truth) is None
    assert pairs.add_image(later_image) is None
    matched = pairs.add_truth(truth)
    assert matched == (image, truth)
    pairs.clear()
    assert pairs.add_truth(truth) is None
    matched = pairs.add_image(image)
    assert matched == (image, truth)

    ambiguous_truth = ns(header=header(12, 34), scenario_id="vsim04",
                         targets=[target, target])
    assert select_truth_target(truth, "pillbox", "pillbox_1") is target
    assert select_truth_target(ambiguous_truth, "pillbox", "") is None
    invalid_target = ns(**vars(target))
    invalid_target.projection_valid = False
    invalid_truth = ns(header=header(12, 34), scenario_id="vsim04",
                       targets=[invalid_target])
    assert select_truth_target(
        invalid_truth, "pillbox", "pillbox_1") is None
    labels = scene_targets(truth)
    assert [value["target_id"] for value in labels] == [
        "bridge_0", "pillbox_1"]
    assert labels[0]["fully_in_frame"] is True
    partial = ns(**vars(co_visible))
    partial.target_id = "partial"
    partial.fully_in_frame = False
    partial_truth = ns(header=header(12, 34), scenario_id="vsim04",
                       targets=[partial, target])
    assert [value["target_id"] for value in scene_targets(partial_truth)] == [
        "pillbox_1"]

    intrinsic = [500.0, 0.0, 320.0, 0.0, 501.0, 240.0, 0.0, 0.0, 1.0]
    projection = [
        500.0, 0.0, 320.0, 0.0,
        0.0, 501.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0]
    info = ns(
        header=header(10, 0), width=640, height=480,
        distortion_model="plumb_bob", D=[0.0] * 5, K=intrinsic,
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=projection, binning_x=0, binning_y=0,
        roi=roi(0, 0, 640, 480))
    profile = freeze_camera_info_profile(None, info, image)
    # CameraInfo is a latched profile contract; its stamp need not equal image.
    restamped_info = copy.deepcopy(info)
    restamped_info.header.stamp = stamp(999, 1)
    assert freeze_camera_info_profile(profile, restamped_info, image) == profile
    for mutate in (
            lambda value: setattr(value, "width", 320),
            lambda value: setattr(value.header, "frame_id", "other_camera"),
            lambda value: setattr(value, "D", [0.1] * 5),
            lambda value: setattr(value, "K", [math.nan] + intrinsic[1:]),
            lambda value: setattr(value, "P", [0.0] + projection[1:])):
        changed = copy.deepcopy(info)
        mutate(changed)
        expect_value_error(
            lambda value=changed: freeze_camera_info_profile(
                profile, value, image),
            "invalid or changed CameraInfo profile was accepted")
    wrong_frame_image = copy.deepcopy(image)
    wrong_frame_image.header.frame_id = "wrong_camera"
    expect_value_error(
        lambda: freeze_camera_info_profile(profile, info, wrong_frame_image),
        "CameraInfo/image frame mismatch was accepted")
    assert rgb8_sentinel_to_bgr8((17, 43, 199)) == (199, 43, 17)

    trial = {
        "trial_id": "static_pillbox_h3p6", "kind": "static",
        "class_name": "pillbox", "height_m": 3.6,
    }
    manifest_plan = configure_sampling_plan(sampling_plan(
        {"static": {"center_dwell_sec": 2.0}}, trial, 1),
        2.0, None, 11.0)
    manifest_plan = align_sampling_plan(manifest_plan, 0.1)
    planned_offset = manifest_plan["sample_offsets_sec"][0]
    source_offset = 12.0 + 34.0e-9 - 11.0
    record_timing = sampling_timing(
        source_offset, planned_offset, 0.25)
    record = build_frame_record(
        trial, image, truth, target, info, "frame.png", "frame.json", 1,
        {
            "policy": manifest_plan["policy"],
            "sample_index": 0,
            "planned_fraction": manifest_plan["sample_fractions"][0],
            "planned_offset_sec": planned_offset,
            "actual_offset_sec": source_offset,
            "expected_duration_sec":
                manifest_plan["expected_duration_sec"],
            "sampling_start_stamp_sec":
                manifest_plan["sampling_start_stamp_sec"],
            "window_start_offset_sec":
                manifest_plan["window_start_offset_sec"],
            "window_duration_sec": manifest_plan["window_duration_sec"],
            **record_timing,
            "used_trial_end_fallback": False,
        })
    assert record["schema_version"] == CAPTURE_SCHEMA_VERSION
    assert record["dataset_kind"] == CAPTURE_DATASET_KIND
    assert record["trial"]["class_name"] == "pillbox"
    assert record["trial"]["height_m"] == 3.6
    assert record["image"]["stamp"] == {"secs": 12, "nsecs": 34}
    assert record["image"]["source_encoding"] == "rgb8"
    assert record["image"]["saved_encoding"] == "bgr8"
    assert record["truth"]["roi"]["width"] == 30
    assert record["truth"]["association"] == "exact_header_stamp"
    assert len(record["scene_targets"]) == 2
    assert record["truth"]["target_id"] == "pillbox_1"
    assert len(record["camera_info"]["K"]) == 9
    expect_value_error(
        lambda: build_frame_record(
            trial, later_image, truth, target, info,
            "bad.png", "bad.json", 2),
        "mismatched image/truth stamps were accepted")

    with tempfile.TemporaryDirectory() as output_dir:
        image_path = os.path.join(output_dir, "frame.png")
        metadata_path = os.path.join(output_dir, "frame.json")
        with open(image_path, "wb") as stream:
            stream.write(b"not-empty-png-sentinel")
        with open(metadata_path, "w", encoding="utf-8") as stream:
            json.dump(record, stream, sort_keys=True)
        manifest = {
            "schema_version": CAPTURE_SCHEMA_VERSION,
            "dataset_kind": CAPTURE_DATASET_KIND,
            "status": "DIAGNOSTIC",
            "run_complete": True,
            "selected_trial_ids": [trial["trial_id"]],
            "sampling_plans": {trial["trial_id"]: manifest_plan},
            "max_sampling_lateness_sec": 0.25,
            "readiness": {
                "camera_profile_frozen": True,
                "exact_pair_observed": True,
                "ready_before_first_trial": True,
                "ready_pair_stamp": {"secs": 10, "nsecs": 900},
                "sampling_started_trial_ids": [trial["trial_id"]],
            },
            "max_frames": 1,
            "captured_frames": 1,
            "trial_counts": {trial["trial_id"]: 1},
            "trial_quotas": {trial["trial_id"]: 1},
            "records": [record],
        }
        assert validate_capture_manifest(manifest, output_dir)
        incomplete = copy.deepcopy(manifest)
        incomplete["run_complete"] = False
        expect_value_error(
            lambda: validate_capture_manifest(incomplete, output_dir),
            "incomplete capture manifest was accepted")
        wrong_quota = copy.deepcopy(manifest)
        wrong_quota["trial_quotas"][trial["trial_id"]] = 2
        expect_value_error(
            lambda: validate_capture_manifest(wrong_quota, output_dir),
            "capture manifest with unmet quota was accepted")
        missing_ready = copy.deepcopy(manifest)
        missing_ready["readiness"]["ready_before_first_trial"] = False
        expect_value_error(
            lambda: validate_capture_manifest(missing_ready, output_dir),
            "capture manifest without pre-trial readiness was accepted")
        missing_sampling_ack = copy.deepcopy(manifest)
        missing_sampling_ack["readiness"][
            "sampling_started_trial_ids"] = []
        expect_value_error(
            lambda: validate_capture_manifest(
                missing_sampling_ack, output_dir),
            "capture manifest without sampling_start ACK was accepted")
        late_sample = copy.deepcopy(manifest)
        late_sampling = late_sample["records"][0]["sampling"]
        late_sampling["actual_offset_sec"] = (
            late_sampling["planned_offset_sec"] + 0.3)
        late_sampling["sampling_lateness_sec"] = 0.3
        expect_value_error(
            lambda: validate_capture_manifest(late_sample, output_dir),
            "capture manifest with an over-late sample was accepted")
        wrong_source_offset = copy.deepcopy(manifest)
        wrong_sampling = wrong_source_offset["records"][0]["sampling"]
        wrong_sampling["actual_offset_sec"] += 0.01
        wrong_sampling["sampling_lateness_sec"] += 0.01
        expect_value_error(
            lambda: validate_capture_manifest(
                wrong_source_offset, output_dir),
            "capture offset detached from its source stamp was accepted")
        wrong_window = copy.deepcopy(manifest)
        wrong_window["sampling_plans"][trial["trial_id"]][
            "window_duration_sec"] = 1.5
        expect_value_error(
            lambda: validate_capture_manifest(wrong_window, output_dir),
            "capture manifest with false window geometry was accepted")
        nondeterministic_plan = copy.deepcopy(manifest)
        nondeterministic_plan["sampling_plans"][trial["trial_id"]][
            "sample_fractions"] = [0.0]
        expect_value_error(
            lambda: validate_capture_manifest(
                nondeterministic_plan, output_dir),
            "capture manifest with a non-deterministic fraction was accepted")
        bad_schema = copy.deepcopy(manifest)
        bad_schema["records"][0].pop("scene_targets")
        expect_value_error(
            lambda: validate_capture_manifest(bad_schema, output_dir),
            "capture manifest with incomplete frame schema was accepted")
        os.unlink(metadata_path)
        expect_value_error(
            lambda: validate_capture_manifest(manifest, output_dir),
            "capture manifest with a missing JSON file was accepted")

    print("V-SIM-04 failure capture mock/schema PASS")


def test_failure_capture_contract():
    main()


if __name__ == "__main__":
    main()
