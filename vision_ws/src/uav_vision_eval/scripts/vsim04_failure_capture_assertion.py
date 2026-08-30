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
    allocate_trial_quotas,
    build_frame_record,
    freeze_camera_info_profile,
    resolve_capture_output_dir,
    rgb8_sentinel_to_bgr8,
    sampling_offsets,
    sampling_plan,
    scene_targets,
    select_truth_target,
    validate_capture_config,
    validate_capture_manifest,
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
    static_plan = sampling_plan(
        {"static": {"center_dwell_sec": 2.0}},
        {"kind": "static"}, 2)
    assert static_plan["sample_offsets_sec"] == [0.0, 1.8]

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
    record = build_frame_record(
        trial, image, truth, target, info, "frame.png", "frame.json", 1,
        {
            "policy": static_plan["policy"],
            "sample_index": 0,
            "planned_fraction": 0.0,
            "planned_offset_sec": 0.0,
            "actual_offset_sec": 0.02,
            "expected_duration_sec": 2.0,
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
