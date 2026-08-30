#!/usr/bin/env python3
"""Pure mock/schema assertions for bounded V-SIM failure capture."""

from types import SimpleNamespace

from uav_vision_eval.failure_capture import (
    ExactStampPairBuffer,
    build_frame_record,
    select_truth_target,
    validate_capture_config,
)


def ns(**values):
    return SimpleNamespace(**values)


def stamp(secs, nsecs):
    return ns(secs=secs, nsecs=nsecs)


def header(secs, nsecs, frame_id="camera"):
    return ns(stamp=stamp(secs, nsecs), frame_id=frame_id)


def roi(x=10, y=20, width=30, height=40):
    return ns(x_offset=x, y_offset=y, width=width, height=height,
              do_rectify=False)


def main():
    validate_capture_config(False, "", 0, "")
    for arguments in (
            (True, "", 3, "/tmp/out"),
            (True, "static_pillbox_h3p6", 0, "/tmp/out"),
            (True, "static_pillbox_h3p6", 3, "")):
        try:
            validate_capture_config(*arguments)
            raise AssertionError("unsafe capture configuration was accepted")
        except ValueError:
            pass
    validate_capture_config(
        True, "static_pillbox_h3p6", 3, "/tmp/out")

    image = ns(header=header(12, 34), width=640, height=480,
               encoding="bgr8", step=1920)
    later_image = ns(header=header(12, 35), width=640, height=480,
                     encoding="bgr8", step=1920)
    target = ns(
        target_id="pillbox_1", class_name="pillbox", pose_valid=True,
        projection_valid=True, fully_in_frame=True, distance_m=3.6,
        roi=roi())
    truth = ns(header=header(12, 34), scenario_id="vsim04",
               targets=[target])
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
    assert select_truth_target(invalid_truth, "pillbox", "pillbox_1") is None

    info = ns(
        header=header(10, 0), width=640, height=480,
        distortion_model="plumb_bob", D=[0.0] * 5, K=[1.0] * 9,
        R=[1.0] * 9, P=[1.0] * 12, binning_x=0, binning_y=0,
        roi=roi(0, 0, 640, 480))
    trial = {
        "trial_id": "static_pillbox_h3p6", "kind": "static",
        "class_name": "pillbox", "height_m": 3.6,
    }
    record = build_frame_record(
        trial, image, truth, target, info, "frame.png", 1)
    assert record["schema_version"] == 1
    assert record["dataset_kind"] == "sim-small-target"
    assert record["trial"]["class_name"] == "pillbox"
    assert record["trial"]["height_m"] == 3.6
    assert record["image"]["stamp"] == {"secs": 12, "nsecs": 34}
    assert record["truth"]["roi"]["width"] == 30
    assert record["truth"]["association"] == "exact_header_stamp"
    assert len(record["camera_info"]["K"]) == 9
    try:
        build_frame_record(
            trial, later_image, truth, target, info, "bad.png", 2)
        raise AssertionError("mismatched image/truth stamps were accepted")
    except ValueError:
        pass
    bad_info = ns(**vars(info))
    bad_info.width = 320
    try:
        build_frame_record(
            trial, image, truth, target, bad_info, "bad_info.png", 2)
        raise AssertionError("mismatched CameraInfo dimensions were accepted")
    except ValueError:
        pass

    print("V-SIM-04 failure capture mock/schema PASS")


if __name__ == "__main__":
    main()
