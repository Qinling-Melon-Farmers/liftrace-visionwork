"""Pure helpers for bounded, exact-stamp V-SIM diagnostic capture."""

from collections import OrderedDict
import json
import math
import os


CAPTURE_SCHEMA_VERSION = 2
CAPTURE_DATASET_KIND = "sim-small-target"
SAMPLING_POLICY = "source_stamp_expected_duration_0_to_90_percent"


def stamp_key(message):
    """Return an exact ROS stamp key without floating-point conversion."""
    stamp = message.header.stamp
    return int(stamp.secs), int(stamp.nsecs)


def stamp_dict(message):
    secs, nsecs = stamp_key(message)
    return {"secs": secs, "nsecs": nsecs}


def validate_capture_config(enabled, trial_selector, trial_slice, max_frames,
                            output_dir):
    """Reject configurations that could silently affect a formal run."""
    if not enabled:
        return
    selector = str(trial_selector).strip()
    slice_name = str(trial_slice).strip()
    if bool(selector) == bool(slice_name):
        raise ValueError(
            "failure capture requires exactly one of diagnostic "
            "trial_selector or trial_slice")
    if int(max_frames) <= 0:
        raise ValueError("failure capture max_frames must be positive")
    if not str(output_dir).strip():
        raise ValueError("failure capture output_dir must be non-empty")


def resolve_capture_output_dir(output_root, relative_output_dir):
    """Resolve one non-escaping run-relative capture directory."""
    root_value = str(output_root).strip()
    relative = str(relative_output_dir).strip()
    if not root_value or not relative:
        raise ValueError("capture output root and relative directory required")
    root = os.path.abspath(os.path.expanduser(root_value))
    components = relative.split("/")
    if (os.path.isabs(relative) or "\\" in relative or
            any(value in ("", ".", "..") for value in components)):
        raise ValueError("capture output_dir must be run-relative")
    normalized = os.path.normpath(relative)
    if normalized in ("", ".", "..") or normalized.startswith(".." + os.sep):
        raise ValueError("capture output_dir escapes its run output root")
    resolved = os.path.abspath(os.path.join(root, normalized))
    if os.path.commonpath((root, resolved)) != root:
        raise ValueError("capture output_dir escapes its run output root")
    return resolved


def allocate_trial_quotas(trial_ids, max_frames):
    """Split one bounded capture budget across ordered selected trials."""
    identifiers = [str(trial_id).strip() for trial_id in trial_ids]
    if not identifiers or any(not trial_id for trial_id in identifiers):
        raise ValueError("capture trial IDs must be non-empty")
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("capture trial IDs must be unique")
    total = int(max_frames)
    if total < len(identifiers):
        raise ValueError(
            "failure capture max_frames must cover every selected trial")
    base, remainder = divmod(total, len(identifiers))
    return OrderedDict(
        (trial_id, base + (1 if index < remainder else 0))
        for index, trial_id in enumerate(identifiers))


def expected_trial_duration(matrix, trial):
    """Return the matrix-defined source-time duration used for sampling."""
    if trial["kind"] == "static":
        duration = float(matrix.get("static", {}).get(
            "center_dwell_sec", 2.0))
    elif trial["kind"] == "dynamic":
        half_length = float(matrix.get("dynamic", {}).get(
            "path_half_length_m", 3.5))
        speed = float(trial.get("speed_mps", 0.0))
        if speed <= 0.0:
            raise ValueError("dynamic capture trial speed must be positive")
        duration = 2.0 * half_length / speed
    else:
        raise ValueError("unknown capture trial kind: {}".format(
            trial.get("kind")))
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("capture trial duration must be finite and positive")
    return duration


def sampling_offsets(expected_duration_sec, quota):
    """Spread a trial quota deterministically across its expected duration."""
    duration = float(expected_duration_sec)
    count = int(quota)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("sampling duration must be finite and positive")
    if count <= 0:
        raise ValueError("sampling quota must be positive")
    if count == 1:
        fractions = [0.45]
    else:
        fractions = [0.9 * index / float(count - 1)
                     for index in range(count)]
    return [duration * fraction for fraction in fractions]


def sampling_plan(matrix, trial, quota):
    duration = expected_trial_duration(matrix, trial)
    offsets = sampling_offsets(duration, quota)
    return {
        "policy": SAMPLING_POLICY,
        "expected_duration_sec": duration,
        "quota": int(quota),
        "sample_fractions": [offset / duration for offset in offsets],
        "sample_offsets_sec": offsets,
    }


class ExactStampPairBuffer:
    """Pair image and truth messages only when their header stamps are equal."""

    def __init__(self, max_pending=64):
        if int(max_pending) <= 0:
            raise ValueError("max_pending must be positive")
        self._max_pending = int(max_pending)
        self._images = OrderedDict()
        self._truth = OrderedDict()

    def clear(self):
        self._images.clear()
        self._truth.clear()

    @staticmethod
    def _trim(mapping, limit):
        while len(mapping) > limit:
            mapping.popitem(last=False)

    def add_image(self, message):
        return self._add(message, self._images, self._truth, image_first=True)

    def add_truth(self, message):
        return self._add(message, self._truth, self._images, image_first=False)

    def _add(self, message, own, other, image_first):
        key = stamp_key(message)
        counterpart = other.pop(key, None)
        if counterpart is not None:
            if image_first:
                return message, counterpart
            return counterpart, message
        own[key] = message
        own.move_to_end(key)
        self._trim(own, self._max_pending)
        return None


def select_truth_target(truth_message, class_name, target_id=""):
    """Select one valid projected target, rejecting absent or ambiguous truth."""
    matches = []
    for target in truth_message.targets:
        if target.class_name != class_name:
            continue
        if target_id and target.target_id != target_id:
            continue
        if (target.pose_valid and target.projection_valid and
                target.roi.width > 0 and target.roi.height > 0):
            matches.append(target)
    if len(matches) != 1:
        return None
    return matches[0]


def scene_targets(truth_message):
    """Return all valid fully visible labels so co-visible objects stay labeled."""
    visible = []
    for target in truth_message.targets:
        roi = target.roi
        if not (target.pose_valid and target.projection_valid and
                target.fully_in_frame and roi.width > 0 and roi.height > 0):
            continue
        visible.append({
            "target_id": str(target.target_id),
            "class_name": str(target.class_name),
            "roi": {
                "x_offset": int(roi.x_offset),
                "y_offset": int(roi.y_offset),
                "width": int(roi.width),
                "height": int(roi.height),
            },
            "distance_m": float(target.distance_m),
            "fully_in_frame": True,
        })
    visible.sort(key=lambda value: (
        value["target_id"], value["class_name"],
        value["roi"]["x_offset"], value["roi"]["y_offset"],
        value["roi"]["width"], value["roi"]["height"]))
    return visible


def _finite_values(name, values, expected_length=None):
    converted = tuple(float(value) for value in values)
    if expected_length is not None and len(converted) != expected_length:
        raise ValueError("CameraInfo {} has invalid length".format(name))
    if any(not math.isfinite(value) for value in converted):
        raise ValueError("CameraInfo {} contains non-finite values".format(
            name))
    return converted


def camera_info_profile(message):
    """Validate and return the stamp-independent fixed-camera profile."""
    frame_id = str(message.header.frame_id).strip()
    width, height = int(message.width), int(message.height)
    if not frame_id:
        raise ValueError("CameraInfo frame_id must be non-empty")
    if width <= 0 or height <= 0:
        raise ValueError("CameraInfo dimensions must be positive")
    distortion = _finite_values("D", message.D)
    intrinsic = _finite_values("K", message.K, 9)
    rectification = _finite_values("R", message.R, 9)
    projection = _finite_values("P", message.P, 12)
    if (intrinsic[0] <= 0.0 or intrinsic[4] <= 0.0 or
            projection[0] <= 0.0 or projection[5] <= 0.0):
        raise ValueError("CameraInfo fx/fy must be positive in K and P")
    roi = message.roi
    return {
        "frame_id": frame_id,
        "width": width,
        "height": height,
        "distortion_model": str(message.distortion_model),
        "D": distortion,
        "K": intrinsic,
        "R": rectification,
        "P": projection,
        "binning_x": int(message.binning_x),
        "binning_y": int(message.binning_y),
        "roi": (
            int(roi.x_offset), int(roi.y_offset), int(roi.width),
            int(roi.height), bool(roi.do_rectify)),
    }


def freeze_camera_info_profile(frozen_profile, message, image=None):
    """Freeze the first legal profile and fail on any later profile drift."""
    profile = camera_info_profile(message)
    if frozen_profile is not None and profile != frozen_profile:
        raise ValueError("CameraInfo profile changed during capture")
    if image is not None:
        if str(image.header.frame_id) != profile["frame_id"]:
            raise ValueError("CameraInfo frame_id does not match image")
        if (int(image.width) != profile["width"] or
                int(image.height) != profile["height"]):
            raise ValueError("CameraInfo dimensions do not match image")
    return profile


def rgb8_sentinel_to_bgr8(rgb_pixel):
    """Pure channel-order sentinel for the explicit RGB-to-BGR contract."""
    values = tuple(int(value) for value in rgb_pixel)
    if len(values) != 3 or any(value < 0 or value > 255 for value in values):
        raise ValueError("RGB sentinel must contain three uint8 values")
    return values[2], values[1], values[0]


def camera_info_dict(message):
    return {
        "stamp": stamp_dict(message),
        "frame_id": str(message.header.frame_id),
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": str(message.distortion_model),
        "D": [float(value) for value in message.D],
        "K": [float(value) for value in message.K],
        "R": [float(value) for value in message.R],
        "P": [float(value) for value in message.P],
        "binning_x": int(message.binning_x),
        "binning_y": int(message.binning_y),
        "roi": {
            "x_offset": int(message.roi.x_offset),
            "y_offset": int(message.roi.y_offset),
            "width": int(message.roi.width),
            "height": int(message.roi.height),
            "do_rectify": bool(message.roi.do_rectify),
        },
    }


def build_frame_record(trial, image, truth_message, target, camera_info,
                       image_filename, metadata_filename, capture_index,
                       sample=None):
    if stamp_key(image) != stamp_key(truth_message):
        raise ValueError("image and truth header stamps differ")
    freeze_camera_info_profile(None, camera_info, image)
    if target.class_name != trial["class_name"]:
        raise ValueError("truth class does not match active trial")
    roi = target.roi
    record = {
        "schema_version": CAPTURE_SCHEMA_VERSION,
        "dataset_kind": CAPTURE_DATASET_KIND,
        "capture_index": int(capture_index),
        "image_file": str(image_filename),
        "metadata_file": str(metadata_filename),
        "trial": {
            "trial_id": str(trial["trial_id"]),
            "kind": str(trial["kind"]),
            "class_name": str(trial["class_name"]),
            "height_m": float(trial["height_m"]),
            "speed_mps": (
                None if trial.get("speed_mps") is None else
                float(trial["speed_mps"])),
        },
        "image": {
            "stamp": stamp_dict(image),
            "frame_id": str(image.header.frame_id),
            "width": int(image.width),
            "height": int(image.height),
            "encoding": str(image.encoding),
            "source_encoding": str(image.encoding),
            "saved_encoding": "bgr8",
            "step": int(image.step),
        },
        "truth": {
            "stamp": stamp_dict(truth_message),
            "association": "exact_header_stamp",
            "scenario_id": str(truth_message.scenario_id),
            "target_id": str(target.target_id),
            "class_name": str(target.class_name),
            "fully_in_frame": bool(target.fully_in_frame),
            "distance_m": float(target.distance_m),
            "roi": {
                "x_offset": int(roi.x_offset),
                "y_offset": int(roi.y_offset),
                "width": int(roi.width),
                "height": int(roi.height),
            },
        },
        "scene_targets": scene_targets(truth_message),
        "camera_info": camera_info_dict(camera_info),
    }
    if sample is not None:
        record["sampling"] = {
            "policy": str(sample["policy"]),
            "sample_index": int(sample["sample_index"]),
            "planned_fraction": float(sample["planned_fraction"]),
            "planned_offset_sec": float(sample["planned_offset_sec"]),
            "actual_offset_sec": float(sample["actual_offset_sec"]),
            "expected_duration_sec": float(
                sample["expected_duration_sec"]),
            "used_trial_end_fallback": bool(
                sample.get("used_trial_end_fallback", False)),
        }
    return record


def validate_capture_manifest(payload, output_dir):
    """Validate a successful capture manifest and every declared frame file."""
    if not isinstance(payload, dict):
        raise ValueError("capture manifest must be a mapping")
    if int(payload.get("schema_version", -1)) != CAPTURE_SCHEMA_VERSION:
        raise ValueError("capture manifest schema_version mismatch")
    if payload.get("dataset_kind") != CAPTURE_DATASET_KIND:
        raise ValueError("capture manifest dataset_kind mismatch")
    if payload.get("status") != "DIAGNOSTIC" or payload.get(
            "run_complete") is not True:
        raise ValueError("capture manifest is not a complete diagnostic run")
    counts = payload.get("trial_counts")
    quotas = payload.get("trial_quotas")
    if not isinstance(counts, dict) or not isinstance(quotas, dict):
        raise ValueError("capture manifest lacks trial counts or quotas")
    if set(counts) != set(quotas) or not counts:
        raise ValueError("capture trial count/quota keys differ")
    normalized_counts = {key: int(value) for key, value in counts.items()}
    normalized_quotas = {key: int(value) for key, value in quotas.items()}
    if any(value <= 0 for value in normalized_quotas.values()):
        raise ValueError("capture trial quotas must be positive")
    if normalized_counts != normalized_quotas:
        raise ValueError("capture trial counts do not satisfy quotas")
    captured = int(payload.get("captured_frames", -1))
    max_frames = int(payload.get("max_frames", -1))
    if (captured != sum(normalized_counts.values()) or
            captured != sum(normalized_quotas.values()) or
            captured != max_frames):
        raise ValueError("capture frame totals do not match quotas")
    records = payload.get("records")
    if not isinstance(records, list) or len(records) != captured:
        raise ValueError("capture record count mismatch")
    root = os.path.abspath(output_dir)
    indices = []
    for record in records:
        if (not isinstance(record, dict) or
                int(record.get("schema_version", -1)) !=
                CAPTURE_SCHEMA_VERSION or
                record.get("dataset_kind") != CAPTURE_DATASET_KIND):
            raise ValueError("capture frame record schema mismatch")
        indices.append(int(record.get("capture_index", -1)))
        image_record = record.get("image", {})
        if (not str(image_record.get("source_encoding", "")).strip() or
                image_record.get("saved_encoding") != "bgr8"):
            raise ValueError("capture frame is not declared as saved bgr8")
        truth_record = record.get("truth")
        labels = record.get("scene_targets")
        if not isinstance(truth_record, dict) or not isinstance(labels, list):
            raise ValueError("capture frame lacks truth or scene_targets")
        label_keys = []
        for label in labels:
            roi = label.get("roi", {}) if isinstance(label, dict) else {}
            if (not str(label.get("target_id", "")).strip() or
                    not str(label.get("class_name", "")).strip() or
                    label.get("fully_in_frame") is not True or
                    int(roi.get("width", 0)) <= 0 or
                    int(roi.get("height", 0)) <= 0):
                raise ValueError("capture scene_targets schema is invalid")
            label_keys.append((
                label["target_id"], label["class_name"],
                int(roi.get("x_offset", 0)), int(roi.get("y_offset", 0)),
                int(roi["width"]), int(roi["height"])))
        if label_keys != sorted(label_keys):
            raise ValueError("capture scene_targets are not deterministic")
        sampling = record.get("sampling")
        if (not isinstance(sampling, dict) or
                sampling.get("policy") != SAMPLING_POLICY):
            raise ValueError("capture frame lacks source-time sampling schema")
        for field in ("image_file", "metadata_file"):
            filename = str(record.get(field, ""))
            if not filename or os.path.basename(filename) != filename:
                raise ValueError("capture record has unsafe {}".format(field))
            path = os.path.join(root, filename)
            if not os.path.isfile(path) or os.path.getsize(path) <= 0:
                raise ValueError("capture artifact is missing: " + filename)
        metadata_path = os.path.join(root, record["metadata_file"])
        with open(metadata_path, "r", encoding="utf-8") as stream:
            if json.load(stream) != record:
                raise ValueError("capture metadata does not match manifest")
    if indices != list(range(1, captured + 1)):
        raise ValueError("capture indices are not contiguous and ordered")
    return True
