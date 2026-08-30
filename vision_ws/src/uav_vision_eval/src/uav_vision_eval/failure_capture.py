"""Pure helpers for bounded, exact-stamp V-SIM diagnostic capture."""

from collections import OrderedDict
import json
import math
import os


CAPTURE_SCHEMA_VERSION = 3
CAPTURE_DATASET_KIND = "sim-small-target"
SAMPLING_POLICY = "source_stamp_visible_window_0_to_90_percent"
CAPTURE_STATUS_COMPONENT = "vsim04_failure_capture"
CAPTURE_STATUS_STATES = frozenset({
    "STARTING", "READY", "RUNNING", "FINALIZED", "FAIL",
})


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
    root = os.path.realpath(os.path.abspath(os.path.expanduser(root_value)))
    components = relative.split("/")
    if (os.path.isabs(relative) or "\\" in relative or
            any(value in ("", ".", "..") for value in components)):
        raise ValueError("capture output_dir must be run-relative")
    normalized = os.path.normpath(relative)
    if normalized in ("", ".", "..") or normalized.startswith(".." + os.sep):
        raise ValueError("capture output_dir escapes its run output root")
    resolved = os.path.realpath(os.path.abspath(os.path.join(
        root, normalized)))
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
        "trial_kind": str(trial["kind"]),
        "expected_duration_sec": duration,
        "quota": int(quota),
        "sample_fractions": [offset / duration for offset in offsets],
        "sample_offsets_sec": [],
        "aligned": False,
        "window_start_offset_sec": None,
        "window_duration_sec": None,
        "target_center_offset_sec": (
            duration * 0.5 if trial["kind"] == "dynamic" else None),
        "sampling_start_stamp_sec": None,
    }


def configure_sampling_plan(plan, expected_duration_sec,
                            target_center_offset_sec=None,
                            sampling_start_stamp_sec=None):
    """Bind a pending plan to the runner's actual clipped trajectory."""
    configured = dict(plan)
    if configured.get("aligned"):
        raise ValueError("aligned sampling plan cannot be reconfigured")
    expected = float(expected_duration_sec)
    if not math.isfinite(expected) or expected <= 0.0:
        raise ValueError("sampling trajectory duration must be positive")
    start_stamp = float(sampling_start_stamp_sec)
    if not math.isfinite(start_stamp) or start_stamp < 0.0:
        raise ValueError("sampling trajectory start stamp must be non-negative")
    if configured.get("trial_kind") == "dynamic":
        center = float(target_center_offset_sec)
        if (not math.isfinite(center) or center <= 0.0 or
                center >= expected):
            raise ValueError(
                "dynamic target center offset must lie inside trajectory")
        configured["target_center_offset_sec"] = center
    elif configured.get("trial_kind") == "static":
        if target_center_offset_sec is not None:
            raise ValueError("static sampling plan has no target center offset")
        configured["target_center_offset_sec"] = None
    else:
        raise ValueError("unknown sampling plan trial kind")
    configured["expected_duration_sec"] = expected
    configured["sampling_start_stamp_sec"] = start_stamp
    return configured


def align_sampling_plan(plan, first_eligible_offset_sec):
    """Anchor 0..90% to the target's observable source-time window."""
    aligned = dict(plan)
    if aligned.get("aligned"):
        raise ValueError("sampling plan is already aligned")
    start = float(first_eligible_offset_sec)
    expected = float(aligned.get("expected_duration_sec", 0.0))
    if not math.isfinite(start) or start < 0.0:
        raise ValueError("first eligible sampling offset must be non-negative")
    if not math.isfinite(expected) or expected <= 0.0:
        raise ValueError("sampling plan expected duration must be positive")
    if aligned.get("trial_kind") == "dynamic":
        # Mirror the first projected sample about the runner-declared target
        # crossing. This remains correct when arena clipping makes the target
        # crossing differ from half of the actual trajectory duration.
        center = float(aligned.get("target_center_offset_sec", math.nan))
        if not math.isfinite(center) or center <= 0.0 or center >= expected:
            raise ValueError("dynamic sampling plan target center is invalid")
        window_end = min(expected, 2.0 * center - start)
        window_duration = window_end - start
    elif aligned.get("trial_kind") == "static":
        # The static dwell starts after the camera is placed over the target.
        window_duration = expected
    else:
        raise ValueError("unknown sampling plan trial kind")
    if not math.isfinite(window_duration) or window_duration <= 0.0:
        raise ValueError(
            "first eligible target arrived too late for sampling window")
    fractions = [float(value) for value in aligned["sample_fractions"]]
    if any(not math.isfinite(value) or value < 0.0 or value > 0.9
           for value in fractions):
        raise ValueError("sampling plan fractions are invalid")
    aligned["sample_offsets_sec"] = [
        start + fraction * window_duration for fraction in fractions]
    aligned["aligned"] = True
    aligned["window_start_offset_sec"] = start
    aligned["window_duration_sec"] = window_duration
    return aligned


def sampling_timing(actual_offset_sec, planned_offset_sec,
                    max_lateness_sec, used_trial_end_fallback=False):
    """Validate one source-time sample and describe its lateness contract."""
    actual = float(actual_offset_sec)
    planned = float(planned_offset_sec)
    limit = float(max_lateness_sec)
    if not all(math.isfinite(value) for value in (actual, planned, limit)):
        raise ValueError("sampling timing values must be finite")
    if actual < 0.0 or planned < 0.0 or limit <= 0.0:
        raise ValueError("sampling offsets and lateness limit are invalid")
    lateness = actual - planned
    fallback = bool(used_trial_end_fallback)
    if fallback:
        return {
            "sampling_lateness_sec": lateness,
            "max_sampling_lateness_sec": limit,
            "lateness_limit_applies": False,
            "lateness_within_limit": None,
        }
    if lateness < -1.0e-6:
        raise ValueError("non-fallback sample precedes its planned offset")
    if lateness > limit + 1.0e-6:
        raise ValueError(
            "non-fallback sample lateness {:.6f}s exceeds {:.6f}s".format(
                lateness, limit))
    return {
        "sampling_lateness_sec": lateness,
        "max_sampling_lateness_sec": limit,
        "lateness_limit_applies": True,
        "lateness_within_limit": True,
    }


def build_capture_status(selected_trial_ids, state, ready=False,
                         active_trial="", active_event="", active_event_seq=0,
                         completed_trial_count=0, run_complete=False,
                         error=""):
    identifiers = [str(value).strip() for value in selected_trial_ids]
    if (not identifiers or any(not value for value in identifiers) or
            len(identifiers) != len(set(identifiers))):
        raise ValueError("capture status requires selected trial IDs")
    normalized_state = str(state).strip().upper()
    if normalized_state not in CAPTURE_STATUS_STATES:
        raise ValueError("invalid failure capture status state")
    return {
        "evaluation_id": "V-SIM-04",
        "component": CAPTURE_STATUS_COMPONENT,
        "schema_version": CAPTURE_SCHEMA_VERSION,
        "state": normalized_state,
        "ready": bool(ready),
        "selected_trial_ids": identifiers,
        "active_trial": str(active_trial),
        "active_event": str(active_event),
        "active_event_seq": int(active_event_seq),
        "completed_trial_count": int(completed_trial_count),
        "run_complete": bool(run_complete),
        "error": str(error),
    }


def validate_capture_status(payload, selected_trial_ids):
    """Validate the latched runner/capture readiness handshake payload."""
    if not isinstance(payload, dict):
        raise ValueError("failure capture status must be a mapping")
    expected = [str(value).strip() for value in selected_trial_ids]
    if (not expected or any(not value for value in expected) or
            len(expected) != len(set(expected))):
        raise ValueError("failure capture expected trial IDs are invalid")
    if (payload.get("evaluation_id") != "V-SIM-04" or
            payload.get("component") != CAPTURE_STATUS_COMPONENT or
            int(payload.get("schema_version", -1)) !=
            CAPTURE_SCHEMA_VERSION or
            payload.get("state") not in CAPTURE_STATUS_STATES or
            payload.get("selected_trial_ids") != expected):
        raise ValueError("failure capture status contract mismatch")
    if not isinstance(payload.get("ready"), bool):
        raise ValueError("failure capture status ready must be boolean")
    if int(payload.get("active_event_seq", -1)) < 0:
        raise ValueError("failure capture active event sequence is invalid")
    completed = int(payload.get("completed_trial_count", -1))
    if completed < 0 or completed > len(expected):
        raise ValueError("failure capture completed trial count is invalid")
    state = payload["state"]
    ready = payload["ready"]
    active = str(payload.get("active_trial", ""))
    active_event = str(payload.get("active_event", ""))
    sequence = int(payload.get("active_event_seq", -1))
    run_complete = payload.get("run_complete")
    error = str(payload.get("error", ""))
    if not isinstance(run_complete, bool):
        raise ValueError("failure capture run_complete must be boolean")
    if state == "STARTING" and (
            ready or completed != 0 or active or active_event or
            sequence != 0 or
            run_complete or error):
        raise ValueError("failure capture STARTING state is incoherent")
    if state == "READY" and (
            not ready or active or active_event or sequence != 0 or
            run_complete or error):
        raise ValueError("failure capture READY state is incoherent")
    if state == "RUNNING" and (
            not ready or active not in expected or sequence <= 0 or
            active_event not in ("trial_start", "sampling_start") or
            run_complete or error):
        raise ValueError("failure capture RUNNING state is incoherent")
    if state == "FINALIZED" and (
            not ready or active or active_event or sequence != 0 or
            completed != len(expected) or run_complete is not True or error):
        raise ValueError("failure capture FINALIZED state is incoherent")
    if state == "FAIL" and (ready or not error):
        raise ValueError("failure capture FAIL state is incoherent")
    return payload


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
            "sampling_start_stamp_sec": float(
                sample["sampling_start_stamp_sec"]),
            "window_start_offset_sec": float(
                sample["window_start_offset_sec"]),
            "window_duration_sec": float(sample["window_duration_sec"]),
            "sampling_lateness_sec": float(
                sample["sampling_lateness_sec"]),
            "max_sampling_lateness_sec": float(
                sample["max_sampling_lateness_sec"]),
            "lateness_limit_applies": bool(
                sample["lateness_limit_applies"]),
            "lateness_within_limit": sample["lateness_within_limit"],
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
    max_lateness = float(payload.get("max_sampling_lateness_sec", 0.0))
    if not math.isfinite(max_lateness) or max_lateness <= 0.0:
        raise ValueError("capture manifest sampling lateness limit is invalid")
    readiness = payload.get("readiness", {})
    if (readiness.get("camera_profile_frozen") is not True or
            readiness.get("exact_pair_observed") is not True or
            readiness.get("ready_before_first_trial") is not True):
        raise ValueError("capture manifest readiness evidence is incomplete")
    ready_stamp = readiness.get("ready_pair_stamp")
    if (not isinstance(ready_stamp, dict) or
            int(ready_stamp.get("secs", -1)) < 0 or
            int(ready_stamp.get("nsecs", -1)) < 0 or
            int(ready_stamp.get("nsecs", -1)) >= 1000000000):
        raise ValueError("capture manifest ready pair stamp is invalid")
    sampling_started_trial_ids = readiness.get(
        "sampling_started_trial_ids")
    counts = payload.get("trial_counts")
    quotas = payload.get("trial_quotas")
    if not isinstance(counts, dict) or not isinstance(quotas, dict):
        raise ValueError("capture manifest lacks trial counts or quotas")
    if set(counts) != set(quotas) or not counts:
        raise ValueError("capture trial count/quota keys differ")
    selected_trial_ids = payload.get("selected_trial_ids")
    if (not isinstance(selected_trial_ids, list) or
            any(not str(value).strip() for value in selected_trial_ids) or
            len(selected_trial_ids) != len(set(selected_trial_ids)) or
            set(selected_trial_ids) != set(quotas)):
        raise ValueError("capture selected trials differ from quotas")
    if sampling_started_trial_ids != selected_trial_ids:
        raise ValueError("capture sampling_start ACK evidence is incomplete")
    normalized_counts = {key: int(value) for key, value in counts.items()}
    normalized_quotas = {key: int(value) for key, value in quotas.items()}
    if any(value <= 0 for value in normalized_quotas.values()):
        raise ValueError("capture trial quotas must be positive")
    if normalized_counts != normalized_quotas:
        raise ValueError("capture trial counts do not satisfy quotas")
    plans = payload.get("sampling_plans")
    if not isinstance(plans, dict) or set(plans) != set(normalized_quotas):
        raise ValueError("capture sampling plan keys differ from quotas")
    for trial_id, quota in normalized_quotas.items():
        plan = plans[trial_id]
        offsets = plan.get("sample_offsets_sec", [])
        fractions = plan.get("sample_fractions", [])
        window_start = float(plan.get(
            "window_start_offset_sec", math.nan))
        window_duration = float(plan.get(
            "window_duration_sec", math.nan))
        expected_duration = float(plan.get(
            "expected_duration_sec", math.nan))
        sampling_start_stamp = float(plan.get(
            "sampling_start_stamp_sec", math.nan))
        trial_kind = plan.get("trial_kind")
        if (plan.get("policy") != SAMPLING_POLICY or
                plan.get("aligned") is not True or
                int(plan.get("quota", -1)) != quota or
                len(offsets) != quota or len(fractions) != quota or
                not math.isfinite(window_start) or window_start < 0.0 or
                not math.isfinite(window_duration) or
                window_duration <= 0.0 or
                not math.isfinite(expected_duration) or
                expected_duration <= 0.0 or
                not math.isfinite(sampling_start_stamp) or
                sampling_start_stamp < 0.0):
            raise ValueError("capture sampling plan is incomplete: " + trial_id)
        target_center = plan.get("target_center_offset_sec")
        if trial_kind == "dynamic":
            target_center = float(target_center)
            if (not math.isfinite(target_center) or target_center <= 0.0 or
                    target_center >= expected_duration):
                raise ValueError(
                    "capture dynamic target center is invalid: " + trial_id)
        elif trial_kind == "static":
            if target_center is not None:
                raise ValueError(
                    "capture static target center must be null: " + trial_id)
        else:
            raise ValueError("capture sampling trial kind is invalid")
        if trial_kind == "static":
            expected_window_duration = expected_duration
        else:
            expected_window_end = min(
                expected_duration, 2.0 * target_center - window_start)
            expected_window_duration = expected_window_end - window_start
        if (not math.isfinite(expected_window_duration) or
                expected_window_duration <= 0.0 or
                abs(window_duration - expected_window_duration) > 1.0e-6):
            raise ValueError("capture sampling window geometry is invalid")
        values = [float(value) for value in offsets]
        fraction_values = [float(value) for value in fractions]
        expected_fractions = sampling_offsets(1.0, quota)
        if (any(not math.isfinite(value) or value < 0.0 for value in values) or
                any(not math.isfinite(value) for value in fraction_values) or
                any(later <= earlier for earlier, later in zip(
                    values, values[1:]))):
            raise ValueError("capture sampling offsets are not increasing")
        if any(abs(actual - expected) > 1.0e-9 for actual, expected in zip(
                fraction_values, expected_fractions)):
            raise ValueError("capture sampling fractions are not deterministic")
        expected_offsets = [
            window_start + fraction * window_duration
            for fraction in fraction_values]
        if any(abs(actual - expected) > 1.0e-6 for actual, expected in zip(
                values, expected_offsets)):
            raise ValueError("capture sampling offsets do not match window")
    ordered_sampling_starts = [
        float(plans[trial_id]["sampling_start_stamp_sec"])
        for trial_id in selected_trial_ids]
    if any(later <= earlier for earlier, later in zip(
            ordered_sampling_starts, ordered_sampling_starts[1:])):
        raise ValueError("capture sampling_start stamps are not increasing")
    ready_stamp_sec = (
        float(ready_stamp["secs"]) + float(ready_stamp["nsecs"]) * 1.0e-9)
    if ready_stamp_sec > ordered_sampling_starts[0] + 1.0e-6:
        raise ValueError("capture readiness pair follows first sampling_start")
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
    observed_sample_indexes = {
        trial_id: [] for trial_id in normalized_quotas}
    observed_actual_offsets = {
        trial_id: [] for trial_id in normalized_quotas}
    observed_source_stamps = {
        trial_id: [] for trial_id in normalized_quotas}
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
        trial_record = record.get("trial")
        labels = record.get("scene_targets")
        if (not isinstance(truth_record, dict) or
                not isinstance(trial_record, dict) or
                not isinstance(labels, list)):
            raise ValueError("capture frame lacks truth or scene_targets")
        truth_roi = truth_record.get("roi", {})
        if (not str(truth_record.get("target_id", "")).strip() or
                not str(truth_record.get("class_name", "")).strip() or
                truth_record.get("class_name") !=
                trial_record.get("class_name") or
                truth_record.get("association") != "exact_header_stamp" or
                not isinstance(truth_record.get("fully_in_frame"), bool) or
                int(truth_roi.get("width", 0)) <= 0 or
                int(truth_roi.get("height", 0)) <= 0):
            raise ValueError("capture active truth schema is invalid")
        truth_distance = float(truth_record.get("distance_m", math.nan))
        if not math.isfinite(truth_distance) or truth_distance < 0.0:
            raise ValueError("capture active truth distance is invalid")
        image_stamp = image_record.get("stamp", {})
        truth_stamp = truth_record.get("stamp", {})
        if (not isinstance(image_stamp, dict) or
                not isinstance(truth_stamp, dict) or
                image_stamp != truth_stamp):
            raise ValueError("capture image/truth stamps are not exact")
        source_stamp = (
            int(image_stamp.get("secs", -1)),
            int(image_stamp.get("nsecs", -1)))
        if (source_stamp[0] < 0 or source_stamp[1] < 0 or
                source_stamp[1] >= 1000000000):
            raise ValueError("capture source stamp is invalid")
        label_keys = []
        for label in labels:
            roi = label.get("roi", {}) if isinstance(label, dict) else {}
            if (not str(label.get("target_id", "")).strip() or
                    not str(label.get("class_name", "")).strip() or
                    label.get("fully_in_frame") is not True or
                    not math.isfinite(float(label.get(
                        "distance_m", math.nan))) or
                    float(label.get("distance_m", -1.0)) < 0.0 or
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
        trial_id = str(trial_record.get("trial_id", ""))
        if trial_id not in plans:
            raise ValueError("capture record trial is not selected")
        if trial_record.get("kind") != plans[trial_id].get("trial_kind"):
            raise ValueError("capture record trial kind differs from plan")
        sample_index = int(sampling.get("sample_index", -1))
        if sample_index < 0 or sample_index >= normalized_quotas[trial_id]:
            raise ValueError("capture sample index is out of range")
        observed_sample_indexes[trial_id].append(sample_index)
        actual_offset = float(sampling.get(
            "actual_offset_sec", math.nan))
        observed_actual_offsets[trial_id].append(actual_offset)
        observed_source_stamps[trial_id].append(source_stamp)
        planned = float(sampling.get("planned_offset_sec", math.nan))
        plan = plans[trial_id]
        if abs(planned - float(
                plan["sample_offsets_sec"][sample_index])) > 1.0e-6:
            raise ValueError("capture record does not match sampling plan")
        for field, expected in (
                ("planned_fraction",
                 plan["sample_fractions"][sample_index]),
                ("expected_duration_sec", plan["expected_duration_sec"]),
                ("sampling_start_stamp_sec",
                 plan["sampling_start_stamp_sec"]),
                ("window_start_offset_sec",
                 plan["window_start_offset_sec"]),
                ("window_duration_sec", plan["window_duration_sec"])):
            actual = float(sampling.get(field, math.nan))
            if not math.isfinite(actual) or abs(
                    actual - float(expected)) > 1.0e-6:
                raise ValueError(
                    "capture sampling {} differs from plan".format(field))
        fallback = bool(sampling.get("used_trial_end_fallback", False))
        if fallback and sample_index != normalized_quotas[trial_id] - 1:
            raise ValueError("only the final sample may use trial-end fallback")
        timing = sampling_timing(
            sampling.get("actual_offset_sec"), planned, max_lateness,
            fallback)
        source_offset = (
            float(source_stamp[0]) + float(source_stamp[1]) * 1.0e-9 -
            float(plan["sampling_start_stamp_sec"]))
        if abs(source_offset - actual_offset) > 1.0e-6:
            raise ValueError(
                "capture actual offset does not match source stamp")
        for field in (
                "sampling_lateness_sec", "max_sampling_lateness_sec"):
            actual = float(sampling.get(field, math.nan))
            if (not math.isfinite(actual) or
                    abs(actual - float(timing[field])) > 1.0e-6):
                raise ValueError("capture sampling lateness record mismatch")
        if (sampling.get("lateness_limit_applies") is not
                timing["lateness_limit_applies"] or
                sampling.get("lateness_within_limit") is not
                timing["lateness_within_limit"]):
            raise ValueError("capture sampling lateness verdict mismatch")
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
    for trial_id, quota in normalized_quotas.items():
        if observed_sample_indexes[trial_id] != list(range(quota)):
            raise ValueError("capture sample indexes are not complete")
        if (any(not math.isfinite(value) for value in
                observed_actual_offsets[trial_id]) or
                any(later <= earlier for earlier, later in zip(
                    observed_actual_offsets[trial_id],
                    observed_actual_offsets[trial_id][1:])) or
                any(later <= earlier for earlier, later in zip(
                    observed_source_stamps[trial_id],
                    observed_source_stamps[trial_id][1:]))):
            raise ValueError(
                "capture samples are not strictly source-time ordered")
    return True
