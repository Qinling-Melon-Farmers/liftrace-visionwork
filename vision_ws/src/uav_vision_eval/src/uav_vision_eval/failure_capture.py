"""Pure helpers for bounded, exact-stamp V-SIM diagnostic capture."""

from collections import OrderedDict


CAPTURE_SCHEMA_VERSION = 1


def stamp_key(message):
    """Return an exact ROS stamp key without floating-point conversion."""
    stamp = message.header.stamp
    return int(stamp.secs), int(stamp.nsecs)


def stamp_dict(message):
    secs, nsecs = stamp_key(message)
    return {"secs": secs, "nsecs": nsecs}


def validate_capture_config(enabled, trial_selector, max_frames, output_dir):
    """Reject configurations that could silently affect a formal run."""
    if not enabled:
        return
    if not str(trial_selector).strip():
        raise ValueError(
            "failure capture requires a non-empty diagnostic trial_selector")
    if int(max_frames) <= 0:
        raise ValueError("failure capture max_frames must be positive")
    if not str(output_dir).strip():
        raise ValueError("failure capture output_dir must be non-empty")


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
                       image_filename, capture_index):
    if stamp_key(image) != stamp_key(truth_message):
        raise ValueError("image and truth header stamps differ")
    if (int(camera_info.width) != int(image.width) or
            int(camera_info.height) != int(image.height)):
        raise ValueError("CameraInfo dimensions do not match image")
    if len(camera_info.K) != 9 or len(camera_info.P) != 12:
        raise ValueError("CameraInfo projection matrices are incomplete")
    if target.class_name != trial["class_name"]:
        raise ValueError("truth class does not match active trial")
    roi = target.roi
    return {
        "schema_version": CAPTURE_SCHEMA_VERSION,
        "dataset_kind": "sim-small-target",
        "capture_index": int(capture_index),
        "image_file": str(image_filename),
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
        "camera_info": camera_info_dict(camera_info),
    }
