"""Pure accounting and validation helpers for the V-SIM-04 camera soak."""

import math


REQUIRED_SOAK_ARTIFACTS = (
    "manifest.json",
    "frames.csv",
    "events.csv",
    "summary.json",
    "report.md",
    "vision_search_performance.csv",
)


def finite_positive(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("{} must be a finite positive number".format(name))
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError("{} must be a finite positive number".format(name))
    return number


def validate_soak_config(config):
    """Validate the route and fail-closed monitoring thresholds."""
    result = dict(config)
    positive = (
        "duration_sec", "route_period_sec", "route_update_rate_hz",
        "health_window_sec", "heartbeat_timeout_sec",
        "startup_timeout_sec", "bucket_settle_sec", "max_source_lag_sec",
        "min_input_fps", "min_complete_mapped_fps",
        "service_call_timeout_sec", "node_check_timeout_sec",
        "max_camera_pose_age_sec", "max_camera_pose_error_m",
        "max_camera_orientation_drift_rad",
    )
    for name in positive:
        result[name] = finite_positive(result.get(name), name)
    for name in ("route_center_x_m", "route_center_y_m",
                 "route_radius_x_m", "route_radius_y_m", "route_height_m"):
        try:
            result[name] = float(result.get(name))
        except (TypeError, ValueError, OverflowError):
            raise ValueError("{} must be finite".format(name))
        if not math.isfinite(result[name]):
            raise ValueError("{} must be finite".format(name))
    if result["route_radius_x_m"] <= 0.0 or result["route_radius_y_m"] <= 0.0:
        raise ValueError("route radii must be positive")
    if result["route_height_m"] <= 0.0 or result["route_height_m"] > 4.0:
        raise ValueError("route_height_m must be in (0, 4]")
    arena_limit = finite_positive(result.get("arena_limit_m"), "arena_limit_m")
    result["arena_limit_m"] = arena_limit
    if (abs(result["route_center_x_m"]) + result["route_radius_x_m"] >
            arena_limit or
            abs(result["route_center_y_m"]) + result["route_radius_y_m"] >
            arena_limit):
        raise ValueError("soak route exceeds arena_limit_m")
    try:
        result["bad_windows_to_fail"] = int(result.get("bad_windows_to_fail"))
        result["min_partial_samples"] = int(result.get("min_partial_samples"))
    except (TypeError, ValueError, OverflowError):
        raise ValueError("window counts must be integers")
    if result["bad_windows_to_fail"] < 1 or result["min_partial_samples"] < 1:
        raise ValueError("window counts must be positive")
    partial_ratio = float(result.get("max_partial_only_ratio"))
    if not math.isfinite(partial_ratio) or not 0.0 <= partial_ratio <= 1.0:
        raise ValueError("max_partial_only_ratio must be in [0, 1]")
    result["max_partial_only_ratio"] = partial_ratio
    return result


def route_pose(elapsed_sec, config):
    """Return a deterministic elliptical route and its unique loop id."""
    elapsed = max(0.0, float(elapsed_sec))
    period = float(config["route_period_sec"])
    loop_index = int(math.floor(elapsed / period)) + 1
    phase = 2.0 * math.pi * ((elapsed % period) / period)
    return {
        "trial_id": "soak_loop_{:04d}".format(loop_index),
        "loop_index": loop_index,
        "x": (float(config["route_center_x_m"]) +
              float(config["route_radius_x_m"]) * math.cos(phase)),
        "y": (float(config["route_center_y_m"]) +
              float(config["route_radius_y_m"]) * math.sin(phase)),
        "z": float(config["route_height_m"]),
    }


def camera_pose_tracking_errors(actual_pose, commanded_pose, now_monotonic,
                                max_age_sec, max_error_m):
    """Validate a stamped actual camera pose against the current command."""
    if not actual_pose:
        return ["camera_pose_missing"], None, None
    try:
        age = float(now_monotonic) - float(
            actual_pose["receipt_monotonic"])
        deltas = [
            float(actual_pose[axis]) - float(commanded_pose[axis])
            for axis in ("x", "y", "z")
        ]
        error = math.sqrt(sum(delta * delta for delta in deltas))
    except (KeyError, TypeError, ValueError, OverflowError):
        return ["camera_pose_invalid"], None, None
    if not math.isfinite(age) or not math.isfinite(error) or age < 0.0:
        return ["camera_pose_invalid"], age, error
    errors = []
    if age > float(max_age_sec):
        errors.append("camera_pose_stale")
    if error > float(max_error_m):
        errors.append("camera_pose_tracking_error")
    return errors, age, error


def camera_orientation_drift_errors(actual_quaternion,
                                    baseline_quaternion,
                                    max_drift_rad):
    """Measure sign-invariant quaternion drift from the admitted baseline."""
    try:
        actual = [float(value) for value in actual_quaternion]
        baseline = [float(value) for value in baseline_quaternion]
    except (TypeError, ValueError, OverflowError):
        return ["camera_orientation_invalid"], None
    if (len(actual) != 4 or len(baseline) != 4 or
            not all(math.isfinite(value) for value in actual + baseline)):
        return ["camera_orientation_invalid"], None
    actual_norm = math.sqrt(sum(value * value for value in actual))
    baseline_norm = math.sqrt(sum(value * value for value in baseline))
    if actual_norm <= 1.0e-9 or baseline_norm <= 1.0e-9:
        return ["camera_orientation_invalid"], None
    dot = abs(sum(a * b for a, b in zip(actual, baseline)) /
              (actual_norm * baseline_norm))
    dot = min(1.0, max(-1.0, dot))
    drift = 2.0 * math.acos(dot)
    errors = []
    if drift > float(max_drift_rad):
        errors.append("camera_orientation_drift")
    return errors, drift


def truth_catalog_errors(scenario_id, target_records,
                         expected_scenario_id, expected_target_ids):
    """Require the exact configured target set and a valid pose for each."""
    records = [dict(record) for record in target_records]
    identifiers = [str(record.get("target_id", "")).strip()
                   for record in records]
    actual = {value for value in identifiers if value}
    expected = {str(value).strip() for value in expected_target_ids
                if str(value).strip()}
    errors = []
    if str(scenario_id).strip() != str(expected_scenario_id).strip():
        errors.append("truth_scenario_mismatch")
    duplicates = sorted({value for value in identifiers
                         if value and identifiers.count(value) > 1})
    missing = sorted(expected - actual)
    unexpected = sorted(actual - expected)
    invalid = sorted(
        str(record.get("target_id", "")).strip()
        for record in records
        if str(record.get("target_id", "")).strip() in expected and
        not bool(record.get("pose_valid", False)))
    if duplicates:
        errors.append("truth_targets_duplicate:" + ",".join(duplicates))
    if missing:
        errors.append("truth_targets_missing:" + ",".join(missing))
    if unexpected:
        errors.append("truth_targets_unexpected:" + ",".join(unexpected))
    if invalid:
        errors.append("truth_pose_invalid:" + ",".join(invalid))
    return errors


def measurement_presence_errors(required_streams, counts,
                                truth_valid_messages,
                                actual_camera_pose_samples,
                                truth_projection_valid_messages=0,
                                truth_fully_in_frame_messages=0):
    """Prove required evidence arrived after the measured epoch began."""
    errors = [
        "measurement_stream_missing:" + str(name)
        for name in required_streams
        if int(counts.get(name, 0)) <= 0
    ]
    if int(truth_valid_messages) <= 0:
        errors.append("measurement_truth_valid_missing")
    if int(actual_camera_pose_samples) <= 0:
        errors.append("measurement_actual_camera_pose_missing")
    if int(truth_projection_valid_messages) <= 0:
        errors.append("measurement_truth_projection_missing")
    if int(truth_fully_in_frame_messages) <= 0:
        errors.append("measurement_truth_fully_in_frame_missing")
    return errors


def selected_candidate_errors(record, now_source_sec, allowed_classes,
                              confirm_frames, selected_max_age_sec):
    """Validate one actually published selected candidate."""
    errors = []
    class_name = str(record.get("class_name", "")).strip()
    if class_name not in set(allowed_classes):
        errors.append("selected_class_disallowed:" + class_name)
    if class_name == "tank":
        errors.append("tank_selected")
    try:
        age = float(now_source_sec) - float(record.get("last_seen_sec"))
    except (TypeError, ValueError, OverflowError):
        age = float("inf")
    if (not math.isfinite(age) or age < 0.0 or
            age > float(selected_max_age_sec)):
        errors.append("selected_stale")
    if int(record.get("state", -1)) != 2:
        errors.append("selected_not_confirmed")
    if int(record.get("consecutive_observe_count", 0)) < int(confirm_frames):
        errors.append("selected_consecutive_frames_insufficient")
    if not bool(record.get("map_valid", False)):
        errors.append("selected_map_invalid")
    if not bool(record.get("association_valid", False)):
        errors.append("selected_association_invalid")
    if str(record.get("reject_reason", "")).strip():
        errors.append("selected_has_reject_reason")
    return errors


def camera_info_snapshot(message):
    """Return the immutable CameraInfo fields or reject an invalid profile."""
    try:
        frame_id = str(message.header.frame_id).strip()
        width = int(message.width)
        height = int(message.height)
        distortion_model = str(message.distortion_model).strip()
        k_values = [float(value) for value in message.K]
        d_values = [float(value) for value in message.D]
        r_values = [float(value) for value in message.R]
        p_values = [float(value) for value in message.P]
    except (AttributeError, TypeError, ValueError, OverflowError):
        raise ValueError("CameraInfo profile is invalid")
    values = k_values + d_values + r_values + p_values
    if (width <= 0 or height <= 0 or not frame_id or not distortion_model or
            len(k_values) != 9 or len(r_values) != 9 or
            len(p_values) != 12 or not all(math.isfinite(v) for v in values) or
            k_values[0] <= 0.0 or k_values[4] <= 0.0):
        raise ValueError("CameraInfo profile is invalid")
    return {
        "frame_id": frame_id,
        "width": width,
        "height": height,
        "distortion_model": distortion_model,
        "K": k_values,
        "D": d_values,
        "R": r_values,
        "P": p_values,
    }


class SoakAccounting:
    """Thread-compatible cumulative accounting with rolling trend checks."""

    def __init__(self, config, required_streams):
        self.config = validate_soak_config(config)
        self.required_streams = tuple(sorted(set(required_streams)))
        self.started_monotonic = None
        self.last_source_stamp = {}
        self.source_reorder_counts = {
            name: 0 for name in self.required_streams}
        self.last_receipt = {}
        self.max_heartbeat_gap = {name: 0.0 for name in self.required_streams}
        self.counts = {name: 0 for name in self.required_streams}
        self.errors = []
        self._error_set = set()
        self._mapped_buckets = {}
        self.complete_mapped_frames = 0
        self.partial_only_mapped_frames = 0
        self._window_started = None
        self._window_counts = None
        self._bad_streaks = {
            "input_throughput": 0,
            "complete_mapped_throughput": 0,
            "partial_only_trend": 0,
            "source_backlog": 0,
        }
        self._window_max_source_lag = 0.0
        self.windows = []

    def add_error(self, reason):
        reason = str(reason)
        if reason and reason not in self._error_set:
            self._error_set.add(reason)
            self.errors.append(reason)

    def start(self, monotonic_sec):
        self.started_monotonic = float(monotonic_sec)
        self._window_started = float(monotonic_sec)
        self._window_counts = self._count_snapshot()

    def begin_measurement(self, monotonic_sec):
        """Clear warm-up measurements while preserving fresh stream baselines.

        Startup admission deliberately receives messages before the measured
        interval.  Latest receipt times remain useful for detecting the first
        post-start heartbeat gap, but source stamps and all measurement
        counters must restart at the measured epoch because delayed pre-reset
        derived messages are filtered by the runner.
        """
        self.max_heartbeat_gap = {
            name: 0.0 for name in self.required_streams}
        self.counts = {name: 0 for name in self.required_streams}
        self.last_source_stamp = {}
        self.source_reorder_counts = {
            name: 0 for name in self.required_streams}
        self.errors = []
        self._error_set = set()
        self._mapped_buckets = {}
        self.complete_mapped_frames = 0
        self.partial_only_mapped_frames = 0
        self._bad_streaks = {
            "input_throughput": 0,
            "complete_mapped_throughput": 0,
            "partial_only_trend": 0,
            "source_backlog": 0,
        }
        self._window_max_source_lag = 0.0
        self.windows = []
        self.start(monotonic_sec)

    def _count_snapshot(self):
        return {
            "image": int(self.counts.get("image", 0)),
            "complete": int(self.complete_mapped_frames),
            "partial": int(self.partial_only_mapped_frames),
        }

    def note_stream(self, name, receipt_monotonic, source_stamp=None,
                    source_order_required=True):
        receipt = float(receipt_monotonic)
        previous_receipt = self.last_receipt.get(name)
        if previous_receipt is not None:
            gap = max(0.0, receipt - previous_receipt)
            self.max_heartbeat_gap[name] = max(
                self.max_heartbeat_gap.get(name, 0.0), gap)
            if (self.started_monotonic is not None and
                    previous_receipt >= self.started_monotonic and
                    gap > self.config["heartbeat_timeout_sec"]):
                self.add_error("{}_heartbeat_gap_exceeded".format(name))
        self.last_receipt[name] = receipt
        self.counts[name] = self.counts.get(name, 0) + 1
        if source_stamp is None:
            return
        stamp = float(source_stamp)
        if not math.isfinite(stamp) or stamp < 0.0:
            self.add_error("{}_source_stamp_invalid".format(name))
            return
        previous_stamp = self.last_source_stamp.get(name)
        if previous_stamp is not None and stamp + 1.0e-9 < previous_stamp:
            self.source_reorder_counts[name] = (
                self.source_reorder_counts.get(name, 0) + 1)
            if source_order_required:
                self.add_error("{}_source_time_regressed".format(name))
        self.last_source_stamp[name] = max(
            stamp, previous_stamp if previous_stamp is not None else stamp)

    def note_mapped(self, receipt_monotonic, source_stamp, complete,
                    source_now_sec=None):
        # Fusion branches may finish in a different receipt order.  Image,
        # truth and camera-pose streams still enforce source monotonicity;
        # mapped reorder is counted and bounded by source-lag/backlog checks.
        self.note_stream(
            "mapped", receipt_monotonic, source_stamp,
            source_order_required=False)
        key = round(float(source_stamp), 9)
        bucket = self._mapped_buckets.setdefault(key, {
            "last_receipt": float(receipt_monotonic),
            "complete": False,
        })
        bucket["last_receipt"] = float(receipt_monotonic)
        bucket["complete"] = bucket["complete"] or bool(complete)
        if complete:
            self.note_stream(
                "mapped_complete", receipt_monotonic, source_stamp,
                source_order_required=False)
            if source_now_sec is not None:
                lag = max(0.0, float(source_now_sec) - float(source_stamp))
                self._window_max_source_lag = max(
                    self._window_max_source_lag, lag)

    def _finalize_buckets(self, now_monotonic, force=False):
        cutoff = float(now_monotonic) - self.config["bucket_settle_sec"]
        ready = [key for key, value in self._mapped_buckets.items()
                 if force or value["last_receipt"] <= cutoff]
        for key in ready:
            bucket = self._mapped_buckets.pop(key)
            if bucket["complete"]:
                self.complete_mapped_frames += 1
            else:
                self.partial_only_mapped_frames += 1

    def heartbeat_errors(self, now_monotonic):
        now = float(now_monotonic)
        errors = []
        for name in self.required_streams:
            receipt = self.last_receipt.get(name)
            if receipt is None:
                errors.append("{}_heartbeat_missing".format(name))
            else:
                gap = max(0.0, now - receipt)
                self.max_heartbeat_gap[name] = max(
                    self.max_heartbeat_gap.get(name, 0.0), gap)
                if gap > self.config["heartbeat_timeout_sec"]:
                    errors.append("{}_heartbeat_stale".format(name))
        return errors

    def evaluate(self, now_monotonic, force=False):
        now = float(now_monotonic)
        self._finalize_buckets(now, force=force)
        if self._window_started is None:
            self.start(now)
            return None
        elapsed = now - self._window_started
        if not force and elapsed < self.config["health_window_sec"]:
            return None
        if elapsed <= 0.0:
            return None
        current = self._count_snapshot()
        previous = self._window_counts
        image_delta = current["image"] - previous["image"]
        complete_delta = current["complete"] - previous["complete"]
        partial_delta = current["partial"] - previous["partial"]
        mapped_total = complete_delta + partial_delta
        partial_ratio = (
            partial_delta / float(mapped_total) if mapped_total else 0.0)
        window = {
            "elapsed_sec": elapsed,
            "input_fps": image_delta / elapsed,
            "complete_mapped_fps": complete_delta / elapsed,
            "complete_mapped_frames": complete_delta,
            "partial_only_mapped_frames": partial_delta,
            "partial_only_ratio": partial_ratio,
            "max_source_lag_sec": self._window_max_source_lag,
        }
        bad = {
            "input_throughput": (
                window["input_fps"] < self.config["min_input_fps"]),
            "complete_mapped_throughput": (
                window["complete_mapped_fps"] <
                self.config["min_complete_mapped_fps"]),
            "partial_only_trend": (
                mapped_total >= self.config["min_partial_samples"] and
                partial_ratio > self.config["max_partial_only_ratio"]),
            "source_backlog": (
                window["max_source_lag_sec"] >
                self.config["max_source_lag_sec"]),
        }
        for name, failed in bad.items():
            self._bad_streaks[name] = (
                self._bad_streaks[name] + 1 if failed else 0)
            if self._bad_streaks[name] >= self.config["bad_windows_to_fail"]:
                self.add_error("{}_sustained".format(name))
        window["bad_streaks"] = dict(self._bad_streaks)
        self.windows.append(window)
        self._window_started = now
        self._window_counts = current
        self._window_max_source_lag = 0.0
        return window

    def final_summary(self, requested_duration_sec, actual_wall_duration_sec,
                      actual_source_duration_sec):
        requested = float(requested_duration_sec)
        actual_wall = float(actual_wall_duration_sec)
        actual_source = float(actual_source_duration_sec)
        if actual_wall + 1.0e-3 < requested:
            self.add_error("soak_duration_incomplete")
        elapsed = max(actual_wall, 1.0e-9)
        return {
            "status": "FAIL" if self.errors else "SOAK_MEASURED",
            "qualification_status": (
                "SOAK_600S_MEASURED" if requested >= 600.0 and not self.errors
                else "SMOKE_ONLY" if not self.errors else "FAIL"),
            "soak_600s_pass": bool(requested >= 600.0 and not self.errors),
            "requested_duration_sec": requested,
            "actual_duration_sec": actual_wall,
            "actual_wall_duration_sec": actual_wall,
            "actual_source_duration_sec": actual_source,
            "errors": list(self.errors),
            "counts": dict(self.counts),
            "complete_mapped_frames": self.complete_mapped_frames,
            "partial_only_mapped_frames": self.partial_only_mapped_frames,
            "input_fps": self.counts.get("image", 0) / elapsed,
            "complete_mapped_fps": self.complete_mapped_frames / elapsed,
            "max_heartbeat_gap_sec": dict(self.max_heartbeat_gap),
            "source_reorder_counts": dict(self.source_reorder_counts),
            "health_windows": list(self.windows),
            "p_interrupt": None,
        }
