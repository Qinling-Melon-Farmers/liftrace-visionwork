"""Deterministic V-SIM-04 D50 trajectory and association contracts.

The D surface is intentionally kept independent from the formal 23-trial Gate
and from the C25 lateral surface.  It provides a pure-Python source of
per-sample Gazebo poses so the online runner can consume the same trajectory
contract later, without reinterpreting aircraft ZYX yaw as target image angle.
"""

import csv
import copy
import json
import math
import os

import yaml

from uav_vision.target_selection_policy import resolve_class_profile


D50_MATRIX_KIND = "vsim04_d50_trajectory_association"
D50_YAW_DEG = (0.0, 45.0, 90.0, 135.0)
D50_MOTION_PROFILES = ("constant", "accel_decel", "turn")
D50_FRAMING = ("center", "quadrant", "edge", "partial")
D50_ASSOCIATION_REQUIRED_FIELDS = (
    "frame_seq", "truth_target_id", "associated_truth_target_id",
    "stable_id", "visible", "confirmed", "selected",
)


def _finite(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("{} must be finite".format(name)) from error
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def _vector(values, name):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("{} must contain three values".format(name))
    return tuple(_finite(value, name) for value in values)


def _dot(left, right):
    return sum(a * b for a, b in zip(left, right))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _norm(values):
    return math.sqrt(_dot(values, values))


def _normalize(values, name):
    magnitude = _norm(values)
    if not math.isfinite(magnitude) or magnitude <= 1.0e-12:
        raise ValueError("{} has zero magnitude".format(name))
    return tuple(value / magnitude for value in values)


def _scale(values, factor):
    return tuple(value * factor for value in values)


def _add(*vectors):
    return tuple(sum(values) for values in zip(*vectors))


def _subtract(left, right):
    return tuple(a - b for a, b in zip(left, right))


def _quaternion_from_rotation_columns(column_x, column_y, column_z):
    """Return xyzw quaternion for a finite right-handed rotation matrix."""
    matrix = (
        (column_x[0], column_y[0], column_z[0]),
        (column_x[1], column_y[1], column_z[1]),
        (column_x[2], column_y[2], column_z[2]),
    )
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w_value = 0.25 * scale
        x_value = (matrix[2][1] - matrix[1][2]) / scale
        y_value = (matrix[0][2] - matrix[2][0]) / scale
        z_value = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(
            1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w_value = (matrix[2][1] - matrix[1][2]) / scale
        x_value = 0.25 * scale
        y_value = (matrix[0][1] + matrix[1][0]) / scale
        z_value = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(
            1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w_value = (matrix[0][2] - matrix[2][0]) / scale
        x_value = (matrix[0][1] + matrix[1][0]) / scale
        y_value = 0.25 * scale
        z_value = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w_value = (matrix[1][0] - matrix[0][1]) / scale
        x_value = (matrix[0][2] + matrix[2][0]) / scale
        y_value = (matrix[1][2] + matrix[2][1]) / scale
        z_value = 0.25 * scale
    quaternion = _normalize(
        (x_value, y_value, z_value, w_value), "camera quaternion")
    return quaternion


def _rotate_by_quaternion(vector, quaternion):
    x_value, y_value, z_value, w_value = quaternion
    q_vector = (x_value, y_value, z_value)
    first = _scale(_cross(q_vector, vector), 2.0)
    return _add(vector, _scale(first, w_value), _cross(q_vector, first))


def camera_basis_for_relative_angle(relative_angle_deg,
                                    optical_axis_world=(0.0, 0.0, -1.0),
                                    target_heading_world=(1.0, 0.0, 0.0)):
    """Build image right/down axes around the optical axis.

    ``relative_angle_deg`` is the angle of the projected target heading in the
    image ``right/down`` basis.  This stays correct for a downward camera and
    does not depend on the singular ZYX yaw of that camera pose.
    """
    optical = _normalize(
        _vector(optical_axis_world, "optical_axis_world"),
        "optical_axis_world")
    heading = _vector(target_heading_world, "target_heading_world")
    projected = _subtract(heading, _scale(optical, _dot(heading, optical)))
    projected = _normalize(projected, "projected target heading")
    perpendicular = _normalize(
        _cross(optical, projected), "image-plane perpendicular")
    angle = math.radians(_finite(relative_angle_deg, "relative_angle_deg"))
    right = _add(
        _scale(projected, math.cos(angle)),
        _scale(perpendicular, -math.sin(angle)))
    down = _add(
        _scale(projected, math.sin(angle)),
        _scale(perpendicular, math.cos(angle)))
    right = _normalize(right, "image_right_world")
    down = _normalize(down, "image_down_world")
    if _dot(_cross(right, down), optical) < 1.0 - 1.0e-9:
        raise ValueError("camera basis is not right handed")
    return {
        "optical_axis_world": optical,
        "image_right_world": right,
        "image_down_world": down,
    }


def quaternion_from_camera_basis(basis):
    """Map the Gazebo camera-model axes to an optical image basis.

    The evaluation camera uses model +X as optical, -Y as image-right and -Z
    as image-down.  Rotation columns therefore are ``optical, -right, -down``.
    """
    optical = _vector(basis["optical_axis_world"], "optical_axis_world")
    right = _vector(basis["image_right_world"], "image_right_world")
    down = _vector(basis["image_down_world"], "image_down_world")
    return _quaternion_from_rotation_columns(
        optical, _scale(right, -1.0), _scale(down, -1.0))


def relative_image_angle_deg(target_heading_world, quaternion):
    """Measure target heading with camera optical/image basis vectors."""
    heading = _normalize(
        _vector(target_heading_world, "target_heading_world"),
        "target_heading_world")
    if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4:
        raise ValueError("quaternion must contain four values")
    quaternion = _normalize(
        tuple(_finite(value, "quaternion") for value in quaternion),
        "quaternion")
    right = _rotate_by_quaternion((0.0, -1.0, 0.0), quaternion)
    down = _rotate_by_quaternion((0.0, 0.0, -1.0), quaternion)
    optical = _rotate_by_quaternion((1.0, 0.0, 0.0), quaternion)
    projected = _subtract(heading, _scale(optical, _dot(heading, optical)))
    projected = _normalize(projected, "projected target heading")
    angle = math.degrees(math.atan2(
        _dot(projected, down), _dot(projected, right)))
    return angle % 360.0


def _factor_pairs(rows, first, second):
    return {(row[first], row[second]) for row in rows}


def d50_pairwise_coverage(single_trials):
    """Return required/covered/missing D pairwise factor combinations."""
    factor_domains = {
        "relative_angle_deg": D50_YAW_DEG,
        "motion_profile": D50_MOTION_PROFILES,
        "framing": D50_FRAMING,
    }
    pairs = (
        ("relative_angle_deg", "motion_profile"),
        ("relative_angle_deg", "framing"),
        ("motion_profile", "framing"),
    )
    report = {}
    complete = True
    for first, second in pairs:
        required = {
            (left, right) for left in factor_domains[first]
            for right in factor_domains[second]
        }
        covered = _factor_pairs(single_trials, first, second)
        missing = sorted(required - covered, key=lambda item: str(item))
        key = "{}__{}".format(first, second)
        report[key] = {
            "required_count": len(required),
            "covered_count": len(required & covered),
            "missing": [list(values) for values in missing],
        }
        complete = complete and not missing
    return {"complete": complete, "pairs": report}


def _validate_truth_targets(trial, allowed_classes):
    targets = trial.get("truth_targets", [])
    if not isinstance(targets, list) or not targets:
        raise ValueError("{} truth_targets must be non-empty".format(
            trial["trial_id"]))
    identifiers = []
    for target in targets:
        if not isinstance(target, dict):
            raise ValueError("truth target must be a mapping")
        target_id = str(target.get("target_id", "")).strip()
        class_name = str(target.get("class_name", "")).strip()
        if not target_id or not class_name:
            raise ValueError("truth targets require target_id and class_name")
        if class_name not in allowed_classes and class_name != "landing_h":
            raise ValueError("truth class is not admitted: " + class_name)
        if class_name == "landing_h" and bool(target.get("eligible", False)):
            raise ValueError("landing_h cannot be eligible during search")
        offset = target.get("offset_xy_m", [0.0, 0.0])
        if not isinstance(offset, (list, tuple)) or len(offset) != 2:
            raise ValueError("truth target offset_xy_m must have two values")
        target["offset_xy_m"] = [
            _finite(value, "offset_xy_m") for value in offset]
        target["target_id"] = target_id
        target["class_name"] = class_name
        target["eligible"] = bool(target.get(
            "eligible", class_name != "landing_h"))
        target["priority_weight"] = _finite(
            target.get("priority_weight", 0.0), "priority_weight")
        identifiers.append(target_id)
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("{} has duplicate truth target IDs".format(
            trial["trial_id"]))
    primary = str(trial.get("expected_primary_target_id", "")).strip()
    if primary not in identifiers:
        raise ValueError("{} expected primary is missing".format(
            trial["trial_id"]))
    primary_target = next(
        target for target in targets if target["target_id"] == primary)
    if not primary_target["eligible"]:
        raise ValueError("expected primary target must be eligible")
    if trial.get("kind") == "multi_directed":
        contract = trial.get("association_contract")
        if not isinstance(contract, dict):
            raise ValueError("multi-target trial requires association_contract")
        for field in (
                "max_duplicate_stable_ids", "max_merged_truth_targets",
                "max_wrong_associations", "max_priority_starvation_frames"):
            value = contract.get(field)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError("invalid association contract: " + field)
        for field in (
                "minimum_observation_frames",
                "minimum_truth_coverage_frames_per_target"):
            value = contract.get(field)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError("invalid association contract: " + field)
        if contract.get("observation_coverage_threshold_status") != \
                "UNFROZEN_DIAGNOSTIC_ONLY":
            raise ValueError(
                "multi-target coverage threshold must remain UNFROZEN")
        if contract.get("require_search_target_selection") is not True or \
                contract.get("forbid_landing_h_selection") is not True:
            raise ValueError("D50 SEARCH target/H gates must be fail-closed")


def load_d50_matrix(path):
    with open(path, "r", encoding="utf-8") as stream:
        matrix = yaml.safe_load(stream)
    if not isinstance(matrix, dict):
        raise ValueError("D50 matrix must be a mapping")
    if matrix.get("evaluation_id") != "V-SIM-04":
        raise ValueError("D50 evaluation_id must be V-SIM-04")
    if matrix.get("matrix_kind") != D50_MATRIX_KIND:
        raise ValueError("D50 matrix_kind is invalid")
    profile, allowed = resolve_class_profile(matrix.get("class_profile", ""))
    matrix["class_profile"] = profile
    defaults = matrix.get("defaults", {})
    height = _finite(defaults.get("height_m"), "defaults.height_m")
    speed = _finite(defaults.get("nominal_speed_mps"),
                    "defaults.nominal_speed_mps")
    if height <= 0.0 or speed <= 0.0:
        raise ValueError("D50 height and speed must be positive")
    intrinsics = matrix.get("camera", {}).get("intrinsics", {})
    try:
        image_width = int(intrinsics["width"])
        image_height = int(intrinsics["height"])
        horizontal_fov = _finite(
            intrinsics["horizontal_fov_rad"], "horizontal_fov_rad")
        fx_value = _finite(intrinsics["fx"], "camera fx")
        fy_value = _finite(intrinsics["fy"], "camera fy")
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("D50 camera intrinsics are incomplete") from error
    if (image_width <= 0 or image_height <= 0 or
            not 0.0 < horizontal_fov < math.pi):
        raise ValueError("D50 camera dimensions/HFOV are invalid")
    expected_focal = image_width / (2.0 * math.tan(horizontal_fov / 2.0))
    if (abs(fx_value - expected_focal) > 1.0e-6 or
            abs(fy_value - expected_focal) > 1.0e-6):
        raise ValueError(
            "D50 fx/fy must match the Gazebo horizontal_fov contract")
    anchors = matrix.get("target_anchors", {})
    for class_name in allowed:
        if class_name in anchors:
            anchors[class_name]["xyz"] = list(_vector(
                anchors[class_name]["xyz"], "target anchor"))
            size = anchors[class_name].get("size_m", [])
            if not isinstance(size, (list, tuple)) or len(size) != 2:
                raise ValueError("D50 target size_m must contain two values")
            size = [_finite(value, "target size_m") for value in size]
            if any(value <= 0.0 for value in size):
                raise ValueError("D50 target size_m must be positive")
            anchors[class_name]["size_m"] = size
    single_section = matrix.get("single_target_pairwise", {})
    multi_section = matrix.get("multi_target_directed", {})
    single_rows = single_section.get("rows", [])
    multi_rows = multi_section.get("rows", [])
    if len(single_rows) != 40 or len(multi_rows) != 10:
        raise ValueError("D50 requires 40 single and 10 multi-target trials")
    trials = []
    for kind, rows in (("single_pairwise", single_rows),
                       ("multi_directed", multi_rows)):
        for source in rows:
            if not isinstance(source, dict):
                raise ValueError("D50 trial rows must be mappings")
            trial = dict(source)
            trial["kind"] = kind
            trial["trial_id"] = str(trial.get("trial_id", "")).strip()
            trial["class_name"] = str(trial.get("class_name", "")).strip()
            trial["height_m"] = _finite(
                trial.get("height_m", height), "height_m")
            trial["speed_mps"] = _finite(
                trial.get("speed_mps", speed), "speed_mps")
            trial["relative_angle_deg"] = _finite(
                trial.get("relative_angle_deg"), "relative_angle_deg")
            trial["motion_profile"] = str(
                trial.get("motion_profile", "")).strip()
            trial["framing"] = str(trial.get("framing", "")).strip()
            trial["mission_phase"] = str(trial.get(
                "mission_phase",
                multi_section.get("mission_phase", "search")
                if kind == "multi_directed" else "search")).strip().lower()
            if not trial["trial_id"]:
                raise ValueError("D50 trial_id must be non-empty")
            if trial["class_name"] not in allowed:
                raise ValueError("D50 primary class is not admitted")
            if trial["relative_angle_deg"] not in D50_YAW_DEG:
                raise ValueError("D50 relative angle is outside the domain")
            if trial["motion_profile"] not in D50_MOTION_PROFILES:
                raise ValueError("D50 motion profile is outside the domain")
            if trial["framing"] not in D50_FRAMING:
                raise ValueError("D50 framing is outside the domain")
            if trial["mission_phase"] != "search":
                raise ValueError("D50 currently freezes mission_phase=search")
            if kind == "single_pairwise":
                anchor = anchors.get(trial["class_name"], {})
                target_id = str(anchor.get("target_id", "")).strip()
                if not target_id:
                    raise ValueError("D50 class anchor target_id is missing")
                trial["expected_primary_target_id"] = target_id
                trial["truth_targets"] = [{
                    "target_id": target_id,
                    "class_name": trial["class_name"],
                    "offset_xy_m": [0.0, 0.0],
                    "eligible": True,
                    "priority_weight": _finite(
                        anchor.get("priority_weight", 0.0),
                        "anchor priority_weight"),
                }]
            _validate_truth_targets(trial, set(allowed))
            trials.append(trial)
    identifiers = [trial["trial_id"] for trial in trials]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("D50 trial IDs must be unique")
    expected = int(matrix.get("expected_trial_count", 50))
    if expected != 50 or len(trials) != expected:
        raise ValueError("D50 expected_trial_count must be 50")
    coverage = d50_pairwise_coverage(trials[:40])
    if not coverage["complete"]:
        raise ValueError("D50 single-target pairwise coverage is incomplete")
    matrix["trials"] = trials
    matrix["pairwise_coverage"] = coverage
    return matrix


def load_d50_runtime_matrix(path):
    """Adapt D50 to the established V-SIM-04 recorder/runner contract.

    Runtime trials remain diagnostic and reuse the existing dynamic telemetry
    path.  ``design_kind`` preserves the D split while ``kind=dynamic`` lets
    the recorder apply its already-tested motion and visibility assertions.
    """
    matrix = copy.deepcopy(load_d50_matrix(path))
    for trial in matrix["trials"]:
        trial["design_kind"] = trial["kind"]
        trial["kind"] = "dynamic"
        # The established recorder's eligibility window requires the complete
        # target in frame. D edge/partial need a different, not-yet-wired
        # center-in-frame window and must not masquerade as C25 partial.
        trial["visibility_profile"] = "full"
        if trial["design_kind"] == "multi_directed":
            not_run_reason = "multi_target_spawner_truth_not_wired"
            visibility_preflight = None
        elif trial["framing"] in {"edge", "partial"}:
            not_run_reason = "clipped_target_observation_window_not_wired"
            visibility_preflight = None
        else:
            trajectory = generate_d50_pose_samples(matrix, trial)
            outside = any(
                abs(float(sample["position_x_m"])) > 4.8 or
                abs(float(sample["position_y_m"])) > 4.8
                for sample in trajectory["samples"])
            if outside:
                not_run_reason = "camera_trajectory_exceeds_arena_limit"
                visibility_preflight = None
            else:
                visibility_preflight = d50_visibility_preflight(
                    matrix, trial, trajectory)
                not_run_reason = (
                    "fully_in_frame_enter_leave_preflight_failed"
                    if not visibility_preflight["preflight_pass"] else "")
        trial["d50_runtime_status"] = (
            "READY_FOR_SINGLE_SMOKE" if not not_run_reason else "NOT_RUN")
        trial["d50_not_run_reason"] = not_run_reason
        trial["d50_visibility_preflight"] = visibility_preflight
    trial_ids = [trial["trial_id"] for trial in matrix["trials"]]
    smoke_ids = [
        trial["trial_id"] for trial in matrix["trials"]
        if trial["d50_runtime_status"] == "READY_FOR_SINGLE_SMOKE"]
    matrix["trial_slices"] = {
        "single40": trial_ids[:40],
        "single_smoke_supported": smoke_ids,
        "multi10": trial_ids[40:],
        "d50": trial_ids,
    }
    matrix["formal_expected_trial_count"] = 50
    matrix["diagnostic_only"] = True
    matrix["design_id"] = "vsim04-d50-trajectory-association"
    matrix["performance_contract"] = {
        "contract_id": "vsim04-d50-diagnostic",
        "sources": [
            "config/vsim04_trajectory_d50_matrix.yaml",
            "docs/VSIM04_D50_TRAJECTORY_ASSOCIATION.md",
        ],
        "thresholds": {},
        "unfrozen_thresholds": [
            "min_p_confirm", "min_p_selected",
            "max_duplicate_stable_ids", "max_merged_truth_targets",
            "max_wrong_associations", "max_priority_starvation_frames",
        ],
    }
    matrix["runner"] = {
        "camera_model": "vision_eval_camera",
        "offscreen_offset_m": 3.5,
        "pretrial_settle_sec": 0.5,
        "posttrial_settle_sec": 1.2,
        "reset_service": "/uav_vision/reset_memory",
        "trial_event_topic": "/uav_vision_eval/vsim04/trial_event",
        # Used only by the base runner while parked offscreen. D samples carry
        # their own full quaternion and never derive target angle from ZYX yaw.
        "camera_rpy": [0.0, math.pi / 2.0, 0.0],
    }
    matrix["dynamic"] = {
        "path_half_length_m": matrix["motion"]["path_length_m"] / 2.0,
        "update_rate_hz": matrix["motion"]["sample_rate_hz"],
    }
    return matrix


def _primary_anchor(matrix, trial):
    target = next(
        item for item in trial["truth_targets"]
        if item["target_id"] == trial["expected_primary_target_id"])
    values = matrix["target_anchors"][target["class_name"]]["xyz"]
    return (
        float(values[0]) + target["offset_xy_m"][0],
        float(values[1]) + target["offset_xy_m"][1],
        float(values[2]),
    )


def _nominal_camera_position(matrix, trial, basis):
    anchor = _primary_anchor(matrix, trial)
    camera = matrix["camera"]
    intrinsics = camera["intrinsics"]
    offsets = camera["framing_offsets_half_frame"]
    u_norm, v_norm = offsets[trial["framing"]]
    height = trial["height_m"]
    u_metric = (
        _finite(u_norm, "framing u") * float(intrinsics["width"]) * 0.5 /
        float(intrinsics["fx"]) * height)
    v_metric = (
        _finite(v_norm, "framing v") * float(intrinsics["height"]) * 0.5 /
        float(intrinsics["fy"]) * height)
    ray = _add(
        _scale(basis["optical_axis_world"], height),
        _scale(basis["image_right_world"], u_metric),
        _scale(basis["image_down_world"], v_metric))
    return _subtract(anchor, ray)


def _horizontal_unit(vector, name):
    return _normalize((vector[0], vector[1], 0.0), name)


def generate_d50_pose_samples(matrix, trial):
    """Generate deterministic per-source-time Gazebo poses for one D trial."""
    motion = matrix["motion"]
    sample_rate = _finite(motion.get("sample_rate_hz", 20.0),
                          "sample_rate_hz")
    path_length = _finite(motion.get("path_length_m", 4.0), "path_length_m")
    speed = trial["speed_mps"]
    if sample_rate <= 0.0 or path_length <= 0.0 or speed <= 0.0:
        raise ValueError("D50 motion values must be positive")
    duration = path_length / speed
    steps = max(2, int(math.ceil(duration * sample_rate)))
    target_heading = tuple(matrix["camera"].get(
        "target_heading_world", [1.0, 0.0, 0.0]))
    optical = tuple(matrix["camera"].get(
        "optical_axis_world", [0.0, 0.0, -1.0]))
    midpoint_basis = camera_basis_for_relative_angle(
        trial["relative_angle_deg"], optical, target_heading)
    nominal = _nominal_camera_position(matrix, trial, midpoint_basis)
    tangent = _horizontal_unit(
        midpoint_basis["image_right_world"], "trajectory tangent")
    normal = (-tangent[1], tangent[0], 0.0)
    turn_total = math.radians(_finite(
        motion.get("turn_heading_change_deg", 90.0),
        "turn_heading_change_deg"))
    if turn_total <= 0.0 or turn_total >= math.pi:
        raise ValueError("turn_heading_change_deg must be in (0, 180)")
    digits = [int(value) for value in trial["trial_id"] if value.isdigit()]
    turn_direction = float(trial.get(
        "turn_direction", 1.0 if sum(digits) % 2 else -1.0))
    turn_direction = 1.0 if turn_direction >= 0.0 else -1.0
    samples = []
    for index in range(steps + 1):
        u_value = index / float(steps)
        if trial["motion_profile"] == "accel_decel":
            progress = 3.0 * u_value ** 2 - 2.0 * u_value ** 3
        else:
            progress = u_value
        relative_angle = trial["relative_angle_deg"]
        if trial["motion_profile"] == "turn":
            phi = (u_value - 0.5) * turn_total * turn_direction
            radius = path_length / turn_total
            position = _add(
                nominal,
                _scale(tangent, radius * math.sin(phi)),
                _scale(normal, turn_direction * radius * (1.0 - math.cos(phi))))
            relative_angle += math.degrees(phi)
        else:
            position = _add(
                nominal, _scale(tangent, path_length * (progress - 0.5)))
        basis = camera_basis_for_relative_angle(
            relative_angle, optical, target_heading)
        quaternion = quaternion_from_camera_basis(basis)
        measured_angle = relative_image_angle_deg(target_heading, quaternion)
        samples.append({
            "trial_id": trial["trial_id"],
            "sample_index": index,
            "source_time_offset_sec": duration * u_value,
            "position_x_m": position[0],
            "position_y_m": position[1],
            "position_z_m": position[2],
            "orientation_x": quaternion[0],
            "orientation_y": quaternion[1],
            "orientation_z": quaternion[2],
            "orientation_w": quaternion[3],
            "relative_image_angle_deg": measured_angle,
            "optical_axis_x": basis["optical_axis_world"][0],
            "optical_axis_y": basis["optical_axis_world"][1],
            "optical_axis_z": basis["optical_axis_world"][2],
            "image_right_x": basis["image_right_world"][0],
            "image_right_y": basis["image_right_world"][1],
            "image_right_z": basis["image_right_world"][2],
            "image_down_x": basis["image_down_world"][0],
            "image_down_y": basis["image_down_world"][1],
            "image_down_z": basis["image_down_world"][2],
        })
    for index, sample in enumerate(samples):
        if index == 0:
            sample["linear_speed_mps"] = 0.0
            sample["optical_twist_rate_radps"] = 0.0
            continue
        previous = samples[index - 1]
        delta_time = (sample["source_time_offset_sec"] -
                      previous["source_time_offset_sec"])
        displacement = math.sqrt(sum(
            (sample[field] - previous[field]) ** 2 for field in
            ("position_x_m", "position_y_m", "position_z_m")))
        angle_delta = math.radians(
            sample["relative_image_angle_deg"] -
            previous["relative_image_angle_deg"])
        angle_delta = math.atan2(math.sin(angle_delta), math.cos(angle_delta))
        sample["linear_speed_mps"] = displacement / delta_time
        sample["optical_twist_rate_radps"] = angle_delta / delta_time
    distance = sum(
        samples[index]["linear_speed_mps"] *
        (samples[index]["source_time_offset_sec"] -
         samples[index - 1]["source_time_offset_sec"])
        for index in range(1, len(samples)))
    return {
        "trial_id": trial["trial_id"],
        "motion_profile": trial["motion_profile"],
        "expected_duration_sec": duration,
        "path_distance_m": distance,
        "mean_speed_mps": distance / duration,
        "sample_rate_hz": sample_rate,
        "samples": samples,
    }


def d50_visibility_preflight(matrix, trial, trajectory=None):
    """Project frozen target corners and require an enter/leave observation.

    This mirrors the existing recorder's fully-in-frame eligibility semantics
    closely enough to reject geometrically impossible D smoke cases before
    Gazebo.  It is a launch preflight, not a detector performance result.
    """
    trajectory = trajectory or generate_d50_pose_samples(matrix, trial)
    anchor = _primary_anchor(matrix, trial)
    target = next(
        item for item in trial["truth_targets"]
        if item["target_id"] == trial["expected_primary_target_id"])
    size_x, size_y = matrix["target_anchors"][
        target["class_name"]]["size_m"]
    intrinsics = matrix["camera"]["intrinsics"]
    width = float(intrinsics["width"])
    height = float(intrinsics["height"])
    fx_value = float(intrinsics["fx"])
    fy_value = float(intrinsics["fy"])
    center_x = width * 0.5
    center_y = height * 0.5
    optical_offset = _finite(matrix["camera"].get(
        "optical_center_offset_m", 0.0), "optical_center_offset_m")
    states = []
    center_states = []
    for sample in trajectory["samples"]:
        optical = (
            sample["optical_axis_x"], sample["optical_axis_y"],
            sample["optical_axis_z"])
        right = (
            sample["image_right_x"], sample["image_right_y"],
            sample["image_right_z"])
        down = (
            sample["image_down_x"], sample["image_down_y"],
            sample["image_down_z"])
        model_position = (
            sample["position_x_m"], sample["position_y_m"],
            sample["position_z_m"])
        camera_position = _add(
            model_position, _scale(optical, optical_offset))

        def project(point):
            relative = _subtract(point, camera_position)
            depth = _dot(relative, optical)
            if depth <= 1.0e-6:
                return None
            return (
                center_x + fx_value * _dot(relative, right) / depth,
                center_y + fy_value * _dot(relative, down) / depth,
            )

        center_pixel = project(anchor)
        center_states.append(bool(
            center_pixel is not None and
            0.0 <= center_pixel[0] < width and
            0.0 <= center_pixel[1] < height))
        corners = [
            project((anchor[0] + dx, anchor[1] + dy, anchor[2]))
            for dx in (-size_x / 2.0, size_x / 2.0)
            for dy in (-size_y / 2.0, size_y / 2.0)
        ]
        states.append(bool(
            all(pixel is not None for pixel in corners) and
            all(0.0 <= pixel[0] < width and 0.0 <= pixel[1] < height
                for pixel in corners)))
    first_full = next(
        (index for index, value in enumerate(states) if value), None)
    entered = first_full is not None
    left_after_entry = bool(
        entered and any(not value for value in states[first_full + 1:]))
    return {
        "entered_fully_in_frame": entered,
        "left_fully_in_frame_after_entry": left_after_entry,
        "fully_in_frame_sample_count": sum(states),
        "center_in_frame_sample_count": sum(center_states),
        "sample_count": len(states),
        "first_fully_in_frame_sample": first_full,
        "last_fully_in_frame_sample": (
            max(index for index, value in enumerate(states) if value)
            if entered else None),
        "preflight_pass": entered and left_after_entry,
        "semantics": "frozen_corner_projection_not_detector_result",
    }


def audit_multitarget_associations(trial, observations):
    """Evaluate stable-ID association and search-phase target/H gating.

    Observation records carry ground-truth ``truth_target_id`` and the visual
    chain's ``associated_truth_target_id``.  The split is deliberate: it makes
    wrong association distinguishable from duplicate and merge failures.
    """
    truths = {item["target_id"]: item for item in trial["truth_targets"]}
    truth_to_stable = {target_id: set() for target_id in truths}
    stable_to_truth = {}
    wrong = 0
    selected_target_ids = []
    selected_h = 0
    empty_stable_ids = 0
    frames = {}
    truth_frame_coverage = {target_id: set() for target_id in truths}
    for observation_index, observation in enumerate(observations):
        if not isinstance(observation, dict):
            raise ValueError("association observation must be a mapping")
        missing = [field for field in D50_ASSOCIATION_REQUIRED_FIELDS
                   if field not in observation]
        if missing:
            raise ValueError(
                "association observation {} missing fields: {}".format(
                    observation_index, ",".join(missing)))
        frame_value = observation["frame_seq"]
        if isinstance(frame_value, bool):
            raise ValueError("association frame_seq must be a positive integer")
        try:
            frame_seq = int(frame_value)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "association frame_seq must be a positive integer") from error
        if frame_seq <= 0 or (isinstance(frame_value, float) and
                              not frame_value.is_integer()):
            raise ValueError("association frame_seq must be a positive integer")
        for field in ("visible", "confirmed", "selected"):
            if not isinstance(observation[field], bool):
                raise ValueError(
                    "association {} must be boolean".format(field))
        actual = str(observation["truth_target_id"]).strip()
        associated = str(
            observation["associated_truth_target_id"]).strip()
        stable_id = str(observation["stable_id"]).strip()
        if actual not in truths:
            raise ValueError("observation references unknown truth target")
        truth_frame_coverage[actual].add(frame_seq)
        if associated not in truths:
            wrong += 1
        elif associated != actual:
            wrong += 1
        if stable_id:
            truth_to_stable[actual].add(stable_id)
            stable_to_truth.setdefault(stable_id, set()).add(actual)
        else:
            empty_stable_ids += 1
        selected = observation["selected"]
        confirmed = observation["confirmed"]
        visible = observation["visible"]
        frame = frames.setdefault(frame_seq, {
            "eligible_confirmed": set(), "selected": set()})
        if visible and confirmed and truths[actual]["eligible"]:
            frame["eligible_confirmed"].add(actual)
        if selected:
            selected_target_ids.append(associated)
            frame["selected"].add(associated)
            selected_truth = truths.get(associated)
            if selected_truth is not None and (
                    selected_truth["class_name"] == "landing_h" or
                    not selected_truth["eligible"]):
                selected_h += 1
    duplicate_excess = sum(
        max(0, len(stable_ids) - 1)
        for stable_ids in truth_to_stable.values())
    merge_excess = sum(
        max(0, len(truth_ids) - 1)
        for truth_ids in stable_to_truth.values())
    max_starvation = 0
    current_starvation = 0
    previous_frame = None
    starvation_frames = 0
    for frame_seq in sorted(frames):
        frame = frames[frame_seq]
        if previous_frame is None or frame_seq != previous_frame + 1:
            current_starvation = 0
        previous_frame = frame_seq
        eligible = frame["eligible_confirmed"]
        if eligible:
            max_priority = max(
                truths[target_id]["priority_weight"] for target_id in eligible)
            highest = {
                target_id for target_id in eligible
                if truths[target_id]["priority_weight"] == max_priority}
            starved = not bool(highest & frame["selected"])
        else:
            starved = False
        if starved:
            starvation_frames += 1
            current_starvation += 1
            max_starvation = max(max_starvation, current_starvation)
        else:
            current_starvation = 0
    contract = trial.get("association_contract", {})
    allowed_starvation = int(contract.get(
        "max_priority_starvation_frames", 0))
    selected_eligible = sum(
        1 for target_id in selected_target_ids
        if target_id in truths and truths[target_id]["eligible"])
    violations = []
    if empty_stable_ids:
        violations.append(
            "empty_stable_id:{}>0".format(empty_stable_ids))
    minimum_frames = int(contract.get("minimum_observation_frames", 1))
    if len(frames) < minimum_frames:
        violations.append("insufficient_frame_coverage:{}<{}".format(
            len(frames), minimum_frames))
    minimum_truth_frames = int(contract.get(
        "minimum_truth_coverage_frames_per_target", 1))
    for target_id, covered_frames in sorted(truth_frame_coverage.items()):
        if len(covered_frames) < minimum_truth_frames:
            violations.append(
                "truth_frame_coverage:{}:{}<{}".format(
                    target_id, len(covered_frames), minimum_truth_frames))
    for reason, actual, allowed in (
            ("duplicate_stable_id", duplicate_excess,
             int(contract.get("max_duplicate_stable_ids", 0))),
            ("merged_truth_targets", merge_excess,
             int(contract.get("max_merged_truth_targets", 0))),
            ("wrong_association", wrong,
             int(contract.get("max_wrong_associations", 0))),
            ("priority_starvation", max_starvation, allowed_starvation),
            ("landing_h_selected_in_search", selected_h, 0)):
        if actual > allowed:
            violations.append("{}:{}>{}".format(reason, actual, allowed))
    if bool(contract.get("require_search_target_selection", True)) and not \
            selected_eligible:
        violations.append("search_target_never_selected")
    return {
        "trial_id": trial["trial_id"],
        "duplicate_stable_id_excess": duplicate_excess,
        "merged_truth_target_excess": merge_excess,
        "wrong_association_count": wrong,
        "observation_frame_count": len(frames),
        "empty_stable_id_count": empty_stable_ids,
        "truth_frame_coverage": {
            target_id: len(frame_ids) for target_id, frame_ids in
            sorted(truth_frame_coverage.items())},
        "observation_coverage_threshold_status": contract.get(
            "observation_coverage_threshold_status", "UNSPECIFIED"),
        "priority_starvation_frame_count": starvation_frames,
        "max_priority_starvation_streak_frames": max_starvation,
        "selected_eligible_target_count": selected_eligible,
        "landing_h_selected_count": selected_h,
        "search_target_gate_pass": selected_eligible > 0,
        "search_h_gate_pass": selected_h == 0,
        "passed": not violations,
        "violations": violations,
    }


def _write_csv(path, fieldnames, rows):
    with open(path, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_d50_dry_run(matrix_path, output_dir):
    matrix = load_d50_matrix(matrix_path)
    runtime_matrix = load_d50_runtime_matrix(matrix_path)
    runtime_trials = runtime_matrix["trials"]
    runtime_ready_ids = [
        trial["trial_id"] for trial in runtime_trials
        if trial["d50_runtime_status"] == "READY_FOR_SINGLE_SMOKE"]
    runtime_not_run = {}
    for trial in runtime_trials:
        reason = trial["d50_not_run_reason"]
        if reason:
            runtime_not_run.setdefault(reason, []).append(trial["trial_id"])
    os.makedirs(output_dir, exist_ok=True)
    trajectories = [
        generate_d50_pose_samples(matrix, trial) for trial in matrix["trials"]]
    trial_rows = []
    sample_rows = []
    association_contracts = []
    for trial, trajectory in zip(matrix["trials"], trajectories):
        trial_rows.append({
            "trial_id": trial["trial_id"],
            "kind": trial["kind"],
            "class_name": trial["class_name"],
            "height_m": trial["height_m"],
            "speed_mps": trial["speed_mps"],
            "relative_angle_deg": trial["relative_angle_deg"],
            "motion_profile": trial["motion_profile"],
            "framing": trial["framing"],
            "expected_primary_target_id":
                trial["expected_primary_target_id"],
            "truth_target_count": len(trial["truth_targets"]),
            "sample_count": len(trajectory["samples"]),
            "path_distance_m": trajectory["path_distance_m"],
            "mean_speed_mps": trajectory["mean_speed_mps"],
        })
        sample_rows.extend(trajectory["samples"])
        if trial["kind"] == "multi_directed":
            association_contracts.append({
                "trial_id": trial["trial_id"],
                "expected_primary_target_id":
                    trial["expected_primary_target_id"],
                "truth_targets": trial["truth_targets"],
                "association_contract": trial.get(
                    "association_contract", {}),
            })
    association_document = {
        "schema_version": 1,
        "mission_phase": "search",
        "required_observation_fields": list(
            D50_ASSOCIATION_REQUIRED_FIELDS),
        "metric_definitions": {
            "duplicate_stable_id_excess": (
                "stable IDs beyond one that are associated with one truth "
                "target during a trial"),
            "merged_truth_target_excess": (
                "truth targets beyond one that share one stable ID during a "
                "trial"),
            "wrong_association_count": (
                "observations whose associated truth ID differs from the "
                "source truth ID"),
            "max_priority_starvation_streak_frames": (
                "maximum consecutive frames in which a visible confirmed "
                "highest-weight eligible target is not selected"),
            "search_target_gate_pass": (
                "at least one eligible delivery target is selected"),
            "search_h_gate_pass": (
                "no landing H or other ineligible object is selected"),
        },
        "trials": association_contracts,
    }
    summary = {
        "evaluation_id": "V-SIM-04",
        "matrix_kind": D50_MATRIX_KIND,
        "status": "DRY_RUN",
        "trial_count": len(matrix["trials"]),
        "single_target_pairwise_count": sum(
            trial["kind"] == "single_pairwise"
            for trial in matrix["trials"]),
        "multi_target_directed_count": sum(
            trial["kind"] == "multi_directed"
            for trial in matrix["trials"]),
        "multi_target_with_landing_h_count": sum(
            trial["kind"] == "multi_directed" and any(
                target["class_name"] == "landing_h"
                for target in trial["truth_targets"])
            for trial in matrix["trials"]),
        "association_metrics": sorted(
            association_document["metric_definitions"]),
        "pairwise_coverage": matrix["pairwise_coverage"],
        "relative_angle_counts": {
            str(int(value)): sum(
                trial["relative_angle_deg"] == value
                for trial in matrix["trials"])
            for value in D50_YAW_DEG},
        "motion_profile_counts": {
            value: sum(trial["motion_profile"] == value
                       for trial in matrix["trials"])
            for value in D50_MOTION_PROFILES},
        "framing_counts": {
            value: sum(trial["framing"] == value
                       for trial in matrix["trials"])
            for value in D50_FRAMING},
        "trajectory_mean_speed_min_mps": min(
            trajectory["mean_speed_mps"] for trajectory in trajectories),
        "trajectory_mean_speed_max_mps": max(
            trajectory["mean_speed_mps"] for trajectory in trajectories),
        "runtime_readiness": {
            "supported_single_smoke_trial_ids": runtime_ready_ids,
            "not_run_by_reason": runtime_not_run,
            "meaning": (
                "Readiness for the existing fully-in-frame recorder only; "
                "no listed trial is an algorithm result until Gazebo runs."),
        },
        "gazebo_execution_status": "NOT_RUN",
        "meaning": (
            "Schema/trajectory/association contract validation only; this is "
            "not a Gazebo or algorithm Gate result."),
    }
    manifest = {
        "schema_version": matrix.get("schema_version"),
        "evaluation_id": matrix["evaluation_id"],
        "matrix_kind": matrix["matrix_kind"],
        "seed": matrix["seed"],
        "class_profile": matrix["class_profile"],
        "source_matrix": os.path.abspath(matrix_path),
        "camera": matrix["camera"],
        "motion": matrix["motion"],
        "trials": matrix["trials"],
        "runtime_readiness": summary["runtime_readiness"],
    }
    with open(os.path.join(output_dir, "d50_manifest.json"), "w",
              encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with open(os.path.join(output_dir, "d50_coverage.json"), "w",
              encoding="utf-8") as stream:
        json.dump(matrix["pairwise_coverage"], stream, indent=2,
                  sort_keys=True)
        stream.write("\n")
    with open(os.path.join(output_dir, "d50_association_contracts.json"), "w",
              encoding="utf-8") as stream:
        json.dump(association_document, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with open(os.path.join(output_dir, "summary.json"), "w",
              encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    _write_csv(os.path.join(output_dir, "d50_trials.csv"), [
        "trial_id", "kind", "class_name", "height_m", "speed_mps",
        "relative_angle_deg", "motion_profile", "framing",
        "expected_primary_target_id", "truth_target_count", "sample_count",
        "path_distance_m", "mean_speed_mps",
    ], trial_rows)
    _write_csv(os.path.join(output_dir, "d50_trajectory_samples.csv"), [
        "trial_id", "sample_index", "source_time_offset_sec",
        "position_x_m", "position_y_m", "position_z_m",
        "orientation_x", "orientation_y", "orientation_z", "orientation_w",
        "relative_image_angle_deg", "linear_speed_mps",
        "optical_twist_rate_radps", "optical_axis_x", "optical_axis_y",
        "optical_axis_z", "image_right_x", "image_right_y", "image_right_z",
        "image_down_x", "image_down_y", "image_down_z",
    ], sample_rows)
    return summary
