"""Pure policy and state machine for guarded VCL06 mission start.

This module deliberately has no ROS dependency.  The ROS shell supplies the
latest latched JSON documents and the parsed, durable truth document; the
policy only decides whether invoking the navigation-owned Trigger service is
currently permitted.
"""

from dataclasses import dataclass
import math
import os


SCHEMA_VERSION = 1
R2026_PROFILE = "r2026"
R2026_TARGET_CLASSES = frozenset(
    ("tent", "pillbox", "bridge", "panzer", "red_cross"))
R2026_TARGET_MODELS = frozenset(
    "random_%s" % class_name for class_name in R2026_TARGET_CLASSES)


def _is_document(value):
    return isinstance(value, dict) and not value.get("_decode_error")


def _exact_string_set(value, expected):
    if not isinstance(value, (list, tuple)):
        return False
    if not all(isinstance(item, str) and item for item in value):
        return False
    return len(value) == len(expected) and set(value) == set(expected)


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _safe_int(value):
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _normalized_path(value):
    if not isinstance(value, str) or not value.strip():
        return ""
    return os.path.abspath(os.path.normpath(value))


@dataclass(frozen=True)
class StartGateConfig:
    """Frozen inputs that identify one intended random-field run."""

    expected_seed: int
    expected_truth_path: str
    profile: str = R2026_PROFILE
    nav_feature_profile: str = "baseline"
    retry_initial_sec: float = 0.5
    retry_max_sec: float = 5.0

    def __post_init__(self):
        if self.profile != R2026_PROFILE:
            raise ValueError("mission start gate only permits r2026")
        if (isinstance(self.expected_seed, bool) or
                not isinstance(self.expected_seed, int) or
                self.expected_seed < 0):
            raise ValueError("expected_seed must be a nonnegative integer")
        if not self.nav_feature_profile.strip():
            raise ValueError("nav_feature_profile must not be empty")
        if not _normalized_path(self.expected_truth_path):
            raise ValueError("expected_truth_path must not be empty")
        for name, value in (
                ("retry_initial_sec", self.retry_initial_sec),
                ("retry_max_sec", self.retry_max_sec)):
            if not _finite(value) or float(value) <= 0.0:
                raise ValueError("%s must be finite and positive" % name)
        if self.retry_initial_sec > self.retry_max_sec:
            raise ValueError(
                "retry_initial_sec must not exceed retry_max_sec")


@dataclass(frozen=True)
class GateEvaluation:
    """Deterministic readiness result and all atomic contract checks."""

    ready: bool
    reason: str
    checks: dict


class MissionStartPolicy:
    """Validate field, truth, anchor, manager and control readiness."""

    _CHECK_ORDER = (
        ("field_document", "field_status_missing_or_invalid"),
        ("field_ready", "field_not_ready"),
        ("field_identity", "field_identity_mismatch"),
        ("field_classes", "field_target_classes_mismatch"),
        ("field_models", "field_target_models_mismatch"),
        ("field_footprint", "field_footprint_invalid"),
        ("truth_path", "truth_path_mismatch"),
        ("truth_durable", "truth_not_durable"),
        ("truth_document", "truth_missing_or_invalid"),
        ("truth_identity", "truth_identity_mismatch"),
        ("truth_targets", "truth_target_manifest_mismatch"),
        ("truth_geometry", "truth_footprint_manifest_invalid"),
        ("anchor_document", "anchor_status_missing_or_invalid"),
        ("anchor_ready", "anchor_not_ready"),
        ("anchor_identity", "anchor_profile_mismatch"),
        ("anchor_models", "anchor_models_mismatch"),
        ("manager_document", "manager_status_missing_or_invalid"),
        ("manager_profile", "manager_profile_mismatch"),
        ("manager_idle", "manager_not_idle"),
        ("control_ready", "control_not_ready"),
    )

    def __init__(self, config):
        self.config = config

    def _truth_checks(self, truth):
        identity = False
        targets_valid = False
        geometry_valid = False
        if _is_document(truth):
            identity = bool(
                truth.get("profile") == self.config.profile and
                _safe_int(truth.get("seed")) == self.config.expected_seed)
            targets = truth.get("targets")
            if isinstance(targets, list):
                classes = [item.get("class") for item in targets
                           if isinstance(item, dict)]
                models = [item.get("model") for item in targets
                          if isinstance(item, dict)]
                targets_valid = bool(
                    len(targets) == len(R2026_TARGET_CLASSES) and
                    _exact_string_set(classes, R2026_TARGET_CLASSES) and
                    _exact_string_set(models, R2026_TARGET_MODELS) and
                    all(item.get("model") == "random_%s" %
                        item.get("class") for item in targets
                        if isinstance(item, dict)))
                geometry_valid = bool(targets_valid and all(
                    all(_finite(item.get(key)) for key in (
                        "x", "y", "world_x", "world_y", "yaw",
                        "footprint_radius")) and
                    float(item.get("footprint_radius")) > 0.0
                    for item in targets))
        return identity, targets_valid, geometry_valid

    def evaluate(self, field, truth, truth_durable, anchor, manager,
                 control_ready=False):
        field_document = _is_document(field)
        truth_document = _is_document(truth)
        anchor_document = _is_document(anchor)
        manager_document = _is_document(manager)

        expected_truth = _normalized_path(
            self.config.expected_truth_path)
        field_truth = (_normalized_path(field.get("truth_path"))
                       if field_document else "")
        field_models = bool(field_document and all(
            _exact_string_set(field.get(key), R2026_TARGET_MODELS)
            for key in (
                "expected_models", "spawned_models", "verified_models")))
        truth_identity, truth_targets, truth_geometry = (
            self._truth_checks(truth))

        anchor_expected = (anchor.get("expected_models")
                           if anchor_document else None)
        anchor_expected_valid = bool(
            isinstance(anchor_expected, (list, tuple)) and
            all(isinstance(item, str) and item for item in anchor_expected) and
            len(anchor_expected) == len(set(anchor_expected)))
        anchor_models = bool(
            anchor_document and anchor_expected_valid and
            _exact_string_set(
                anchor.get("spawned_models"), set(anchor_expected)) and
            _exact_string_set(
                anchor.get("verified_models"), set(anchor_expected)))

        checks = {
            "field_document": field_document,
            "field_ready": bool(
                field_document and field.get("status") == "READY" and
                field.get("ready") is True),
            "field_identity": bool(
                field_document and
                field.get("profile") == self.config.profile and
                _safe_int(field.get("seed")) == self.config.expected_seed),
            "field_classes": bool(
                field_document and _exact_string_set(
                    field.get("allowed_classes"), R2026_TARGET_CLASSES)),
            "field_models": field_models,
            "field_footprint": bool(
                field_document and field.get("footprint_valid") is True),
            "truth_path": bool(
                field_document and field_truth == expected_truth),
            "truth_durable": bool(truth_durable),
            "truth_document": truth_document,
            "truth_identity": truth_identity,
            "truth_targets": truth_targets,
            "truth_geometry": truth_geometry,
            "anchor_document": anchor_document,
            "anchor_ready": bool(
                anchor_document and anchor.get("status") == "READY" and
                anchor.get("ready") is True),
            "anchor_identity": bool(
                anchor_document and anchor.get("profile") ==
                self.config.nav_feature_profile),
            "anchor_models": anchor_models,
            "manager_document": manager_document,
            "manager_profile": bool(
                manager_document and
                manager.get("profile") == self.config.profile),
            "manager_idle": bool(
                manager_document and manager.get("phase") == "IDLE"),
            "control_ready": control_ready is True,
        }
        for name, reason in self._CHECK_ORDER:
            if not checks[name]:
                return GateEvaluation(False, reason, checks)
        return GateEvaluation(True, "ready_to_start", checks)


class MissionStartGate:
    """One-shot start latch with capped exponential retry backoff."""

    def __init__(self, config, enabled=False):
        self.config = config
        self.enabled = bool(enabled)
        self.policy = MissionStartPolicy(config)
        self.field = None
        self.truth = None
        self.truth_durable = False
        self.anchor = None
        self.manager = None
        self.control_ready = False
        self.started_latched = False
        self.call_in_flight = False
        self.service_call_count = 0
        self.service_success_count = 0
        self.service_unavailable_count = 0
        self.service_rejection_count = 0
        self.retry_failure_count = 0
        self.next_retry_at = 0.0
        self.last_service_message = ""

    def update_field(self, field, truth=None, truth_durable=False):
        self.field = field
        self.truth = truth
        self.truth_durable = bool(truth_durable)

    def update_anchor(self, anchor):
        self.anchor = anchor

    def update_manager(self, manager):
        self.manager = manager

    def update_control_ready(self, ready):
        self.control_ready = ready is True

    def evaluate(self):
        return self.policy.evaluate(
            self.field, self.truth, self.truth_durable,
            self.anchor, self.manager, self.control_ready)

    def may_probe_service(self, now):
        if not self.enabled or self.started_latched or self.call_in_flight:
            return False
        if not _finite(now) or float(now) < self.next_retry_at:
            return False
        return self.evaluate().ready

    def begin_service_call(self, now):
        if not self.may_probe_service(now):
            return False
        self.call_in_flight = True
        self.service_call_count += 1
        return True

    def _record_retry(self, now, message):
        self.retry_failure_count += 1
        exponent = min(self.retry_failure_count - 1, 30)
        delay = min(
            self.config.retry_initial_sec * (2 ** exponent),
            self.config.retry_max_sec)
        self.next_retry_at = float(now) + delay
        self.last_service_message = str(message)

    def record_service_unavailable(self, now, message):
        if not self.may_probe_service(now):
            return False
        self.service_unavailable_count += 1
        self._record_retry(now, message)
        return True

    def complete_service_call(self, now, success, message):
        if not self.call_in_flight:
            raise RuntimeError("no mission start service call is in flight")
        self.call_in_flight = False
        self.last_service_message = str(message)
        if success:
            self.service_success_count += 1
            self.started_latched = True
            self.next_retry_at = math.inf
            return
        self.service_rejection_count += 1
        self._record_retry(now, message)

    def status(self, now):
        evaluation = self.evaluate()
        if not self.enabled:
            state, reason = "DISABLED", "explicit_enable_required"
        elif self.started_latched:
            state, reason = "STARTED", "start_succeeded_latched"
        elif self.call_in_flight:
            state, reason = "STARTING", "start_service_call_in_flight"
        elif not evaluation.ready:
            state, reason = "WAITING", evaluation.reason
        elif not _finite(now):
            state, reason = "WAITING", "monotonic_clock_invalid"
        elif float(now) < self.next_retry_at:
            state, reason = "RETRY_WAIT", "bounded_retry_backoff"
        else:
            state, reason = "READY", "ready_to_start"
        remaining = 0.0
        if _finite(now) and _finite(self.next_retry_at):
            remaining = max(0.0, self.next_retry_at - float(now))
        return {
            "component": "navigation_mission_start_gate",
            "schema_version": SCHEMA_VERSION,
            "status": state,
            "reason": reason,
            "enabled": self.enabled,
            "profile": self.config.profile,
            "expected_seed": self.config.expected_seed,
            "nav_feature_profile": self.config.nav_feature_profile,
            "expected_truth_path": _normalized_path(
                self.config.expected_truth_path),
            "conditions": dict(evaluation.checks),
            "control_ready": self.control_ready,
            "service_call_count": self.service_call_count,
            "service_success_count": self.service_success_count,
            "service_unavailable_count": self.service_unavailable_count,
            "service_rejection_count": self.service_rejection_count,
            "retry_failure_count": self.retry_failure_count,
            "next_retry_in_sec": round(remaining, 3),
            "call_in_flight": self.call_in_flight,
            "started_latched": self.started_latched,
            "last_service_message": self.last_service_message,
        }
