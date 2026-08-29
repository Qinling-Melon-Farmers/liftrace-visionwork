#!/usr/bin/env python3
"""Deterministic regression for profile and current-selection admission."""

import sys
import math
from types import SimpleNamespace

from uav_vision.target_selection_policy import (
    candidate_is_currently_selectable,
    choose_selected_candidate,
    resolve_class_profile,
)


PRIORITIES = {
    "tent": 1.0,
    "pillbox": 1.5,
    "bridge": 2.0,
    "panzer": 2.5,
    "tank": 5.0,
    "red_cross": 10.0,
}


def _candidate(class_name="panzer", **overrides):
    values = {
        "id": 3,
        "class_name": class_name,
        "class_confidence": 0.90,
        "map_quality": 0.85,
        "state": 2,
        "consecutive_observe_count": 3,
        "map_valid": True,
        "association_valid": True,
        "reject_reason": "",
        "map_point": SimpleNamespace(x=1.0, y=2.0, z=0.0),
        "map_frame": "camera_init",
        "last_seen": 9.8,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def main():
    full_name, full = resolve_class_profile("full")
    r2026_name, r2026 = resolve_class_profile("r2026")
    assert full_name == "full" and "tank" in full
    assert r2026_name == "r2026" and "tank" not in r2026
    try:
        resolve_class_profile("future_profile")
        raise AssertionError("unknown profile did not fail closed")
    except ValueError:
        pass

    tank = _candidate("tank", id=1)
    panzer = _candidate("panzer", id=2)
    assert choose_selected_candidate(
        [panzer, tank], 10.0, 3, 0.5, PRIORITIES, full) is tank
    assert choose_selected_candidate(
        [panzer, tank], 10.0, 3, 0.5, PRIORITIES, r2026) is panzer

    invalid_variants = [
        _candidate(state=3),
        _candidate(state=4),
        _candidate(consecutive_observe_count=0),
        _candidate(map_valid=False),
        _candidate(association_valid=False),
        _candidate(reject_reason="association_invalid"),
        _candidate(last_seen=9.0),
        _candidate(last_seen=10.01),
        _candidate(last_seen=float("nan")),
        _candidate(last_seen=float("inf")),
        _candidate(class_confidence=float("nan")),
        _candidate(class_confidence=float("inf")),
        _candidate(map_quality=float("nan")),
        _candidate(map_quality=float("inf")),
        _candidate(map_point=SimpleNamespace(
            x=float("nan"), y=2.0, z=0.0)),
        _candidate(map_point=SimpleNamespace(
            x=1.0, y=float("inf"), z=0.0)),
        _candidate(map_frame=""),
    ]
    for candidate in invalid_variants:
        assert not candidate_is_currently_selectable(
            candidate, 10.0, 3, 0.5, PRIORITIES, r2026)
    assert candidate_is_currently_selectable(
        _candidate(), 10.0, 3, 0.5, PRIORITIES, r2026)
    assert not candidate_is_currently_selectable(
        _candidate(), float("nan"), 3, 0.5, PRIORITIES, r2026)
    assert not candidate_is_currently_selectable(
        _candidate(), 10.0, 3, float("inf"), PRIORITIES, r2026)
    invalid_priorities = dict(PRIORITIES)
    invalid_priorities["panzer"] = math.nan
    assert not candidate_is_currently_selectable(
        _candidate(), 10.0, 3, 0.5, invalid_priorities, r2026)
    print("V-CL target selection profile/current-admission PASS")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("V-CL target selection profile/current-admission FAIL: {}".format(
            error), file=sys.stderr)
        sys.exit(1)
