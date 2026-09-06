#!/usr/bin/env python3
"""ROS-free assertions for typed V-SIM-04 navigation evidence."""

import copy
import csv
import json
import os
import shutil
import sys
import tempfile
import traceback

from uav_vision_eval.vsim04_metrics import (
    correlate_navigation_events,
    navigation_drain_ready,
    write_artifacts,
)


FIRST_SEEN_NS = 1_000_000_000
OBSERVATION_NS = 1_500_000_000


def completed_trial():
    return {
        "trial_id": "dynamic_panzer_h1p8_v0p5",
        "kind": "dynamic",
        "class_name": "panzer",
        "height_m": 1.8,
        "speed_mps": 0.5,
        "status": "completed",
        "p_confirm": True,
        "p_selected": True,
        "p_interrupt": None,
        "stable_id": 42,
        "selected_target_first_seen_ns": FIRST_SEEN_NS,
        "selected_target_observation_stamps_ns": [OBSERVATION_NS],
        "entered_fully_in_frame": True,
        "left_fully_in_frame": True,
    }


def decision(**updates):
    record = {
        "schema_version": 1,
        "mission_id": "vcl06-unit",
        "decision_seq": 7,
        "header_stamp_ns": 2_000_000_000,
        "deadline_ns": 10_000_000_000,
        "command": 1,
        "class_profile": "r2026",
        "has_goal": True,
        "has_target": True,
        "target_id": 42,
        "target_first_seen_ns": FIRST_SEEN_NS,
        "target_observation_stamp_ns": OBSERVATION_NS,
        "target_class": "panzer",
        "attempt": 1,
        "payload_slot": 1,
        "receipt_monotonic": 2.0,
    }
    record.update(updates)
    return record


def result(event_seq, status, stage, reason="", **updates):
    record = {
        "schema_version": 1,
        "mission_id": "vcl06-unit",
        "executor_id": "adapter-unit",
        "event_seq": event_seq,
        "decision_seq": 7,
        "header_stamp_ns": (2 + event_seq) * 1_000_000_000,
        "command": 1,
        "status": status,
        "stage": stage,
        "terminal": False,
        "retryable": False,
        "payload_committed": False,
        "has_target": True,
        "target_id": 42,
        "target_first_seen_ns": FIRST_SEEN_NS,
        "target_class": "panzer",
        "attempt": 1,
        "payload_slot": 1,
        "reason": reason,
        "evidence_source": "unit",
        "receipt_monotonic": 2.0 + event_seq,
    }
    record.update(updates)
    return record


def evaluate(mode, decisions=None, results=None, trials=None):
    return correlate_navigation_events(
        trials or [completed_trial()],
        decisions if decisions is not None else [decision()],
        results if results is not None else [],
        mode=mode,
        class_profile="r2026",
        allowed_classes={
            "tent", "pillbox", "bridge", "panzer", "red_cross"})


def only_trial(evaluation):
    assert len(evaluation["trials"]) == 1
    return evaluation["trials"][0]


def main():
    dispatch = result(1, status=0, stage=0)
    arrival = result(
        2, status=2, stage=1, reason="approach_arrival_confirmed")
    capture = result(3, status=1, stage=2)

    visual = evaluate(
        "visual_only", [decision(schema_version=99)], [capture])
    visual_trial = only_trial(visual)
    assert visual["validation_errors"] == []
    assert visual_trial["p_decision"] is None
    assert visual_trial["p_dispatch"] is None
    assert visual_trial["p_planner_arrival"] is None
    assert visual_trial["p_interrupt"] is None
    assert visual_trial["p_interrupt_reason"] == \
        "visual_only_no_navigation_acceptance_event"

    # Results are deliberately evaluated after the trial object is already
    # completed: snapshot-time correlation must backfill late callbacks.
    typed = evaluate("typed_contract", results=[dispatch, arrival, capture])
    typed_trial = only_trial(typed)
    assert typed["validation_errors"] == []
    assert typed_trial["p_decision"]
    assert typed_trial["p_dispatch"]
    assert typed_trial["p_planner_arrival"]
    assert typed_trial["p_interrupt"] is None
    assert typed_trial["p_interrupt_reason"] == \
        "typed_navigation_contract_without_target_stage_capability"
    assert json.loads(typed_trial["navigation_binding_keys"]) == [[
        "vcl06-unit", 7, 42, FIRST_SEEN_NS, 1, 1]]

    target_stage = evaluate(
        "target_stage", results=[dispatch, arrival, capture])
    target_trial = only_trial(target_stage)
    assert target_stage["target_stage_capability"]
    assert target_trial["p_interrupt"]

    late_arrival = copy.deepcopy(arrival)
    late_arrival["header_stamp_ns"] = 12_000_000_000
    late_capture = copy.deepcopy(capture)
    late_capture["header_stamp_ns"] = 13_000_000_000
    post_lease = evaluate(
        "target_stage", results=[dispatch, late_arrival, late_capture])
    post_lease_trial = only_trial(post_lease)
    assert sum("result_at_or_after_deadline" in error
               for error in post_lease["validation_errors"]) == 2
    assert post_lease_trial["p_dispatch"]
    assert not post_lease_trial["p_planner_arrival"]
    assert not post_lease_trial["p_interrupt"]

    # The target observation stamp assigns a decision to exactly one visual
    # trial even when stable ID and first-seen identity persist across trials.
    later_trial = completed_trial()
    later_trial["trial_id"] = "dynamic_panzer_h3p0_v0p5"
    later_trial["selected_target_observation_stamps_ns"] = [1_600_000_000]
    cross_trial = evaluate(
        "target_stage", results=[dispatch, arrival, capture],
        trials=[completed_trial(), later_trial])
    assert cross_trial["trials"][0]["p_interrupt"]
    assert not cross_trial["trials"][1]["p_decision"]
    assert not cross_trial["trials"][1]["p_dispatch"]
    assert not cross_trial["trials"][1]["p_planner_arrival"]
    assert not cross_trial["trials"][1]["p_interrupt"]

    zero_trial = completed_trial()
    zero_trial["stable_id"] = 0
    zero_identity = correlate_navigation_events(
        [zero_trial], [decision(target_id=0)], [
            result(1, status=0, stage=0, target_id=0)],
        mode="typed_contract", class_profile="r2026",
        allowed_classes={"panzer"})
    assert zero_identity["validation_errors"] == []
    assert only_trial(zero_identity)["p_dispatch"]

    empty_allowlist = correlate_navigation_events(
        [completed_trial()], [decision()], [dispatch],
        mode="typed_contract", class_profile="r2026",
        allowed_classes=[])
    assert empty_allowlist["validation_errors"] == []
    assert only_trial(empty_allowlist)["p_dispatch"]
    derived_r2026 = correlate_navigation_events(
        [completed_trial()], [decision(target_class="tank")], [],
        mode="typed_contract", class_profile="r2026", allowed_classes=[])
    assert any("target_class_disallowed_by_profile" in error
               for error in derived_r2026["validation_errors"])
    empty_profile = correlate_navigation_events(
        [completed_trial()], [decision()], [dispatch],
        mode="typed_contract", class_profile="", allowed_classes=[])
    assert any("profile_mismatch" in error
               for error in empty_profile["validation_errors"])
    assert not any("target_class_disallowed_by_profile" in error
                   for error in empty_profile["validation_errors"])

    # No weaker observation is allowed to stand in for a stronger milestone.
    selected_only = evaluate("target_stage", decisions=[], results=[])
    assert not only_trial(selected_only)["p_decision"]
    decision_only = evaluate("target_stage")
    decision_trial = only_trial(decision_only)
    assert decision_trial["p_decision"]
    assert not decision_trial["p_dispatch"]
    assert not decision_trial["p_planner_arrival"]
    assert not decision_trial["p_interrupt"]
    dispatch_only = evaluate("target_stage", results=[dispatch])
    dispatch_trial = only_trial(dispatch_only)
    assert dispatch_trial["p_dispatch"]
    assert not dispatch_trial["p_planner_arrival"]
    assert not dispatch_trial["p_interrupt"]
    arrival_only = evaluate("target_stage", results=[arrival])
    arrival_trial = only_trial(arrival_only)
    assert arrival_trial["p_planner_arrival"]
    assert not arrival_trial["p_dispatch"]
    assert not arrival_trial["p_interrupt"]
    wrong_capture = evaluate(
        "target_stage", results=[result(1, status=1, stage=1)])
    assert not only_trial(wrong_capture)["p_interrupt"]

    # Exact retransmissions are idempotent; identity/event-seq conflicts are
    # not.  Callback receipt metadata is deliberately different.
    duplicate_decision = copy.deepcopy(decision())
    duplicate_decision["receipt_monotonic"] = 99.0
    duplicate_dispatch = copy.deepcopy(dispatch)
    duplicate_dispatch["receipt_monotonic"] = 99.0
    idempotent = evaluate(
        "typed_contract", [decision(), duplicate_decision],
        [dispatch, duplicate_dispatch])
    assert idempotent["validation_errors"] == []
    assert idempotent["deduplicated_decision_count"] == 1
    assert idempotent["deduplicated_result_count"] == 1

    decision_conflict = evaluate(
        "typed_contract",
        [decision(), decision(target_id=43)], [])
    assert any("decision_identity_conflict" in error
               for error in decision_conflict["validation_errors"])
    event_conflict = evaluate(
        "typed_contract", results=[
            dispatch, result(1, status=1, stage=2)])
    assert any("event_seq_conflict" in error
               for error in event_conflict["validation_errors"])
    out_of_order = evaluate(
        "typed_contract", results=[arrival, dispatch])
    assert any("event_seq_out_of_order" in error
               for error in out_of_order["validation_errors"])
    out_of_order_trial = only_trial(out_of_order)
    assert not out_of_order_trial["p_dispatch"]
    assert out_of_order_trial["p_planner_arrival"]

    invalid_cases = (
        ("schema_version_mismatch", [decision(schema_version=2)], []),
        ("deadline_not_after_decision", [decision(
            deadline_ns=2_000_000_000)], []),
        ("profile_mismatch", [decision(class_profile="full")], []),
        ("command_invalid", [decision(command=99)], []),
        ("has_target_invalid", [decision(has_target="true")], []),
        ("target_observation_precedes_first_seen", [decision(
            target_observation_stamp_ns=FIRST_SEEN_NS - 1)], []),
        ("target_observation_after_decision", [decision(
            target_observation_stamp_ns=2_000_000_001)], []),
        ("target_class_disallowed_by_profile", [decision(
            target_class="tank")], []),
        ("dispatch_at_or_after_deadline", [decision()], [result(
            1, status=0, stage=0, header_stamp_ns=10_000_000_000)]),
        ("full_binding_key_mismatch", [decision()], [result(
            1, status=0, stage=0,
            target_first_seen_ns=FIRST_SEEN_NS + 1)]),
        ("command_mismatch", [decision()], [result(
            1, status=0, stage=0, command=2)]),
    )
    for expected, decisions, results in invalid_cases:
        evaluation = evaluate("typed_contract", decisions, results)
        assert any(expected in error
                   for error in evaluation["validation_errors"]), (
                       expected, evaluation["validation_errors"])

    output_dir = tempfile.mkdtemp(prefix="vsim04_navigation_metrics_")
    try:
        artifact_summary = write_artifacts(
            output_dir,
            {"class_profile": "r2026"},
            [], [], [completed_trial()], "unit_target_stage",
            terminal_context={
                "run_complete": True,
                "evaluation_scope": "diagnostic",
                "expected_trial_count": 1,
                "validation_errors": [],
                "class_profile": "r2026",
                "allowed_classes": [
                    "tent", "pillbox", "bridge", "panzer", "red_cross"],
                "navigation_metrics_mode": "target_stage",
                "navigation_decision_topic":
                    "/navigation/mission_command_raw",
                "navigation_result_topic": "/navigation/mission_result",
                "navigation_decision_records": [decision()],
                "navigation_result_records": [
                    dispatch, arrival, capture],
            })
        assert artifact_summary["navigation_metrics"]["mode"] == \
            "target_stage"
        assert artifact_summary["navigation_metrics"][
            "target_stage_capability"]
        assert artifact_summary["metrics"]["p_decision"] == 1.0
        assert artifact_summary["metrics"]["p_dispatch"] == 1.0
        assert artifact_summary["metrics"]["p_planner_arrival"] == 1.0
        assert artifact_summary["metrics"]["p_interrupt"] == 1.0
        with open(os.path.join(output_dir, "manifest.json"),
                  "r", encoding="utf-8") as stream:
            manifest = json.load(stream)
        assert manifest["navigation_metrics"]["mode"] == "target_stage"
        assert manifest["navigation_metrics"]["target_stage_capability"]
        assert manifest["navigation_metrics"]["decision_event_count"] == 1
        assert manifest["navigation_metrics"]["result_event_count"] == 3
        assert manifest["navigation_metrics"]["validation_errors"] == []
        with open(os.path.join(output_dir, "report.md"),
                  "r", encoding="utf-8") as stream:
            report = stream.read()
        assert "Navigation metrics mode: `target_stage`" in report
        assert "P_interrupt: `1.0`" in report
        with open(os.path.join(
                output_dir, "vision_search_performance.csv"),
                "r", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 1
        assert rows[0]["navigation_metrics_mode"] == "target_stage"
        assert rows[0]["p_decision"] == "True"
        assert rows[0]["p_dispatch"] == "True"
        assert rows[0]["p_planner_arrival"] == "True"
        assert rows[0]["p_interrupt"] == "True"
    finally:
        shutil.rmtree(output_dir)

    assert navigation_drain_ready(10.49, 10.0, None, 0.5, 3.0) == \
        (False, False)
    assert navigation_drain_ready(10.5, 10.0, None, 0.5, 3.0) == \
        (True, False)
    assert navigation_drain_ready(11.0, 10.0, 10.8, 0.5, 3.0) == \
        (False, False)
    assert navigation_drain_ready(13.0, 10.0, 12.9, 0.5, 3.0) == \
        (True, True)
    assert navigation_drain_ready(10.0, 10.0, None, 1.0, 0.5) == \
        (False, False)

    try:
        evaluate("invented_mode")
        raise AssertionError("unknown navigation mode was accepted")
    except ValueError:
        pass

    print("V-SIM-04 typed navigation metrics PASS")


def test_typed_navigation_metrics_contract():
    """Expose the same pure assertion through catkin_add_nosetests."""
    main()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # pylint: disable=broad-except
        print("V-SIM-04 typed navigation metrics FAIL: {}".format(error),
              file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
