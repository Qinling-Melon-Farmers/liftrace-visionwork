#!/usr/bin/env python3
"""Gate the real new-vision fixed three-drop SITL route.

The assertion is passive. It accepts only fresh, confirmed, map-valid
standard candidates with usable association metadata, then waits for the
guarded release chain and visual audit to finish in slot order.
"""

import sys
import time
import json
import os
import math

import rospy
from std_msgs.msg import String, UInt8

from uav_mission.msg import ReleaseResult
from uav_vision.msg import (
    DropOffset, DropReady, ReleaseEvidence, TargetCandidateArray,
)


REQUIRED_CLASSES = ("pillbox", "bridge", "tent")
EXPECTED_SLOTS = (1, 2, 3)


class NewVisionFixed3Assertion:
    def __init__(self):
        rospy.init_node("newvision_fixed3_assertion")
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 600.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "gate_status.json"))
        self._candidate_ids = {}
        self._raw_calls = []
        self._successes = []
        self._denied_results = []
        self._audit_status = "WAITING"
        self._failure = ""
        self._drop_ready_reasons = {}
        self._drop_ready_true_count = 0
        self._evidence_valid_count = 0
        self._latest_evidence = {}
        self._min_offset_px = None
        self._circle_diagnostics = {
            "observed": 0,
            "fresh_confirmed_refined": 0,
            "latest_ids": [],
        }

        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray,
                         self._on_targets, queue_size=4)
        rospy.Subscriber("/uav_mission/mock_raw_servo_calls", UInt8,
                         self._on_raw_call, queue_size=8)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_result, queue_size=12)
        rospy.Subscriber("/mission/visual_delivery_audit_status", String,
                         self._on_audit_status, queue_size=2)
        rospy.Subscriber("/uav_vision/drop_offset", DropOffset,
                         self._on_drop_offset, queue_size=4)
        rospy.Subscriber("/uav_vision/drop_ready", DropReady,
                         self._on_drop_ready, queue_size=4)
        rospy.Subscriber("/uav_vision/release_evidence", ReleaseEvidence,
                         self._on_evidence, queue_size=4)
        self._write_status("WAITING", "")

    def _write_status(self, status, reason):
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        payload = {
            "gate": "newvision_fixed3",
            "status": status,
            "reason": reason,
            "candidate_ids": self._candidate_ids,
            "raw_calls": self._raw_calls,
            "success_slots": self._successes,
            "denied_results": self._denied_results,
            "audit_status": self._audit_status,
            "drop_ready_reasons": self._drop_ready_reasons,
            "drop_ready_true_count": self._drop_ready_true_count,
            "evidence_valid_count": self._evidence_valid_count,
            "latest_evidence": self._latest_evidence,
            "min_offset_px": self._min_offset_px,
            "circle_diagnostics": self._circle_diagnostics,
        }
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def _fail(self, reason):
        if self._failure:
            return
        self._failure = reason
        self._write_status("FAIL", reason)
        rospy.logerr("[NewVisionFixed3Assertion] FAIL: %s", reason)
        rospy.signal_shutdown(reason)

    @staticmethod
    def _fresh(candidate):
        if candidate.last_seen.to_sec() <= 0.0:
            return False
        return max(0.0, (rospy.Time.now() - candidate.last_seen).to_sec()) <= 0.5

    def _on_targets(self, msg):
        circles = [candidate for candidate in msg.targets
                   if candidate.class_name == "circle"]
        fresh_confirmed = [candidate for candidate in circles
                           if candidate.state >= 2 and
                           candidate.center_refined and
                           candidate.association_valid and
                           self._fresh(candidate)]
        self._circle_diagnostics = {
            "observed": len(circles),
            "fresh_confirmed_refined": len(fresh_confirmed),
            "latest_ids": [int(candidate.id) for candidate in circles],
        }
        for candidate in msg.targets:
            if candidate.class_name not in REQUIRED_CLASSES:
                continue
            if (candidate.state < 2 or not candidate.map_valid or
                    candidate.map_frame != "camera_init" or
                    not candidate.association_valid or
                    candidate.reject_reason or not candidate.center_refined or
                    not self._fresh(candidate)):
                continue
            self._candidate_ids[candidate.class_name] = int(candidate.id)

        present = [self._candidate_ids.get(name) for name in REQUIRED_CLASSES]
        if all(value is not None for value in present) and len(set(present)) == 3:
            rospy.loginfo_throttle(
                2.0, "[NewVisionFixed3Assertion] valid candidates %s",
                self._candidate_ids)

    def _on_raw_call(self, msg):
        self._raw_calls.append(int(msg.data))
        if self._raw_calls != list(EXPECTED_SLOTS[:len(self._raw_calls)]):
            self._fail("raw_slots_%s" % self._raw_calls)

    def _on_result(self, msg):
        if not msg.success:
            denied = {
                "slot": int(msg.payload_slot),
                "reason": msg.reason,
                "target_id": int(msg.target_id),
                "target_class": msg.target_class,
            }
            self._denied_results.append(denied)
            self._fail("release_denied_%s" % denied)
            return
        if msg.success:
            self._successes.append(int(msg.payload_slot))
            if self._successes != list(EXPECTED_SLOTS[:len(self._successes)]):
                self._fail("success_slots_%s" % self._successes)

    def _on_audit_status(self, msg):
        self._audit_status = msg.data.strip()
        if self._audit_status == "FAIL":
            self._fail("visual_delivery_audit_failed")

    def _on_drop_offset(self, msg):
        offset = math.hypot(float(msg.dx_px), float(msg.dy_px))
        if self._min_offset_px is None or offset < self._min_offset_px:
            self._min_offset_px = offset

    def _on_drop_ready(self, msg):
        reason = msg.reason or "<empty>"
        self._drop_ready_reasons[reason] = \
            self._drop_ready_reasons.get(reason, 0) + 1
        if msg.ready:
            self._drop_ready_true_count += 1

    def _on_evidence(self, msg):
        self._latest_evidence = {
            "align_mode": msg.align_mode,
            "target_id": int(msg.target_id),
            "target_class": msg.target_class,
            "aligned": bool(msg.aligned),
            "stable_frames": int(msg.stable_frames),
            "evidence_valid": bool(msg.evidence_valid),
            "observation_age_sec": float(msg.observation_age_sec),
            "rejection_reasons": list(msg.rejection_reasons),
        }
        if msg.evidence_valid:
            self._evidence_valid_count += 1

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            candidate_ids = [self._candidate_ids.get(name)
                             for name in REQUIRED_CLASSES]
            candidates_ok = (all(value is not None for value in candidate_ids)
                             and len(set(candidate_ids)) == 3)
            drops_ok = (self._raw_calls == list(EXPECTED_SLOTS) and
                        self._successes == list(EXPECTED_SLOTS))
            if candidates_ok and drops_ok and self._audit_status == "PASS":
                self._write_status("PASS", "three_drop_chain_complete")
                rospy.loginfo(
                    "[NewVisionFixed3Assertion] PASS classes=%s ids=%s slots=%s audit=%s",
                    REQUIRED_CLASSES, candidate_ids, self._successes,
                    self._audit_status)
                return 0
            if time.monotonic() >= self._deadline:
                self._fail(
                    "wall_timeout candidates=%s raw=%s success=%s audit=%s" %
                    (self._candidate_ids, self._raw_calls,
                     self._successes, self._audit_status))
                return 1
            rate.sleep()
        return 1 if self._failure else 0


if __name__ == "__main__":
    sys.exit(NewVisionFixed3Assertion().run())
