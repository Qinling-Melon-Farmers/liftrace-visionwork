#!/usr/bin/env python3
"""Record and validate the visual-to-legacy three-drop event chain.

This node is passive: it never calls flight-control or actuator interfaces.
It snapshots the evidence and legacy controller context at every raw mock
Servo call, writes an atomic JSON report, and publishes WAITING/PASS/FAIL.
"""
import json
import os
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String, UInt8

from uav_mission.msg import ReleasePermission, ReleaseResult
from uav_mission.visual_delivery_audit_policy import (
    permission_matches_audit_view, resolve_audit_evidence,
)
from uav_vision.msg import (
    ReleaseEvidence, ReleaseEvidenceContext, TargetCandidate,
)


class VisualDeliveryAudit:
    def __init__(self):
        rospy.init_node("visual_delivery_audit")
        self._report_path = rospy.get_param(
            "~report_path", "/tmp/visual_delivery_audit.json")
        self._required_slots = list(range(
            1, int(rospy.get_param("~payload_slots", 3)) + 1))
        self._required_control_state = int(
            rospy.get_param("~required_control_state", 2))
        self._min_stable_frames = int(
            rospy.get_param("~min_stable_frames", 5))
        self._min_altitude = float(
            rospy.get_param("~min_release_altitude", -0.05))
        self._max_altitude = float(
            rospy.get_param("~max_release_altitude", 0.25))
        self._require_evidence_context = bool(
            rospy.get_param("~require_evidence_context", False))
        self._evidence_context_topic = rospy.get_param(
            "~evidence_context_topic",
            "/uav_vision/release_evidence_context")

        self._lock = threading.RLock()
        self._events = []
        self._raw_calls = []
        self._raw_contexts = []
        self._results = []
        self._status = "WAITING"
        self._failure_reason = ""
        self._control_state = None
        self._align_mode = "disabled"
        self._pose_z = None
        self._evidence = None
        self._evidence_context = None
        self._locked_evidence = None
        self._locked_evidence_context = None
        self._permission_evidence = None
        self._permission_evidence_context = None
        self._permission = None
        self._selected_target = None
        self._last_signatures = {}

        self._status_pub = rospy.Publisher(
            "/mission/visual_delivery_audit_status", String,
            queue_size=1, latch=True)
        rospy.Subscriber("/detect/point_class", Int8,
                         self._on_control_state, queue_size=4)
        rospy.Subscriber("/uav_vision/align_mode", String,
                         self._on_align_mode, queue_size=4)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._on_pose, queue_size=4)
        rospy.Subscriber("/uav_vision/selected_target", TargetCandidate,
                         self._on_selected_target, queue_size=4)
        rospy.Subscriber("/uav_vision/release_evidence", ReleaseEvidence,
                         self._on_evidence, queue_size=4)
        if self._require_evidence_context:
            rospy.Subscriber(
                self._evidence_context_topic, ReleaseEvidenceContext,
                self._on_evidence_context, queue_size=4)
        rospy.Subscriber("/mission/release_commitment_evidence",
                         ReleaseEvidence,
                         self._on_commitment_evidence, queue_size=4)
        rospy.Subscriber("/mission/release_permission", ReleasePermission,
                         self._on_permission, queue_size=8)
        rospy.Subscriber("/uav_mission/mock_raw_servo_calls", UInt8,
                         self._on_raw_call, queue_size=8)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_result, queue_size=12)
        rospy.on_shutdown(self._write_report)
        self._publish_status()
        self._write_report()
        rospy.loginfo(
            "[VisualDeliveryAudit] ready report=%s required_slots=%s",
            self._report_path, self._required_slots)

    @staticmethod
    def _now_sec():
        return rospy.Time.now().to_sec()

    def _record_change(self, kind, signature, data):
        if self._last_signatures.get(kind) == signature:
            return
        self._last_signatures[kind] = signature
        self._events.append({"time": self._now_sec(), "type": kind,
                             "data": data})

    def _on_control_state(self, msg):
        with self._lock:
            self._control_state = int(msg.data)
            self._record_change("control_state", self._control_state,
                                {"value": self._control_state})

    def _on_align_mode(self, msg):
        with self._lock:
            self._align_mode = msg.data.strip()
            self._record_change("align_mode", self._align_mode,
                                {"value": self._align_mode})

    def _on_pose(self, msg):
        with self._lock:
            self._pose_z = float(msg.pose.position.z)

    @staticmethod
    def _candidate_dict(msg):
        return {
            "id": int(msg.id),
            "class_name": msg.class_name,
            "class_confidence": float(msg.class_confidence),
            "geometry_confidence": float(msg.geometry_confidence),
            "state": int(msg.state),
            "observe_count": int(msg.observe_count),
            "consecutive_observe_count": int(msg.consecutive_observe_count),
            "map_valid": bool(msg.map_valid),
            "map_frame": msg.map_frame,
            "map_point": [float(msg.map_point.x), float(msg.map_point.y),
                          float(msg.map_point.z)],
        }

    def _on_selected_target(self, msg):
        with self._lock:
            self._selected_target = self._candidate_dict(msg)
            signature = (
                self._selected_target["id"],
                self._selected_target["class_name"],
                self._selected_target["state"],
                self._selected_target["consecutive_observe_count"],
            )
            self._record_change("selected_target", signature,
                                self._selected_target)

    @staticmethod
    def _evidence_dict(msg):
        return {
            "stamp": msg.header.stamp.to_sec(),
            "stamp_nsec": int(msg.header.stamp.to_nsec()),
            "align_mode": msg.align_mode,
            "target_id": int(msg.target_id),
            "target_class": msg.target_class,
            "target_confirmed": bool(msg.target_confirmed),
            "geometry_verified": bool(msg.geometry_verified),
            "center_refined": bool(msg.center_refined),
            "observation_fresh": bool(msg.observation_fresh),
            "aligned": bool(msg.aligned),
            "stable_frames": int(msg.stable_frames),
            "evidence_valid": bool(msg.evidence_valid),
        }

    def _on_evidence(self, msg):
        with self._lock:
            self._evidence = self._evidence_dict(msg)

    @classmethod
    def _evidence_context_dict(cls, msg):
        return {
            "stamp_nsec": int(msg.header.stamp.to_nsec()),
            "context_valid": bool(msg.context_valid),
            "context_reason": msg.context_reason,
            "context_active": bool(msg.context_active),
            "mission_id": msg.mission_id,
            "decision_seq": int(msg.decision_seq),
            "deadline": msg.deadline.to_sec(),
            "command": int(msg.command),
            "class_profile": msg.class_profile,
            "align_mode": msg.align_mode,
            "has_semantic_target": bool(msg.has_semantic_target),
            "semantic_target_id": int(msg.semantic_target_id),
            "semantic_target_class": msg.semantic_target_class,
            "attempt": int(msg.attempt),
            "payload_slot": int(msg.payload_slot),
            "geometry_target_present": bool(msg.geometry_target_present),
            "geometry_target_id": int(msg.geometry_target_id),
            "geometry_target_class": msg.geometry_target_class,
            "semantic_geometry_match": bool(msg.semantic_geometry_match),
            "evidence": cls._evidence_dict(msg.evidence),
        }

    def _on_evidence_context(self, msg):
        with self._lock:
            self._evidence_context = self._evidence_context_dict(msg)
            if (self._locked_evidence is not None and
                    self._locked_evidence["stamp_nsec"] ==
                    self._evidence_context["evidence"]["stamp_nsec"]):
                self._locked_evidence_context = dict(self._evidence_context)

    def _on_commitment_evidence(self, msg):
        with self._lock:
            self._locked_evidence = self._evidence_dict(msg)
            self._locked_evidence_context = None
            if (self._evidence_context is not None and
                    self._evidence_context["evidence"]["stamp_nsec"] ==
                    self._locked_evidence["stamp_nsec"]):
                self._locked_evidence_context = dict(self._evidence_context)
            signature = (
                self._locked_evidence["align_mode"],
                self._locked_evidence["target_id"],
                self._locked_evidence["stamp_nsec"],
            )
            self._record_change(
                "release_evidence_lock", signature,
                self._locked_evidence)

    @staticmethod
    def _permission_dict(msg):
        return {
            "stamp": msg.header.stamp.to_sec(),
            "permitted": bool(msg.permitted),
            "payload_slot": int(msg.payload_slot),
            "align_mode": msg.align_mode,
            "target_id": int(msg.target_id),
            "target_class": msg.target_class,
            "reason": msg.reason,
            "evidence_stamp": msg.evidence_stamp.to_sec(),
            "evidence_stamp_nsec": int(msg.evidence_stamp.to_nsec()),
            "valid_until": msg.valid_until.to_sec(),
        }

    def _on_permission(self, msg):
        with self._lock:
            self._permission = self._permission_dict(msg)
            if (self._locked_evidence is not None and
                    self._locked_evidence["stamp_nsec"] ==
                    self._permission["evidence_stamp_nsec"]):
                self._permission_evidence = dict(self._locked_evidence)
            else:
                self._permission_evidence = None
            self._permission_evidence_context = None
            if self._require_evidence_context:
                candidates = (
                    self._locked_evidence_context,
                    self._evidence_context,
                )
                self._permission_evidence_context = next((
                    dict(item) for item in candidates
                    if item is not None and
                    item["evidence"]["stamp_nsec"] ==
                    self._permission["evidence_stamp_nsec"]
                ), None)
            signature = (
                self._permission["permitted"],
                self._permission["payload_slot"],
                self._permission["align_mode"],
                self._permission["target_id"],
                self._permission["reason"],
            )
            self._record_change("permission", signature, self._permission)

    def _context(self):
        return {
            "control_state": self._control_state,
            "align_mode": self._align_mode,
            "pose_z": self._pose_z,
            "selected_target": self._selected_target,
            "release_evidence": self._evidence,
            "release_evidence_context": self._evidence_context,
            "locked_release_evidence": self._locked_evidence,
            "permission_release_evidence": self._permission_evidence,
            "permission_release_evidence_context":
                self._permission_evidence_context,
            "release_permission": self._permission,
        }

    def _on_raw_call(self, msg):
        with self._lock:
            slot = int(msg.data)
            context = self._context()
            self._raw_calls.append(slot)
            self._raw_contexts.append({
                "time": self._now_sec(), "slot": slot, "context": context})
            self._events.append({"time": self._now_sec(), "type": "raw_servo",
                                 "data": {"slot": slot, "context": context}})
            self._update_status()
            self._write_report()

    @staticmethod
    def _result_dict(msg):
        return {
            "stamp": msg.header.stamp.to_sec(),
            "execution_id": int(msg.execution_id),
            "payload_slot": int(msg.payload_slot),
            "success": bool(msg.success),
            "align_mode": msg.align_mode,
            "target_id": int(msg.target_id),
            "target_class": msg.target_class,
            "reason": msg.reason,
        }

    def _on_result(self, msg):
        with self._lock:
            result = self._result_dict(msg)
            self._results.append(result)
            self._events.append({"time": self._now_sec(), "type": "result",
                                 "data": result})
            self._update_status()
            self._write_report()

    def _validate_complete(self):
        successes = [item for item in self._results if item["success"]]
        slots = [item["payload_slot"] for item in successes]
        if slots != self._required_slots:
            return False, "success_slots_%s" % slots
        if self._raw_calls != self._required_slots:
            return False, "raw_slots_%s" % self._raw_calls

        target_keys = []
        for slot in self._required_slots:
            raw = next((item for item in self._raw_contexts
                        if item["slot"] == slot), None)
            if raw is None:
                return False, "raw_context_missing_slot_%d" % slot
            context = raw["context"]
            if context["control_state"] != self._required_control_state:
                return False, "control_not_aligning_slot_%d" % slot
            if context["align_mode"] not in ("drop_circle", "drop_cross"):
                return False, "align_mode_invalid_slot_%d" % slot
            pose_z = context["pose_z"]
            if pose_z is None or not self._min_altitude <= pose_z <= self._max_altitude:
                return False, "altitude_invalid_slot_%d" % slot
            permission = context["release_permission"]
            evidence_context = context[
                "permission_release_evidence_context"]
            evidence = context["permission_release_evidence"]
            if not self._require_evidence_context:
                if evidence is None:
                    evidence = context["release_evidence"]
                if evidence is None or not evidence["evidence_valid"]:
                    evidence = context["locked_release_evidence"]
            view, reason = resolve_audit_evidence(
                evidence, evidence_context,
                self._require_evidence_context)
            if view is None:
                return False, "%s_slot_%d" % (reason, slot)
            evidence = view["geometry_evidence"]
            if not evidence["evidence_valid"]:
                return False, "evidence_lock_missing_slot_%d" % slot
            if evidence["stable_frames"] < self._min_stable_frames:
                return False, "evidence_unstable_slot_%d" % slot
            if permission is None or not permission["permitted"]:
                return False, "permission_invalid_slot_%d" % slot
            if permission["payload_slot"] != slot:
                return False, "permission_slot_mismatch_%d" % slot
            if not permission_matches_audit_view(permission, view):
                return False, "permission_evidence_mismatch_slot_%d" % slot
            if permission["evidence_stamp_nsec"] != evidence["stamp_nsec"]:
                return False, "permission_evidence_stamp_mismatch_slot_%d" % slot
            target_keys.append((permission["align_mode"],
                                permission["target_id"]))
        if len(set(target_keys)) != len(target_keys):
            return False, "duplicate_release_target"
        return True, "three_drop_chain_complete"

    def _update_status(self):
        if self._status != "WAITING":
            return
        success_count = sum(1 for item in self._results if item["success"])
        if success_count < len(self._required_slots) or \
                len(self._raw_calls) < len(self._required_slots):
            return
        passed, reason = self._validate_complete()
        self._status = "PASS" if passed else "FAIL"
        self._failure_reason = "" if passed else reason
        self._publish_status()
        log = rospy.loginfo if passed else rospy.logerr
        log("[VisualDeliveryAudit] %s reason=%s", self._status, reason)

    def _publish_status(self):
        self._status_pub.publish(String(data=self._status))

    def _report(self):
        return {
            "schema_version": 2,
            "status": self._status,
            "failure_reason": self._failure_reason,
            "required_slots": self._required_slots,
            "raw_calls": self._raw_calls,
            "results": self._results,
            "latest_context": self._context(),
            "events": self._events,
        }

    def _write_report(self):
        with self._lock:
            directory = os.path.dirname(self._report_path) or "."
            os.makedirs(directory, exist_ok=True)
            temporary = self._report_path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(self._report(), handle, ensure_ascii=False,
                          indent=2, sort_keys=True)
                handle.write("\n")
            os.replace(temporary, self._report_path)


def main():
    VisualDeliveryAudit()
    rospy.spin()


if __name__ == "__main__":
    main()
