#!/usr/bin/env python3
"""Publish one strictly live/profile-valid candidate for the nav manager.

This node deliberately owns no queue, retry, terminal-ID, or mission state.
Those behaviours remain the navigation manager's responsibility.
"""

import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy

from coverage_policy import (
    SelectorCandidate,
    adapter_candidate_accepting,
    profile_allowed_classes,
    select_current_candidate,
)
from std_msgs.msg import String
from uav_vision.msg import TargetCandidate, TargetCandidateArray


class ProfileCandidateSelector:
    def __init__(self):
        rospy.init_node("profile_candidate_selector")
        self._profile = rospy.get_param("~class_profile", "r2026")
        try:
            self._allowed = profile_allowed_classes(self._profile)
        except ValueError as exc:
            rospy.logfatal("[ProfileSelector] %s", exc)
            raise
        self._frame = rospy.get_param("~mission_frame", "camera_init")
        self._max_age = float(rospy.get_param("~max_age", 0.5))
        self._min_streak = int(rospy.get_param("~min_streak", 3))
        self._target_max_z = float(rospy.get_param("~target_max_z", 4.0))
        self._require_field_ready = bool(
            rospy.get_param("~require_field_ready", True))
        self._require_adapter_accepting = bool(
            rospy.get_param("~require_adapter_candidate_accepting", True))
        self._bounds = (
            float(rospy.get_param("~field/min_x", -3.992)),
            float(rospy.get_param("~field/max_x", 4.008)),
            float(rospy.get_param("~field/min_y", -1.132)),
            float(rospy.get_param("~field/max_y", 8.718)),
        )
        if self._max_age <= 0.0 or self._min_streak <= 0:
            raise ValueError("max_age and min_streak must be positive")
        self._field_ready = not self._require_field_ready
        self._field_reason = (
            "field_ready_not_required" if self._field_ready
            else "waiting_for_field_ready")
        self._adapter_status_seen = False
        self._adapter_accepting = not self._require_adapter_accepting
        self._adapter_state = None
        self._selected_count = 0
        self._tank_rejected_count = 0
        self._invalid_rejected_count = 0
        self._selected_pub = rospy.Publisher(
            rospy.get_param(
                "~selected_topic", "/mission/profile_selected_target"),
            TargetCandidate,
            queue_size=1)
        self._status_pub = rospy.Publisher(
            rospy.get_param(
                "~status_topic", "/mission/profile_selector_status"), String,
            queue_size=1, latch=True)
        rospy.Subscriber(
            rospy.get_param("~targets_topic", "/uav_vision/targets"),
            TargetCandidateArray, self._on_targets, queue_size=1)
        if self._require_field_ready:
            rospy.Subscriber(
                rospy.get_param(
                    "~field_status_topic", "/mission/random_field_status"),
                String, self._on_field_status, queue_size=1)
        if self._require_adapter_accepting:
            rospy.Subscriber(
                rospy.get_param(
                    "~adapter_status_topic",
                    "/mission/target_search_status"),
                String, self._on_adapter_status, queue_size=2)
        self._publish_status()

    def _publish_status(self):
        if not self._field_ready:
            reason = self._field_reason
        elif self._require_adapter_accepting and not self._adapter_status_seen:
            reason = "waiting_for_adapter_status"
        elif not self._adapter_accepting:
            reason = "adapter_not_accepting_%s" % str(
                self._adapter_state or "unknown").lower()
        else:
            reason = "candidate_publication_enabled"
        payload = {
            "component": "profile_candidate_selector",
            "ready": self._field_ready,
            "reason": reason,
            "profile": self._profile,
            "allowed_classes": list(self._allowed),
            "adapter_status_seen": self._adapter_status_seen,
            "adapter_state": self._adapter_state,
            "adapter_candidate_accepting": self._adapter_accepting,
            "publishing_enabled": (
                self._field_ready and self._adapter_accepting),
            "selected_count": self._selected_count,
            "tank_rejected_count": self._tank_rejected_count,
            "invalid_rejected_count": self._invalid_rejected_count,
        }
        self._status_pub.publish(String(data=json.dumps(
            payload, sort_keys=True)))

    def _on_field_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            self._field_ready = False
            self._field_reason = "invalid_field_status_json"
            self._publish_status()
            return
        same_profile = payload.get("profile") == self._profile
        self._field_ready = bool(
            payload.get("ready") and payload.get("status") == "READY" and
            same_profile)
        if self._field_ready:
            self._field_reason = "field_ready"
        elif not same_profile and payload.get("profile"):
            self._field_reason = "field_profile_mismatch"
        else:
            self._field_reason = "field_%s" % payload.get(
                "status", "not_ready").lower()
        self._publish_status()

    def _on_adapter_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            self._adapter_status_seen = True
            self._adapter_accepting = False
            self._adapter_state = "INVALID_JSON"
            self._publish_status()
            return
        self._adapter_status_seen = True
        self._adapter_state = payload.get("state")
        self._adapter_accepting = adapter_candidate_accepting(
            payload, self._profile)
        self._publish_status()

    @staticmethod
    def _facts(target):
        return SelectorCandidate(
            target_id=int(target.id),
            class_name=target.class_name,
            confidence=float(target.class_confidence),
            geometry_confidence=float(target.geometry_confidence),
            map_quality=float(target.map_quality),
            last_seen=target.last_seen.to_sec(),
            state=int(target.state),
            consecutive_observe_count=int(
                target.consecutive_observe_count),
            map_valid=bool(target.map_valid),
            map_frame=target.map_frame,
            association_valid=bool(target.association_valid),
            reject_reason=target.reject_reason,
            x=float(target.map_point.x),
            y=float(target.map_point.y),
            z=float(target.map_point.z),
        )

    def _on_targets(self, message):
        if not self._field_ready or not self._adapter_accepting:
            self._publish_status()
            return
        by_id = {int(target.id): target for target in message.targets}
        if "tank" not in self._allowed:
            self._tank_rejected_count += sum(
                target.class_name == "tank" for target in message.targets)
        selected = select_current_candidate(
            [self._facts(target) for target in message.targets],
            rospy.Time.now().to_sec(), self._frame, self._max_age,
            self._min_streak, self._allowed, self._bounds,
            self._target_max_z)
        if selected is not None:
            self._selected_pub.publish(by_id[selected.target_id])
            self._selected_count += 1
        else:
            self._invalid_rejected_count += len(message.targets)
        self._publish_status()


if __name__ == "__main__":
    ProfileCandidateSelector()
    rospy.spin()
