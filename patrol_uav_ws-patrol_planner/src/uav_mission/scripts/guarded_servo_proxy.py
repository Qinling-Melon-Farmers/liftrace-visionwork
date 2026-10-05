#!/usr/bin/env python3
"""Fence the unchanged bool Servo service; never retry an uncertain raw call."""
import threading
import time

import rospy
from patrol_control.srv import Servo, ServoResponse, ServoAction, ServoActionResponse
from uav_mission.msg import ReleasePermission, ReleaseResult
from uav_vision.msg import AlignmentTargetContext
from uav_mission.release_transactions import (
    EXECUTION_UNKNOWN, NOT_STARTED, RAW_CALL_STARTED, COMPLETED, action_identity,
)


class GuardedServoProxy:
    def __init__(self):
        rospy.init_node("guarded_servo_proxy")
        self._service_name = rospy.get_param("~service_name", "/Servo")
        self._raw_service_name = rospy.get_param("~raw_service_name", "/legacy/Servo_raw")
        self._raw_wait_timeout = float(rospy.get_param("~raw_service_wait_timeout", 0.25))
        self._permission_max_age = float(rospy.get_param("~permission_max_age", 0.5))
        self._permission_refresh_wait = float(rospy.get_param("~permission_refresh_wait", 0.25))
        if not 0.0 <= self._permission_refresh_wait <= 1.0:
            raise ValueError("permission_refresh_wait must be in [0, 1] wall seconds")
        self._permission = None
        self._consumed_permission_stamp = rospy.Time(0)
        self._completed_slots = set()
        self._locked_slots = set()
        self._locked_actions = set()
        self._inflight_slots = set()
        self._revoked_actions = set()
        self._execution_id = 0
        self._lock = threading.RLock()
        self._permission_changed = threading.Condition(self._lock)
        self._result_pub = rospy.Publisher(rospy.get_param(
            "~result_topic", "/mission/release_result"), ReleaseResult, queue_size=8)
        rospy.Subscriber(rospy.get_param("~permission_topic", "/mission/release_permission"),
                         ReleasePermission, self._on_permission, queue_size=2)
        rospy.Subscriber(rospy.get_param("~alignment_context_topic",
                         "/uav_vision/alignment_target_context"), AlignmentTargetContext,
                         self._on_alignment_context, queue_size=4)
        self._raw_client = rospy.ServiceProxy(self._raw_service_name, Servo)
        self._service = rospy.Service(self._service_name, Servo, self._on_servo_request)
        self._action_service = rospy.Service(rospy.get_param(
            "~action_service_name", "/mission/servo_action"), ServoAction,
            self._on_servo_action_request)
        rospy.loginfo("[GuardedServo] ready public=%s raw=%s", self._service_name,
                      self._raw_service_name)

    def _on_permission(self, msg):
        with self._lock:
            self._permission = msg
            self._permission_changed.notify_all()

    def _on_alignment_context(self, msg):
        if msg.active:
            return
        # The same lock serializes revoke vs raw-call entry. A cancellation ACK
        # is therefore positive proof that this fenced action cannot start later.
        permission = ReleasePermission()
        permission.payload_slot = msg.payload_slot
        permission.align_mode = msg.align_mode
        permission.target_id = msg.semantic_target_id
        permission.target_class = msg.semantic_target_class
        permission.mission_id = msg.mission_id
        permission.decision_seq = msg.decision_seq
        permission.attempt = msg.attempt
        permission.target_first_seen = msg.semantic_target_first_seen
        key = action_identity(permission)
        if key is None:
            return
        with self._lock:
            if key in self._revoked_actions or key in self._locked_actions:
                return
            self._revoked_actions.add(key)
            self._permission_changed.notify_all()
            self._publish_result(msg.payload_slot, False, "alignment_context_revoked",
                                 permission, NOT_STARTED)

    def _permission_reason(self, slot, now, permission=None, own_reservation=False):
        permission = self._permission if permission is None else permission
        if slot < 1 or slot > 3:
            return "payload_slot_invalid"
        if slot in self._completed_slots:
            return "payload_slot_already_used"
        if slot in self._locked_slots:
            return "payload_slot_execution_uncertain"
        if slot in self._inflight_slots and not own_reservation:
            return "payload_slot_busy"
        if permission is None:
            return "permission_missing"
        key = action_identity(permission)
        if key in self._locked_actions:
            return "action_already_called"
        if key in self._revoked_actions:
            return "action_revoked"
        if not permission.permitted:
            return permission.reason or "permission_denied"
        if permission.payload_slot != slot:
            return "payload_slot_mismatch"
        if permission.header.stamp <= self._consumed_permission_stamp:
            return "permission_already_consumed"
        if permission.valid_until.to_sec() <= 0.0 or now > permission.valid_until:
            return "permission_expired"
        age = (now - permission.header.stamp).to_sec()
        if age < 0.0:
            return "permission_clock_ahead"
        if permission.header.stamp.to_sec() <= 0.0 or age > self._permission_max_age:
            return "permission_stale"
        if permission.align_mode not in ("drop_circle", "drop_cross"):
            return "permission_mode_invalid"
        return ""

    def _publish_result(self, slot, success, reason, permission,
                        state=NOT_STARTED, execution_id=None, terminal=True):
        if execution_id is None:
            self._execution_id += 1
            execution_id = self._execution_id
        result = ReleaseResult()
        result.header.stamp = rospy.Time.now()
        result.execution_id = execution_id
        result.payload_slot = slot
        result.success = success
        result.reason = reason
        result.execution_state = state
        result.terminal = terminal
        if permission is not None:
            for field in ("align_mode", "target_id", "target_class", "mission_id",
                          "decision_seq", "attempt", "target_first_seen", "evidence_stamp"):
                setattr(result, field, getattr(permission, field, getattr(result, field)))
        self._result_pub.publish(result)
        return execution_id

    def _on_servo_request(self, request):
        success, _, _ = self._execute_request(int(request.req))
        return ServoResponse(res=success)

    def _on_servo_action_request(self, request):
        success, state, reason = self._execute_request(int(request.payload_slot), request)
        return ServoActionResponse(request_id=request.request_id,
            payload_slot=request.payload_slot, res=success, execution_state=state,
            terminal=True, reason=reason)

    def _await_permission(self, slot, fence=None, own_reservation=False,
                          previous=None):
        """Wait only for temporal admission of the SAME fenced action.

        Caller holds _lock. Condition.wait releases it, so new permits and
        cancellation remain responsive. This is the service worker, never the
        control timer. A future-dated permit is never accepted early.
        """
        deadline = time.monotonic() + self._permission_refresh_wait
        while True:
            permission = self._permission
            reason = self._permission_reason(slot, rospy.Time.now(), permission,
                                             own_reservation=own_reservation)
            if permission is not None and fence is not None:
                if (action_identity(fence) is None or
                        action_identity(fence) != action_identity(permission) or
                        str(fence.align_mode) != str(permission.align_mode)):
                    return permission, "request_action_identity_mismatch"
            if previous is not None and permission is not previous:
                if (permission is None or not permission.permitted or
                        action_identity(permission) != action_identity(previous) or
                        permission.align_mode != previous.align_mode or
                        permission.target_id != previous.target_id or
                        permission.target_class != previous.target_class):
                    return permission, "permission_changed_before_call"
            # Legacy unfenced calls keep their immediate admission semantics.
            # Invalid geometry, revoked context, used slots and unknown
            # executions are never made retryable by this small clock wait.
            temporal = reason in ("permission_clock_ahead", "permission_expired",
                                  "permission_stale")
            remaining = deadline - time.monotonic()
            if (not temporal or fence is None or permission is None or
                    not permission.permitted or permission.header.stamp.to_sec() <= 0.0 or
                    remaining <= 0.0):
                return permission, reason
            self._permission_changed.wait(min(remaining, 0.01))

    def _execute_request(self, slot, fence=None):
        with self._lock:
            permission, reason = self._await_permission(slot, fence)
            if reason:
                # A rejected duplicate proves nothing about a previous call.
                if slot not in self._locked_slots and slot not in self._inflight_slots:
                    self._publish_result(slot, False, reason,
                                         fence if fence is not None else permission, NOT_STARTED)
                    return False, NOT_STARTED, reason
                return False, EXECUTION_UNKNOWN, reason
            self._inflight_slots.add(slot)
        try:
            # Discovery failure is distinguishable from a failed RPC after entry.
            self._raw_client.wait_for_service(timeout=self._raw_wait_timeout)
        except (rospy.ROSException, rospy.ServiceException):
            with self._lock:
                self._inflight_slots.discard(slot)
                self._consumed_permission_stamp = permission.header.stamp
                self._publish_result(slot, False, "raw_service_not_available",
                                     permission, NOT_STARTED)
            return False, NOT_STARTED, "raw_service_not_available"
        with self._lock:
            permission, reason = self._await_permission(
                slot, fence, own_reservation=True, previous=permission)
            if reason:
                self._inflight_slots.discard(slot)
                self._publish_result(slot, False, reason, permission, NOT_STARTED)
                return False, NOT_STARTED, reason
            self._consumed_permission_stamp = permission.header.stamp
            self._locked_slots.add(slot)
            key = action_identity(permission)
            if key is not None:
                self._locked_actions.add(key)
            execution_id = self._publish_result(slot, False, "raw_actuator_call_started",
                permission, RAW_CALL_STARTED, terminal=False)
        # No shared lock is held across the slow raw RPC. Neither a renewed
        # permit nor another callback can unlock this action or slot.
        try:
            success = bool(self._raw_client(slot).res)
            reason = "raw_actuator_ack" if success else "raw_actuator_failed_unknown"
        except (rospy.ROSException, rospy.ServiceException):
            success = False
            reason = "raw_actuator_rpc_failed_unknown"
        with self._lock:
            self._inflight_slots.discard(slot)
            if success:
                self._completed_slots.add(slot)
            self._publish_result(slot, success, reason, permission,
                                 COMPLETED if success else RAW_CALL_STARTED,
                                 execution_id=execution_id)
        return success, COMPLETED if success else RAW_CALL_STARTED, reason


def main():
    GuardedServoProxy()
    rospy.spin()


if __name__ == "__main__":
    main()
