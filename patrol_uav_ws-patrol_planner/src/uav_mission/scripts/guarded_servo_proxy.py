#!/usr/bin/env python3
"""Guard the legacy patrol_control/Servo service with release permission.

The old controller continues to call /Servo with sequential payload slots.
Only a fresh, matching /mission/release_permission is forwarded to the raw
legacy actuator service.  This node is fail-closed and consumes a permission
after one call, including failed raw calls.
"""
import threading

import rospy

from patrol_control.srv import Servo, ServoResponse
from uav_mission.msg import ReleasePermission, ReleaseResult


class GuardedServoProxy:
    def __init__(self):
        rospy.init_node("guarded_servo_proxy")
        self._service_name = rospy.get_param("~service_name", "/Servo")
        self._raw_service_name = rospy.get_param(
            "~raw_service_name", "/legacy/Servo_raw")
        permission_topic = rospy.get_param(
            "~permission_topic", "/mission/release_permission")
        result_topic = rospy.get_param(
            "~result_topic", "/mission/release_result")
        self._raw_wait_timeout = float(
            rospy.get_param("~raw_service_wait_timeout", 0.25))
        self._permission_max_age = float(
            rospy.get_param("~permission_max_age", 0.5))

        self._permission = None
        self._consumed_permission_stamp = rospy.Time(0)
        self._completed_slots = set()
        self._execution_id = 0
        self._lock = threading.Lock()

        self._result_pub = rospy.Publisher(
            result_topic, ReleaseResult, queue_size=4)
        rospy.Subscriber(permission_topic, ReleasePermission,
                         self._on_permission, queue_size=2)
        self._raw_client = rospy.ServiceProxy(self._raw_service_name, Servo)
        self._service = rospy.Service(
            self._service_name, Servo, self._on_servo_request)
        rospy.loginfo(
            "[GuardedServo] ready public=%s raw=%s",
            self._service_name, self._raw_service_name)

    def _on_permission(self, msg):
        with self._lock:
            self._permission = msg

    def _permission_reason(self, slot, now):
        permission = self._permission
        if slot < 1 or slot > 3:
            return "payload_slot_invalid"
        if slot in self._completed_slots:
            return "payload_slot_already_used"
        if permission is None:
            return "permission_missing"
        if not permission.permitted:
            return permission.reason or "permission_denied"
        if permission.payload_slot != slot:
            return "payload_slot_mismatch"
        if permission.header.stamp <= self._consumed_permission_stamp:
            return "permission_already_consumed"
        if permission.valid_until.to_sec() <= 0.0 or now > permission.valid_until:
            return "permission_expired"
        if permission.header.stamp.to_sec() <= 0.0 or \
                (now - permission.header.stamp).to_sec() > self._permission_max_age:
            return "permission_stale"
        if permission.align_mode not in ("drop_circle", "drop_cross"):
            return "permission_mode_invalid"
        return ""

    def _publish_result(self, slot, success, reason, permission):
        self._execution_id += 1
        result = ReleaseResult()
        result.header.stamp = rospy.Time.now()
        result.execution_id = self._execution_id
        result.payload_slot = slot
        result.success = success
        result.reason = reason
        if permission is not None:
            result.align_mode = permission.align_mode
            result.target_id = permission.target_id
            result.target_class = permission.target_class
        self._result_pub.publish(result)

    def _on_servo_request(self, request):
        slot = int(request.req)
        with self._lock:
            now = rospy.Time.now()
            permission = self._permission
            reason = self._permission_reason(slot, now)
            if reason:
                rospy.logwarn(
                    "[GuardedServo] denied slot=%d reason=%s", slot, reason)
                self._publish_result(slot, False, reason, permission)
                return ServoResponse(res=False)

            # Consume before calling raw hardware so concurrent/replayed calls
            # cannot reuse the same permission.
            self._consumed_permission_stamp = permission.header.stamp

            try:
                self._raw_client.wait_for_service(timeout=self._raw_wait_timeout)
                raw_response = self._raw_client(slot)
                success = bool(raw_response.res)
                reason = "raw_actuator_ack" if success else "raw_actuator_rejected"
            except (rospy.ROSException, rospy.ServiceException) as exc:
                success = False
                reason = "raw_actuator_unavailable"
                rospy.logerr("[GuardedServo] raw service failed: %s", exc)

            if success:
                self._completed_slots.add(slot)
            self._publish_result(slot, success, reason, permission)
            return ServoResponse(res=success)


def main():
    GuardedServoProxy()
    rospy.spin()


if __name__ == "__main__":
    main()
