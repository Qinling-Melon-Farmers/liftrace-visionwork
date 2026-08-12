#!/usr/bin/env python3
"""Finish the fixed-drop SITL Gate after exactly three guarded ACKs."""

import sys
import time

import rospy
from std_msgs.msg import UInt8

from uav_mission.msg import ReleaseResult


class FixedThreeDropAssertion:
    def __init__(self):
        rospy.init_node("fixed_three_drop_assertion")
        self._expected = [1, 2, 3]
        self._raw_calls = []
        self._successes = []
        self._failure = ""
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 600.0))
        rospy.Subscriber("/uav_mission/mock_raw_servo_calls", UInt8,
                         self._on_raw_call, queue_size=8)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_result, queue_size=12)

    def _fail(self, reason):
        self._failure = reason
        rospy.logerr("[FixedThreeDropAssertion] FAIL: %s", reason)
        rospy.signal_shutdown(reason)

    def _on_raw_call(self, msg):
        self._raw_calls.append(int(msg.data))
        if self._raw_calls != self._expected[:len(self._raw_calls)]:
            self._fail("raw_calls_%s" % self._raw_calls)

    def _on_result(self, msg):
        if msg.success:
            self._successes.append(int(msg.payload_slot))
            if self._successes != self._expected[:len(self._successes)]:
                self._fail("success_slots_%s" % self._successes)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._raw_calls == self._expected and \
                    self._successes == self._expected:
                rospy.loginfo(
                    "[FixedThreeDropAssertion] PASS guarded_ack_slots=%s",
                    self._successes)
                return 0
            if time.monotonic() >= self._deadline:
                self._fail("wall_timeout raw=%s success=%s" %
                           (self._raw_calls, self._successes))
                return 1
            rate.sleep()
        return 1 if self._failure else 0


if __name__ == "__main__":
    sys.exit(FixedThreeDropAssertion().run())
