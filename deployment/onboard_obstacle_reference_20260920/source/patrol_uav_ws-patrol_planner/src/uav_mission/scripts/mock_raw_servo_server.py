#!/usr/bin/env python3
"""Simulation-only raw Servo endpoint; never touches PWM or hardware."""
import rospy
from std_msgs.msg import UInt8

from patrol_control.srv import Servo, ServoResponse


class MockRawServoServer:
    def __init__(self):
        rospy.init_node("mock_raw_servo_server")
        service_name = rospy.get_param("~service_name", "/legacy/Servo_raw")
        self._success = bool(rospy.get_param("~success", True))
        self._delay = float(rospy.get_param("~delay", 0.0))
        calls_topic = rospy.get_param(
            "~calls_topic", "/uav_mission/mock_raw_servo_calls")
        self._calls_pub = rospy.Publisher(calls_topic, UInt8, queue_size=10)
        self._service = rospy.Service(service_name, Servo, self._on_request)
        rospy.loginfo(
            "[MockRawServo] ready service=%s success=%s",
            service_name, self._success)

    def _on_request(self, request):
        slot = int(request.req)
        if self._delay > 0.0:
            rospy.sleep(self._delay)
        self._calls_pub.publish(UInt8(data=slot))
        valid = 1 <= slot <= 3
        rospy.loginfo(
            "[MockRawServo] slot=%d result=%s", slot,
            self._success and valid)
        return ServoResponse(res=self._success and valid)


def main():
    MockRawServoServer()
    rospy.spin()


if __name__ == "__main__":
    main()
