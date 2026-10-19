#!/usr/bin/python3
import threading

import rospy
from mavros_msgs.msg import State
from patrol_control.srv import Servo, ServoResponse
from single_drop_core import OnceRelease


class RightServoOnce:
    def __init__(self):
        self.lock = threading.Lock()
        self.once = OnceRelease(rospy.get_param("~receipt_path"))
        self.state = None
        self.state_at = 0.0
        self.raw = rospy.ServiceProxy("/legacy/Servo_raw", Servo)
        self.subscriber = rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=1)
        self.service = rospy.Service("/test_4x5/right_once", Servo, self.release)

    def on_state(self, message):
        with self.lock:
            self.state, self.state_at = message, rospy.Time.now().to_sec()

    def release(self, request):
        with self.lock:
            try:
                if (self.state is None or not self.state.connected or not self.state.armed
                        or self.state.mode != "OFFBOARD"
                        or not 0 <= rospy.Time.now().to_sec() - self.state_at <= 0.5):
                    return ServoResponse(res=False)
                if not self.once.consume(int(request.req)):
                    return ServoResponse(res=False)
                self.raw.wait_for_service(timeout=0.5)
                response = self.raw(2)
                rospy.logwarn("Single right-servo request acknowledged=%s", response.res)
                return ServoResponse(res=bool(response.res))
            except (OSError, rospy.ROSException, rospy.ServiceException) as error:
                rospy.logerr("Right-servo attempt consumed; no retry: %s", error)
                return ServoResponse(res=False)


if __name__ == "__main__":
    rospy.init_node("right_servo_once")
    RightServoOnce()
    rospy.spin()
