#!/usr/bin/env python3
"""Isolated, immediate software ACK. No PWM, GPIO or hardware connection."""
import json,rospy
from std_msgs.msg import String
from patrol_control.srv import Servo,ServoResponse
if __name__=='__main__':
    rospy.init_node('board_mock_servo')
    publisher=rospy.Publisher('/board_trials/mock_release',String,queue_size=10,latch=True)
    def acknowledge(request):
        publisher.publish(String(data=json.dumps(dict(time=rospy.Time.now().to_sec(),request=request.req,simulated=True,hardware_output=False))))
        return ServoResponse(res=True)
    service=rospy.Service('/board_trials/mock_servo',Servo,acknowledge)
    rospy.logwarn('MOCK RELEASE ONLY: no real actuator; no artificial blocking release delay')
    rospy.spin()
