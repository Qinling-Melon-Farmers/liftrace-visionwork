#!/usr/bin/env python3
"""
Simulation helpers: mock circle detection + mock YOLO + mock Servo.
Replaces broken cv_bridge-based YOLO and missing actuator_pwm in Gazebo SITL.
Timer-driven — does not depend on camera image callbacks.
"""
import rospy
import std_msgs.msg
import geometry_msgs.msg
from patrol_control.srv import Servo, ServoResponse


class SimHelpers:
    def __init__(self):
        # ---- Circle mock ----
        self.detect_control = False
        self.detect_sub = rospy.Subscriber(
            "/detect/control", std_msgs.msg.Bool,
            self.detect_cb, queue_size=1)
        self.waypoint_pub = rospy.Publisher(
            "/detect/waypoint_mark_point", geometry_msgs.msg.PoseStamped,
            queue_size=1)

        # ---- YOLO mock ----
        self.class_control = False
        self.class_sub = rospy.Subscriber(
            "/detect/class_control", std_msgs.msg.Bool,
            self.class_cb, queue_size=1)
        self.class_pub = rospy.Publisher(
            "/yolo_detect", std_msgs.msg.String, queue_size=1)

        # Drone position + current setpoint target from MAVROS
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", geometry_msgs.msg.PoseStamped,
            self.pose_cb, queue_size=1)

        # Current target point (what patrol_control is flying to)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.setpoint_sub = rospy.Subscriber(
            "/mavros/setpoint_position/local", geometry_msgs.msg.PoseStamped,
            self.setpoint_cb, queue_size=1)

        # Timer: publish mocks at 10 Hz (independent of camera callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("[sim_helpers] Timer-driven mock ready (circle+YOLO @10Hz)")

        # ---- Servo mock ----
        self.servo_srv = rospy.Service("Servo", Servo, self.servo_cb)
        rospy.loginfo("[sim_helpers] Servo mock service ready")

    def pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_z = msg.pose.position.z

    def setpoint_cb(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z

    def detect_cb(self, msg):
        self.detect_control = msg.data

    def class_cb(self, msg):
        self.class_control = msg.data

    def timer_cb(self, event):
        # Circle mock: publish waypoint_mark_point at the current TARGET
        # point (from /mavros/setpoint_position/local).  This makes the
        # alignment error exactly 0 and lets WayPointDetectDone converge.
        if self.detect_control:
            wp = geometry_msgs.msg.PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = "map"
            wp.pose.position.x = self.target_x
            wp.pose.position.y = self.target_y
            wp.pose.position.z = self.target_z
            wp.pose.orientation.w = 1.0
            self.waypoint_pub.publish(wp)

        # YOLO mock: publish "panzer" (matches default goal list)
        if self.class_control:
            self.class_pub.publish(std_msgs.msg.String("panzer"))

    def servo_cb(self, req):
        servo_id = req.req
        rospy.loginfo("[sim_helpers] >>> SERVO CALLED: id=%d — MOCK ACK <<<", servo_id)
        return ServoResponse(res=True)


def main():
    rospy.init_node("sim_helpers", anonymous=False)
    node = SimHelpers()
    rospy.spin()


if __name__ == "__main__":
    main()
