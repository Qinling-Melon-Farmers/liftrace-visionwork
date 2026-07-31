#!/usr/bin/env python3
"""
Simulation helpers: mock circle detection + dynamic YOLO class + mock Servo.

Replaces broken cv_bridge-based YOLO and missing actuator_pwm in Gazebo SITL.
Timer-driven — does not depend on camera image callbacks.

YOLO mock is DYNAMIC: publishes the class of the nearest standard target to
the current reference point (setpoint target or drone position), instead of
a hardcoded "panzer".  All parameters come from private nh (~), no hardcoded
internal parameters (AGENTS.md rule 7).

Parameters (all ~):
  target_table   XmlRpc array of [name, class, x, y]  (world truth table)
  class_radius   publish class only if nearest target within this distance (m)
  class_reference setpoint | drone   (default: setpoint)
  circle_mode    setpoint | target_true  (default: setpoint)
  servo_mock_enable  bool (default: true; disable in guarded-chain runs)
"""
import math

import rospy
import std_msgs.msg
import geometry_msgs.msg
from patrol_control.srv import Servo, ServoResponse

# toudi3.world 内嵌标准靶真值（世界绝对坐标, 来自 compute_free_waypoints.py）
# 类别名与 merged_standard_6cls_metadata.yaml 一致
DEFAULT_TARGET_TABLE = [
    # (name, class, x, y)
    ("dibao",        "pillbox", -0.602, -1.041),
    ("qiaoliang",    "bridge",  -1.903, -0.023),
    ("zhangpeng",    "tent",     1.016,  0.256),
    ("zhuangjiache", "panzer",  -1.589,  3.022),
    ("tanke",        "tank",     0.283,  3.856),
]


class SimHelpers:
    def __init__(self):
        pnh = rospy.get_param("~", {})
        self.class_radius = float(pnh.get("class_radius", 1.2))
        self.class_reference = pnh.get("class_reference", "setpoint")
        self.circle_mode = pnh.get("circle_mode", "setpoint")
        self.servo_mock_enable = bool(pnh.get("servo_mock_enable", True))

        # 目标真值表：支持 ~target_table 覆盖
        table = pnh.get("target_table", None)
        if table:
            self.target_table = []
            for row in table:
                if len(row) >= 4:
                    self.target_table.append((row[0], row[1], float(row[2]), float(row[3])))
        else:
            self.target_table = DEFAULT_TARGET_TABLE
        rospy.loginfo("[sim_helpers] target_table: %d entries, radius=%.2fm, ref=%s",
                      len(self.target_table), self.class_radius, self.class_reference)

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

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.setpoint_sub = rospy.Subscriber(
            "/mavros/setpoint_position/local", geometry_msgs.msg.PoseStamped,
            self.setpoint_cb, queue_size=1)

        # Timer: publish mocks at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("[sim_helpers] Timer-driven mock ready (circle+YOLO @10Hz)")

        # ---- Servo mock (可关) ----
        if self.servo_mock_enable:
            self.servo_srv = rospy.Service("Servo", Servo, self.servo_cb)
            rospy.loginfo("[sim_helpers] Servo mock service ready")
        else:
            rospy.loginfo("[sim_helpers] Servo mock DISABLED (guarded chain provides /Servo)")

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

    def _nearest_target(self):
        """返回 (类别, 距离) 最近靶标；无则 (None, inf)。"""
        if self.class_reference == "drone":
            rx, ry = self.drone_x, self.drone_y
        else:
            rx, ry = self.target_x, self.target_y
        best_cls, best_dist = None, float("inf")
        for _name, cls, tx, ty in self.target_table:
            d = math.hypot(rx - tx, ry - ty)
            if d < best_dist:
                best_cls, best_dist = cls, d
        return best_cls, best_dist

    def timer_cb(self, event):
        # ---- Circle mock ----
        if self.detect_control:
            wp = geometry_msgs.msg.PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = "map"
            if self.circle_mode == "target_true":
                # 严格模式：发布最近靶标真值坐标
                cls, dist = self._nearest_target()
                if cls is not None and dist <= self.class_radius:
                    for _name, _cls, tx, ty in self.target_table:
                        if _cls == cls:
                            wp.pose.position.x = tx
                            wp.pose.position.y = ty
                            break
                else:
                    wp.pose.position.x = self.target_x
                    wp.pose.position.y = self.target_y
            else:
                # 默认：发布当前目标点，对准误差恒 0（状态机最稳）
                wp.pose.position.x = self.target_x
                wp.pose.position.y = self.target_y
            wp.pose.position.z = self.target_z
            wp.pose.orientation.w = 1.0
            self.waypoint_pub.publish(wp)

        # ---- YOLO mock: 就近类别（动态，不写死） ----
        if self.class_control:
            cls, dist = self._nearest_target()
            if cls is not None and dist <= self.class_radius:
                self.class_pub.publish(std_msgs.msg.String(cls))
            else:
                self.class_pub.publish(std_msgs.msg.String("Nothing"))

    def servo_cb(self, req):
        rospy.loginfo("[sim_helpers] >>> SERVO CALLED: id=%d — MOCK ACK <<<", req.req)
        return ServoResponse(res=True)


def main():
    rospy.init_node("sim_helpers", anonymous=False)
    node = SimHelpers()
    rospy.spin()


if __name__ == "__main__":
    main()
