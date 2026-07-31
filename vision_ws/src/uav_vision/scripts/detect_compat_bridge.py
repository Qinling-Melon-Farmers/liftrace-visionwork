#!/usr/bin/env python3
"""detect_compat_bridge: 订阅 uav_vision 新接口，转发到 patrol_control 旧话题。

旧话题消息类型（与 patrol_control.cpp 实际订阅一致）：
  /yolo_detect           → std_msgs::String       (单个类别名，与 goal[] 逐一比较)
  /detect/waypoint_mark_point → geometry_msgs::PoseStamped  (pose.position)
  /detect/tank_status    → geometry_msgs::PoseStamped  (pose.position)
  /detect/cross_mark_point    → geometry_msgs::PoseStamped  (pose.position)
  /detect/cross_status   → std_msgs::Bool

注意：新链路的 `drop_offset` / `detections.center_px` 是图像域结果，不等价于旧世界系 Pose。
因此这些 Pose 兼容输出默认关闭，仅在显式 `publish_pixel_pose_compat:=true` 时用于临时调试。
"""
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

from uav_vision.msg import TargetDetectionArray, DropOffset, DropReady

STANDARD_TARGET_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}


class DetectCompatBridge:
    def __init__(self):
        rospy.init_node("detect_compat_bridge")
        self._detections_topic = rospy.get_param("~detections_topic", "/uav_vision/detections")
        self._publish_pixel_pose_compat = rospy.get_param("~publish_pixel_pose_compat", False)
        self._suppress_bridge_on_red_cross = rospy.get_param("~suppress_bridge_on_red_cross", True)
        self._suppress_bridge_on_landing_pad = rospy.get_param("~suppress_bridge_on_landing_pad", True)
        self._aux_geometry_confidence = rospy.get_param("~aux_geometry_confidence", 0.85)

        # 订阅新接口
        rospy.Subscriber(self._detections_topic, TargetDetectionArray,
                         self._on_detections)
        rospy.Subscriber("/uav_vision/drop_offset", DropOffset,
                         self._on_drop_offset)
        rospy.Subscriber("/uav_vision/drop_ready", DropReady,
                         self._on_drop_ready)

        # 旧话题发布 — 类型与 patrol_control 订阅一致
        self._yolo_detect_pub = rospy.Publisher("/yolo_detect",
                                                String, queue_size=1)
        self._waypoint_pub = rospy.Publisher("/detect/waypoint_mark_point",
                                             PoseStamped, queue_size=1)
        self._tank_status_pub = rospy.Publisher("/detect/tank_status",
                                                PoseStamped, queue_size=1)
        self._cross_mark_pub = rospy.Publisher("/detect/cross_mark_point",
                                               PoseStamped, queue_size=1)
        self._cross_status_pub = rospy.Publisher("/detect/cross_status",
                                                 Bool, queue_size=1)
        self._land_mark_pub = rospy.Publisher("/detect/land_mark_point",
                                              PoseStamped, queue_size=1)

        rospy.loginfo("[CompatBridge] ready  detections_topic=%s  publish_pixel_pose_compat=%s  suppress_bridge_on_red_cross=%s  suppress_bridge_on_landing_pad=%s",
                      self._detections_topic,
                      self._publish_pixel_pose_compat,
                      self._suppress_bridge_on_red_cross,
                      self._suppress_bridge_on_landing_pad)

    # ------------------------------------------------------------------
    def _on_detections(self, msg):
        has_tank = False
        tank_pose = None
        has_cross = False
        cross_pose = None
        has_landing = False
        landing_pose = None
        suppress_bridge = False

        for det in msg.detections:
            if det.class_name == "red_cross" and det.geometry_verified and \
               det.geometry_confidence >= self._aux_geometry_confidence and \
               self._suppress_bridge_on_red_cross:
                suppress_bridge = True
            if det.class_name == "landing_pad" and det.geometry_verified and \
               det.geometry_confidence >= self._aux_geometry_confidence and \
               self._suppress_bridge_on_landing_pad:
                suppress_bridge = True

        # 选出最可信标准目标用于 /yolo_detect（单类别名 String）
        # 旧接口只承担“标准目标分类”语义，不混入 cross/circle/landing。
        best_class = None
        best_conf = -1.0
        for det in msg.detections:
            if suppress_bridge and det.class_name == "bridge":
                continue
            if (det.class_name in STANDARD_TARGET_CLASSES and
                    det.geometry_verified and det.center_refined and
                    det.class_confidence > best_conf):
                best_conf = det.class_confidence
                best_class = det.class_name

            if det.class_name == "tank" and det.geometry_verified and det.center_refined:
                has_tank = True
                tank_pose = det.center_px
            if det.class_name == "red_cross":
                has_cross = True
                cross_pose = det.center_px
            if det.class_name == "landing_pad":
                has_landing = True
                landing_pose = det.center_px

        # /yolo_detect — std_msgs::String
        yolo_str = String()
        yolo_str.data = best_class if best_class is not None else "Nothing"
        self._yolo_detect_pub.publish(yolo_str)

        # /detect/tank_status — geometry_msgs::PoseStamped
        if self._publish_pixel_pose_compat and has_tank and tank_pose is not None:
            ts = PoseStamped()
            ts.header = msg.header
            ts.pose.position.x = tank_pose.x
            ts.pose.position.y = tank_pose.y
            ts.pose.position.z = 0
            ts.pose.orientation.w = 1.0
            self._tank_status_pub.publish(ts)

        # /detect/cross_status — std_msgs::Bool
        cs = Bool()
        cs.data = has_cross
        self._cross_status_pub.publish(cs)

        # /detect/cross_mark_point — geometry_msgs::PoseStamped
        if self._publish_pixel_pose_compat and has_cross and cross_pose is not None:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = cross_pose.x
            pose.pose.position.y = cross_pose.y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            self._cross_mark_pub.publish(pose)

        # /detect/land_mark_point — geometry_msgs::PoseStamped
        if self._publish_pixel_pose_compat and has_landing and landing_pose is not None:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = landing_pose.x
            pose.pose.position.y = landing_pose.y
            pose.pose.position.z = landing_pose.z
            pose.pose.orientation.w = 1.0
            self._land_mark_pub.publish(pose)

    # ------------------------------------------------------------------
    def _on_drop_offset(self, msg):
        if not self._publish_pixel_pose_compat:
            return
        # /detect/waypoint_mark_point — geometry_msgs::PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.dx_px
        pose.pose.position.y = msg.dy_px
        pose.pose.position.z = msg.radius_px
        pose.pose.orientation.w = msg.quality
        self._waypoint_pub.publish(pose)

    def _on_drop_ready(self, msg):
        pass  # 旧接口无对应话题，预留


def main():
    DetectCompatBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
