#!/usr/bin/env python3
"""detect_compat_bridge: 订阅 uav_vision 新接口，转发到 patrol_control 旧话题。

旧话题消息类型（与 patrol_control.cpp 实际订阅一致）：
  /yolo_detect           → std_msgs::String       (单个类别名，与 goal[] 逐一比较)
  /detect/waypoint_mark_point → geometry_msgs::PoseStamped  (pose.position)
  /detect/tank_status    → geometry_msgs::PoseStamped  (pose.position)
  /detect/cross_mark_point    → geometry_msgs::PoseStamped  (pose.position)
  /detect/cross_status   → std_msgs::Bool
  /detect/land_mark_point → geometry_msgs::PoseStamped  (pose.position)

注意：新链路的 `drop_offset` / `detections.center_px` 是图像域结果，不等价于旧世界系 Pose。
因此这些像素 Pose 兼容输出默认关闭，仅在显式 `publish_pixel_pose_compat:=true` 时用于临时调试。

2026 起补充：`detections_mapped` 的 `map_point` 是真实地图坐标。本节点按旧链语义恢复：
  - `/detect/control` 为 true 时，把当前可见圆环的地图点发布到 `/detect/waypoint_mark_point`；
  - `/detect/landing_control` 为 true 时，把 H 的地图点发布到 `/detect/land_mark_point`。
两者默认开启（`publish_circle_mark_compat` / `publish_landing_mark_compat`），
且只发布 `map_valid` 的地图点，不把像素伪装成世界坐标。
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
        # 地图点兼容开关：恢复旧链 circle/landing 检测器的地图点输出语义。
        self._publish_circle_mark_compat = rospy.get_param("~publish_circle_mark_compat", True)
        self._publish_landing_mark_compat = rospy.get_param("~publish_landing_mark_compat", True)

        self._detect_control = False
        self._landing_control = False
        self._latest = None  # (header, best_circle_map, best_landing_map)

        # 订阅新接口
        rospy.Subscriber(self._detections_topic, TargetDetectionArray,
                         self._on_detections)
        rospy.Subscriber("/uav_vision/drop_offset", DropOffset,
                         self._on_drop_offset)
        rospy.Subscriber("/uav_vision/drop_ready", DropReady,
                         self._on_drop_ready)
        # 旧控制自己的检测开关，镜像旧检测器语义
        rospy.Subscriber("/detect/control", Bool, self._on_detect_control)
        rospy.Subscriber("/detect/landing_control", Bool, self._on_landing_control)

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

        rospy.loginfo("[CompatBridge] ready  detections_topic=%s  publish_pixel_pose_compat=%s  "
                      "circle_mark_compat=%s  landing_mark_compat=%s",
                      self._detections_topic,
                      self._publish_pixel_pose_compat,
                      self._publish_circle_mark_compat,
                      self._publish_landing_mark_compat)

    # ------------------------------------------------------------------
    def _on_detect_control(self, msg):
        self._detect_control = bool(msg.data)
        self._publish_map_marks()

    def _on_landing_control(self, msg):
        self._landing_control = bool(msg.data)
        self._publish_map_marks()

    @staticmethod
    def _best_mapped(detections, class_name):
        """返回 map_valid 且几何有效的该类检测中质量最高者。"""
        best = None
        best_conf = -1.0
        for det in detections:
            if det.class_name != class_name:
                continue
            if not det.map_valid or not det.geometry_verified:
                continue
            if det.geometry_confidence > best_conf:
                best_conf = det.geometry_confidence
                best = det
        return best

    def _publish_map_marks(self):
        """按旧控制开关发布圆环/H 的地图点（仅 map_valid 检测）。"""
        if self._latest is None:
            return
        header, circle_det, landing_det = self._latest
        if (self._publish_circle_mark_compat and self._detect_control and
                circle_det is not None):
            pose = PoseStamped()
            pose.header.stamp = circle_det.header.stamp
            pose.header.frame_id = circle_det.map_frame
            pose.pose.position = circle_det.map_point
            pose.pose.orientation.w = 1.0
            self._waypoint_pub.publish(pose)
        if (self._publish_landing_mark_compat and self._landing_control and
                landing_det is not None):
            pose = PoseStamped()
            pose.header.stamp = landing_det.header.stamp
            pose.header.frame_id = landing_det.map_frame
            pose.pose.position = landing_det.map_point
            pose.pose.orientation.w = 1.0
            self._land_mark_pub.publish(pose)

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

        # 记录最新地图点候选，供控制开关触发时发布
        circle_det = self._best_mapped(msg.detections, "circle")
        landing_det = self._best_mapped(msg.detections, "landing_pad")
        self._latest = (msg.header, circle_det, landing_det)
        self._publish_map_marks()

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

        # /detect/land_mark_point — geometry_msgs::PoseStamped（像素调试路径；
        # 正式路径使用上方的地图点输出）
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
