#!/usr/bin/env python3
"""drop_aligner: 计算像素偏差，判定对准条件，发布 DropOffset + DropReady。"""
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

from uav_vision.msg import (
    TargetCandidate, TargetCandidateArray, DropOffset, DropReady, ReleaseEvidence,
)

CONFIRMED_STATE = 2
VALID_ALIGN_MODES = {"disabled", "drop_circle", "drop_cross", "landing"}
MODE_CLASS_MAP = {
    "drop_circle": {"circle"},
    "drop_cross": {"red_cross"},
    "landing": {"landing_pad"},
}


class DropAligner:
    def __init__(self):
        rospy.init_node("drop_aligner")

        self._align_mode_topic = rospy.get_param("~align_mode_topic", "/uav_vision/align_mode")
        self._default_mode = self._sanitize_mode(rospy.get_param("~default_mode", "disabled"))
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/camera_info")
        self._use_camera_info = bool(rospy.get_param("~use_camera_info", True))

        # 参数
        self._target_cx = rospy.get_param("~target_center_x", 640.0)
        self._target_cy = rospy.get_param("~target_center_y", 480.0)
        self._max_offset_px = rospy.get_param("~max_offset_px", 30.0)
        self._stable_frames = rospy.get_param("~stable_frames", 3)
        self._min_confidence = rospy.get_param("~min_confidence", 0.6)
        self._target_max_age = float(rospy.get_param("~target_max_age", 0.5))
        self._camera_model = PinholeCameraModel()
        self._camera_ready = False

        self._consecutive_ok = 0
        self._align_mode = self._default_mode
        self._selected_target = None

        self._offset_pub = rospy.Publisher("/uav_vision/drop_offset",
                                           DropOffset, queue_size=1)
        self._ready_pub = rospy.Publisher("/uav_vision/drop_ready",
                                          DropReady, queue_size=1)
        self._evidence_pub = rospy.Publisher("/uav_vision/release_evidence",
                                             ReleaseEvidence, queue_size=1)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray, self._on_targets)
        rospy.Subscriber(self._align_mode_topic, String, self._on_align_mode)
        rospy.Subscriber("/uav_vision/selected_target", TargetCandidate, self._on_selected_target)
        if self._use_camera_info:
            rospy.Subscriber(self._camera_info_topic, CameraInfo,
                             self._on_camera_info, queue_size=1)

        rospy.loginfo("[DropAligner] ready  target=(%.0f,%.0f)  max_offset=%.0fpx  stable=%d  mode=%s",
                      self._target_cx, self._target_cy, self._max_offset_px, self._stable_frames, self._align_mode)

    def _on_camera_info(self, msg):
        self._camera_model.fromCameraInfo(msg)
        self._target_cx = float(self._camera_model.cx())
        self._target_cy = float(self._camera_model.cy())
        self._camera_ready = True

    def _sanitize_mode(self, mode):
        return mode if mode in VALID_ALIGN_MODES else "disabled"

    def _on_align_mode(self, msg):
        new_mode = self._sanitize_mode(msg.data.strip())
        if new_mode != self._align_mode:
            self._align_mode = new_mode
            self._consecutive_ok = 0
            rospy.loginfo("[DropAligner] align mode -> %s", self._align_mode)

    def _on_selected_target(self, msg):
        self._selected_target = msg

    def _target_sort_key(self, target):
        return (target.geometry_confidence, target.class_confidence, target.observe_count)

    def _mode_reason(self):
        return {
            "disabled": "align disabled",
            "drop_circle": "no confirmed circle",
            "drop_cross": "no confirmed red_cross",
            "landing": "no confirmed landing_pad",
        }.get(self._align_mode, "invalid mode")

    def _choose_target(self, msg):
        if self._align_mode == "disabled":
            return None, "align disabled"

        allowed_classes = MODE_CLASS_MAP.get(self._align_mode, set())
        if self._align_mode == "drop_cross" and self._selected_target is not None:
            for target in msg.targets:
                if (
                    target.id == self._selected_target.id
                    and target.class_name == "red_cross"
                    and target.state >= CONFIRMED_STATE
                    and target.center_refined
                    and self._observation_age(target) <= self._target_max_age
                ):
                    return target, None

        confirmed_candidates = [
            target for target in msg.targets
            if target.class_name in allowed_classes and
            target.state >= CONFIRMED_STATE and target.center_refined
        ]
        if not confirmed_candidates:
            return None, "no confirmed refined target"

        # 地图记忆会有意保留到目标离开当前视野之后。因此投递对准必须先丢弃过期记录，再按
        # 质量排序；否则历史高质量圆环可能一直遮蔽当前位于飞机下方、质量较低的圆环。
        candidates = [
            target for target in confirmed_candidates
            if self._observation_age(target) <= self._target_max_age
        ]
        if not candidates:
            return None, "stale observation"

        candidates.sort(key=self._target_sort_key, reverse=True)
        return candidates[0], None

    def _on_targets(self, msg):
        if self._align_mode == "disabled":
            self._consecutive_ok = 0
            self._publish_state(None, False, ["align_disabled"])
            return

        if not msg.targets:
            self._consecutive_ok = 0
            self._publish_state(None, False, ["no_targets"])
            return

        best, reason = self._choose_target(msg)
        if best is None:
            self._consecutive_ok = 0
            normalized_reason = (reason or self._mode_reason()).replace(" ", "_")
            self._publish_state(None, False, [normalized_reason])
            return

        age = self._observation_age(best)
        if age > self._target_max_age:
            self._consecutive_ok = 0
            self._publish_state(best, False, ["stale_observation"])
            return

        dx = best.center_px.x - self._target_cx
        dy = best.center_px.y - self._target_cy
        dist = (dx * dx + dy * dy) ** 0.5

        # 置信度低于阈值的，不发有效偏移
        if best.class_confidence < self._min_confidence:
            self._consecutive_ok = 0
            self._publish_state(best, False, ["low_confidence"])
            return

        offset = DropOffset()
        offset.header = best.header
        offset.dx_px = dx
        offset.dy_px = dy
        offset.radius_px = best.center_px.z
        offset.quality = best.geometry_confidence
        self._offset_pub.publish(offset)

        aligned = dist <= self._max_offset_px
        if aligned:
            self._consecutive_ok += 1
        else:
            self._consecutive_ok = 0

        ready = self._consecutive_ok >= self._stable_frames
        reasons = []
        if not aligned:
            reasons.append("offset_exceeds_limit")
        elif not ready:
            reasons.append("insufficient_stable_frames")
        self._publish_state(best, aligned, reasons)

    @staticmethod
    def _observation_age(target):
        if target.last_seen.to_sec() <= 0.0:
            return float("inf")
        return max(0.0, (rospy.Time.now() - target.last_seen).to_sec())

    def _publish_state(self, target, aligned, rejection_reasons):
        evidence = ReleaseEvidence()
        evidence.header.stamp = rospy.Time.now()
        evidence.align_mode = self._align_mode
        evidence.aligned = aligned
        evidence.stable_frames = self._consecutive_ok
        evidence.rejection_reasons = rejection_reasons
        if target is not None:
            evidence.header.frame_id = target.header.frame_id
            evidence.target_present = True
            evidence.target_id = target.id
            evidence.target_class = target.class_name
            evidence.target_confirmed = target.state >= CONFIRMED_STATE
            evidence.center_refined = target.center_refined
            evidence.geometry_verified = (
                target.center_refined and
                target.geometry_confidence >= self._min_confidence
            )
            evidence.observation_age_sec = self._observation_age(target)
            evidence.observation_fresh = evidence.observation_age_sec <= self._target_max_age
        evidence.evidence_valid = (
            evidence.target_present and evidence.target_confirmed and
            evidence.geometry_verified and evidence.center_refined and
            evidence.observation_fresh and aligned and
            self._consecutive_ok >= self._stable_frames and
            not rejection_reasons
        )
        self._evidence_pub.publish(evidence)
        reason = "evidence_valid" if evidence.evidence_valid else \
            (rejection_reasons[0] if rejection_reasons else "evidence_invalid")
        self._publish_ready(evidence.evidence_valid, reason)

    def _publish_ready(self, ready, reason):
        msg = DropReady()
        msg.header.stamp = rospy.Time.now()
        msg.ready = ready
        msg.reason = reason
        self._ready_pub.publish(msg)


def main():
    DropAligner()
    rospy.spin()


if __name__ == "__main__":
    main()
