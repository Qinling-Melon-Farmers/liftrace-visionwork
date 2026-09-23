#!/usr/bin/env python3
"""Phase D mock 回归断言节点。

根据参数等待指定话题条件出现，满足则 0 退出，超时则 1 退出。
用于验证当前 Phase D 软链路，而不是替代完整实机/板端验证。
"""
import sys

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, String

from uav_vision.msg import DropReady, TargetCandidate, TargetCandidateArray


def _parse_tristate(raw):
    if raw is None:
        return None
    text = str(raw).strip().lower()
    if text in ("", "ignore", "none"):
        return None
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise ValueError("invalid tristate value: %r" % raw)


class PhaseDAssertion:
    def __init__(self):
        rospy.init_node("phase_d_assertion")
        self._timeout = float(rospy.get_param("~timeout", 8.0))
        self._expect_target_class = rospy.get_param("~expect_target_class", "")
        self._expect_selected_class = rospy.get_param("~expect_selected_class", "")
        self._expect_align_mode = rospy.get_param("~expect_align_mode", "")
        self._expect_yolo_detect = rospy.get_param("~expect_yolo_detect", "")
        self._expect_drop_ready = _parse_tristate(rospy.get_param("~expect_drop_ready", "ignore"))
        self._expect_cross_status = _parse_tristate(rospy.get_param("~expect_cross_status", "ignore"))
        self._expect_perf_name = rospy.get_param("~expect_perf_name", "")
        self._expect_perf_backend = rospy.get_param("~expect_perf_backend", "")

        self._target_ok = (self._expect_target_class == "")
        self._selected_ok = (self._expect_selected_class == "")
        self._align_ok = (self._expect_align_mode == "")
        self._yolo_ok = (self._expect_yolo_detect == "")
        self._drop_ready_ok = (self._expect_drop_ready is None)
        self._cross_status_ok = (self._expect_cross_status is None)
        self._perf_name_ok = (self._expect_perf_name == "")
        self._perf_backend_ok = (self._expect_perf_backend == "")

        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray, self._on_targets, queue_size=2)
        rospy.Subscriber("/uav_vision/selected_target", TargetCandidate, self._on_selected, queue_size=2)
        rospy.Subscriber("/uav_vision/align_mode", String, self._on_align_mode, queue_size=2)
        rospy.Subscriber("/uav_vision/drop_ready", DropReady, self._on_drop_ready, queue_size=2)
        rospy.Subscriber("/yolo_detect", String, self._on_yolo_detect, queue_size=2)
        rospy.Subscriber("/detect/cross_status", Bool, self._on_cross_status, queue_size=2)
        rospy.Subscriber("/uav_vision/perf", DiagnosticArray, self._on_perf, queue_size=2)

        rospy.loginfo(
            "[PhaseDAssertion] timeout=%.1fs target=%s selected=%s align=%s yolo=%s drop_ready=%s cross_status=%s perf_name=%s perf_backend=%s",
            self._timeout,
            self._expect_target_class or "<ignore>",
            self._expect_selected_class or "<ignore>",
            self._expect_align_mode or "<ignore>",
            self._expect_yolo_detect or "<ignore>",
            self._expect_drop_ready if self._expect_drop_ready is not None else "<ignore>",
            self._expect_cross_status if self._expect_cross_status is not None else "<ignore>",
            self._expect_perf_name or "<ignore>",
            self._expect_perf_backend or "<ignore>",
        )

    def _on_targets(self, msg):
        if self._target_ok:
            return
        for target in msg.targets:
            if target.class_name == self._expect_target_class:
                self._target_ok = True
                rospy.loginfo("[PhaseDAssertion] target observed: %s", self._expect_target_class)
                return

    def _on_selected(self, msg):
        if self._selected_ok:
            return
        if msg.class_name == self._expect_selected_class:
            self._selected_ok = True
            rospy.loginfo("[PhaseDAssertion] selected_target observed: %s", self._expect_selected_class)

    def _on_align_mode(self, msg):
        if self._align_ok:
            return
        if msg.data == self._expect_align_mode:
            self._align_ok = True
            rospy.loginfo("[PhaseDAssertion] align_mode observed: %s", self._expect_align_mode)

    def _on_drop_ready(self, msg):
        if self._drop_ready_ok:
            return
        if msg.ready == self._expect_drop_ready:
            self._drop_ready_ok = True
            rospy.loginfo("[PhaseDAssertion] drop_ready observed: %s (%s)",
                          msg.ready, msg.reason)

    def _on_yolo_detect(self, msg):
        if self._yolo_ok:
            return
        if msg.data == self._expect_yolo_detect:
            self._yolo_ok = True
            rospy.loginfo("[PhaseDAssertion] /yolo_detect observed: %s", self._expect_yolo_detect)

    def _on_cross_status(self, msg):
        if self._cross_status_ok:
            return
        if msg.data == self._expect_cross_status:
            self._cross_status_ok = True
            rospy.loginfo("[PhaseDAssertion] /detect/cross_status observed: %s", msg.data)

    def _on_perf(self, msg):
        if self._perf_name_ok and self._perf_backend_ok:
            return
        for status in msg.status:
            backend = ""
            for item in status.values:
                if item.key == "backend":
                    backend = item.value
                    break
            if not self._perf_name_ok and status.name == self._expect_perf_name:
                self._perf_name_ok = True
                rospy.loginfo("[PhaseDAssertion] perf status observed: %s", status.name)
            if not self._perf_backend_ok and backend == self._expect_perf_backend:
                self._perf_backend_ok = True
                rospy.loginfo("[PhaseDAssertion] perf backend observed: %s", backend)

    def _all_satisfied(self):
        return all((
            self._target_ok,
            self._selected_ok,
            self._align_ok,
            self._yolo_ok,
            self._drop_ready_ok,
            self._cross_status_ok,
            self._perf_name_ok,
            self._perf_backend_ok,
        ))

    def _missing(self):
        missing = []
        if not self._target_ok:
            missing.append("target_class=%s" % self._expect_target_class)
        if not self._selected_ok:
            missing.append("selected_class=%s" % self._expect_selected_class)
        if not self._align_ok:
            missing.append("align_mode=%s" % self._expect_align_mode)
        if not self._yolo_ok:
            missing.append("yolo_detect=%s" % self._expect_yolo_detect)
        if not self._drop_ready_ok:
            missing.append("drop_ready=%s" % self._expect_drop_ready)
        if not self._cross_status_ok:
            missing.append("cross_status=%s" % self._expect_cross_status)
        if not self._perf_name_ok:
            missing.append("perf_name=%s" % self._expect_perf_name)
        if not self._perf_backend_ok:
            missing.append("perf_backend=%s" % self._expect_perf_backend)
        return missing

    def run(self):
        deadline = rospy.Time.now() + rospy.Duration(self._timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._all_satisfied():
                rospy.loginfo("[PhaseDAssertion] success")
                return 0
            if rospy.Time.now() > deadline:
                rospy.logerr("[PhaseDAssertion] timeout; missing: %s",
                             ", ".join(self._missing()) or "<unknown>")
                return 1
            rate.sleep()
        return 1


def main():
    rc = PhaseDAssertion().run()
    sys.exit(rc)


if __name__ == "__main__":
    main()
