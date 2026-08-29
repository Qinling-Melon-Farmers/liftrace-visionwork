#!/usr/bin/env python3
"""Gate target-area search, weighted three-drop delivery and landing."""

import json
import os
import sys
import time

# catkin devel 下节点经 relay 执行时 __file__ 指向源码，但 sys.path[0] 是 relay
# 目录；先插入源码目录，保证 from coverage_policy import ... 命中真实模块。
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import rosgraph
import rospy
from std_msgs.msg import String, UInt8

from uav_mission.msg import ReleaseResult
from uav_vision.msg import TargetCandidateArray

from coverage_policy import (
    accumulate_run_facts,
    expected_delivery_classes,
    profile_standard_classes,
)

EXPECTED_SLOTS = (1, 2, 3)


class CoverageR6Assertion:
    def __init__(self):
        rospy.init_node("coverage_r6_assertion")
        self._deadline = time.monotonic() + float(
            rospy.get_param("~wall_timeout", 3000.0))
        self._report_path = rospy.get_param(
            "~report_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "gate_status.json"))
        self._red_cross_truth_path = rospy.get_param(
            "~red_cross_truth_path",
            os.path.join(os.environ.get("SIM_RUN_DIR", "/tmp"),
                         "red_cross_truth.yaml"))
        # 类目 profile 与 manager 保持一致：full=五类（历史回归），
        # r2026=规则书四类标准靶（无 tank）。“发现完整”按 profile 集合判定。
        self._class_profile = rospy.get_param("~class_profile", "full")
        self._standard_classes = profile_standard_classes(self._class_profile)
        self._manager = None
        self._candidate_ids = {}
        # 发现/选择属于全程事实：按每个状态快照累计合并，避免最终快照受
        # landing 模式过滤影响而丢失历史发现证据。
        self._discovered_by_class = {}
        self._discovered_ids = set()
        self._selection_accum = []
        self._raw_calls = []
        self._successes = []
        self._denied = []
        self._audit_status = "WAITING"
        self._goal_publishers = set()
        self._unexpected_publishers = set()
        self._max_publishers = 0
        self._master = rosgraph.Master(rospy.get_name())

        rospy.Subscriber("/mission/coverage_status", String,
                         self._on_manager, queue_size=4)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray,
                         self._on_targets, queue_size=4)
        rospy.Subscriber("/uav_mission/mock_raw_servo_calls", UInt8,
                         self._on_raw, queue_size=8)
        rospy.Subscriber("/mission/release_result", ReleaseResult,
                         self._on_result, queue_size=12)
        rospy.Subscriber("/mission/visual_delivery_audit_status", String,
                         self._on_audit, queue_size=2)
        self._write("RUNNING", "waiting")

    @staticmethod
    def _fresh(candidate):
        if candidate.last_seen.to_sec() <= 0.0:
            return False
        age = max(0.0, (rospy.Time.now() - candidate.last_seen).to_sec())
        return age <= 0.5

    def _on_manager(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self._manager = {"status": "FAIL", "reason": "invalid_json"}
            return
        self._manager = payload
        accumulate_run_facts(
            self._discovered_by_class, self._discovered_ids,
            self._selection_accum, payload)

    def _on_targets(self, msg):
        for candidate in msg.targets:
            if candidate.class_name not in self._standard_classes:
                continue
            if (candidate.state < 2 or not candidate.map_valid or
                    candidate.map_frame != "camera_init" or
                    not candidate.association_valid or
                    candidate.reject_reason or not self._fresh(candidate)):
                continue
            self._candidate_ids[candidate.class_name] = int(candidate.id)

    def _on_raw(self, msg):
        self._raw_calls.append(int(msg.data))

    def _on_result(self, msg):
        record = {
            "slot": int(msg.payload_slot),
            "success": bool(msg.success),
            "target_id": int(msg.target_id),
            "target_class": msg.target_class,
            "reason": msg.reason,
        }
        if msg.success:
            self._successes.append(record)
        else:
            self._denied.append(record)

    def _on_audit(self, msg):
        self._audit_status = msg.data.strip()

    def _sample_publishers(self):
        for topic, nodes in self._master.getSystemState()[0]:
            if topic != "/fastplanner/goal":
                continue
            self._max_publishers = max(self._max_publishers, len(nodes))
            self._goal_publishers.update(nodes)
            self._unexpected_publishers.update(
                node for node in nodes
                if node != "/coverage_search_manager")
            return

    def _payload(self, status, reason):
        return {
            "gate": "coverage_r6",
            "status": status,
            "reason": reason,
            "class_profile": self._class_profile,
            "standard_classes": list(self._standard_classes),
            "manager": self._manager,
            "candidate_ids": self._candidate_ids,
            "raw_calls": self._raw_calls,
            "successes": self._successes,
            "denied_results": self._denied,
            "audit_status": self._audit_status,
            "goal_publishers": sorted(self._goal_publishers),
            "unexpected_goal_publishers": sorted(
                self._unexpected_publishers),
            "max_goal_publishers": self._max_publishers,
        }

    def _write(self, status, reason):
        directory = os.path.dirname(self._report_path) or "."
        os.makedirs(directory, exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(self._payload(status, reason), handle,
                      indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._report_path)

    def _final_checks(self):
        manager = self._manager
        # 发现/选择使用全程累计事实；其余检查使用最终状态快照。
        discovered_classes = set(self._discovered_by_class)
        discovered_ids = set(self._discovered_ids)
        selected_classes = [
            class_name for _target_id, class_name in
            self._selection_accum[:3]]
        delivered = manager.get("delivered", [])
        delivered_classes = [item.get("class") for item in delivered]
        delivered_ids = [item.get("id") for item in delivered]
        delivered_slots = [item.get("slot") for item in delivered]
        success_slots = [item["slot"] for item in self._successes]
        success_ids = [item["target_id"] for item in self._successes]
        commands = manager.get("command_sequence", [])
        # 动态期望：按已发现候选权重排序的 top-3（随机十字被发现则期望第一投为十字）。
        expected_classes = expected_delivery_classes(
            self._discovered_by_class)
        # 随机十字摆放成功（真值文件存在）才要求发现十字；未摆放不检查。
        red_cross_required = (
            "red_cross" in discovered_classes
            if os.path.exists(self._red_cross_truth_path) else True)
        return [
            manager.get("reason") == "three_deliveries_landed",
            manager.get("state") == "COMPLETE",
            set(self._standard_classes).issubset(discovered_classes),
            red_cross_required,
            len(discovered_ids) >= len(self._standard_classes),
            selected_classes == expected_classes,
            delivered_classes == expected_classes,
            len(delivered_ids) == 3 and len(set(delivered_ids)) == 3,
            delivered_slots == list(EXPECTED_SLOTS),
            self._raw_calls == list(EXPECTED_SLOTS),
            success_slots == list(EXPECTED_SLOTS),
            len(success_ids) == 3 and len(set(success_ids)) == 3,
            not self._denied,
            self._audit_status == "PASS",
            manager.get("collision_count") == 0,
            manager.get("boundary_violations") == 0,
            manager.get("mission_elapsed", 601.0) <= 600.0,
            manager.get("execute_candidates") is True,
            manager.get("collect_before_delivery") is True,
            manager.get("final_land") is True,
            commands.count(1) == 3,
            commands.count(2) == 3,
            commands.count(3) == 3,
            4 in commands and 5 in commands,
            "/coverage_search_manager" in self._goal_publishers,
            not self._unexpected_publishers,
            self._max_publishers <= 1,
        ]

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self._sample_publishers()
            if self._unexpected_publishers or self._max_publishers > 1:
                self._write("FAIL", "planner_goal_not_exclusive")
                return 1
            if self._raw_calls != list(EXPECTED_SLOTS[:len(self._raw_calls)]):
                self._write("FAIL", "raw_slot_order")
                return 1
            if self._denied:
                self._write("FAIL", "release_denied")
                return 1
            if self._manager is not None and self._manager.get("status") in (
                    "PASS", "FAIL"):
                if self._manager.get("status") != "PASS":
                    self._write(
                        "FAIL", "manager_%s" %
                        self._manager.get("reason", "failed"))
                    return 1
                if not all(self._final_checks()):
                    self._write("FAIL", "coverage_r6_contract_failed")
                    return 1
                self._write("PASS", "weighted_three_drop_landed")
                rospy.loginfo("[CoverageR6Gate] PASS")
                return 0
            if time.monotonic() >= self._deadline:
                self._write("FAIL", "wall_timeout")
                return 1
            rate.sleep()
        return 1


if __name__ == "__main__":
    sys.exit(CoverageR6Assertion().run())
