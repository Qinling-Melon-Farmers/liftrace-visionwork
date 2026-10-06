#!/usr/bin/env python3
"""试飞工作台离线单测：阶段解析、就绪判定、命令拼装、回报文本。

不需要板端、不需要 ROS：
    cd tools/flight_workbench && python3 tests/test_status.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import wb_board  # noqa: E402
import wb_ssh  # noqa: E402
import wb_status  # noqa: E402

CONFIG = wb_board.load_config()

TRANSCRIPT = [
    "INITIALIZING {'pose_samples': 12, 'camera_info': True, 'image_seen': True, "
    "'compressed_fresh': True, 'mapping_alignment': {'reason': 'settling', 'stable_for': 0.4}, "
    "'armed': False, 'yaw_deg': 1.2, 'xyz_span': [0.01, 0.01, 0.005]}",
    "INITIALIZING {'pose_samples': 88, 'camera_info': True, 'image_seen': True, "
    "'compressed_fresh': True, 'mapping_alignment': {'reason': 'stable'}, 'armed': False, "
    "'yaw_deg': 1.37, 'xyz_span': [0.006, 0.004, 0.003]}",
    "MAPPING_READY {'mapping_started': 1.0, 'map_ready': 3.2, 'first_cloud': 1.1, "
    "'last_cloud': 3.1, 'distinct_clouds': 21, 'alignment': {'reason': 'stable'}}",
    "READY: /home/orangepi/liftrace_board_trials_20260928/logs/board_high_priority_20261002_101500",
    "AUTO_SEQUENCE: manual arm -> OFFBOARD -> low hover -> mission. Never auto-arms.",
    "FLIGHT_STATUS {'mode': 'AUTO.LOITER', 'armed': False, 'phase': 'IDLE', 'reason': ''}",
    "FLIGHT_STATUS {'mode': 'OFFBOARD', 'armed': True, 'phase': 'LOW_HOVER', 'reason': ''}",
    "AUTO_MISSION_START True ",
    "FLIGHT_STATUS {'mode': 'OFFBOARD', 'armed': True, 'phase': 'HIGH_VIEW_SEARCH', 'reason': ''}",
    "Trial application stopped; device MAVROS/driver2 left running. Logs: "
    "/home/orangepi/liftrace_board_trials_20260928/logs/board_high_priority_20261002_101500",
]


def run_transcript(lines):
    tracker = wb_status.StageTracker()
    events = []
    for line in lines:
        events.extend(tracker.feed(line + "\n"))
    return tracker, events


class StageTrackerTest(unittest.TestCase):
    def test_full_ready_flow(self):
        tracker, events = run_transcript(TRANSCRIPT)
        names = [item["name"] for item in tracker.history]
        self.assertEqual(names[:4], ["INITIALIZING", "MAPPING_READY", "READY", "IN_FLIGHT"])
        self.assertEqual(tracker.name, "STOPPED")
        self.assertTrue(tracker.ever_armed)
        self.assertEqual(tracker.alignment, "stable")
        self.assertEqual(tracker.run_dir,
                         "/home/orangepi/liftrace_board_trials_20260928/logs/"
                         "board_high_priority_20261002_101500")
        self.assertTrue(tracker.ready_at)
        self.assertTrue(any(event.get("kind") == "stage" for event in events))
        snapshot = tracker.snapshot()
        self.assertEqual(snapshot["label"], wb_status.STAGE_LABELS["STOPPED"])
        self.assertEqual(snapshot["detail"].get("distinct_clouds"), 21)

    def test_partial_line_is_buffered(self):
        tracker = wb_status.StageTracker()
        tracker.feed("INITIALIZ")
        self.assertEqual(tracker.name, "IDLE")
        tracker.feed("ING {'pose_samples': 3, 'mapping_alignment': {'reason': 'settling'}}\n")
        self.assertEqual(tracker.name, "INITIALIZING")
        self.assertEqual(tracker.detail.get("pose_samples"), 3)

    def test_fc_lio_disagreement_is_alerted_and_throttled(self):
        tracker = wb_status.StageTracker()
        line = ("INITIALIZING {'pose_samples': 40, 'mapping_alignment': "
                "{'reason': 'fc_lio_disagreement', 'position_delta_m': 0.31, "
                "'yaw_delta_deg': 12.5}, 'armed': False}\n")
        tracker.feed(line)
        tracker.feed(line)
        warnings = [a for a in tracker.alerts if a["level"] == "warn"]
        self.assertEqual(len(warnings), 1, "同一原因应被节流")
        self.assertIn("fc_lio_disagreement", warnings[0]["text"])
        self.assertIn("不要转动", warnings[0]["hint"])

    def test_armed_during_initializing_warns(self):
        tracker = wb_status.StageTracker()
        tracker.feed("INITIALIZING {'pose_samples': 5, 'armed': True, "
                     "'mapping_alignment': {'reason': 'settling'}}\n")
        self.assertTrue(any("已解锁" in alert["text"] for alert in tracker.alerts))

    def test_runtime_error_marks_failed_with_hint(self):
        tracker = wb_status.StageTracker()
        tracker.feed("INITIALIZING {'pose_samples': 4, 'mapping_alignment': {'reason': 'stable'}}\n")
        tracker.feed("RuntimeError: Stop the old application first: /patrol_control,/freedom\n")
        self.assertEqual(tracker.name, "FAILED")
        errors = [a for a in tracker.alerts if a["level"] == "error"]
        self.assertTrue(errors)
        self.assertIn("旧应用仍在运行", errors[0]["hint"])

    def test_vision_chain_hint(self):
        tracker = wb_status.StageTracker()
        tracker.feed("RuntimeError: Vision chain not live; inspect application.log: /uav_vision/targets\n")
        self.assertEqual(tracker.name, "FAILED")
        self.assertIn("视觉链未上线", tracker.alerts[-1]["hint"])

    def test_corridor_waypoint_hint(self):
        tracker = wb_status.StageTracker()
        tracker.feed("ValueError: Fill corridor_waypoints with at least two measured waypoints; "
                     "no default route is permitted\n")
        self.assertEqual(tracker.name, "FAILED")
        self.assertIn("走廊航点留空", tracker.alerts[-1]["hint"])

    def test_landing_xy_hint(self):
        tracker = wb_status.StageTracker()
        tracker.feed("ValueError: Fill landing_xy with the measured H center\n")
        self.assertIn("H 中心坐标留空", tracker.alerts[-1]["hint"])

    def test_argparse_usage_is_failure(self):
        tracker = wb_status.StageTracker()
        tracker.feed("usage: run_trial.py [-h] [--root ROOT]\nrun_trial.py: error: bad args\n")
        self.assertEqual(tracker.name, "FAILED")
        self.assertIn("命令行参数不合法", tracker.alerts[-1]["hint"])

    def test_config_valid_and_manual_start_hint(self):
        tracker = wb_status.StageTracker()
        tracker.feed("CONFIG_VALID; no ROS nodes started\n")
        self.assertEqual(tracker.name, "IDLE")
        self.assertTrue(any("配置检查通过" in item["text"] for item in tracker.timeline))
        tracker.feed("WAITING_FOR_MANUAL_MISSION_START: flight READY is not route START.\n")
        self.assertTrue(any("人工启动" in alert["text"] for alert in tracker.alerts))

    def test_disarm_after_flight(self):
        tracker = wb_status.StageTracker()
        for line in ["READY: /tmp/board_x_1\n",
                     "FLIGHT_STATUS {'mode': 'OFFBOARD', 'armed': True, 'phase': 'SEARCH'}\n",
                     "FLIGHT_STATUS {'mode': 'OFFBOARD', 'armed': False, 'phase': 'LANDED'}\n"]:
            tracker.feed(line)
        self.assertEqual(tracker.name, "DISARMED")

    def test_report_contains_sections(self):
        tracker, _ = run_transcript(TRANSCRIPT)
        report = tracker.report({"host": "orangepi@192.168.3.15", "board_root": "/root"},
                                {"group": "5", "mode": "flight", "release": "real"}, {})
        for token in ("# 试飞工作台状态回报", "## 当前阶段", "## 阶段时间线", "## 告警",
                      "不代表飞行验收结论"):
            self.assertIn(token, report)


class ParseTest(unittest.TestCase):
    def test_python_payload(self):
        self.assertEqual(wb_status.parse_python_payload("{'a': 1, 'b': None}"), {"a": 1, "b": None})
        self.assertIsNone(wb_status.parse_python_payload("not a dict"))
        self.assertIsNone(wb_status.parse_python_payload("[1, 2]"))

    def test_probe_line(self):
        line = '{"t": 1.0, "master": true, "state": {"connected": true, "armed": false}}'
        parsed = wb_status.parse_probe_line(line)
        self.assertTrue(parsed["master"])
        self.assertIsNone(wb_status.parse_probe_line("=== log line ==="))
        self.assertIsNone(wb_status.parse_probe_line("{broken"))

    def test_strip_ansi(self):
        self.assertEqual(wb_ssh.strip_ansi("\x1b[1mREADY\x1b[0m: /x"), "READY: /x")


class ReadyCheckTest(unittest.TestCase):
    TELEMETRY = {
        "master": True,
        "state": {"connected": True, "armed": False, "mode": "AUTO.LOITER"},
        "topics": {"/mavros/state": {"age": 0.1},
                   "/camera/image_raw": {"count": 120, "hz": 10.0, "age": 0.2},
                   "/livox/lidar": {"count": 0, "hz": 0.0, "age": None}},
        "services": {"/legacy/Servo_raw": "patrol_control/Servo"},
        "nodes": ["/mavros", "/patrol_control", "/flight_workbench_probe"],
    }

    def test_kinds(self):
        self.assertTrue(wb_status.ready_check("ros_master", {}, self.TELEMETRY)[0])
        self.assertTrue(wb_status.ready_check("mavros_connected", {}, self.TELEMETRY)[0])
        self.assertTrue(wb_status.ready_check("topic", {"topic": "/camera/image_raw"},
                                              self.TELEMETRY)[0])
        self.assertFalse(wb_status.ready_check("topic", {"topic": "/livox/lidar"},
                                               self.TELEMETRY)[0])
        self.assertTrue(wb_status.ready_check("service", {"service": "/legacy/Servo_raw"},
                                              self.TELEMETRY)[0])
        self.assertFalse(wb_status.ready_check("service", {"service": "/navigation/start_mission"},
                                               self.TELEMETRY)[0])

    def test_conflict_nodes(self):
        found = wb_status.conflict_nodes(self.TELEMETRY, CONFIG["checks"]["conflict_nodes"])
        self.assertEqual(found, ["/patrol_control"])
        self.assertNotIn("/flight_workbench_probe", found)


class CommandBuildTest(unittest.TestCase):
    def group(self, gid):
        return next(g for g in CONFIG["groups"] if g["id"] == gid)

    def test_site_route(self):
        command, _ = wb_board.build_group_command(CONFIG, self.group("site1"), "flight", route="site")
        self.assertEqual(command, "bash deployment/site_20260928/start_test.sh 1 flight")
        command, _ = wb_board.build_group_command(CONFIG, self.group("site3"), "preview", route="site")
        self.assertEqual(command, "bash deployment/site_20260928/start_test.sh 3 preview")

    def test_module_real_release(self):
        command, _ = wb_board.build_group_command(CONFIG, self.group("site2"), "flight",
                                                  route="module", real_release=True)
        self.assertEqual(command, "bash deployment/board_trials_4x4/05_low_multi/start_real.sh "
                                  "--site-config deployment/site_20260928/test_area.yaml")

    def test_real_release_rejected_without_entry(self):
        with self.assertRaises(ValueError):
            wb_board.build_group_command(CONFIG, self.group("mod03"), "flight", real_release=True)

    def test_check_config_uses_module_entry(self):
        command, _ = wb_board.build_group_command(CONFIG, self.group("site6"), "preview",
                                                  check_config=True)
        self.assertIn("09_high_speed_capture/start.sh preview", command)
        self.assertTrue(command.endswith("--check-config"))

    def test_capture_speed_site_and_module(self):
        command, _ = wb_board.build_group_command(CONFIG, self.group("site6"), "flight",
                                                  route="module", capture_speed=1.0)
        self.assertIn("--capture-speed 1.0", command)
        command, _ = wb_board.build_group_command(CONFIG, self.group("site6"), "flight", route="site",
                                                  capture_speed=1.0)
        self.assertTrue(command.endswith('6 flight --capture-speed 1.0'))

    def test_terminal_commands_substituted(self):
        roscore = next(t for t in CONFIG["terminals"] if t["id"] == "roscore")
        self.assertEqual(wb_board.terminal_command(CONFIG, roscore), "roscore")
        lidar = next(t for t in CONFIG["terminals"] if t["id"] == "lidar")
        self.assertIn("/home/orangepi/liftrace_board_trials_20260928/deployment/site_20260928/"
                      "MID360_config.json", wb_board.terminal_command(CONFIG, lidar))
        servo = next(t for t in CONFIG["terminals"] if t["id"] == "servo")
        self.assertIn("patrol_uav_ws-patrol_planner/devel/lib/actuator_pwm/pwm_node1 /Servo:=/legacy/Servo_raw",
                      wb_board.terminal_command(CONFIG, servo))
        camera = next(t for t in CONFIG["terminals"] if t["id"] == "camera")
        self.assertTrue(wb_board.terminal_command(CONFIG, camera).endswith("/dev/video0"))

    def test_wrapped_command(self):
        wrapped = wb_board.terminal_wrapped_command(CONFIG, "roscore")
        self.assertIn("cd /home/orangepi/liftrace_board_trials_20260928", wrapped)
        self.assertIn("source /home/orangepi/liftrace_board_trials_20260928/deployment/"
                      "site_20260928/environment.sh", wrapped)
        self.assertTrue(wrapped.endswith("&& roscore"))

    def test_config_sanity(self):
        ids = [t["id"] for t in CONFIG["terminals"]]
        self.assertEqual(len(ids), len(set(ids)))
        for group in CONFIG["groups"]:
            command, _ = wb_board.build_group_command(CONFIG, group, "preview",
                                                      route=group["channel"])
            self.assertTrue(command.startswith("bash "))


class HostOptionsTest(unittest.TestCase):
    """外场地址：默认是当前现场地址，历史地址作为可选清单。"""

    HISTORICAL = ("orangepi@192.168.43.99", "orangepi@192.168.3.15",
                  "orangepi@192.168.43.59", "orangepi@192.168.156.193",
                  "orangepi@10.231.47.193", "orangepi@192.168.3.126")

    def test_default_host_is_current_field_address(self):
        self.assertEqual(CONFIG["connection"]["host"], "orangepi@192.168.3.126")

    def test_historical_addresses_are_options(self):
        options = CONFIG["connection"]["host_options"]
        hosts = [item["host"] for item in options]
        for host in self.HISTORICAL:
            self.assertIn(host, hosts)
        self.assertEqual(len(hosts), len(set(hosts)), "地址清单不应重复")
        self.assertTrue(all(item["label"] for item in options), "每项都应有出处说明")
        self.assertEqual(hosts[0], CONFIG["connection"]["host"], "当前地址应排在最前")

    def test_host_options_helper_handles_strings_and_custom_current(self):
        config = {"connection": {"host": "orangepi@10.0.0.9",
                                 "host_options": ["orangepi@192.168.3.15",
                                                  {"host": "orangepi@192.168.3.15", "label": "重复项"},
                                                  {"host": "", "label": "空项"},
                                                  {"host": "orangepi@192.168.43.99", "label": "外场"}]}}
        options = wb_board.host_options(config)
        hosts = [item["host"] for item in options]
        self.assertEqual(hosts, ["orangepi@10.0.0.9", "orangepi@192.168.3.15",
                                 "orangepi@192.168.43.99"])
        self.assertEqual(options[0]["label"], "当前配置")

    def test_profile_can_override_host(self):
        config = {"connection": {"host": "orangepi@192.168.43.99", "host_options": []}}
        wb_board.apply_profile(config, {"host": "orangepi@192.168.3.15"})
        self.assertEqual(config["connection"]["host"], "orangepi@192.168.3.15")


class SshFailureTest(unittest.TestCase):
    """ssh 连不上时必须如实报认证/网络原因，不能误报成"板端文件缺失"。"""

    def test_permission_denied_is_reported_as_auth(self):
        reason, hint = wb_board.classify_ssh_error(
            "orangepi@192.168.43.59: Permission denied (publickey,password).")
        self.assertIn("Permission denied", reason)
        self.assertIn("口令", hint)

    def test_host_key_changed_and_network_cases(self):
        reason, hint = wb_board.classify_ssh_error(
            "@@@ WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED! @@@")
        self.assertIn("IDENTIFICATION HAS CHANGED", reason)
        self.assertIn("ssh-keygen -R", hint)
        reason, hint = wb_board.classify_ssh_error("ssh: connect to host 10.0.0.1 port 22: "
                                                   "Connection timed out")
        self.assertIn("timed out", reason)
        self.assertIn("网络不通", hint)

    def test_clean_output_has_no_ssh_error(self):
        self.assertEqual(wb_board.classify_ssh_error("HOST=orangepi\nROOT=OK\n"), (None, ""))

    def test_connection_detail_prefers_ssh_reason(self):
        class StubBoard(wb_board.BoardClient):
            def __init__(self):
                self.config = {"connection": dict(CONFIG["connection"])}
                self.target = wb_ssh.Target(host="orangepi@192.168.43.59")

            def run(self, command, timeout=30.0):
                return 255, "orangepi@192.168.43.59: Permission denied (publickey,password).\n"

        info = StubBoard().test_connection()
        self.assertFalse(info["ok"])
        self.assertTrue(info["auth_failed"])
        self.assertIn("SSH 层失败", info["detail"])
        self.assertNotIn("工程根目录不存在", info["detail"])

    def test_batchmode_only_without_password(self):
        target = wb_ssh.Target(host="orangepi@192.168.43.59",
                               ssh_options=["-o", "LogLevel=ERROR"])
        self.assertIn("BatchMode=yes", target.remote_argv("echo hi", force_tty=False))
        target.password = "secret"
        self.assertNotIn("BatchMode=yes", target.remote_argv("echo hi", force_tty=False))


if __name__ == "__main__":
    unittest.main(verbosity=2)
