#!/usr/bin/env python3
"""本机端到端自检：transport=local + 纸板工程（tests/fake_board），不需要板端与 ROS。

覆盖：连接自检 → 单实例检查 → 会话输出采集 → 设备顺序启动编排 → 任务组命令拼装 →
阶段解析（INITIALIZING/MAPPING_READY/READY/FLIGHT_STATUS/STOPPED）→ 安全检查 → 回报 →
板端产物浏览 → 事件流。

    cd tools/flight_workbench && python3 tests/selfcheck.py
"""
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_DIR = os.path.dirname(HERE)
sys.path.insert(0, TOOL_DIR)

import server  # noqa: E402
import wb_board  # noqa: E402
import wb_status  # noqa: E402

# 自检不碰真实状态目录（profile/日志/回报）
_TMP_STATE = tempfile.mkdtemp(prefix="wb-selfcheck-state-")
server.PROFILE_DIR = _TMP_STATE
wb_board.DEFAULT_PROFILE_DIR = _TMP_STATE

FAKE = os.path.join(HERE, "fake_board")
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((bool(ok), name, detail))
    print("%-4s %s%s" % ("PASS" if ok else "FAIL", name,
                         ("  —— %s" % detail) if detail else ""))
    return ok


def wait_until(predicate, timeout=30.0, interval=0.1):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def build_config():
    config = wb_board.load_config()
    connection = config["connection"]
    connection.update({
        "host": "fake@localhost",
        "port": 22,
        "board_root": FAKE,
        "site_dir": "deployment/site_20260928",
        "env_script": "deployment/site_20260928/environment.sh",
        "board_python": sys.executable or "/usr/bin/python3",
        "model": "runtime_models/fake.rknn",
        "metadata": "vision_ws/src/uav_vision/config/fake_metadata.yaml",
        "probe_path": "logs/flight_workbench/board_probe.py",
    })
    config["checks"]["min_free_gb"] = 0.0
    config["terminals"] = [
        {"id": "roscore", "seq": 1, "title": "1 · roscore（纸板）",
         "command": "echo FAKE_ROSCORE_STARTED; sleep 120",
         "ready": {"kind": "ros_master", "timeout": 10}},
        {"id": "mavros", "seq": 2, "title": "2 · MAVROS（纸板）",
         "command": "echo FAKE_MAVROS_STARTED; sleep 120",
         "ready": {"kind": "mavros_connected", "timeout": 10}},
        {"id": "trial", "seq": 3, "title": "专项入口", "command": None},
    ]
    return config


FAKE_TELEMETRY = {
    "t": time.time(), "master": True,
    "state": {"connected": True, "armed": False, "mode": "AUTO.LOITER"},
    "extended": {"landed_state": 1},
    "topics": {"/mavros/state": {"age": 0.1},
               "/camera/image_raw": {"count": 120, "hz": 10.0, "age": 0.1}},
    "services": {"/legacy/Servo_raw": "patrol_control/Servo"},
    "nodes": ["/mavros", "/flight_workbench_probe"],
}


def ensure_fake_board():
    """纸板工程里可再生的部分（占位文件、模块入口）随用随建，保证新克隆也能直接自检。"""
    required = (os.path.join(FAKE, "emit_transcript.sh"),
                os.path.join(FAKE, "runtime_models", "fake.rknn"),
                os.path.join(FAKE, "deployment", "board_trials_4x4", "06_high_priority", "start.sh"))
    if all(os.path.exists(path) for path in required):
        return True
    script = os.path.join(HERE, "fake_board_setup.sh")
    if os.path.exists(script):
        subprocess.run(["bash", script], check=False)
    for path in (os.path.join(FAKE, "runtime_models", "fake.rknn"),
                 os.path.join(FAKE, "vision_ws", "src", "uav_vision", "config", "fake_metadata.yaml")):
        if not os.path.exists(path):
            directory = os.path.dirname(path)
            if not os.path.isdir(directory):
                os.makedirs(directory)
            open(path, "a").close()
    return all(os.path.exists(path) for path in required)


def main():
    check("纸板工程可再生部分就绪", ensure_fake_board())
    config = build_config()
    workbench = server.Workbench(config, {"transport": "local", "password": None,
                                          "allow_local_commands": True})
    server.Handler.workbench = workbench
    events = workbench.subscribe()

    # 0) 自检模式默认拒绝执行设备命令（避免误在本机拉起 roscore 等节点）
    strict = server.Workbench(config, {"transport": "local", "password": None})
    try:
        strict.open_session({"id": "roscore"})
        check("transport=local 默认拒绝执行设备命令", False, "本应拒绝")
    except ValueError as error:
        check("transport=local 默认拒绝执行设备命令", "allow-local-commands" in str(error),
              str(error)[:50])
    strict.sessions.close_all()

    # 1) 连接自检
    result = workbench.connect({})
    check("连接自检（纸板工程 + rospy shim）", result.get("ok"), workbench.connection.get("detail"))

    # 1b) 历史外场地址清单 + 切换
    snapshot = workbench.snapshot()
    hosts = [item["host"] for item in snapshot["connection"]["host_options"]]
    check("快照暴露历史外场地址（含 192.168.43.99）",
          "orangepi@192.168.43.99" in hosts and len(hosts) >= 6,
          "共 %d 项" % len(hosts))
    workbench.update_config({"host": "orangepi@192.168.3.15"})
    check("切换板端地址后 target 同步",
          workbench.target.host == "orangepi@192.168.3.15"
          and workbench.snapshot()["connection"]["host"] == "orangepi@192.168.3.15",
          workbench.target.host)
    workbench.update_config({"host": "orangepi@192.168.43.99"})

    # 2) 单实例检查
    preflight = workbench.refresh_preflight()
    check("单实例检查返回结构化结果", bool(preflight and preflight.get("checks")),
          "检查项 %d 个" % len((preflight or {}).get("checks", [])))
    check("单实例检查识别任务组入口",
          any(info.get("start") for info in (preflight or {}).get("groups", {}).values()))

    # 3) 设备顺序启动（注入假遥测作为就绪判据）
    workbench.telemetry = dict(FAKE_TELEMETRY, t=time.time())
    workbench.open_all_devices({"include_servo": False, "confirm": "启动设备"})
    wait_until(lambda: not workbench.orchestration["running"], timeout=30)
    steps = workbench.orchestration["steps"]
    check("设备顺序启动编排完成", all(step["state"] == "ok" for step in steps),
          "；".join("%s=%s(%s)" % (s["id"], s["state"], s["detail"]) for s in steps))
    session = workbench.sessions.get("roscore")
    text = "".join(data for _, data in session.history_since(0))
    check("终端会话输出被采集", "FAKE_ROSCORE_STARTED" in text)

    # 4) 安全检查
    for body, name in (
        ({"group_id": "site5", "mode": "flight"}, "飞行模式缺少确认被拒绝"),
        ({"group_id": "site5", "mode": "flight", "confirm": "启动试飞", "real_release": True},
         "实投缺少确认词被拒绝"),
        ({"group_id": "site5", "mode": "flight", "confirm": "实投",
          "expected_body": "bash deployment/site_20260928/start_test.sh 9 flight"},
         "界面预览与后端命令不一致被拒绝"),
    ):
        try:
            workbench.start_trial(body)
            check(name, False, "本应拒绝")
        except ValueError as error:
            check(name, True, str(error)[:60])

    # 5) 启动任务组并等阶段推进（带上界面预览命令，走一致性校验的正路径）
    workbench.start_trial({"group_id": "site5", "mode": "flight", "confirm": "实投", "real_release": True,
                           "expected_body": "bash deployment/board_trials_4x4/06_high_priority/start_real.sh --site-config deployment/site_20260928/test_area.yaml"})
    check("任务组命令按模块实投入口拼装",
          workbench.trial["command"] == "bash deployment/board_trials_4x4/06_high_priority/start_real.sh --site-config deployment/site_20260928/test_area.yaml",
          workbench.trial["command"])
    reached = wait_until(lambda: workbench.stage.name == "STOPPED", timeout=30)
    names = [item["name"] for item in workbench.stage.history]
    check("阶段解析：INITIALIZING→MAPPING_READY→READY→IN_FLIGHT→STOPPED",
          reached and names[:5] == ["INITIALIZING", "MAPPING_READY", "READY", "IN_FLIGHT", "STOPPED"],
          "→".join(names))
    check("READY 回报时刻与产物目录已解析",
          bool(workbench.stage.ready_at) and str(workbench.stage.run_dir).startswith(FAKE),
          str(workbench.stage.run_dir))
    check("阶段关键量解析（distinct_clouds / pose_samples）",
          (workbench.stage.detail or {}).get("distinct_clouds") == 18,
          str(workbench.stage.detail)[:80])
    check("无残留错误告警",
          not [a for a in workbench.stage.alerts if a["level"] == "error"],
          "；".join(a["text"][:40] for a in workbench.stage.alerts))

    # 6) 回报
    report = workbench.make_report()
    markdown = report.get("markdown") or ""
    check("生成回报并落盘", os.path.isfile(report.get("path", "")),
          str(report.get("path")))
    check("回报包含阶段/时间线/告警章节",
          all(token in markdown for token in ("## 当前阶段", "## 阶段时间线", "## 告警")))

    # 7) 板端产物浏览
    logs = workbench.refresh_logs()["logs"]
    check("列出板端 logs/board_* 产物", bool(logs), "；".join(item["run"] for item in logs[:3]))
    if logs:
        tail = workbench.tail_log(logs[0]["run"], "supervisor_result.json")
        check("产物 tail 可用", "landed_after_flight" in (tail.get("text") or ""))

    # 8) 事件流
    kinds = set()
    while not events.empty():
        import json
        kinds.add(json.loads(events.get_nowait()).get("t"))
    check("SSE 事件流包含 out/session/stage/timeline",
          {"out", "session", "stage", "timeline"}.issubset(kinds), ",".join(sorted(kinds)))

    # 9) 收尾
    workbench.stop_all()
    wait_until(lambda: all(s.state not in ("running", "starting")
                           for s in workbench.sessions.sessions.values()), timeout=15)
    check("全部会话可停止", all(s.state not in ("running", "starting")
                                for s in workbench.sessions.sessions.values()))
    shutil.rmtree(_TMP_STATE, ignore_errors=True)

    failed = [name for ok, name, _ in RESULTS if not ok]
    print("\n%d 项检查，%d 项失败%s" % (len(RESULTS), len(failed),
                                       ("：" + "；".join(failed)) if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
