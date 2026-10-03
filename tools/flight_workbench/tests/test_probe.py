#!/usr/bin/env python3
"""板端探针离线测试：用假 rospy/mavros_msgs/std_msgs 跑 board_probe.py。

覆盖：无 master 时输出结构化错误；有 master 时 State/ExtendedState/String 取值正确
（回归点：mavros_msgs/State 没有 .data 字段，早期实现会在板端刷异常日志）、
服务类型读取、节点列表、以及"只订阅不下发"的边界（假 rospy 里没有任何 Publisher/ServiceProxy）。

    cd tools/flight_workbench && python3 tests/test_probe.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_DIR = os.path.dirname(HERE)
RESULTS = []

ROSPY_UP = '''
ERROR = 40
WARN = 30
INFO = 20
Subscribers = []

class AnyMsg(object):
    pass

def init_node(name, **kwargs):
    if not _up:
        raise RuntimeError("Unable to communicate with master!")
    globals()["_node"] = name

def is_shutdown():
    return False

def Subscriber(topic, msg_type, callback, **kwargs):
    Subscribers.append((topic, msg_type, callback))
    # 订阅瞬间投递一条合成消息，便于离线校验取值路径
    try:
        callback(_sample(topic, msg_type))
    except Exception as error:
        print("SAMPLE_ERROR %s %s" % (topic, error))

def _sample(topic, msg_type):
    name = getattr(msg_type, "__name__", "")
    if name == "State":
        return msg_type()
    if name == "ExtendedState":
        return msg_type()
    if name == "String":
        payload = '{"phase": "HIGH_VIEW_SEARCH", "reason": "ok"}'
        return msg_type(payload)
    return msg_type()
'''

FAKE_MSGS = {
    "mavros_msgs/__init__.py": "",
    "mavros_msgs/msg/__init__.py": "",
    "mavros_msgs/msg/_mod.py": textwrap.dedent('''
        class State(object):
            def __init__(self):
                self.connected = True
                self.armed = False
                self.mode = "OFFBOARD"
                self.guided = True
                self.system_status = 3

        class ExtendedState(object):
            def __init__(self):
                self.landed_state = 1
                self.vtol_state = 0
    '''),
    "std_msgs/__init__.py": "",
    "std_msgs/msg/__init__.py": "",
    "std_msgs/msg/_mod.py": textwrap.dedent('''
        class String(object):
            def __init__(self, data=""):
                self.data = data
    '''),
    "rosnode.py": textwrap.dedent('''
        def get_node_names():
            return ["/mavros", "/flight_workbench_probe", "/patrol_control"]
    '''),
    "rosservice.py": textwrap.dedent('''
        def get_service_list():
            return ["/legacy/Servo_raw", "/navigation/start_mission"]

        def get_service_type(name):
            if name == "/legacy/Servo_raw":
                return "patrol_control/Servo"
            return "std_srvs/Trigger"
    '''),
}


def write(path, text):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)


def make_shim(root, master_up):
    write(os.path.join(root, "rospy.py"),
          ("_up = True\n" if master_up else "_up = False\n") + ROSPY_UP)
    for name, body in FAKE_MSGS.items():
        path = os.path.join(root, name)
        if name.endswith("_mod.py"):
            path = os.path.join(root, name.replace("_mod.py", "msg.py"))
            write(path, body)
            continue
        write(path, body)
    # 把 msg.py 里的两个类重新导出到包级别
    write(os.path.join(root, "mavros_msgs", "msg", "__init__.py"),
          "from .msg import State, ExtendedState\n")
    write(os.path.join(root, "std_msgs", "msg", "__init__.py"), "from .msg import String\n")


def run_probe(shim, topics, services):
    env = dict(os.environ)
    env["PYTHONPATH"] = shim + os.pathsep + env.get("PYTHONPATH", "")
    result = subprocess.run(
        [sys.executable, os.path.join(TOOL_DIR, "board_probe.py"), "--once",
         "--interval", "0.2", "--topics", topics, "--services", services],
        cwd=TOOL_DIR, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    lines = [line for line in result.stdout.decode("utf-8", "replace").splitlines()
             if line.strip().startswith("{")]
    if not lines and result.stderr:
        sys.stderr.write("[probe stderr] %s\n" % result.stderr.decode("utf-8", "replace")[-1200:])
    return result, (json.loads(lines[-1]) if lines else None)


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print("%-4s %s%s" % ("PASS" if ok else "FAIL", name, ("  —— %s" % detail) if detail else ""))


def main():
    root = tempfile.mkdtemp(prefix="wb-probe-")
    try:
        shim_down = os.path.join(root, "down")
        make_shim(shim_down, master_up=False)
        result, row = run_probe(shim_down, "/mavros/state,/camera/image_raw", "/legacy/Servo_raw")
        check("无 master 时输出 master=false 且不崩溃",
              row is not None and row.get("master") is False and "error" in row,
              (row or {}).get("error", "无输出"))
        check("无 master 时不输出 state/nodes", row is not None and "state" not in row)

        shim_up = os.path.join(root, "up")
        make_shim(shim_up, master_up=True)
        result, row = run_probe(shim_up, "/mavros/state,/camera/image_raw,/navigation/mission_status",
                                "/legacy/Servo_raw")
        check("有 master 时输出一行 JSON", row is not None and row.get("master") is True,
              (row or {}).get("error", ""))
        row = row or {}
        state = row.get("state") or {}
        check("mavros_msgs/State 取值正确（回归：没有 .data 字段）",
              state.get("connected") is True and state.get("mode") == "OFFBOARD"
              and state.get("armed") is False and state.get("system_status") == 3,
              json.dumps(state, ensure_ascii=False))
        extended = row.get("extended") or {}
        check("ExtendedState 取值正确", extended.get("landed_state") == 1,
              json.dumps(extended, ensure_ascii=False))
        mission = row.get("mission")
        check("std_msgs/String 里的 JSON 被解析",
              isinstance(mission, dict) and mission.get("phase") == "HIGH_VIEW_SEARCH",
              json.dumps(mission, ensure_ascii=False))
        check("只订阅请求的话题 + 三个已知状态话题",
              set(row.get("topics", {})) == {"/mavros/state", "/camera/image_raw",
                                             "/navigation/mission_status",
                                             "/mavros/extended_state",
                                             "/uav_high_view/probe_status",
                                             "/board_trials/auto_land_status"},
              ",".join(sorted(row.get("topics", {}))))
        check("节点列表来自 rosnode", "/patrol_control" in (row.get("nodes") or []),
              ",".join(row.get("nodes") or []))
        check("服务类型读取正确",
              (row.get("services") or {}).get("/legacy/Servo_raw") == "patrol_control/Servo",
              json.dumps(row.get("services"), ensure_ascii=False))
        source = open(os.path.join(TOOL_DIR, "board_probe.py"), encoding="utf-8").read()
        check("探针不发布、不调用服务（源码边界）",
              "Publisher(" not in source and "ServiceProxy(" not in source
              and "wait_for_service(" not in source and "set_mode" not in source)
    finally:
        shutil.rmtree(root, ignore_errors=True)

    failed = RESULTS.count(False)
    print("\n%d 项检查，%d 项失败" % (len(RESULTS), failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
