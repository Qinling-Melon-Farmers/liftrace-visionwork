#!/usr/bin/env python3
"""HTTP 层自检：真起一次服务（transport=local），用 urllib 检查接口、静态文件与 SSE。

    cd tools/flight_workbench && python3 tests/smoke_http.py
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_DIR = os.path.dirname(HERE)
sys.path.insert(0, TOOL_DIR)

PORT = int(os.environ.get("WB_SMOKE_PORT", "8799"))
BASE = "http://127.0.0.1:%d" % PORT
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print("%-4s %s%s" % ("PASS" if ok else "FAIL", name, ("  —— %s" % detail) if detail else ""))
    return ok


def free_port(start):
    port = start
    while port < start + 20:
        with socket.socket() as probe:
            if probe.connect_ex(("127.0.0.1", port)) != 0:
                return port
        port += 1
    raise RuntimeError("没有空闲端口")


def request(path, method="GET", body=None, timeout=8):
    data = json.dumps(body).encode("utf-8") if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.status, response.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as error:
        return error.code, error.read().decode("utf-8", "replace")
    except (urllib.error.URLError, ConnectionError, OSError) as error:
        return 0, str(error)


def main():
    global PORT, BASE
    PORT = free_port(PORT)
    BASE = "http://127.0.0.1:%d" % PORT
    profile_dir = tempfile.mkdtemp(prefix="wb-smoke-profile-")
    log_path = "/tmp/wb_smoke_http.log"
    with open(log_path, "w", encoding="utf-8") as log:
        process = subprocess.Popen([sys.executable, os.path.join(TOOL_DIR, "server.py"),
                                    "--transport", "local", "--port", str(PORT),
                                    "--profile-dir", profile_dir,
                                    "--allow-local-commands"],
                                   cwd=TOOL_DIR, stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.time() + 20
        while time.time() < deadline:
            status, _ = request("/api/snapshot")
            if status == 200:
                break
            time.sleep(0.3)
        status, text = request("/api/snapshot")
        snapshot = json.loads(text) if status == 200 else {}
        check("GET /api/snapshot", status == 200, "HTTP %s" % status)
        check("快照包含终端/任务组/阶段字段",
              all(key in snapshot for key in ("terminals", "groups", "stage", "telemetry")),
              "terminals=%d groups=%d" % (len(snapshot.get("terminals", [])),
                                          len(snapshot.get("groups", []))))

        for path, keyword in (("/", "<html"), ("/static/app.js", "EventSource"),
                              ("/static/style.css", "{")):
            status, body = request(path)
            check("GET %s" % path, status == 200 and keyword in body,
                  "HTTP %s，%d 字节" % (status, len(body)))

        status, body = request("/api/config", "POST",
                               {"host": "orangepi@192.168.3.15", "board_root":
                                "/home/orangepi/liftrace_board_trials_20260928"})
        config_ok = status == 200 and json.loads(body).get("ok")
        check("POST /api/config", config_ok, "HTTP %s" % status)

        status, body = request("/api/snapshot")
        connection = (json.loads(body).get("connection") or {})
        options = connection.get("host_options") or []
        hosts = [item.get("host") for item in options]
        check("快照带历史地址清单（含历史 192.168.43.99）",
              "orangepi@192.168.43.99" in hosts and "orangepi@10.231.47.193" in hosts,
              "共 %d 项：%s" % (len(hosts), ", ".join(hosts[:3])))
        check("切换后的 host 生效", connection.get("host") == "orangepi@192.168.3.15",
              str(connection.get("host")))

        status, body = request("/api/trial/start", "POST",
                               {"group_id": "site5", "mode": "flight"})
        check("飞行模式缺少确认被拒绝", status == 400 and "确认" in body, body[:80])

        status, body = request("/api/trial/start", "POST",
                               {"group_id": "site5", "mode": "flight", "confirm": "启动试飞",
                                "real_release": True})
        check("实投缺少确认词被拒绝", status == 400 and "实投" in body, body[:80])

        status, body = request("/api/trial/start", "POST",
                               {"group_id": "site5", "mode": "flight", "confirm": "实投",
                                "expected_body": "bash deployment/site_20260928/start_test.sh 9 flight"})
        check("界面预览与后端命令不一致被拒绝", status == 400 and "不一致" in body, body[:100])

        status, body = request("/api/session/open", "POST", {"id": "servo"})
        check("需要现场确认的终端被拒绝", status == 400 and "确认" in body, body[:80])

        status, body = request("/api/action/mission_start", "POST", {"confirm": "启动任务"})
        check("非 READY 阶段拒绝启动任务", status == 400 and "READY" in body, body[:80])

        # SSE：读少量字节，确认 hello 事件存在
        try:
            with urllib.request.urlopen(BASE + "/api/events", timeout=6) as stream:
                chunk = stream.read(220).decode("utf-8", "replace")
        except Exception as error:  # noqa: BLE001
            chunk = "读取失败：%s" % error
        check("GET /api/events 推送 hello 快照", '"t": "hello"' in chunk, chunk[:80].replace("\n", " "))

        status, body = request("/api/logs/refresh", "POST", {})
        check("POST /api/logs/refresh", status == 200 and json.loads(body).get("ok"),
              "HTTP %s" % status)
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
        shutil.rmtree(profile_dir, ignore_errors=True)

    failed = RESULTS.count(False)
    print("\n%d 项检查，%d 项失败" % (len(RESULTS), failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
