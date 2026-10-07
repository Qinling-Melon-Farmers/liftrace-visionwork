#!/usr/bin/env python3
"""Liftrace 试飞验证看板（本地 Web 工作台服务端）。

职责：
  * 用一条常驻 SSH 会话的方式管理现场手册里的 6~7 个终端；
  * 按顺序启动设备链并逐项等待就绪（roscore → MAVROS → 雷达 → 相机 → 舵机 → 专项入口）；
  * 解析专项入口的 INITIALIZING / MAPPING_READY / READY / FLIGHT_STATUS，做初始化监控与
    READY 回报（事件流 + 告警 + 可复制报告 + 声音提示）；
  * 浏览板端 logs/board_* 产物（tail / 下载）与操作时间线。

边界：本服务只执行现场手册里已有的入口命令，不自动解锁、不自动起飞、不代替飞手接管，
不修改板端代码，不写任何口令进仓库。
"""
import argparse
import json
import os
import queue
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import wb_survey
import wb_board
import wb_status
import wb_ssh
from wb_ssh import SessionManager, Target, run_once

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
PROFILE_DIR = wb_board.DEFAULT_PROFILE_DIR
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
}

# 顺序启动编排里每个终端对应的"已在运行"判据（避免重复启动设备节点）
RUNNING_KEYS = {
    "roscore": ("roscore", "rosmaster"),
    "mavros": ("mavros_node",),
}


class Workbench(object):
    def __init__(self, config, options):
        self.config = config
        self.options = options
        self.target = Target(
            host=config["connection"].get("host", ""),
            port=config["connection"].get("port", 22),
            ssh_options=config["connection"].get("ssh_options", []),
            password=options.get("password"),
            auto_password=config["connection"].get("auto_password", True),
            connect_timeout=config["connection"].get("connect_timeout", 12),
            transport=options.get("transport", "ssh"),
        )
        self.board = wb_board.BoardClient(config, self.target)
        self.log_dir = os.path.join(PROFILE_DIR, "logs")
        self.report_dir = os.path.join(PROFILE_DIR, "reports")
        self.sessions = SessionManager(self.log_dir, on_output=self._on_output,
                                       on_state=self._on_state, on_note=self._on_note)
        self.stage = wb_status.StageTracker()
        self.telemetry = {}
        self.connection = {"state": "unknown", "detail": "尚未连接",
                           "host": self.target.host, "transport": self.target.transport}
        self.trial = {"group_id": None, "group": None, "name": None, "mode": None,
                      "release": None, "real_release": False, "command": None,
                      "run_dir": None, "started_at": None, "check_config": False,
                      "capture_speed": None, "capture_lighting": None,
                      "motion_optimized": False, "survey_pattern": None,
                      "resume_survey": None, "route": None}
        self.orchestration = {"running": False, "step": None, "started_at": None, "steps": [],
                              "cancel": False}
        self.board_state = {"logs": [], "preflight": None}
        self.report = {"path": None, "markdown": None}
        self.probe_state = {"uploaded": False, "detail": "未部署"}
        self.clients = []
        self.lock = threading.RLock()
        self.trial_lock = threading.RLock()
        self.mission_start_attempted = False
        self._probe_buffer = ""
        self._last_telemetry_broadcast = 0.0
        self._profile = wb_board.load_profile()

    # ---------- 事件广播 ----------
    def subscribe(self):
        client = queue.Queue(maxsize=2000)
        with self.lock:
            self.clients.append(client)
        return client

    def unsubscribe(self, client):
        with self.lock:
            if client in self.clients:
                self.clients.remove(client)

    def broadcast(self, event, snapshot_after=False):
        payload = json.dumps(event, ensure_ascii=False, default=str)
        with self.lock:
            dead = []
            for client in self.clients:
                try:
                    client.put_nowait(payload)
                except queue.Full:
                    try:
                        client.get_nowait()
                        client.put_nowait(payload)
                    except queue.Empty:
                        dead.append(client)
            for client in dead:
                self.clients.remove(client)

    def toast(self, level, text):
        self.broadcast({"t": "toast", "level": level, "text": text})

    # ---------- 会话回调 ----------
    def _on_output(self, sid, seq, data):
        self.broadcast({"t": "out", "s": sid, "n": seq, "d": data})
        if sid == "trial":
            for event in self.stage.feed(data):
                self._emit_event(event)
        elif sid == "probe":
            self._feed_probe(data)

    def _on_state(self, sid, snapshot):
        self.broadcast({"t": "session", "s": sid, "session": snapshot})
        if snapshot.get("state") in ("exited", "failed") and sid == "trial":
            if snapshot.get("state") == "failed":
                self._emit_event(self.stage.set_stage("FAILED", {"exit_code": snapshot.get("exit_code")}))
            elif self.stage.name not in ("STOPPED", "FAILED"):
                self._emit_event(self.stage.set_stage("STOPPED", {"exit_code": snapshot.get("exit_code")}))
            self.stage.note_action("专项入口会话结束（exit=%s）" % snapshot.get("exit_code"),
                                   "warn" if snapshot.get("state") == "failed" else "info")
            self.broadcast({"t": "stage", "stage": self.stage.snapshot()})

    def _on_note(self, sid, text):
        item = self.stage.note_action("[%s] %s" % (sid, text))
        self._emit_event(item)

    def _emit_event(self, event):
        if not event:
            return
        kind = event.get("kind")
        if kind == "stage":
            self.broadcast({"t": "stage", "stage": event["stage"]})
            name = event["stage"].get("name")
            if name == "READY":
                self.toast("ok", "READY：应用链已就绪，由飞手人工解锁并拨入 OFFBOARD")
            elif name == "FAILED":
                self.toast("error", "专项入口失败，查看告警与终端日志")
        elif kind == "alert":
            self.broadcast({"t": "alert", "alert": event["alert"]})
        elif kind == "timeline":
            self.broadcast({"t": "timeline", "item": event["item"]})

    def _feed_probe(self, data):
        self._probe_buffer += data
        while "\n" in self._probe_buffer:
            line, self._probe_buffer = self._probe_buffer.split("\n", 1)
            if not line.strip():
                continue
            if line.startswith("===") or line.startswith("Traceback"):
                continue
            telemetry = wb_status.parse_probe_line(line)
            if telemetry is None:
                continue
            telemetry["at"] = time.time()
            with self.lock:
                self.telemetry = telemetry
            trial_session = self.sessions.get("trial")
            if (self.trial.get("route") == "low_observation" and self.trial.get("mode") == "flight"
                    and not self.trial.get("check_config") and trial_session is not None
                    and trial_session.state == "running"):
                status = (telemetry.get("observe") or {}).get("low_hover")
                if isinstance(status, dict):
                    for event in self.stage.feed_line(json.dumps(status)):
                        self._emit_event(event)
            if (self.trial.get("mode") == "flight" and not self.trial.get("check_config")
                    and self.trial.get("route") != "low_observation"
                    and trial_session is not None and trial_session.state == "running"
                    and self.stage.name in ("READY", "IN_FLIGHT", "DISARMED")):
                for event in self.stage.observe_telemetry(
                        telemetry, self.config["probe"].get("terminal_hover_topic", "")):
                    if event["kind"] == "stage":
                        self.broadcast({"t": "stage", "stage": event["stage"]})
                    else:
                        self._emit_event(event)
            self.broadcast({"t": "telemetry", "telemetry": telemetry})
            self._check_orchestration()

    # ---------- 快照 ----------
    def snapshot(self, include_logs=True):
        with self.lock:
            return {
                "ok": True,
                "now": time.time(),
                "connection": dict(self.connection, **{
                    "user": self.target.user, "host": self.target.host,
                    "port": self.target.port,
                    "board_root": self.config["connection"].get("board_root"),
                    "site_dir": self.config["connection"].get("site_dir"),
                    "env_script": self.config["connection"].get("env_script"),
                    "model": self.config["connection"].get("model"),
                    "metadata": self.config["connection"].get("metadata"),
                    "transport": self.target.transport,
                    "password_available": bool(self.target.password),
                    "auto_password": bool(self.target.auto_password),
                    "host_options": wb_board.host_options(self.config),
                }),
                "profile": {"path": wb_board.profile_path(),
                            "auto_password": bool(self.target.auto_password),
                            "saved_password": bool(self._profile.get("password_saved"))},
                "groups": self._groups_snapshot(),
                "observation": dict(self.config.get("observation", {}),
                                    topics=self.config["probe"].get("observe_topics", {})),
                "terminals": self._terminals_snapshot(),
                "sessions": self.sessions.snapshots(),
                "trial": dict(self.trial),
                "stage": self.stage.snapshot(),
                "telemetry": self.telemetry,
                "orchestration": {k: v for k, v in self.orchestration.items() if k != "cancel"},
                "alerts": self.stage.alerts[-40:],
                "timeline": self.stage.timeline[-80:],
                "board": {"logs": self.board_state["logs"] if include_logs else [],
                          "preflight": self.board_state["preflight"]},
                "report": self.report,
                "probe": self.probe_state,
            }

    def _groups_snapshot(self):
        groups = []
        preflight_groups = (self.board_state.get("preflight") or {}).get("groups") or {}
        for group in self.config.get("groups", []):
            info = preflight_groups.get(group.get("folder")) or {}
            item = dict(group)
            item["real_available"] = bool(info.get("real", group.get("folder") in wb_board.REAL_RELEASE_FOLDERS))
            item["available"] = bool(info.get("start", True))
            groups.append(item)
        return groups

    def _terminals_snapshot(self):
        terminals = []
        for terminal in self.config.get("terminals", []):
            item = dict(terminal)
            item["command"] = wb_board.terminal_command(self.config, terminal)
            item["state"] = (self.sessions.get(terminal["id"]).state
                             if self.sessions.get(terminal["id"]) else "idle")
            item["has_ready"] = bool(terminal.get("ready"))
            item.pop("params", None)
            terminals.append(item)
        return terminals

    # ---------- 连接 ----------
    def connect(self, body):
        password = body.get("password")
        if password is not None:
            self.target.password = password or None
        if body.get("auto_password") is not None:
            self.target.auto_password = bool(body["auto_password"])
        self.connection["state"] = "checking"
        self.connection["detail"] = "正在检查板端工程与 ROS Python"
        self.broadcast({"t": "connection", "connection": self.connection})
        result = self.board.test_connection()
        if result.get("ok"):
            self.connection.update({"state": "ok", "detail": "%s；%s" % (result.get("detail", ""),
                                                                        result.get("uname", ""))})
            if body.get("save_password") and password:
                self._save_profile(password=True)
            elif body.get("save_password"):
                self._save_profile(password=False)
            self.stage.note_action("已连接板端 %s（%s）" % (self.target.host, self.target.transport), "ok")
            self.ensure_probe()
            threading.Thread(target=self.refresh_preflight, name="wb-preflight").start()
        else:
            self.connection.update({"state": "failed", "detail": result.get("detail", "连接失败")})
            self.toast("error", "连接失败：%s" % result.get("detail"))
        self.broadcast({"t": "connection", "connection": self.connection})
        self.broadcast({"t": "timeline", "item": self.stage.timeline[-1] if self.stage.timeline else None})
        return {"ok": bool(result.get("ok")), "connection": self.connection,
                "error": None if result.get("ok") else result.get("detail")}

    def disconnect(self):
        self.protect_observation_dependencies()
        self.sessions.close_all()
        self.connection.update({"state": "unknown", "detail": "已断开（会话已停止）"})
        self.broadcast({"t": "connection", "connection": self.connection})
        return {"ok": True, "connection": self.connection}

    def _save_profile(self, password=False):
        profile = dict(self._profile)
        connection = self.config["connection"]
        for key in ("host", "board_root", "site_dir", "env_script", "model", "metadata"):
            profile[key] = connection.get(key)
        profile["port"] = connection.get("port", 22)
        profile["auto_password"] = bool(self.target.auto_password)
        if password:
            profile["password_saved"] = True
            profile["password"] = self.target.password
        try:
            wb_board.save_profile(profile)
            self._profile = profile
        except OSError as error:
            self.toast("warn", "档案保存失败：%s" % error)

    def update_config(self, body):
        connection = self.config["connection"]
        changed = any(body.get(k) not in (None, "", connection.get(k))
                      for k in ("host", "port", "board_root", "site_dir", "env_script", "model", "metadata"))
        if changed and (self.orchestration.get("running") or any(
                s.state in ("running", "starting") for s in self.sessions.sessions.values())):
            raise ValueError("会话仍在运行；请落地、收尾并断开后再修改板端地址或工程参数")
        if changed:
            self.telemetry = {}
            self.board_state = {"logs": [], "preflight": None}
            self.stage.reset()
            self.connection.update(state="unknown", detail="连接参数已修改，请重新连接")
        for key in ("host", "board_root", "site_dir", "env_script", "model", "metadata"):
            if body.get(key):
                connection[key] = body[key]
        if body.get("port"):
            connection["port"] = int(body["port"])
            self.target.port = int(body["port"])
        if body.get("auto_password") is not None:
            self.target.auto_password = bool(body["auto_password"])
            connection["auto_password"] = bool(body["auto_password"])
        if body.get("host"):
            self.target.host = body["host"]
            # 从「板端地址」下拉选中的地址要落到本机 profile，下次启动仍是它（不写回仓库配置）
            self._save_profile(password=False)
        if body.get("password"):
            self.target.password = body["password"]
        if body.get("save_password"):
            self._save_profile(password=bool(body.get("password")))
        self.connection.update({"host": self.target.host, "port": self.target.port})
        self.broadcast({"t": "connection", "connection": self.snapshot(include_logs=False)["connection"]})
        return {"ok": True, "connection": self.snapshot(include_logs=False)["connection"],
                "profile": {"path": wb_board.profile_path(), "saved_password":
                            bool(self._profile.get("password_saved"))}}

    # ---------- 探针 ----------
    def ensure_probe(self):
        if self.target.transport != "ssh":
            self.probe_state = {"uploaded": False, "detail": "本机自检模式：不部署板端探针"}
            return False
        session = self.sessions.get("probe")
        if session is not None and session.state in ("running", "starting"):
            return True
        try:
            self.board.upload_probe()
            self.probe_state = {"uploaded": True, "detail": "探针已上传"}
        except Exception as error:
            self.probe_state = {"uploaded": False, "detail": "探针上传失败：%s" % str(error)[:200]}
            self.toast("warn", self.probe_state["detail"])
            return False
        command = self.board.probe_command()
        self.sessions.open("probe", "板端探针（只读遥测）", command, self.target,
                           dimensions=(24, 120))
        return True

    # ---------- 前置检查 ----------
    def refresh_preflight(self):
        try:
            preflight = self.board.preflight()
            conflicts = wb_status.conflict_nodes(self.telemetry,
                                                 self.config["checks"].get("conflict_nodes", []))
            preflight["conflicts"] = conflicts
            with self.lock:
                self.board_state["preflight"] = preflight
            self.broadcast({"t": "board", "board": {"logs": self.board_state["logs"], "preflight": preflight}})
            for check in preflight["checks"]:
                if not check["ok"]:
                    self.toast("warn", "单实例检查：%s —— %s" % (check["name"], check["detail"]))
        except Exception as error:
            self.toast("error", "单实例检查失败：%s" % str(error)[:200])
        return self.board_state.get("preflight")

    # ---------- 终端 ----------
    def _require_command_transport(self):
        """自检模式（transport=local）默认只用于界面预览，避免误在本机拉起 roscore 等节点。"""
        if self.target.transport == "local" and not self.options.get("allow_local_commands"):
            raise ValueError("当前是 --transport local 自检模式：只用于界面预览。"
                             "确实要在本机执行这些命令请加 --allow-local-commands；连板端请用默认 ssh。")

    def open_session(self, body):
        self._require_command_transport()
        sid = body.get("id")
        terminal = next((t for t in self.config.get("terminals", []) if t["id"] == sid), None)
        if terminal is None:
            raise ValueError("未知终端 %s" % sid)
        if terminal.get("command") is None:
            raise ValueError("%s 由任务组面板生成命令，请用「启动」按钮" % sid)
        if terminal.get("confirm") and body.get("confirm") != "确认":
            raise ValueError("该终端需要现场确认：%s" % terminal["confirm"])
        command = wb_board.terminal_wrapped_command(self.config, wb_board.terminal_command(self.config, terminal))
        # Only the separately confirmed 5a init command may reuse a connection
        # password for sudo. Shells/probes/other terminals and one-off checks cannot.
        allow_sudo = (sid == "servo_init" and bool(terminal.get("confirm"))
                      and terminal.get("command", "").strip().startswith("sudo bash "))
        session = self.sessions.open(sid, terminal.get("title", sid), command, self.target,
                                     allow_sudo_password=allow_sudo)
        item = self.stage.note_action("启动终端 %s：%s" % (sid, wb_board.terminal_command(self.config, terminal)))
        self._emit_event(item)
        self._journal({"action": "session_open", "session": sid, "command": command})
        return {"ok": True, "session": session.snapshot()}

    def open_all_devices(self, body):
        self._require_command_transport()
        if self.orchestration.get("running"):
            raise ValueError("设备启动流程已在运行")
        if body.get("confirm") != "启动设备":
            raise ValueError("需要确认：启动设备命令会真实占用板端设备节点")
        group = next((g for g in self.config.get("groups", []) if g["id"] == body.get("group_id")), None)
        if body.get("group_id") and group is None:
            raise ValueError("未知任务组")
        observation = bool(group and wb_board.is_observation(group))
        if observation and body.get("include_servo"):
            raise ValueError("低空观察不启动舵机")
        include_servo = bool(body.get("include_servo"))
        order = [t for t in self.config.get("terminals", [])
                 if t.get("command") and t["id"] not in ("trial", "monitor")]
        if not include_servo:
            order = [t for t in order if not t.get("optional")]
        if observation:
            order = [t for t in self.config.get("terminals", [])
                     if t["id"] in ("roscore", "mavros", "lidar", "observation_localization")]
        else:
            order = [t for t in order if t["id"] != "observation_localization"]
        self.orchestration = {"running": True, "step": None, "started_at": time.time(),
                              "cancel": False,
                              "steps": [{"id": t["id"], "title": t.get("title", t["id"]),
                                         "state": "pending", "detail": ""} for t in order]}
        thread = threading.Thread(target=self._run_orchestration, args=(order,), name="wb-start-all")
        thread.daemon = True
        thread.start()
        return {"ok": True, "orchestration": self.orchestration}

    def _run_orchestration(self, terminals):
        def publish():
            self.broadcast({"t": "orchestration",
                            "orchestration": {k: v for k, v in self.orchestration.items() if k != "cancel"}})

        try:
            preflight = self.board_state.get("preflight") or self.refresh_preflight() or {}
            leftovers = preflight.get("leftovers") or {}
            if self.target.transport == "ssh":
                self.ensure_probe()
            publish()
            for terminal in terminals:
                if self.orchestration.get("cancel"):
                    break
                step = next(s for s in self.orchestration["steps"] if s["id"] == terminal["id"])
                step["state"] = "running"
                self.orchestration["step"] = terminal["id"]
                publish()
                existing = self.sessions.get(terminal["id"])
                running = (existing is not None and existing.state in ("running", "starting"))
                running = running or any(leftovers.get(key) for key in RUNNING_KEYS.get(terminal["id"], ()))
                if terminal["id"] in ("lidar", "camera", "servo", "observation_localization"):
                    spec = terminal.get("ready") or {}
                    running = running or (self.telemetry.get("at", 0) >= time.time() - 5
                                          and wb_status.ready_check(spec.get("kind"), spec, self.telemetry)[0])
                try:
                    if running:
                        step["detail"] = "板端已有同名进程，跳过重复启动"
                    elif terminal.get("confirm") and not self.options.get("auto_confirm_servo"):
                        step["state"] = "skipped"
                        step["detail"] = "需要单独确认（会复位机构）：请手动启动"
                        publish()
                        continue
                    else:
                        command = wb_board.terminal_wrapped_command(
                            self.config, wb_board.terminal_command(self.config, terminal))
                        self.sessions.open(terminal["id"], terminal.get("title", terminal["id"]),
                                           command, self.target)
                        step["detail"] = "已启动，等待就绪"
                    publish()
                    ready = terminal.get("ready")
                    if ready:
                        ok, detail = self._wait_ready(ready)
                        step["state"] = "ok" if ok else "failed"
                        step["detail"] = detail
                    else:
                        step["state"] = "ok"
                    publish()
                    self._emit_event(self.stage.note_action(
                        "设备步骤 %s：%s" % (terminal["id"], step["detail"]),
                        "ok" if step["state"] == "ok" else "error"))
                    if step["state"] == "failed":
                        self.toast("error", "设备步骤未就绪，已停止后续启动：%s" % terminal["id"])
                        break
                except Exception as error:
                    step["state"] = "failed"
                    step["detail"] = str(error)[:300]
                    publish()
                    self.toast("error", "启动 %s 失败：%s" % (terminal["id"], step["detail"]))
                    break
        finally:
            self.orchestration["running"] = False
            self.orchestration["step"] = None
            publish()

    def _wait_ready(self, ready, timeout=None):
        spec_kind = ready.get("kind")
        timeout = float(timeout or ready.get("timeout", 60))
        deadline = time.time() + timeout
        last = ""
        while time.time() < deadline:
            if self.orchestration.get("cancel"):
                return False, "已取消"
            if time.time() - self.telemetry.get("at", self.telemetry.get("t", 0)) > 5:
                ok, detail = False, "等待新鲜板端遥测（探针可能断开）"
            else:
                ok, detail = wb_status.ready_check(spec_kind, ready, self.telemetry)
            if detail != last:
                last = detail
                step = next((s for s in self.orchestration["steps"]
                             if s["id"] == self.orchestration.get("step")), None)
                if step is not None:
                    step["detail"] = detail
                    self.broadcast({"t": "orchestration",
                                    "orchestration": {k: v for k, v in self.orchestration.items()
                                                      if k != "cancel"}})
            if ok:
                return True, detail
            time.sleep(0.5)
        return False, "超时未就绪（%.0fs）：%s" % (timeout, last)

    def _wait_for(self, predicate, timeout, what):
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.4)
        return False

    def _check_orchestration(self):
        """遥测刷新时唤醒等待中的步骤（等待循环本身也在轮询，这里只做状态提示）。"""
        return

    def stop_all(self):
        self.protect_observation_dependencies()
        if self.orchestration.get("running"):
            self.orchestration["cancel"] = True
            self.toast("warn", "已请求取消设备启动流程")
        self.sessions.close_all()
        self._emit_event(self.stage.note_action("已停止全部工作台会话（设备侧按现场流程确认）", "warn"))
        self._journal({"action": "stop_all"})
        return {"ok": True}

    # ---------- 任务组 ----------
    def protect_observation_dependencies(self, sid=None):
        trial = self.sessions.get("trial")
        if (self.trial.get("route") == "low_observation"
                and self.trial.get("mode") == "flight" and not self.trial.get("check_config")
                and trial is not None and trial.state in ("starting", "running")
                and (sid is None or sid in ("roscore", "mavros", "lidar", "observation_localization"))):
            raise ValueError("低空观察 flight 尚未退出；先飞手手动落地上锁并等待 OBSERVATION_CLOSED，保留定位与MAVROS")

    def trial_command(self, body):
        """Pure command preview: usable offline, no connection or file writes."""
        group = next((g for g in self.config.get("groups", []) if g["id"] == body.get("group_id")), None)
        if group is None:raise ValueError("未知任务组")
        command, note = wb_board.build_group_command(self.config, group, body.get("mode", "preview"),
            route=body.get("route"), real_release=body.get("real_release"),
            check_config=body.get("check_config",False), capture_speed=body.get("capture_speed"),
            capture_lighting=body.get("capture_lighting"), motion_optimized=body.get("motion_optimized",False),
            survey_pattern=body.get("survey_pattern"), resume_survey=body.get("resume_survey"),
            site_geometry=body.get("site_geometry"), geometry_revision=body.get("geometry_revision"))
        return {"ok":True,"body":command,"note":note}

    def start_trial(self, body):
        with self.trial_lock:
            return self._start_trial(body)

    def _start_trial(self, body):
        self._require_command_transport()
        group_id = body.get("group_id")
        group = next((g for g in self.config.get("groups", []) if g["id"] == group_id), None)
        if group is None:
            raise ValueError("未知任务组 %s" % group_id)
        mode = body.get("mode", "flight")
        real_release = body.get("real_release", group.get("release") == "real")
        check_config = body.get("check_config", False)
        motion_optimized = body.get("motion_optimized", False)
        for key, value in (("real_release", real_release), ("check_config", check_config),
                           ("motion_optimized", motion_optimized)):
            if not isinstance(value, bool):
                raise ValueError("%s 必须是布尔值" % key)
        route = body.get("route") or group.get("channel")
        if wb_board.is_observation(group) and real_release:
            raise ValueError("低空观察不支持真实投递")
        # site/start_test.sh selects real hardware itself; never trust a client's
        # real_release=False to turn a delivery flight into a mock flight.
        real_release = (mode == "flight" and not check_config and
                        (real_release or (route == "site" and group.get("release") == "real")))
        if real_release and body.get("confirm") != "实投":
            raise ValueError("实投入口必须输入确认词「实投」")
        if mode == "flight" and not check_config:
            expected_confirm = "实投" if real_release else "启动试飞"
            if body.get("confirm") != expected_confirm:
                raise ValueError("飞行模式必须确认：飞机已回到起飞点、未解锁、机头朝场内")
        if group.get("needs_waypoints") and not check_config and body.get("confirm") != "实投":
            # 仍然允许，但给出明确提醒：入口自身会拒绝空航点
            self.toast("warn", "%s 的走廊航点/H 坐标必须实测填写，留空时入口会拒绝启动（预期行为）" % group["name"])
        current = self.sessions.get("trial")
        if current is not None and current.state in ("running", "starting"):
            raise ValueError("专项入口已在运行；先停止（Ctrl+C）再启动下一组")
        command_body, note = wb_board.build_group_command(
            self.config, group, mode, route=route, real_release=real_release,
            check_config=check_config, capture_speed=body.get("capture_speed"),
            capture_lighting=body.get("capture_lighting"), motion_optimized=motion_optimized,
            survey_pattern=body.get("survey_pattern"), resume_survey=body.get("resume_survey"),
            site_geometry=body.get("site_geometry"), geometry_revision=body.get("geometry_revision"))
        # 界面预览命令必须与后端实际命令一致，否则拒绝启动：防止"给人看的命令"与"真正执行的命令"漂移
        expected = str(body.get("expected_body") or "").strip()
        if body.get("site_geometry") is not None and not expected:
            raise ValueError("手动坐标必须先生成并确认完整命令")
        if expected and expected != command_body.strip():
            raise ValueError("界面预览与后端实际命令不一致，已拒绝启动。后端实际命令：%s" % command_body)
        command = wb_board.terminal_wrapped_command(self.config, command_body)
        self.stage.reset()
        self.stage.observation_mode = wb_board.is_observation(group)
        self.mission_start_attempted = False
        self.stage.note_action("任务组 %s（%s，%s）：%s" % (group["name"], mode, note, command_body), "info")
        self.trial = {
            "group_id": group_id, "group": group.get("key"), "name": group.get("name"),
            "mode": mode, "release": ("none" if mode == "preview" or check_config else
                                     "real" if real_release else
                                     "none" if group.get("release") == "none" else "mock"),
            "real_release": real_release, "command": command_body, "run_dir": None,
            "started_at": time.time(), "check_config": check_config,
            "capture_speed": body.get("capture_speed"), "capture_lighting": body.get("capture_lighting"),
            "motion_optimized": motion_optimized, "survey_pattern": body.get("survey_pattern"),
            "resume_survey": body.get("resume_survey"), "route": route, "note": note,
            "site_geometry": body.get("site_geometry"), "geometry_revision": body.get("geometry_revision"),
        }
        if wb_board.is_observation(group) and mode == "flight" and not check_config:
            session = self.sessions.open("trial", "低空观察 · %s" % group.get("name"), command,
                                         self.target, graceful_only=True)
        else:
            session = self.sessions.open("trial", "专项入口 · %s" % group.get("name"), command, self.target)
        self.broadcast({"t": "trial", "trial": dict(self.trial)})
        self.broadcast({"t": "stage", "stage": self.stage.snapshot()})
        self._journal({"action": "trial_start", "group": group_id, "mode": mode,
                       "real_release": real_release, "command": command})
        return {"ok": True, "trial": dict(self.trial), "session": session.snapshot()}

    def stop_trial(self):
        session = self.sessions.get("trial")
        if session is None or session.state not in ("running", "starting"):
            return {"ok": True, "detail": "专项入口未在运行"}
        try:
            session.send_key("C-c")
        except Exception as error:
            raise ValueError("发送 Ctrl+C 失败：%s" % error)
        detail = ("已请求保持等待飞手接管；落地上锁后等待 OBSERVATION_CLOSED，定位/MAVROS保留"
                  if self.trial.get("route") == "low_observation" else
                  "已发送 Ctrl+C；落地停机后等待 BAG_CLOSED 再断电")
        self._emit_event(self.stage.note_action(detail, "warn"))
        self._journal({"action": "trial_stop"})
        return {"ok": True, "detail": detail}

    def mission_start(self, body):
        self._require_command_transport()
        if self.trial.get("route") == "low_observation":
            raise ValueError("低空观察没有任务管理器，由飞手人工解锁并重新拨入 OFFBOARD")
        if body.get("confirm") != "启动任务":
            raise ValueError("需要确认：仅在 READY 且飞手完成解锁/悬停后调用一次")
        state = self.stage.name
        if state not in ("READY", "IN_FLIGHT"):
            raise ValueError("当前阶段是 %s，只有 READY 之后才允许启动任务" % state)
        session = self.sessions.get("trial")
        if (session is None or session.state not in ("running", "starting")
                or self.trial.get("mode") != "flight" or self.trial.get("check_config")):
            raise ValueError("需要运行中的 flight 专项入口，preview/配置检查不能启动任务")
        if self.stage.auto_sequence:
            raise ValueError("当前入口采用自动时序；由板端监督器启动任务，请勿重复下发")
        with self.trial_lock:
            if self.mission_start_attempted:
                raise ValueError("本轮已经请求启动任务；请查看返回结果，不重复调用")
            self.mission_start_attempted = True
        command = wb_board.terminal_wrapped_command(
            self.config, 'timeout 15 rosservice call /navigation/start_mission "{}"')
        code, output = self.board.run(command, timeout=25.0)
        text = output.strip()
        ok = "success: True" in text or "success: true" in text
        self._emit_event(self.stage.note_action(
            "启动任务服务返回（exit=%s）：%s" % (code, text.replace("\n", " ")[:200]),
            "ok" if ok else "error"))
        self._journal({"action": "mission_start", "exit": code, "output": text[:400]})
        return {"ok": ok, "output": text, "error": None if ok else "服务未返回 success=true，请人工确认"}

    # ---------- 回报 ----------
    def make_report(self):
        markdown = self.stage.report(self.snapshot(include_logs=False)["connection"], self.trial,
                                     self.telemetry)
        if not os.path.isdir(self.report_dir):
            os.makedirs(self.report_dir, mode=0o700)
        name = "%s_%s.md" % (time.strftime("%Y%m%d_%H%M%S"),
                             (self.trial.get("group_id") or "idle"))
        path = os.path.join(self.report_dir, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(markdown)
        self.report = {"path": path, "markdown": markdown, "at": time.time()}
        self.broadcast({"t": "report", "report": self.report})
        self._journal({"action": "report", "path": path})
        return self.report

    def _journal(self, record):
        record = dict(record, at=time.time(), at_text=time.strftime("%Y-%m-%d %H:%M:%S"))
        path = os.path.join(PROFILE_DIR, "ops.jsonl")
        try:
            if not os.path.isdir(PROFILE_DIR):
                os.makedirs(PROFILE_DIR, mode=0o700)
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError:
            pass

    # ---------- 日志浏览 ----------
    def refresh_logs(self):
        runs = self.board.list_logs()
        with self.lock:
            self.board_state["logs"] = runs
        self.broadcast({"t": "board", "board": {"logs": runs,
                                                "preflight": self.board_state["preflight"]}})
        return {"ok": True, "logs": runs}

    def tail_log(self, run, name, lines=200):
        return self.board.tail(run, name, lines)


class Handler(BaseHTTPRequestHandler):
    workbench = None
    server_version = "LiftraceFlightWorkbench/1.0"

    def log_message(self, fmt, *args):
        if os.environ.get("WB_VERBOSE"):
            sys.stderr.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), fmt % args))

    # -- 工具 --
    def _json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False, default=str).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _error(self, message, status=400):
        self._json({"ok": False, "error": str(message)}, status)

    def _body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except ValueError:
            raise ValueError("请求体不是合法 JSON")

    def _static(self, path):
        relative = path.lstrip("/")
        if relative.startswith("static/"):
            relative = relative[len("static/"):]
        if relative in ("", "/"):
            relative = "index.html"
        target = os.path.normpath(os.path.join(WEB_DIR, relative))
        if not target.startswith(WEB_DIR) or not os.path.isfile(target):
            self._error("未找到 %s" % path, 404)
            return
        with open(target, "rb") as handle:
            body = handle.read()
        self.send_response(200)
        self.send_header("Content-Type", CONTENT_TYPES.get(os.path.splitext(target)[1],
                                                           "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    # -- GET --
    def do_GET(self):
        parsed = urlparse(self.path)
        route = parsed.path
        query = parse_qs(parsed.query)
        if route in ("/observe", "/motor"):
            return self._static("/static/observe.html")
        if route in ("/", "/index.html") or route.startswith("/static/"):
            return self._static(route)
        if route == "/api/snapshot":
            return self._json(self.workbench.snapshot())
        if route == "/api/events":
            return self._events()
        if route == "/api/logs/download":
            run = (query.get("run") or [""])[0]
            name = (query.get("file") or [""])[0]
            return self._download(run, name)
        return self._error("未知路径 %s" % route, 404)

    def _download(self, run, name):
        if not run or not name or "/" in name:
            return self._error("参数不合法")
        path = self.workbench.board.abs_path(os.path.join("logs", run, name))
        code, output = wb_ssh.run_bytes(self.workbench.target, "cat -- %s" % wb_ssh.quote(path),
                                        timeout=120.0)
        if code != 0:
            return self._error("读取失败：%s" % output[:200], 500)
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Disposition", 'attachment; filename="%s"' % name)
        self.send_header("Content-Length", str(len(output)))
        self.end_headers()
        self.wfile.write(output)

    def _events(self):
        workbench = self.workbench
        client = workbench.subscribe()
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            hello = {"t": "hello", "snapshot": workbench.snapshot()}
            self.wfile.write(("data: %s\n\n" % json.dumps(hello, ensure_ascii=False,
                                                          default=str)).encode("utf-8"))
            self.wfile.flush()
            while True:
                try:
                    payload = client.get(timeout=15.0)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
                    continue
                if payload is None:
                    break
                self.wfile.write(("data: %s\n\n" % payload).encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            workbench.unsubscribe(client)

    # -- POST --
    def do_POST(self):
        route = urlparse(self.path).path
        try:
            body = self._body()
        except ValueError as error:
            return self._error(error)
        workbench = self.workbench
        try:
            if route == "/api/config":
                return self._json(workbench.update_config(body))
            if route == "/api/connect":
                return self._json(workbench.connect(body))
            if route == "/api/disconnect":
                return self._json(workbench.disconnect())
            if route == "/api/action/preflight":
                preflight = workbench.refresh_preflight()
                return self._json({"ok": True, "board": {"preflight": preflight}})
            if route == "/api/action/start_all":
                return self._json(workbench.open_all_devices(body))
            if route == "/api/action/stop_all":
                return self._json(workbench.stop_all())
            if route == "/api/action/mission_start":
                return self._json(workbench.mission_start(body))
            if route == "/api/action/report":
                return self._json({"ok": True, "report": workbench.make_report()})
            if route == "/api/session/open":
                return self._json(workbench.open_session(body))
            if route == "/api/session/input":
                session = workbench.sessions.get(body.get("id"))
                if session is None:
                    raise ValueError("会话不存在")
                session.write(body.get("data", ""))
                return self._json({"ok": True})
            if route == "/api/session/close":
                workbench.protect_observation_dependencies(body.get("id"))
                workbench.sessions.close(body.get("id"))
                return self._json({"ok": True})
            if route == "/api/session/clear":
                session = workbench.sessions.get(body.get("id"))
                if session is not None:
                    session.clear()
                return self._json({"ok": True})
            if route == "/api/session/resize":
                session = workbench.sessions.get(body.get("id"))
                if session is not None:
                    session.resize(body.get("rows", 36), body.get("cols", 140))
                return self._json({"ok": True})
            if route == "/api/session/key":
                workbench.protect_observation_dependencies(body.get("id"))
                session = workbench.sessions.get(body.get("id"))
                if session is None:
                    raise ValueError("会话不存在")
                session.send_key(body.get("key", "C-c"))
                return self._json({"ok": True})
            if route == "/api/survey/plan":
                return self._json({"ok": True, "plan": wb_survey.plan(body)})
            if route == "/api/trial/command":
                return self._json(workbench.trial_command(body))
            if route == "/api/trial/start":
                return self._json(workbench.start_trial(body))
            if route == "/api/trial/stop":
                return self._json(workbench.stop_trial())
            if route == "/api/logs/refresh":
                return self._json(workbench.refresh_logs())
            if route == "/api/logs/tail":
                return self._json(dict({"ok": True},
                                       **workbench.tail_log(body.get("run"), body.get("file"),
                                                            body.get("lines", 200))))
        except ValueError as error:
            return self._error(error)
        except Exception as error:  # noqa: BLE001  (把任何后端异常变成可见错误)
            return self._error("%s: %s" % (type(error).__name__, error), 500)
        return self._error("未知路径 %s" % route, 404)


def main():
    parser = argparse.ArgumentParser(description="Liftrace 试飞验证看板")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址（默认只监听本机）")
    parser.add_argument("--port", type=int, default=8791)
    parser.add_argument("--config", default=wb_board.DEFAULT_CONFIG_PATH)
    parser.add_argument("--transport", default="ssh", choices=("ssh", "local"),
                        help="local 用于离线界面预览：默认拒绝执行设备/入口命令")
    parser.add_argument("--allow-local-commands", action="store_true",
                        help="配合 --transport local：允许真的在本机执行设备/入口命令（自检用）")
    parser.add_argument("--password-file", default=None,
                        help="从文件读取一次口令（也可用 ORANGEPI_SSH_PASSWORD 环境变量）")
    parser.add_argument("--open", action="store_true", help="启动后尝试打开浏览器")
    parser.add_argument("--profile-dir", default=None,
                        help="状态目录（profile/日志/回报）；默认 ~/.config/liftrace-flight-workbench，"
                             "自检或离线预览时指向临时目录可避免动到真实状态")
    parser.add_argument("--auto-confirm-servo", action="store_true",
                        help="一键启动设备时把舵机两个可选终端也纳入流程（会复位机构，需现场确认）")
    options = parser.parse_args()

    global PROFILE_DIR
    if options.profile_dir:
        PROFILE_DIR = os.path.abspath(os.path.expanduser(options.profile_dir))
        wb_board.DEFAULT_PROFILE_DIR = PROFILE_DIR
    config = wb_board.load_config(options.config)
    profile = wb_board.load_profile()
    wb_board.apply_profile(config, profile)
    password = None
    if options.password_file:
        with open(os.path.expanduser(options.password_file), "r", encoding="utf-8") as handle:
            password = handle.readline().rstrip("\r\n")
    elif profile.get("password"):
        password = profile.get("password")
    elif os.environ.get("ORANGEPI_SSH_PASSWORD"):
        password = os.environ["ORANGEPI_SSH_PASSWORD"]

    workbench = Workbench(config, {
        "password": password,
        "transport": options.transport,
        "auto_confirm_servo": options.auto_confirm_servo,
        "allow_local_commands": options.allow_local_commands,
    })
    Handler.workbench = workbench

    port = options.port
    for _ in range(20):
        try:
            server = ThreadingHTTPServer((options.host, port), Handler)
            break
        except OSError:
            port += 1
    else:
        raise SystemExit("找不到可用端口，请用 --port 指定")

    server.daemon_threads = True
    url = "http://%s:%d/" % (options.host if options.host != "0.0.0.0" else "127.0.0.1", port)
    print("=" * 68)
    print("Liftrace 试飞验证看板")
    print("  界面地址：%s" % url)
    print("  板端目标：%s（%s）" % (workbench.target.describe(), workbench.target.host))
    print("  工程根目录：%s" % config["connection"].get("board_root"))
    print("  状态目录：%s" % PROFILE_DIR)
    print("  提醒：只启动现场既有入口命令；不自动解锁、不自动起飞、不代替飞手接管。")
    print("=" * 68)
    sys.stdout.flush()
    if options.open:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n收尾：停止全部会话 …")
        workbench.sessions.close_all()
        server.shutdown()


if __name__ == "__main__":
    main()
