#!/usr/bin/env python3
"""试飞看板的状态语义层（纯函数，可离线单测，不依赖 ROS/SSH）。

输入是两类文本：
  1) 板端 run_trial.py 监督进程的控制台输出（INITIALIZING / MAPPING_READY / READY / FLIGHT_STATUS ...）
  2) 板端 board_probe.py 每秒打印的一行 JSON 遥测

输出是阶段机状态、告警和回报文本。这里不启动、不解锁、不调用任何服务。
"""
import ast
import json
import re
import time

# 阶段顺序只用于界面展示；FAILED 不属于顺序推进。
STAGE_ORDER = [
    "IDLE",
    "STARTING",
    "INITIALIZING",
    "MAPPING_READY",
    "READY",
    "IN_FLIGHT",
    "DISARMED",
    "STOPPED",
    "FAILED",
]

STAGE_LABELS = {
    "IDLE": "未启动",
    "STARTING": "启动中",
    "INITIALIZING": "初始化中（定位/相机/坐标一致）",
    "MAPPING_READY": "地图就绪（MAPPING_READY）",
    "READY": "就绪（READY，可人工解锁）",
    "IN_FLIGHT": "飞行中",
    "DISARMED": "已上锁/落地",
    "STOPPED": "应用已退出",
    "FAILED": "失败",
}

# 定位/LIO 一致性拒绝原因 → 现场可执行提示
ALIGNMENT_HINTS = {
    "stable": "飞控与 LIO 坐标一致",
    "settling": "正在稳定计时，等待即可",
    "pose_missing": "缺一路位姿（飞控或 LIO）",
    "vehicle_not_fresh_disarmed": "飞控状态不新鲜或未处于未解锁状态",
    "clock_reset": "检测到时间回退；不要在空中重启应用",
    "pose_stale_or_future": "位姿时间戳过期或来自未来，检查时间同步与链路延迟",
    "pose_stamp_skew": "两路位姿时间戳错开，检查 LIO 延迟",
    "pose_nonfinite": "位姿出现非法数值",
    "fc_lio_disagreement": "飞控与 LIO 坐标不一致：等其自行收敛，不要转动机身去追数值",
}

# 失败文本 → 现场提示（按顺序匹配，先匹配到的先用）
FAILURE_HINTS = [
    (r"Stop the old application first:\s*(.*)",
     "旧应用仍在运行。先退出整机/旧专项应用（设备驱动可保留），再启动本入口。"),
    (r"Start device MAVROS and driver2 first",
     "设备链未就绪：先启动 roscore / MAVROS / MID360 / 相机，再启动专项入口。"),
    (r"No stationary disarmed camera_init reference",
     "没有采到静止未解锁的地面参考：检查 map↔camera_init 转换、初始机头朝向（应朝场内 +X）、"
     "相机与压缩图是否新鲜，以及飞机是否已静置、未解锁。入口不会手填 ground_z。"),
    (r"No fresh nonempty FreeDOM map after localization convergence",
     "定位收敛后仍无新鲜非空 FreeDOM 地图：检查 mapping.log 与雷达点云输入；不要在空中重启。"),
    (r"Mapping initialization lost agreement; stop and rerun from empty map",
     "建图初始化失去一致性：按入口要求停机并从空地图重跑，禁止在空中清图/重启。"),
    (r"Vision chain not live; inspect application.log:\s*(.*)",
     "视觉链未上线，缺这些话题：检查 application.log 与 RKNN 模型/metadata。"),
    (r"Controller setpoints did not become live",
     "控制器设定点未上线：检查 patrol_control 启动错误与 /navigation/setpoint_mission。"),
    (r"Less than 2GB recording space available",
     "板端录像空间不足 2GB：先清理 logs/ 或换盘，再启动。"),
    (r"CameraInfo frame does not match inherited camera optical TF",
     "CameraInfo 的 frame 与继承的光学 TF 不一致：核对相机驱动与 TF，不要在任务里改死内参。"),
    (r"Image and CameraInfo frames differ",
     "图像与 CameraInfo 的 frame 不一致：检查相机驱动发布。"),
    (r"Localization launch exited",
     "localization.launch 退出：看 logs 目录下 localization.log。"),
    (r"Application failed before model readiness",
     "应用在模型就绪前退出：看 application.log（模型路径/metadata/依赖）。"),
    (r"A launch exited; inspect logs",
     "某个 launch 在运行中退出：看对应 .log，先落地再由现场判断。"),
    (r"Unexpected hardware Servo service type",
     "/legacy/Servo_raw 服务类型不是 patrol_control/Servo：核对现场舵机服务。"),
    (r"RKNN model not found",
     "找不到 RKNN 模型：设置 UAV_VISION_RKNN_MODEL_PATH 或用 --model 指定。"),
    (r"Model metadata not found",
     "找不到模型 metadata：用 --metadata 指定与权重匹配的五分类 YAML。"),
    (r"Fill corridor_waypoints with at least two measured waypoints",
     "走廊航点留空：必须填实测航点，入口按设计拒绝启动（不要用仿真航点替代）。"),
    (r"Fill landing_xy with the measured H center",
     "H 中心坐标留空：填入实测 H 中心后再启动。"),
    (r"Waypoint/H outside configured flight_area",
     "实测航点/H 超出配置的飞行范围：核对外场实测坐标与 test_area.yaml。"),
    (r"This module does not permit release",
     "该专项不允许真实投递：改用对应投递组或改回模拟。"),
    (r"Unexpected camera_init origin",
     "起飞点 camera_init 原点异常（偏移 >0.3m）：先检查定位，不要起飞。"),
    (r"Board trials refuse /use_sim_time=true",
     "板端入口拒绝模拟时钟：确认没有把 SITL 环境带到实机。"),
    (r"FreeDOM map stale before READY",
     "READY 前地图变陈旧：检查雷达点云与建图输入是否稳定。"),
    (r"Localization/mapping launch exited before map readiness",
     "定位/建图 launch 在地图就绪前退出：看 localization.log / mapping.log。"),
    (r"Less than 2GiB available for bag",
     "板端录包空间不足 2GiB：先清理 logs/ 或换盘。"),
    (r"^(?:Invalid flight_area|Survey point outside flight_area|Search bounds outside|"
     r"Origin/staging outside|Capture )",
     "现场范围/航点参数不合法：检查 --site-config 与 settings.yaml。"),
    (r"^usage: ",
     "命令行参数不合法：检查任务组、模式与附加参数。"),
]

ERROR_PATTERNS = [
    r"^Traceback \(most recent call last\):",
    r"^RuntimeError:",
    r"^ValueError:",
    r"^ROSInitException:",
    r"^RLException:",
    r"^\[ERROR\]",
    r"^usage: ",
    r"ERROR: cannot launch node",
    r"required process .* has died",
    r"process has died",
]


def parse_python_payload(text):
    """解析 run_trial.py 打印的 Python dict repr（不是 JSON，键用单引号）。"""
    text = (text or "").strip()
    if not text:
        return None
    try:
        value = ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return None
    return value if isinstance(value, dict) else None


def parse_probe_line(line):
    """解析 board_probe.py 的一行 JSON；不是 JSON 就返回 None。"""
    line = (line or "").strip()
    if not line.startswith("{"):
        return None
    try:
        value = json.loads(line)
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


def failure_hint(text):
    for pattern, hint in FAILURE_HINTS:
        match = re.search(pattern, text)
        if match:
            return hint, match.group(1) if match.groups() else ""
    return None, ""


def marker(text):
    """返回监督进程输出的标记名与载荷文本。"""
    for name in ("INITIALIZING", "MAPPING_READY", "FLIGHT_STATUS"):
        if text.startswith(name):
            return name, text[len(name):]
    if text.startswith("READY:"):
        return "READY", text[len("READY:"):].strip()
    if text.startswith("AUTO_SEQUENCE_CANCELLED_PILOT_MODE_CHANGE"):
        return "AUTO_CANCELLED", ""
    if text.startswith("AUTO_SEQUENCE:"):
        return "AUTO_SEQUENCE", text[len("AUTO_SEQUENCE:"):].strip()
    if text.startswith("ARMED_AUTO_OFFBOARD_REQUEST"):
        return "AUTO_OFFBOARD_REQUEST", text[len("ARMED_AUTO_OFFBOARD_REQUEST"):].strip()
    if text.startswith("AUTO_OFFBOARD_FAILED"):
        return "AUTO_OFFBOARD_FAILED", text[len("AUTO_OFFBOARD_FAILED"):].strip()
    if text.startswith("AUTO_MISSION_START_FAILED_NO_RETRY"):
        return "AUTO_MISSION_START_FAILED", text[len("AUTO_MISSION_START_FAILED_NO_RETRY"):].strip()
    if text.startswith("AUTO_MISSION_START"):
        return "AUTO_MISSION_START", text[len("AUTO_MISSION_START"):].strip()
    if text.startswith("WAITING_FOR_MANUAL_MISSION_START"):
        return "WAITING_MANUAL_START", text[len("WAITING_FOR_MANUAL_MISSION_START:"):].strip()
    if text.startswith("Trial application stopped"):
        return "STOPPED", text
    if text.startswith("CONFIG_VALID"):
        return "CONFIG_VALID", text
    if text.startswith("Ground/reference and all local-Z limits generated automatically"):
        return "GROUND_REFERENCE", text
    if text.startswith("After local inspection, operator chooses flight mode/arming"):
        return "MANUAL_START_HINT", text
    return None, ""


class StageTracker:
    """把监督进程控制台输出累积成阶段、告警和回报。"""

    def __init__(self, alert_interval=20.0):
        self.alert_interval = float(alert_interval)
        self.reset()

    def reset(self):
        self.name = "IDLE"
        self.since = None
        self.history = []
        self.detail = {}
        self.alignment = None
        self.armed = None
        self.ever_armed = False
        self.mode = None
        self.phase = None
        self.reason = ""
        self.run_dir = None
        self.ready_at = None
        self.started_at = None
        self.stopped_at = None
        self.last_status_at = None
        self.auto_sequence = None
        self.mission_start = None
        self.alerts = []
        self.timeline = []
        self._pending = ""
        self._last_alert_at = {}
        self._failure = None

    # ---- 基础记录 ----
    def _event(self, kind, **payload):
        payload["kind"] = kind
        return payload

    def _timeline(self, text, level="info", at=None):
        item = {"at": float(at if at is not None else time.time()),
                "level": level, "text": text}
        self.timeline.append(item)
        return self._event("timeline", item=item)

    def _alert(self, level, text, hint="", key=None, at=None, throttle=True):
        now = float(at if at is not None else time.time())
        stamp = now
        if throttle and key:
            last = self._last_alert_at.get(key)
            if last is not None and now - last < self.alert_interval:
                return None
            self._last_alert_at[key] = now
        item = {"id": "%s-%.3f" % (key or level, stamp), "at": stamp,
                "level": level, "text": text, "hint": hint}
        self.alerts.append(item)
        del self.alerts[:-200]
        return self._event("alert", alert=item)

    def set_stage(self, name, detail=None, at=None):
        now = float(at if at is not None else time.time())
        if name == self.name and detail is None:
            return None
        if name != self.name:
            self.history.append({"name": name, "at": now, "detail": detail or {}})
            if name == "READY":
                self.ready_at = now
            if name == "STOPPED":
                self.stopped_at = now
            self.name = name
            self.since = now
        if detail is not None:
            # 关键量合并保留：界面上的"关键量"不应因为阶段推进而消失
            self.detail.update(detail)
        return self._event("stage", stage=self.snapshot())

    # ---- 主入口 ----
    def feed(self, text):
        """喂入一段控制台文本，返回事件列表。"""
        events = []
        if not text:
            return events
        self._pending += text.replace("\r\n", "\n").replace("\r", "\n")
        while "\n" in self._pending:
            line, self._pending = self._pending.split("\n", 1)
            events.extend(self.feed_line(line))
        return [event for event in events if event]

    def feed_line(self, line):
        events = []
        text = line.strip()
        if not text:
            return events
        if self.started_at is None:
            self.started_at = time.time()
        name, payload = marker(text)

        if name == "INITIALIZING":
            events.extend(self._on_initializing(payload))
        elif name == "MAPPING_READY":
            events.extend(self._on_mapping_ready(payload))
        elif name == "READY":
            self.run_dir = payload.strip() or self.run_dir
            events.append(self.set_stage("READY", {"run_dir": self.run_dir}))
            events.append(self._timeline("READY：%s" % (self.run_dir or ""), "ok"))
            events.append(self._alert(
                "ok", "READY：应用链已就绪，可以按现场流程人工解锁。",
                "READY 只是应用就绪，不是起飞/任务开始；飞手确认悬停稳定后才会进入航线。",
                key="ready", throttle=False))
        elif name == "FLIGHT_STATUS":
            events.extend(self._on_flight_status(payload))
        elif name == "STOPPED":
            events.append(self.set_stage("STOPPED", {"line": text}))
            events.append(self._timeline("应用已退出（设备 MAVROS/雷达保留）", "info"))
        elif name == "CONFIG_VALID":
            events.append(self._timeline("配置检查通过（未启动任何 ROS 节点）", "ok"))
        elif name == "AUTO_SEQUENCE":
            self.auto_sequence = payload
            events.append(self._timeline("自动时序：人工解锁 → OFFBOARD → 低空稳定 → 启动任务（不会自动解锁）", "info"))
        elif name == "AUTO_CANCELLED":
            events.append(self._alert("warn", "飞手改了模式，程序不再抢回控制（AUTO_SEQUENCE_CANCELLED）",
                                      "接管优先；如需继续任务，由现场判断，不要在空中重启应用。",
                                      key="auto_cancelled", throttle=False))
        elif name == "AUTO_OFFBOARD_REQUEST":
            self.mission_start = self.mission_start or {}
            events.append(self._timeline("已请求 OFFBOARD%s" % ("（已取消）" if "True" in payload else ""), "info"))
        elif name == "AUTO_OFFBOARD_FAILED":
            events.append(self._alert("error", "请求 OFFBOARD 失败：%s" % payload,
                                      "服务失败不盲目重试；由飞手决定是否重试。", key="offboard_failed", throttle=False))
        elif name == "AUTO_MISSION_START":
            self.mission_start = {"at": time.time(), "detail": payload}
            success = payload.strip().startswith("True")
            events.append(self._timeline("任务开始服务返回：%s" % payload, "ok" if success else "error"))
            if not success:
                events.append(self._alert("error", "启动任务未成功：%s" % payload,
                                          "板端不会自动重试；查看任务状态，由飞手处理。",
                                          key="mission_start_false", throttle=False))
        elif name == "AUTO_MISSION_START_FAILED":
            events.append(self._alert("error", "启动任务失败：%s" % payload,
                                      "确认已 READY、已解锁并进入 OFFBOARD；不要为催促重复调用。",
                                      key="mission_start_failed", throttle=False))
        elif name == "WAITING_MANUAL_START":
            events.append(self._alert("info", "READY 后仍在等待人工启动任务（本组为手动时序）",
                                      "飞手解锁并稳定后，在监测终端执行一次 rosservice call /navigation/start_mission \"{}\"。",
                                      key="waiting_manual_start", throttle=True))
        elif name == "MANUAL_START_HINT" or name == "GROUND_REFERENCE":
            pass
        else:
            events.extend(self._on_other(text))
        return events

    # ---- 分项处理 ----
    def _on_initializing(self, payload):
        events = []
        detail = parse_python_payload(payload) or {}
        self.detail.update(detail)
        alignment = detail.get("mapping_alignment") or {}
        if isinstance(alignment, str):
            alignment = {"reason": alignment}
        self.alignment = alignment.get("reason")
        if self.name in ("IDLE", "STARTING"):
            events.append(self.set_stage("INITIALIZING", detail))
            events.append(self._timeline("进入 INITIALIZING（等待静止未解锁地面参考）", "info"))
        else:
            events.append(self._event("stage", stage=self.snapshot()))
        events.append(self._alignment_alert(alignment))
        armed = detail.get("armed")
        self.armed = armed if armed is not None else self.armed
        if armed is True:
            events.append(self._alert(
                "warn", "初始化期间检测到飞控已解锁",
                "本入口要求未解锁静置采样地面参考；若不是有意操作，请先上锁并检查流程。",
                key="armed_during_init"))
        return events

    def _on_mapping_ready(self, payload):
        detail = parse_python_payload(payload) or {}
        self.detail.update(detail)
        events = [self.set_stage("MAPPING_READY", detail),
                  self._timeline("MAPPING_READY：FreeDOM 地图已有新鲜非空输入", "ok")]
        clouds = detail.get("distinct_clouds")
        if clouds is not None:
            events.append(self._timeline("地图预热帧数 distinct_clouds=%s" % clouds, "info"))
        return events

    def _on_flight_status(self, payload):
        events = []
        status = parse_python_payload(payload) or {}
        self.mode = status.get("mode", self.mode)
        self.armed = status.get("armed", self.armed)
        self.phase = status.get("phase", self.phase)
        self.reason = status.get("reason", self.reason) or ""
        self.last_status_at = time.time()
        if self.armed:
            if not self.ever_armed:
                self.ever_armed = True
                events.append(self._timeline("飞控已解锁（armed=True）", "info"))
            if self.name in ("READY", "MAPPING_READY", "STARTING"):
                events.append(self.set_stage("IN_FLIGHT", {"mode": self.mode, "phase": self.phase}))
                events.append(self._timeline("进入飞行阶段：mode=%s phase=%s" % (self.mode, self.phase), "info"))
        elif self.ever_armed and self.name == "IN_FLIGHT":
            events.append(self.set_stage("DISARMED", {"mode": self.mode, "phase": self.phase}))
            events.append(self._timeline("飞控已上锁（armed=False），等待应用收尾", "info"))
        events.append(self._event("stage", stage=self.snapshot()))
        return events

    def _alignment_alert(self, alignment):
        reason = alignment.get("reason")
        if reason in (None, "stable"):
            return None
        if reason == "settling":
            return None
        if reason == "fc_lio_disagreement":
            delta = []
            if alignment.get("position_delta_m") is not None:
                delta.append("位置差 %.3fm" % float(alignment["position_delta_m"]))
            if alignment.get("yaw_delta_deg") is not None:
                delta.append("航向差 %.1f°" % float(alignment["yaw_delta_deg"]))
            return self._alert(
                "warn",
                "飞控与 LIO 坐标不一致（fc_lio_disagreement%s）" % ("；" + "，".join(delta) if delta else ""),
                ALIGNMENT_HINTS["fc_lio_disagreement"],
                key="fc_lio_disagreement")
        return self._alert("info", "初始化等待：%s" % (ALIGNMENT_HINTS.get(reason, reason),),
                           "", key="align_" + str(reason))

    def _on_other(self, text):
        events = []
        hint, captured = failure_hint(text)
        for pattern in ERROR_PATTERNS:
            if re.search(pattern, text):
                if captured:
                    text = "%s → %s" % (text, captured)
                events.append(self._alert("error", text, hint or "", key="err", throttle=False))
                if self.name not in ("STOPPED", "FAILED"):
                    events.append(self.set_stage("FAILED", {"error": text, "hint": hint or ""}))
                    events.append(self._timeline("失败：%s" % text, "error"))
                return events
        if hint:
            events.append(self._alert("error", text, hint, key="err-hint", throttle=False))
            if self.name not in ("STOPPED", "FAILED"):
                events.append(self.set_stage("FAILED", {"error": text, "hint": hint}))
            return events
        return events

    def note_action(self, text, level="info"):
        return self._timeline(text, level)

    def note_alert(self, level, text, hint="", key=None):
        return self._alert(level, text, hint, key=key, throttle=False)

    # ---- 输出 ----
    def snapshot(self):
        return {
            "name": self.name,
            "label": STAGE_LABELS.get(self.name, self.name),
            "since": self.since,
            "started_at": self.started_at,
            "ready_at": self.ready_at,
            "armed": self.armed,
            "ever_armed": self.ever_armed,
            "mode": self.mode,
            "phase": self.phase,
            "reason": self.reason,
            "alignment": self.alignment,
            "alignment_hint": ALIGNMENT_HINTS.get(self.alignment, ""),
            "detail": self.detail,
            "run_dir": self.run_dir,
            "auto_sequence": self.auto_sequence,
            "mission_start": self.mission_start,
            "last_status_at": self.last_status_at,
            "history": self.history[-40:],
        }

    def report(self, connection=None, trial=None, telemetry=None):
        return build_report(self, connection or {}, trial or {}, telemetry or {})


def build_report(tracker, connection, trial, telemetry):
    """生成可复制/留档的"回报"文本（Markdown）。"""
    stage = tracker.snapshot()
    lines = []
    lines.append("# 试飞工作台状态回报")
    lines.append("")
    lines.append("- 生成时间：%s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("- 板端：`%s` root=`%s`" % (connection.get("host", "?"), connection.get("board_root", "?")))
    if trial:
        lines.append("- 任务组：%s（mode=%s，release=%s）" % (
            trial.get("name") or trial.get("group"), trial.get("mode"), trial.get("release")))
        if trial.get("command"):
            lines.append("- 启动命令：`%s`" % trial["command"])
        if trial.get("run_dir"):
            lines.append("- 产物目录：`%s`" % trial["run_dir"])
    lines.append("")
    lines.append("## 当前阶段")
    lines.append("")
    lines.append("- 阶段：**%s**（%s）" % (stage["label"], stage["name"]))
    if stage.get("ready_at"):
        lines.append("- READY 时刻：%s" % time.strftime("%H:%M:%S", time.localtime(stage["ready_at"])))
    lines.append("- 飞控：armed=%s mode=%s phase=%s reason=%s" % (
        stage.get("armed"), stage.get("mode"), stage.get("phase"), stage.get("reason") or "-"))
    lines.append("- 定位一致性：%s（%s）" % (stage.get("alignment") or "-", stage.get("alignment_hint") or "-"))
    detail = stage.get("detail") or {}
    if detail:
        keep = {k: detail[k] for k in ("pose_samples", "camera_info", "image_seen", "compressed_fresh",
                                       "distinct_clouds", "mapping_started", "map_ready") if k in detail}
        if keep:
            lines.append("- 关键量：%s" % json.dumps(keep, ensure_ascii=False))
    if telemetry:
        state = telemetry.get("state") or {}
        if state:
            lines.append("- 遥测飞控状态：connected=%s armed=%s mode=%s" % (
                state.get("connected"), state.get("armed"), state.get("mode")))
        mission = telemetry.get("mission")
        if mission:
            lines.append("- 任务状态：%s" % json.dumps(mission, ensure_ascii=False))
    lines.append("")
    lines.append("## 阶段时间线")
    lines.append("")
    if stage.get("history"):
        for item in stage["history"]:
            lines.append("- %s  %s" % (time.strftime("%H:%M:%S", time.localtime(item["at"])), item["name"]))
    else:
        lines.append("- （尚无阶段记录）")
    lines.append("")
    lines.append("## 事件时间线")
    lines.append("")
    for item in tracker.timeline[-40:]:
        lines.append("- %s  [%s] %s" % (time.strftime("%H:%M:%S", time.localtime(item["at"])),
                                        item["level"], item["text"]))
    if not tracker.timeline:
        lines.append("- （无）")
    lines.append("")
    lines.append("## 告警")
    lines.append("")
    if tracker.alerts:
        for item in tracker.alerts[-20:]:
            lines.append("- %s  [%s] %s" % (time.strftime("%H:%M:%S", time.localtime(item["at"])),
                                            item["level"], item["text"]))
            if item.get("hint"):
                lines.append("  - 提示：%s" % item["hint"])
    else:
        lines.append("- （无）")
    lines.append("")
    lines.append("> 说明：本回报只描述工作台观测到的应用状态与日志，不代表飞行验收结论；"
                 "录包 PASS 不等于飞行 PASS，是否通过由现场按对应 Gate 判定。")
    return "\n".join(lines) + "\n"


# ---- 设备就绪判定（供顺序启动编排使用）----

def ready_check(kind, spec, telemetry):
    """返回 (是否就绪, 说明)。telemetry 为 board_probe 的最新一条。"""
    spec = spec or {}
    telemetry = telemetry or {}
    if kind == "ros_master":
        if telemetry.get("master"):
            return True, "ROS master 已就绪"
        return False, "等待 ROS master（roscore）"
    if kind == "mavros_connected":
        state = telemetry.get("state") or {}
        age = ((telemetry.get("topics") or {}).get("/mavros/state") or {}).get("age")
        if state.get("connected") and age is not None and 0 <= age <= 2:
            return True, "MAVROS 已连接飞控（mode=%s armed=%s）" % (state.get("mode"), state.get("armed"))
        return False, "等待 MAVROS 连接飞控（当前 connected=%s）" % state.get("connected")
    if kind == "topic":
        topic = spec.get("topic")
        info = (telemetry.get("topics") or {}).get(topic)
        if not info:
            return False, "等待话题 %s" % topic
        age = info.get("age")
        if age is not None and age <= 2.0 and info.get("count"):
            return True, "%s 新鲜（%.1fs 前，%.1fHz）" % (topic, age, info.get("hz") or 0.0)
        return False, "等待 %s 数据（age=%s）" % (topic, age)
    if kind == "service":
        service = spec.get("service")
        found = (telemetry.get("services") or {}).get(service)
        if found:
            return True, "服务 %s 存在（类型 %s）" % (service, found)
        return False, "等待服务 %s" % service
    return True, "无就绪条件"


def conflict_nodes(telemetry, conflicts):
    nodes = set((telemetry or {}).get("nodes") or [])
    return sorted(nodes.intersection(conflicts))
