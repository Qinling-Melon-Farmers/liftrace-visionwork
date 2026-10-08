#!/usr/bin/env python3
"""板端操作：配置装载、命令拼装、单实例检查、探针上传、日志浏览。

这里只生成"现场手册里已有"的命令字符串，不发明新流程；所有命令都会先在界面上
原样展示给飞手确认。
"""
import base64
import json
import os
import posixpath
import re
import time

import yaml
import wb_geometry

from wb_ssh import run_once, quote

TOOL_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(TOOL_DIR, "workbench.yaml")
DEFAULT_PROFILE_DIR = os.path.join(os.path.expanduser("~"), ".config", "liftrace-flight-workbench")

REAL_RELEASE_FOLDERS = ("01_visual_interrupt", "02_high_view_revisit", "05_low_multi",
                        "06_high_priority", "08_full_mission")
SURVEY_FOLDERS = ("02_high_view_revisit", "06_high_priority", "07_memory_only",
                  "08_full_mission", "09_high_speed_capture")
RESUME_FOLDERS = ("06_high_priority", "08_full_mission")
OBSERVATION_PROFILES = ("hover", "forward", "square")
COMPETITION_ENV_SCRIPT = "deployment/competition/environment.sh"


def is_competition_connection(connection):
    return connection.get("env_script") == COMPETITION_ENV_SCRIPT


def competition_asset_paths(connection):
    """Keep model/metadata inside the selected project; never borrow another root."""
    root = posixpath.normpath(connection.get("board_root", ""))
    if not posixpath.isabs(root):
        raise ValueError("独立正赛 board_root 必须是绝对路径")
    paths = {}
    for key in ("model", "metadata"):
        value = connection.get(key)
        if not isinstance(value, str) or not value.strip() or any(c in value for c in "\r\n\x00\\"):
            raise ValueError("独立正赛 %s 必须填写工程内相对路径" % key)
        relative = posixpath.relpath(value, root) if posixpath.isabs(value) else value
        relative = posixpath.normpath(relative)
        if relative in (".", "..") or relative.startswith("../") or ":" in relative:
            raise ValueError("独立正赛 %s 不得引用其他工程；请填写当前根内相对路径" % key)
        paths[key] = relative
    return paths


def is_observation(group):
    return group.get("channel") == "low_observation"

# ssh 层错误 → 现场可执行建议。ssh 失败时远端脚本根本不会执行，不能把"没有标记输出"
# 解释成"板端目录/文件缺失"（换板后免密没配时最容易踩）。
SSH_ERROR_HINTS = (
    (re.compile(r"Permission denied \(publickey,password\)|Permission denied \(publickey\)|"
                r"Permission denied"),
     "SSH 认证失败：这块板没有我们任何免密公钥，且工作台没有口令。"
     "在「连接」里填 SSH 口令（可勾「记住」存到本机 profile，0600），"
     "或把公钥装进板端 ~/.ssh/authorized_keys 后重连。"),
    (re.compile(r"REMOTE HOST IDENTIFICATION HAS CHANGED|Host key verification failed|"
                r"Offending (?:ED25519|RSA|ECDSA) key"),
     "板端主机指纹与 known_hosts 不一致（换板常见）。确认是新板后执行 "
     "ssh-keygen -R <板端地址> 删除旧记录，再连接。"),
    (re.compile(r"Could not resolve hostname|Name or service not known"),
     "主机名无法解析：检查「板端地址」是否写对。"),
    (re.compile(r"Connection timed out|Operation timed out|No route to host|"
                r"Network is unreachable"),
     "网络不通或板端未上电：确认板子已开机、与本机同网段（可先 ping）。"),
    (re.compile(r"Connection refused"),
     "板端拒绝连接：sshd 未运行或端口不是 22。"),
    (re.compile(r"Too many authentication failures"),
     "尝试的密钥过多被拒：在 ~/.ssh/config 里为该主机指定 IdentityFile 并加 "
     "IdentitiesOnly yes。"),
    (re.compile(r"Connection closed by|kex_exchange_identification|Connection reset by peer"),
     "SSH 握手被中断：sshd 可能刚重启或链路抖动，稍后重试。"),
)


def classify_ssh_error(text):
    """从 ssh 输出识别认证/网络层错误，返回 (原因, 处理建议)；没有则 (None, "")。"""
    for pattern, hint in SSH_ERROR_HINTS:
        match = pattern.search(text or "")
        if match:
            return match.group(0), hint
    return None, ""


# ---- 配置 ----

def load_config(path=None):
    with open(path or DEFAULT_CONFIG_PATH, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    config.setdefault("connection", {})
    config.setdefault("probe", {})
    config.setdefault("checks", {})
    config.setdefault("terminals", [])
    config.setdefault("groups", [])
    config["connection"]["host_options"] = host_options(config)
    return config


def host_options(config):
    """历史外场 SSH 地址（界面下拉用）。只接受 {host, label} 形状，去重保序。"""
    options = []
    seen = set()
    for item in (config.get("connection", {}).get("host_options") or []):
        if isinstance(item, str):
            host, label = item, ""
        elif isinstance(item, dict):
            host, label = item.get("host"), item.get("label", "")
        else:
            continue
        host = (host or "").strip()
        if not host or host in seen:
            continue
        seen.add(host)
        options.append({"host": host, "label": str(label or "")})
    current = (config.get("connection", {}).get("host") or "").strip()
    if current and current not in seen:
        options.insert(0, {"host": current, "label": "当前配置"})
    return options


def substitute(template, variables):
    """把 {board_root} 之类的占位符替换成实际值（两遍，允许参数里再引用）。"""
    if template is None:
        return None
    text = str(template)
    for _ in range(2):
        for key, value in variables.items():
            text = text.replace("{%s}" % key, str(value))
    return text


def connection_variables(config):
    connection = config["connection"]
    return {
        "board_root": connection.get("board_root", ""),
        "site_dir": connection.get("site_dir", ""),
        "env_script": connection.get("env_script", ""),
        "board_python": connection.get("board_python", "/usr/bin/python3"),
        "model": connection.get("model", ""),
        "metadata": connection.get("metadata", ""),
        "probe_path": connection.get("probe_path", "logs/flight_workbench/board_probe.py"),
    }


def terminal_command(config, terminal, overrides=None):
    """生成某个终端的完整命令（含参数替换）。"""
    command = terminal.get("command")
    if command is None:
        return None
    variables = connection_variables(config)
    variables.update({k: v for k, v in (terminal.get("params") or {}).items()})
    variables.update(overrides or {})
    return substitute(command, variables)


def group_variables(config, group, overrides=None):
    variables = connection_variables(config)
    variables.update({
        "folder": group.get("folder", ""),
        "key": group.get("key", ""),
        "site_config": group.get("site_config") or "{site_dir}/test_area.yaml",
    })
    variables.update(overrides or {})
    return variables


def build_group_command(config, group, mode, route=None, real_release=None,
                        check_config=False, capture_speed=None, capture_lighting=None,
                        motion_optimized=False, survey_pattern=None, resume_survey=None,
                        site_geometry=None, geometry_revision=None,
                        motion_optimization=None, obstacle_columns=None, competition_config=None, speed_profile=None):
    """按现场手册拼出任务组启动命令。返回 (命令, 说明)。"""
    if mode not in ("preview", "flight"):
        raise ValueError("mode 必须是 preview 或 flight")
    variables = group_variables(config, group)
    folder = group.get("folder", "")
    site_dir = config["connection"].get("site_dir", "deployment/site_20260928")
    route = route or group.get("channel", "module")
    if group.get("channel") == "competition":
        if not isinstance(motion_optimized, bool):
            raise ValueError("motion_optimized 必须是布尔值")
        if route != "competition":
            raise ValueError("正赛卡片仅允许独立 competition 入口")
        if any(value is not None for value in (capture_speed, capture_lighting, survey_pattern,
                site_geometry, geometry_revision, speed_profile)) or motion_optimized:
            raise ValueError("正赛使用独立场地配置和 on/off 开关，不接受专项参数")
        if not isinstance(check_config, bool) or (real_release is not None and not isinstance(real_release, bool)):
            raise ValueError("check_config / real_release 必须是布尔值")
        # This entry is real-only. A client must not relabel it as mock.
        if mode == "flight" and not check_config and real_release is False:
            raise ValueError("独立正赛为实投入口，不支持模拟投递")
        field = competition_config if competition_config is not None else group.get("site_config")
        if not isinstance(field, str) or not field.strip() or any(ch in field for ch in "\r\n\x00"):
            raise ValueError("请填写独立正赛场地配置路径")
        extra = ""
        for option, value in (("motion-optimization", motion_optimization), ("resume-survey", resume_survey), ("obstacle-columns", obstacle_columns)):
            if value is not None:
                if value not in ("on", "off"):
                    raise ValueError("%s 只允许 on/off；省略表示继承所选 YAML" % option)
                extra += " --%s %s" % (option, value)
        entry = group.get("entry", "deployment/competition/start.sh")
        command = "bash %s %s --site-config %s%s" % (
            quote(entry), "preview" if check_config else mode, quote(field.strip()), extra)
        if check_config:
            command += " --check-config"
        return command, "独立正赛实投：所选 YAML 为参数权威；省略开关继承 YAML，on/off 显式覆盖。模板未确认且必填坐标为空；成功测试场地不代表 10×10 正赛。先执行配置检查核对有效参数。"
    if any(value is not None for value in (motion_optimization, obstacle_columns, competition_config)):
        raise ValueError("on/off 开关及正赛配置路径仅支持独立正赛卡片")
    if is_observation(group):
        if route != "low_observation" or group.get("profile") not in OBSERVATION_PROFILES:
            raise ValueError("非法低空观察入口或 profile")
        if real_release or motion_optimized or any(value is not None for value in (
                capture_speed, capture_lighting, survey_pattern, resume_survey,
                site_geometry, geometry_revision, speed_profile)):
            raise ValueError("低空观察继承原 profile，不支持投递或专项参数")
        if not isinstance(check_config, bool):
            raise ValueError("check_config 必须是布尔值")
        return ("bash deployment/low_hover_observation/start.sh %s %s" % (
                    "preview" if check_config else mode, group["profile"]),
                "preview 为离线配置展开（无 ROS）；flight 仅观察动作和诊断录包，飞手人工解锁并重新拨入 OFFBOARD")
    if route not in ("site", "module"):
        raise ValueError("未知入口类型")
    site_config = substitute(variables["site_config"], variables)
    if real_release is None:
        real_release = group.get("release") == "real"
    for name, value in (("real_release", real_release), ("check_config", check_config),
                        ("motion_optimized", motion_optimized)):
        if not isinstance(value, bool):
            raise ValueError("%s 必须是布尔值" % name)
    module_base = "deployment/board_trials_4x4/%s" % folder

    extra = ""
    if speed_profile is not None:
        if folder != "08_full_mission" or route != "module" or speed_profile not in ("limited", "competition"):
            raise ValueError("08速度版本只支持模块08的 limited/competition")
        extra += " --speed-profile " + speed_profile
    if capture_speed is not None:
        if folder != "09_high_speed_capture" or isinstance(capture_speed, bool) or float(capture_speed) not in (.5, 1., 1.2):
            raise ValueError("拍摄速度只允许第6组的 0.5/1.0/1.2 m/s")
        extra += " --capture-speed %.1f" % float(capture_speed)
    if capture_lighting is not None:
        if folder != "09_high_speed_capture" or capture_lighting not in ("normal", "dim", "unspecified"):
            raise ValueError("非法拍摄光照标签")
        extra += " --capture-lighting %s" % capture_lighting
    if motion_optimized:
        extra += " --motion-optimized"
    if survey_pattern is not None:
        if folder not in SURVEY_FOLDERS or survey_pattern not in ("rectangle", "snake2", "snake3"):
            raise ValueError("该组不支持所选扫描路线")
        extra += " --survey-pattern %s" % survey_pattern
    if resume_survey is not None:
        if folder not in RESUME_FOLDERS or resume_survey not in ("on", "off"):
            raise ValueError("高位续扫仅允许第五组/整场的 on 或 off")
        extra += " --resume-survey %s" % resume_survey

    if site_geometry is not None:
        if 'survey_plan' in site_geometry:
            pattern = wb_geometry.validate_geometry(site_geometry)['survey_plan']['pattern']
            if survey_pattern is not None and survey_pattern != pattern:
                raise ValueError('航线下拉选项与自动生成路线不一致，请重新生成')
            survey_pattern = pattern
        if route != "module":raise ValueError("手动坐标需使用模块入口")
        overlay, prepare, clean = wb_geometry.overlay_command(site_config, folder, site_geometry, geometry_revision)
        selected = dict(group, site_config=overlay)
        command, note = build_group_command(config, selected, mode, route=route,
            real_release=real_release, check_config=check_config, capture_speed=capture_speed,
            capture_lighting=capture_lighting, motion_optimized=motion_optimized,
            survey_pattern=survey_pattern, resume_survey=resume_survey, speed_profile=speed_profile)
        return prepare + " && " + command, "先验证实测坐标并生成独立现场配置；" + note

    if check_config:
        return ("bash %s/start.sh preview --site-config %s%s --check-config" % (module_base, quote(site_config), extra),
                "只做配置检查：按模块入口展开参数，不启动任何 ROS 节点")

    if route == "site":
        if motion_optimized or survey_pattern is not None or resume_survey is not None:
            raise ValueError("现场快捷入口不支持优化参数，请改用模块入口")
        return ("bash %s/start_test.sh %s %s%s" % (site_dir, group.get("key"), mode, extra),
                "现场快捷入口：flight 对投递组自动走 start_real.sh（真实舵机），记忆组走 start.sh")
    if real_release and mode == "flight":
        if folder not in REAL_RELEASE_FOLDERS or "real" not in group.get("release_options", ["mock", "real"]):
            raise ValueError("%s 没有 start_real.sh，不能走实投入口" % folder)
        return ("bash %s/start_real.sh --site-config %s%s" % (module_base, quote(site_config), extra),
                "模块实投入口：经释放许可代理调用现场 /legacy/Servo_raw（真实舵机）")
    return ("bash %s/start.sh %s --site-config %s%s" % (module_base, mode, quote(site_config), extra),
            "模块入口：默认模拟投递（mock 舵机），不接 PWM")


def supervisor_command(config, body):
    """把任务组命令包成板端终端的完整命令（cd + source 现场环境 + 原命令）。"""
    connection = config["connection"]
    return "cd %s && source %s && %s" % (
        quote(connection.get("board_root", ".")),
        quote(posixpath.join(connection.get("board_root", "."), connection.get("env_script", ""))),
        body)


def terminal_wrapped_command(config, body):
    connection = config["connection"]
    return "cd %s && source %s && %s" % (
        quote(connection.get("board_root", ".")),
        quote(posixpath.join(connection.get("board_root", "."), connection.get("env_script", ""))),
        body)


# ---- 板端客户端 ----

class BoardClient(object):
    def __init__(self, config, target):
        self.config = config
        self.target = target

    # -- 基础 --
    def run(self, command, timeout=30.0):
        return run_once(self.target, command, timeout=timeout)

    @property
    def root(self):
        return self.config["connection"].get("board_root", "")

    def abs_path(self, relative):
        return posixpath.join(self.root, relative)

    # -- 连接自检 --
    def test_connection(self):
        connection = self.config["connection"]
        env_script = self.abs_path(connection.get("env_script", ""))
        model = self.abs_path(connection.get("model", ""))
        metadata = self.abs_path(connection.get("metadata", ""))
        script = "\n".join([
            'echo "HOST=$(hostname)"',
            'echo "WHOAMI=$(whoami)"',
            'echo "UNAME=$(uname -srm)"',
            'if [ -d %s ]; then echo "ROOT=OK"; else echo "ROOT=MISSING"; fi' % quote(self.root),
            'if [ -f %s ]; then echo "ENV=OK"; else echo "ENV=MISSING"; fi' % quote(env_script),
            'if [ -f %s ]; then echo "MODEL=OK"; else echo "MODEL=MISSING"; fi' % quote(model),
            'if [ -f %s ]; then echo "META=OK"; else echo "META=MISSING"; fi' % quote(metadata),
            'df -Pk %s 2>/dev/null | tail -1 | awk \'{print "DISK_FREE_KB="$4}\'' % quote(self.root),
            "cd %s 2>/dev/null && source %s >/dev/null 2>&1; %s - <<'WBPY'\n"
            "import sys\n"
            "try:\n"
            "    import rospy\n"
            "    print('ROSPY=OK ' + sys.version.split()[0])\n"
            "except Exception as error:\n"
            "    print('ROSPY=FAIL ' + str(error)[:160])\n"
            "WBPY" % (quote(self.root), quote(env_script),
                      quote(connection.get("board_python", "/usr/bin/python3"))),
        ])
        code, output = self.run(script, timeout=max(20.0, self.target.connect_timeout + 10))
        info = {"raw": output, "exit_code": code}
        for line in output.splitlines():
            if "=" in line:
                key, _, value = line.partition("=")
                key = key.strip()
                if key in ("HOST", "WHOAMI", "UNAME", "ROOT", "ENV", "MODEL", "META",
                           "DISK_FREE_KB", "ROSPY"):
                    info[key.lower()] = value.strip()
        ok = (code == 0 and info.get("root") == "OK" and info.get("env") == "OK"
              and str(info.get("rospy", "")).startswith("OK"))
        info["ok"] = ok
        ssh_reason, ssh_hint = classify_ssh_error(output)
        info["ssh_error"] = ssh_reason or ""
        info["ssh_ok"] = not ssh_reason
        if ok:
            info["detail"] = "%s（%s）；rospy %s" % (
                info.get("host", "?"), info.get("uname", "?"), info.get("rospy", ""))
        elif ssh_reason:
            # ssh 都没连上时，远端脚本没执行过；不能把"没有标记输出"说成板端文件缺失
            info["auth_failed"] = "Permission denied" in ssh_reason
            info["detail"] = "SSH 层失败：%s —— %s" % (ssh_reason, ssh_hint)
        else:
            reasons = []
            if info.get("root") != "OK":
                reasons.append("工程根目录不存在：%s" % self.root)
            if info.get("env") != "OK":
                reasons.append("现场环境脚本缺失：%s" % env_script)
            if info.get("model") not in (None, "OK"):
                reasons.append("RKNN 模型缺失：%s" % model)
            if info.get("meta") not in (None, "OK"):
                reasons.append("模型 metadata 缺失：%s" % metadata)
            if not str(info.get("rospy", "")).startswith("OK"):
                reasons.append("板端 ROS Python 不可用：%s" % info.get("rospy"))
            if not reasons:
                tail = " / ".join(line.strip() for line in output.splitlines()[-3:] if line.strip())
                reasons.append("远端命令未返回预期标记（exit=%s）：%s" % (code, tail[:300]))
            info["detail"] = "；".join(reasons)
        return info

    # -- 单实例检查 --
    def preflight(self):
        connection = self.config["connection"]
        competition = is_competition_connection(connection)
        selected_groups = [g for g in self.config.get("groups", [])
                           if not competition or g.get("channel") == "competition"]
        script = []
        for name in self.config["checks"].get("process_names", []):
            script.append('printf "PROC|%s|"; pgrep -x %s | wc -l' % (name, quote(name)))
        script.append('df -Pk %s 2>/dev/null | tail -1 | awk \'{print "DISK|"$4}\'' % quote(self.root))
        script.append('if [ -d %s ]; then echo "LOGSDIR=OK"; else echo "LOGSDIR=MISSING"; fi'
                      % quote(self.abs_path("logs")))
        for group in selected_groups:
            if group.get("channel") == "competition":
                entry = self.abs_path(group.get("entry", "deployment/competition/start.sh"))
                field = self.abs_path(group.get("site_config", "deployment/competition/field.example.yaml"))
                script.append('if [ -f %s ] && [ -f %s ]; then echo "COMPETITION=OK"; else echo "COMPETITION=MISSING"; fi'
                              % (quote(entry), quote(field)))
                continue
            folder = group.get("folder", "")
            base = self.abs_path("deployment/low_hover_observation" if is_observation(group)
                                 else "deployment/board_trials_4x4/%s" % folder)
            settings = "profiles.yaml" if is_observation(group) else "settings.yaml"
            script.append(
                'printf "GROUP|%s|"; if [ -f %s/start.sh ]; then printf "start=1|"; else printf "start=0|"; fi; '
                'if [ -f %s/start_real.sh ]; then printf "real=1|"; else printf "real=0|"; fi; '
                'if [ -f %s/%s ]; then echo "settings=1"; else echo "settings=0"; fi'
                % (folder, quote(base), quote(base), quote(base), settings))
        env_script = self.abs_path(connection.get("env_script", ""))
        site_dir = connection.get("site_dir", "deployment/site_20260928")
        site_config = self.abs_path("deployment/competition/field.example.yaml" if competition
                                    else posixpath.join(site_dir, "test_area.yaml"))
        script.append('if [ -f %s ]; then echo "SITECFG=OK"; else echo "SITECFG=MISSING"; fi' % quote(site_config))
        if competition:
            # Read the template only. Do not generate configuration or contact ROS.
            field_python = r'''import json,sys
try:
    import yaml
    with open(sys.argv[1],encoding="utf-8") as handle:
        field=yaml.safe_load(handle)
    confirmed=field.get("site_confirmed") if isinstance(field,dict) else None
    if not isinstance(confirmed,bool):raise ValueError("site_confirmed must be a YAML boolean")
    report={"ok":True,"site_confirmed":confirmed}
except Exception as error:
    report={"ok":False,"detail":type(error).__name__+": "+str(error)[:200]}
print("FIELD|"+json.dumps(report,separators=(",",":")))
'''
            script.append("%s - %s <<'WBFIELD'\n%sWBFIELD" %
                          (quote(connection.get("board_python", "/usr/bin/python3")), quote(site_config), field_python))
        # Only inspect the checkout and import generated type definitions. This
        # does not query ROS, execute a service, or establish a running revision.
        version_python = r'''import importlib,json,subprocess,sys
from pathlib import Path
root=Path(sys.argv[1])
report={"source":{},"interfaces":{},"resume_cli":False}
try:
    head=subprocess.check_output(["git","-C",str(root),"rev-parse","--short","HEAD"],stderr=subprocess.DEVNULL,text=True).strip()
    dirty=bool(subprocess.check_output(["git","-C",str(root),"status","--porcelain"],stderr=subprocess.DEVNULL,text=True).strip())
    report["source"]={"head":head,"dirty":dirty}
except Exception:
    report["source"]={"error":"source Git revision unavailable"}
try:
    entry=root/(sys.argv[2] if len(sys.argv)>2 else "deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py")
    report["resume_cli"]="--resume-survey" in entry.read_text(encoding="utf-8")
except Exception:
    report["resume_error"]="source trial entry unavailable"
def fields(text):
    result=[]
    for line in text.splitlines():
        line=line.split("#",1)[0].strip()
        if not line or "=" in line:continue
        parts=line.split()
        if len(parts)!=2:raise ValueError("invalid source field declaration")
        kind,name=parts
        result.append((name,"std_msgs/Header" if kind=="Header" else kind))
    return result
def compare(source,generated,required):
    for name,kind in required.items():
        if (name,kind) not in source:raise ValueError("source missing latest field "+name+":"+kind)
    actual=list(zip(generated.__slots__,generated._slot_types))
    if actual!=source:raise ValueError("source/generated field names, order or types differ; rebuild complete message/service consumers")
identity={"mission_id":"string","decision_seq":"uint32","attempt":"uint16","target_first_seen":"time","permission_epoch":"string","permission_revision":"uint64"}
definitions=[("ReleaseAuthorization","patrol_control.msg","patrol_control/msg/ReleaseAuthorization.msg"),
             ("ReleasePermission","uav_mission.msg","uav_mission/msg/ReleasePermission.msg"),
             ("ServoAction","patrol_control.srv","patrol_control/srv/ServoAction.srv")]
for name,module,path in definitions:
    try:
        text=(root/"patrol_uav_ws-patrol_planner/src"/path).read_text(encoding="utf-8")
        generated=getattr(importlib.import_module(module),name)
        expected_type=module.split(".")[0]+"/"+name
        if generated._type!=expected_type:raise ValueError("wrong generated ROS type")
        if name=="ServoAction":
            request,response=text.split("---")
            compare(fields(request),generated._request_class,dict(identity,request_id="uint64",payload_slot="uint8"))
            compare(fields(response),generated._response_class,{"execution_state":"uint8","terminal":"bool","request_id":"uint64"})
        else:
            compare(fields(text),generated,dict(identity,payload_slot="uint8",permitted="bool"))
        report["interfaces"][name]={"ok":True,"detail":"source/generated field schema matches"}
    except Exception as error:
        report["interfaces"][name]={"ok":False,"detail":type(error).__name__+": "+str(error)[:240]}
print("VERSION|"+json.dumps(report,separators=(",",":")))
'''
        cli_source = ("patrol_uav_ws-patrol_planner/src/uav_mission/scripts/competition_supervisor.py" if competition
                      else "deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py")
        script.append("if cd %s && source %s >/dev/null 2>&1; then\n%s - %s %s <<'WBVERSION'\n%sWBVERSION\n"
                      "else echo 'VERSION|{\"error\":\"site environment unavailable\"}'; fi" %
                      (quote(self.root), quote(env_script), quote(connection.get("board_python", "/usr/bin/python3")),
                       quote(self.root), quote(cli_source), version_python))
        code, output = self.run("\n".join(script), timeout=30.0)
        processes = {}
        groups = {}
        disk_free_kb = None
        version = {}
        field_state = {}
        for line in output.splitlines():
            if line.startswith("PROC|"):
                _, name, count = (line.split("|") + ["", ""])[:3]
                processes[name] = count.strip()
            elif line.startswith("DISK|"):
                disk_free_kb = line.split("|")[1].strip()
            elif line.startswith("GROUP|"):
                parts = line.split("|")
                if len(parts) >= 4:
                    groups[parts[1]] = {
                        "start": parts[2].endswith("1"),
                        "real": parts[3].endswith("1"),
                        "settings": len(parts) > 4 and parts[4].endswith("1"),
                    }
            elif line.startswith("VERSION|"):
                try:
                    value = json.loads(line.partition("|")[2])
                    if isinstance(value, dict):version = value
                except (TypeError, ValueError):pass
            elif line.startswith("FIELD|"):
                try:
                    value = json.loads(line.partition("|")[2])
                    if isinstance(value, dict):field_state = value
                except (TypeError, ValueError):pass
        leftovers = {name: int(count) for name, count in processes.items() if count.isdigit() and int(count) > 0}
        free_gb = (float(disk_free_kb) / (1024.0 * 1024.0)) if disk_free_kb and disk_free_kb.isdigit() else None
        min_free = float(self.config["checks"].get("min_free_gb", 2.0))
        results = []
        results.append({"name": "板端登录与工程根", "ok": code == 0, "detail":
                        ("命令退出码 %s" % code) if code else "命令执行完成"})
        results.append({"name": "正赛场地模板文件" if competition else "现场范围配置", "ok": "SITECFG=OK" in output, "detail":
                        "%s %s" % (site_config,
                            "存在" if "SITECFG=OK" in output else "缺失或检查结果不可用；请显式指定有效 --site-config")})
        if competition:
            field_known = field_state.get("ok") is True and isinstance(field_state.get("site_confirmed"), bool)
            results.append({"name": "正赛模板状态（非场地验收）", "ok": field_known, "detail":
                            ("site_confirmed=%s（%s）；仅文件声明，未验证实测几何或飞行配置" %
                             (str(field_state["site_confirmed"]).lower(),
                              "待实测确认" if not field_state["site_confirmed"] else "文件声明已确认"))
                            if field_known else "unknown：%s；不能认定场地已确认" %
                            field_state.get("detail", "模板状态未读到")})
        results.append({"name": "录像空间（≥%.1fGB）" % min_free, "ok":
                        (free_gb is not None and free_gb >= min_free),
                        "detail": ("%.1fGB 可用" % free_gb) if free_gb is not None else "无法读取 df"})
        if leftovers:
            results.append({"name": "本机残留进程", "ok": False,
                            "detail": "；".join("%s×%d" % (k, v) for k, v in sorted(leftovers.items()))})
        elif len(processes) == len(self.config["checks"].get("process_names", [])) and all(v.isdigit() for v in processes.values()):
            results.append({"name": "本机残留进程", "ok": True, "detail": "无 roscore/roslaunch/gzserver/px4/mavros 残留"})
        else:
            results.append({"name": "本机残留进程", "ok": False, "detail": "unknown：未取得完整进程检查结果"})
        expected = {g.get("folder", "") for g in selected_groups if g.get("channel") != "competition"}
        missing = [folder for folder in expected if not groups.get(folder, {}).get("start")
                   or not groups.get(folder, {}).get("settings")]
        if competition:
            results.append({"name": "任务组入口", "ok": bool(selected_groups) and "COMPETITION=OK" in output,
                            "detail": "仅检查独立正赛入口与模板，不要求08或其他专项模块"})
        elif missing:
            results.append({"name": "任务组入口", "ok": False,
                            "detail": "start.sh/settings.yaml（观察组为profiles.yaml）缺失或未读到：%s" % "、".join(sorted(missing))})
        elif expected:
            results.append({"name": "任务组入口", "ok": True,
                            "detail": "%d 个入口与配置均存在（观察组为profiles.yaml）" % len(groups)})
        else:
            results.append({"name": "任务组入口", "ok": False, "detail": "unknown：未配置可检查的模块入口"})
        source = version.get("source") or {}
        if any(g.get("channel") == "competition" for g in selected_groups):
            results.append({"name": "独立正赛入口与模板", "ok": "COMPETITION=OK" in output,
                            "detail": "仅核查文件存在；场地确认、有效参数及CLI支持须执行独立配置检查" if "COMPETITION=OK" in output
                                      else "独立正赛入口/模板缺失或未读到；不以08专项代替"})
        source_known = bool(source.get("head")) and isinstance(source.get("dirty"), bool)
        results.append({"name": "源码版本（非运行版本）", "ok": source_known, "detail":
                        ("HEAD %s；工作区%s；仅源码目录，未核实运行程序版本" %
                         (source["head"], "有改动" if source.get("dirty") else "干净"))
                        if source_known else "unknown：无法读取源码 Git HEAD/dirty"})
        results.append({"name": "高位续扫 CLI", "ok": version.get("resume_cli") is True, "detail":
                        "源码支持 --resume-survey on/off（默认继承）" if version.get("resume_cli") is True
                        else "unknown/FAIL：源码入口不可用或未同步 --resume-survey"})
        for name in ("ReleaseAuthorization", "ReleasePermission", "ServoAction"):
            info = (version.get("interfaces") or {}).get(name) or {}
            results.append({"name": "%s 生成接口" % name, "ok": info.get("ok") is True,
                            "detail": ("源码与已生成字段类型/顺序一致；不代表运行二进制版本") if info.get("ok") is True
                            else "FAIL：%s；需成套同步并重构建控制/任务及消费者，不能只同步 Python" %
                            info.get("detail", version.get("error", "unknown：接口检查结果不可用"))})
        results.append({"name": "LIO 配置提示（非运行证明）", "ok": True, "detail":
                        "最新板端构建要求 FAST_LIO_MATCH_THREADS=3；本检查不证明编译缓存或实际运行线程"})
        return {"at": time.time(), "checks": results, "leftovers": leftovers,
                "disk_free_gb": free_gb, "groups": groups, "version": version,
                "field_template": field_state, "raw": output}

    # -- 探针 --
    def probe_local_source(self):
        with open(os.path.join(TOOL_DIR, "board_probe.py"), "r", encoding="utf-8") as handle:
            return handle.read()

    def upload_probe(self):
        relative = self.config["connection"].get("probe_path", "logs/flight_workbench/board_probe.py")
        remote = self.abs_path(relative)
        payload = base64.b64encode(self.probe_local_source().encode("utf-8")).decode("ascii")
        python = self.config["connection"].get("board_python", "/usr/bin/python3")
        script = "\n".join([
            "mkdir -p %s" % quote(os.path.dirname(remote)),
            "base64 -d > %s <<'WBB64'" % quote(remote),
            payload,
            "WBB64",
            "%s -m py_compile %s && echo PROBE_UPLOAD_OK" % (quote(python), quote(remote)),
        ])
        code, output = self.run(script, timeout=40.0)
        if "PROBE_UPLOAD_OK" not in output:
            raise RuntimeError("探针上传失败（exit=%s）：%s" % (code, output.strip()[-400:]))
        return remote

    def probe_command(self, interval=None):
        connection = self.config["connection"]
        probe = self.config["probe"]
        relative = connection.get("probe_path", "logs/flight_workbench/board_probe.py")
        remote = self.abs_path(relative)
        topics = ",".join(probe.get("topics", []))
        services = ",".join(probe.get("services", []))
        env_script = self.abs_path(connection.get("env_script", ""))
        return "cd %s && source %s >/dev/null 2>&1 && exec %s %s --interval %s --topics %s --services %s --terminal-hover-topic %s --lio-realtime-topic %s --observe-topics %s" % (
            quote(self.root), quote(env_script),
            quote(connection.get("board_python", "/usr/bin/python3")), quote(remote),
            interval or probe.get("interval", 1.0), quote(topics), quote(services),
            quote(probe.get("terminal_hover_topic", "")), quote(probe.get("lio_realtime_topic", "")),
            quote(json.dumps(probe.get("observe_topics", {}), separators=(",", ":"))))

    # -- 日志与产物 --
    def list_logs(self, limit=40):
        script = "\n".join([
            "cd %s 2>/dev/null || exit 0" % quote(self.abs_path("logs")),
            "for d in $(ls -1dt board_* low_hover_* 2>/dev/null | head -%d); do" % int(limit),
            '  [ -d "$d" ] || continue',
            '  echo "RUN|$d|$(stat -c %Y "$d" 2>/dev/null)"',
            '  find "$d" -maxdepth 1 -type f -printf "FILE|%f|%s\\n" 2>/dev/null | sort',
            "done",
        ])
        code, output = self.run(script, timeout=30.0)
        runs = []
        current = None
        for line in output.splitlines():
            parts = line.split("|")
            if line.startswith("RUN|") and len(parts) >= 2:
                current = {"run": parts[1], "mtime": float(parts[2]) if len(parts) > 2 and parts[2] else 0.0,
                           "trial": parts[1].split("_")[1] if parts[1].startswith("board_") and len(parts[1].split("_")) > 1 else "",
                           "files": []}
                runs.append(current)
            elif line.startswith("FILE|") and current is not None and len(parts) >= 3:
                try:
                    size = int(parts[2])
                except ValueError:
                    size = 0
                current["files"].append({"name": parts[1], "size": size})
        return runs

    def tail(self, run, name, lines=200):
        path = self.abs_path(posixpath.join("logs", run, name))
        code, output = self.run("tail -n %d -- %s" % (int(lines), quote(path)), timeout=25.0)
        return {"ok": code == 0, "text": output, "path": path}

    def read_file(self, run, name, max_bytes=2000000):
        path = self.abs_path(posixpath.join("logs", run, name))
        code, output = self.run("if [ -f %s ]; then head -c %d -- %s; else echo __MISSING__; fi"
                                % (quote(path), int(max_bytes), quote(path)), timeout=40.0)
        return {"ok": code == 0 and "__MISSING__" not in output, "text": output, "path": path}


def profile_path(name="profile.json"):
    return os.path.join(DEFAULT_PROFILE_DIR, name)


def load_profile():
    path = profile_path()
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def save_profile(profile):
    directory = os.path.dirname(profile_path())
    if not os.path.isdir(directory):
        os.makedirs(directory, mode=0o700)
    path = profile_path()
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(profile, handle, ensure_ascii=False, indent=2)
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass
    return path


def apply_profile(config, profile):
    """把（不含口令的）连接档案覆盖到配置上。"""
    mapping = {
        "host": "host", "port": "port", "board_root": "board_root", "site_dir": "site_dir",
        "env_script": "env_script", "model": "model", "metadata": "metadata",
    }
    for key, target in mapping.items():
        if profile.get(key) not in (None, ""):
            config["connection"][target] = profile[key]
    if isinstance(profile.get("identity_file"), str):
        config["connection"]["identity_file"] = profile["identity_file"]
    if profile.get("auto_password") is not None:
        config["connection"]["auto_password"] = bool(profile["auto_password"])
    return config
