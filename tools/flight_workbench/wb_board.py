#!/usr/bin/env python3
"""板端操作：配置装载、命令拼装、单实例检查、探针上传、日志浏览。

这里只生成"现场手册里已有"的命令字符串，不发明新流程；所有命令都会先在界面上
原样展示给飞手确认。
"""
import base64
import json
import os
import re
import time

import yaml

from wb_ssh import run_once, quote

TOOL_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(TOOL_DIR, "workbench.yaml")
DEFAULT_PROFILE_DIR = os.path.join(os.path.expanduser("~"), ".config", "liftrace-flight-workbench")

REAL_RELEASE_FOLDERS = ("01_visual_interrupt", "02_high_view_revisit", "05_low_multi",
                        "06_high_priority", "08_full_mission")

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
        "site_config": group.get("site_config") or "deployment/site_20260928/test_area.yaml",
    })
    variables.update(overrides or {})
    return variables


def build_group_command(config, group, mode, route=None, real_release=False,
                        check_config=False, capture_speed=None, capture_lighting=None):
    """按现场手册拼出任务组启动命令。返回 (命令, 说明)。"""
    if mode not in ("preview", "flight"):
        raise ValueError("mode 必须是 preview 或 flight")
    variables = group_variables(config, group)
    folder = group.get("folder", "")
    site_dir = config["connection"].get("site_dir", "deployment/site_20260928")
    route = route or group.get("channel", "module")
    site_config = variables["site_config"]
    module_base = "deployment/board_trials_4x4/%s" % folder

    extra = ""
    if capture_speed is not None:
        if folder != "09_high_speed_capture" or isinstance(capture_speed, bool) or float(capture_speed) not in (.5, 1.):
            raise ValueError("拍摄速度只允许第6组的 0.5/1.0 m/s")
        extra += " --capture-speed %.1f" % float(capture_speed)
    if capture_lighting:
        if folder != "09_high_speed_capture" or capture_lighting not in ("normal", "dim", "unspecified"):
            raise ValueError("非法拍摄光照标签")
        extra += " --capture-lighting %s" % capture_lighting

    if check_config:
        return ("bash %s/start.sh preview --site-config %s%s --check-config" % (module_base, site_config, extra),
                "只做配置检查：按模块入口展开参数，不启动任何 ROS 节点")

    if route == "site":
        if extra and folder != "09_high_speed_capture":
            raise ValueError("现场快捷入口只有拍摄组支持附加参数，请改用模块入口")
        return ("bash %s/start_test.sh %s %s%s" % (site_dir, group.get("key"), mode, extra),
                "现场快捷入口：flight 对投递组自动走 start_real.sh（真实舵机），记忆组走 start.sh")
    if real_release and mode == "flight":
        if folder not in REAL_RELEASE_FOLDERS:
            raise ValueError("%s 没有 start_real.sh，不能走实投入口" % folder)
        return ("bash %s/start_real.sh --site-config %s%s" % (module_base, site_config, extra),
                "模块实投入口：经释放许可代理调用现场 /legacy/Servo_raw（真实舵机）")
    return ("bash %s/start.sh %s --site-config %s%s" % (module_base, mode, site_config, extra),
            "模块入口：默认模拟投递（mock 舵机），不接 PWM")


def supervisor_command(config, body):
    """把任务组命令包成板端终端的完整命令（cd + source 现场环境 + 原命令）。"""
    connection = config["connection"]
    return "cd %s && source %s && %s" % (
        quote(connection.get("board_root", ".")),
        quote(os.path.join(connection.get("board_root", "."), connection.get("env_script", ""))),
        body)


def terminal_wrapped_command(config, body):
    connection = config["connection"]
    return "cd %s && source %s && %s" % (
        quote(connection.get("board_root", ".")),
        quote(os.path.join(connection.get("board_root", "."), connection.get("env_script", ""))),
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
        return os.path.join(self.root, relative)

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
        script = []
        for name in self.config["checks"].get("process_names", []):
            script.append('printf "PROC|%s|"; pgrep -x %s | wc -l' % (name, quote(name)))
        script.append('df -Pk %s 2>/dev/null | tail -1 | awk \'{print "DISK|"$4}\'' % quote(self.root))
        script.append('if [ -d %s ]; then echo "LOGSDIR=OK"; else echo "LOGSDIR=MISSING"; fi'
                      % quote(self.abs_path("logs")))
        for group in self.config.get("groups", []):
            folder = group.get("folder", "")
            base = self.abs_path("deployment/board_trials_4x4/%s" % folder)
            script.append(
                'printf "GROUP|%s|"; if [ -f %s/start.sh ]; then printf "start=1|"; else printf "start=0|"; fi; '
                'if [ -f %s/start_real.sh ]; then printf "real=1|"; else printf "real=0|"; fi; '
                'if [ -f %s/settings.yaml ]; then echo "settings=1"; else echo "settings=0"; fi'
                % (folder, quote(base), quote(base), quote(base)))
        env_script = self.abs_path(self.config["connection"].get("env_script", ""))
        site_config = self.abs_path("deployment/site_20260928/test_area.yaml")
        script.append('if [ -f %s ]; then echo "SITECFG=OK"; else echo "SITECFG=MISSING"; fi' % quote(site_config))
        code, output = self.run("\n".join(script), timeout=30.0)
        processes = {}
        groups = {}
        disk_free_kb = None
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
        leftovers = {name: int(count) for name, count in processes.items() if count.isdigit() and int(count) > 0}
        free_gb = (float(disk_free_kb) / (1024.0 * 1024.0)) if disk_free_kb and disk_free_kb.isdigit() else None
        min_free = float(self.config["checks"].get("min_free_gb", 2.0))
        results = []
        results.append({"name": "板端登录与工程根", "ok": code == 0, "detail":
                        ("命令退出码 %s" % code) if code else "命令执行完成"})
        results.append({"name": "现场范围配置", "ok": True, "detail":
                        "deployment/site_20260928/test_area.yaml %s" % (
                            "存在" if "SITECFG=OK" in output else "缺失（该组入口会用模块默认 4×4，请显式指定 --site-config）")})
        results.append({"name": "录像空间（≥%.1fGB）" % min_free, "ok":
                        (free_gb is None or free_gb >= min_free),
                        "detail": ("%.1fGB 可用" % free_gb) if free_gb is not None else "无法读取 df"})
        if leftovers:
            results.append({"name": "本机残留进程", "ok": False,
                            "detail": "；".join("%s×%d" % (k, v) for k, v in sorted(leftovers.items()))})
        else:
            results.append({"name": "本机残留进程", "ok": True, "detail": "无 roscore/roslaunch/gzserver/px4/mavros 残留"})
        missing = [folder for folder, info in groups.items() if not info.get("start")]
        if missing:
            results.append({"name": "任务组入口", "ok": False,
                            "detail": "缺少 start.sh：%s" % "、".join(sorted(missing))})
        else:
            results.append({"name": "任务组入口", "ok": True,
                            "detail": "%d 个模块入口与 settings.yaml 均存在" % len(groups)})
        return {"at": time.time(), "checks": results, "leftovers": leftovers,
                "disk_free_gb": free_gb, "groups": groups, "raw": output}

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
        return "cd %s && source %s >/dev/null 2>&1; exec %s %s --interval %s --topics %s --services %s" % (
            quote(self.root), quote(env_script),
            quote(connection.get("board_python", "/usr/bin/python3")), quote(remote),
            interval or probe.get("interval", 1.0), quote(topics), quote(services))

    # -- 日志与产物 --
    def list_logs(self, limit=40):
        script = "\n".join([
            "cd %s 2>/dev/null || exit 0" % quote(self.abs_path("logs")),
            "for d in $(ls -1dt board_* 2>/dev/null | head -%d); do" % int(limit),
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
        path = self.abs_path(os.path.join("logs", run, name))
        code, output = self.run("tail -n %d -- %s" % (int(lines), quote(path)), timeout=25.0)
        return {"ok": code == 0, "text": output, "path": path}

    def read_file(self, run, name, max_bytes=2000000):
        path = self.abs_path(os.path.join("logs", run, name))
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
    if profile.get("auto_password") is not None:
        config["connection"]["auto_password"] = bool(profile["auto_password"])
    return config
