#!/usr/bin/env python3
"""SSH / 本地会话层：常驻终端 + 一次性命令。

设计要点：
  * 常驻终端是真实 ssh -tt 交互会话（现场手册里的"每个终端一个 SSH 窗口"），
    因此 sudo 提示、Ctrl+C、交互命令都能用；
  * 口令只存在内存里，用于自动回应 ssh/sudo 提示；工作台不会把口令写进仓库；
  * transport=local 时在本机用 bash 执行同样的命令串，用于离线自检（不碰板端）。
"""
import os
import re
import shlex
import threading
import time

import pexpect

PROMPT_PATTERNS = [
    re.compile(r"(?i)are you sure you want to continue connecting[^\n]*\?"),
    # SSH's login prompt; never treat a remote application's password as SSH auth.
    re.compile(r"(?im)^\s*[^\r\n\s]+@[^\r\n\s]+['’]s password:\s*$"),
]
SUDO_PROMPT = re.compile(r"(?im)^\s*\[sudo\][^\r\n]*(?:password|密码|密碼)[^\r\n]*[:：]\s*$")

ANSI_PATTERN = re.compile(r"\x1b\[[0-9;?]*[a-zA-Z]|\x1b\][^\x07]*\x07|\x1b[=>]")


def strip_ansi(text):
    return ANSI_PATTERN.sub("", text or "")


class PromptResponder:
    """Bounded SSH auth and explicitly authorized sudo, using memory only."""

    def __init__(self, target, child, allow_sudo_password=False, note=None):
        self.target = target
        self.child = child
        self.allow_sudo_password = allow_sudo_password
        self.note = note or (lambda text: None)
        self.attempts = {"host": 0, "ssh": 0, "sudo": 0}
        self.notices = set()
        self.secrets = set()

    def _note_once(self, key, text):
        if key not in self.notices:
            self.notices.add(key)
            self.note(text)

    def answer(self, tail):
        clean = strip_ansi(tail)
        kind = "sudo" if SUDO_PROMPT.search(clean) else None
        if kind is None:
            for index, pattern in enumerate(PROMPT_PATTERNS):
                if pattern.search(clean):
                    kind = "host" if index == 0 else "ssh"
                    break
        if kind is None:
            return False
        if self.target.transport == "local":
            return True
        if kind == "sudo" and not self.allow_sudo_password:
            return True
        if not (self.target.password and self.target.auto_password):
            if kind == "sudo":
                self._note_once("manual", "sudo 需要独立认证；没有可复用的内存口令或自动应答已关闭，请手动输入。")
            return True
        limit = 3 if kind == "ssh" else 1
        if self.attempts[kind] >= limit:
            self._note_once(kind + "-limit", "自动认证次数已达上限；请检查口令并手动处理，不再自动重试。")
            return True
        self.attempts[kind] += 1
        answer = "yes" if kind == "host" else self.target.password
        if kind != "host":
            # Do not type secrets into an echoing PTY. The output filter also
            # handles a remote process echoing the secret back in split chunks.
            self.secrets.add(answer)
        try:
            if kind != "host" and not self.child.waitnoecho(timeout=1.0):
                self._note_once("echo", "PTY 回显未关闭，已跳过自动口令；请手动处理。")
                return True
            self.child.sendline(answer)
        except (OSError, ValueError, pexpect.ExceptionPexpect):
            self._note_once("send", "自动认证输入未发送，请手动处理。")
            return True
        self.note("已确认新的 SSH 主机指纹" if kind == "host" else
                  "已为本次确认的 sudo 初始化填写内存连接口令（不显示口令）" if kind == "sudo" else
                  "已填写 SSH 登录口令（不显示口令）")
        return True


class Target(object):
    """连接目标（工作台进程内共享一份口令）。"""

    def __init__(self, host="orangepi@192.168.3.15", port=22, ssh_options=None,
                 password=None, auto_password=True, connect_timeout=12, transport="ssh", identity_file=""):
        self.host = host
        self.port = int(port or 22)
        self.ssh_options = list(ssh_options or [])
        self.password = password
        self.auto_password = bool(auto_password)
        self.connect_timeout = float(connect_timeout or 12)
        self.transport = transport or "ssh"
        self.identity_file = identity_file or ""

    @property
    def user(self):
        return self.host.split("@")[0] if "@" in self.host else ""

    def base_argv(self):
        if self.transport == "local":
            return ["bash"]
        argv = ["ssh"]
        if self.port and self.port != 22:
            argv += ["-p", str(self.port)]
        argv += ["-o", "ConnectTimeout=%d" % int(self.connect_timeout)]
        if self.identity_file:
            argv += ["-i", os.path.expanduser(self.identity_file), "-o", "IdentitiesOnly=yes"]
        if not self.password:
            # 没有口令可用时不要挂在交互口令提示上：让 ssh 立刻失败，界面才能如实报
            # "认证失败/指纹不一致/网络不通"，而不是被误判成"板端文件缺失"。
            # BatchMode 不影响 agent/密钥认证，也不影响远端命令自己的提示（如 sudo）。
            argv += ["-o", "BatchMode=yes"]
        argv += self.ssh_options
        return argv

    def remote_argv(self, command, force_tty=True):
        argv = self.base_argv()
        if self.transport == "local":
            return argv + ["-lc", command]
        if force_tty:
            argv.append("-tt")
        argv.append(self.host)
        argv.append(command)
        return argv

    def describe(self):
        if self.transport == "local":
            return "local（本机自检，不连板端）"
        return "%s:%s" % (self.host, self.port)


class Session(object):
    """一个常驻终端会话。"""

    def __init__(self, sid, title, command, target, log_path=None,
                 on_output=None, on_state=None, on_note=None,
                 dimensions=(36, 140), env=None, graceful_only=False,
                 allow_sudo_password=False):
        self.id = sid
        self.title = title
        self.command = command
        self.target = target
        self.graceful_only = graceful_only
        self.allow_sudo_password = allow_sudo_password
        self.log_path = log_path
        self.on_output = on_output
        self.on_state = on_state
        self.on_note = on_note
        self.dimensions = dimensions
        self.env = env or dict(os.environ)
        self.env.setdefault("TERM", "xterm-256color")
        self.child = None
        self.thread = None
        self.state = "idle"
        self.exit_code = None
        self.started_at = None
        self.ended_at = None
        self.seq = 0
        self.chars = 0
        self.history = []
        self.history_chars = 0
        self.max_history_chars = 400000
        self.pending = ""
        self._tail = ""
        self._responder = None
        self._redact_pending = ""
        self._log_handle = None
        self._lock = threading.RLock()

    # ---- 生命周期 ----
    def start(self):
        if self.child is not None and self.child.isalive():
            raise RuntimeError("会话 %s 已在运行" % self.id)
        argv = self.target.remote_argv(self.command)
        self._open_log()
        self.state = "starting"
        self._emit_state()
        try:
            if os.name == "nt":
                from wb_native_ssh import ChannelChild
                self.child = ChannelChild(self.target, self.command, self.dimensions)
            else:
                self.child = pexpect.spawn(argv[0], argv[1:], encoding="utf-8",
                                           codec_errors="replace", timeout=None,
                                           env=self.env, dimensions=self.dimensions)
        except Exception:
            self.state = "failed"
            self.exit_code = 1
            self.ended_at = time.time()
            if self._log_handle:
                self._log_handle.close()
                self._log_handle = None
            self._emit_state()
            raise
        self._responder = PromptResponder(self.target, self.child,
                                          self.allow_sudo_password, self._note)
        if os.name == "nt" and self.target.password:
            self._responder.secrets.add(self.target.password)
        self.started_at = time.time()
        self.state = "running"
        self._emit_state()
        self.thread = threading.Thread(target=self._reader, name="wb-%s" % self.id)
        self.thread.daemon = True
        self.thread.start()
        return self

    def _open_log(self):
        if not self.log_path:
            return
        try:
            directory = os.path.dirname(self.log_path)
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            self._log_handle = open(self.log_path, "a", encoding="utf-8", errors="replace")
            self._log_handle.write("\n=== %s | %s | %s ===\n" % (
                time.strftime("%Y-%m-%d %H:%M:%S"), self.id, self.command))
            self._log_handle.flush()
        except OSError:
            self._log_handle = None

    def _reader(self):
        while True:
            try:
                data = self.child.read_nonblocking(size=4096, timeout=0.4)
            except pexpect.TIMEOUT:
                if not self.child.isalive():
                    break
                continue
            except pexpect.EOF:
                break
            except (OSError, ValueError):
                break
            if not data:
                if not self.child.isalive():
                    break
                continue
            self._consume(data)
        self._finish()

    def _consume(self, data):
        self._tail = (self._tail + data)[-600:]
        self._maybe_answer_prompt()
        self._publish(self._redact(data))

    def _redact(self, data, final=False):
        text = ("[口令已隐藏]" if final and self._redact_pending else self._redact_pending) + data
        self._redact_pending = ""
        secrets = self._responder.secrets if self._responder else ()
        for secret in sorted(secrets, key=len, reverse=True):
            text = text.replace(secret, "[口令已隐藏]")
        if not final:
            # Keep only an actual secret prefix, not an arbitrary output tail.
            hold = max((n for secret in secrets for n in range(1, len(secret))
                        if text.endswith(secret[:n])), default=0)
            if hold:
                self._redact_pending, text = text[-hold:], text[:-hold]
        return text

    def _publish(self, data):
        if not data:
            return
        with self._lock:
            self.seq += 1
            seq = self.seq
            self.chars += len(data)
            self.history.append((seq, data))
            self.history_chars += len(data)
            while self.history_chars > self.max_history_chars and len(self.history) > 1:
                _, dropped = self.history.pop(0)
                self.history_chars -= len(dropped)
        if self._log_handle:
            try:
                self._log_handle.write(data)
                self._log_handle.flush()
            except OSError:
                pass
        if self.on_output:
            self.on_output(self.id, seq, data)

    def _maybe_answer_prompt(self):
        if self._responder and self._responder.answer(self._tail):
            self._tail = ""

    def _note(self, text):
        if self.on_note:
            self.on_note(self.id, text)

    def _finish(self):
        self._publish(self._redact("", final=True))
        code = None
        try:
            if not self.child.isalive():
                self.child.close()
            code = self.child.exitstatus
            if code is None and self.child.signalstatus is not None:
                code = 128 + int(self.child.signalstatus)
        except Exception:
            code = None
        self.exit_code = code
        self.ended_at = time.time()
        self.state = "exited" if code in (0, None) else "failed"
        if self._log_handle:
            try:
                self._log_handle.write("\n=== exited code=%s at %s ===\n" % (
                    code, time.strftime("%Y-%m-%d %H:%M:%S")))
                self._log_handle.close()
            except OSError:
                pass
            self._log_handle = None
        self._emit_state()

    def _emit_state(self):
        if self.on_state:
            self.on_state(self.id, self.snapshot())

    # ---- 操作 ----
    def write(self, data):
        if self.child is None or not self.child.isalive():
            raise RuntimeError("会话 %s 未运行" % self.id)
        self.child.send(data)

    def send_key(self, key):
        if self.child is None or not self.child.isalive():
            raise RuntimeError("会话 %s 未运行" % self.id)
        mapping = {"C-c": "\x03", "C-d": "\x04", "C-z": "\x1a", "enter": "\r", "tab": "\t"}
        if key in mapping:
            self.child.send(mapping[key])
            return
        raise ValueError("不支持的按键 %s" % key)

    def resize(self, rows, cols):
        try:
            self.child.setwinsize(max(8, int(rows)), max(20, int(cols)))
        except Exception:
            pass

    def clear(self):
        with self._lock:
            self.history = []
            self.history_chars = 0

    def close(self, wait=6.0):
        if self.child is None:
            return
        if self.graceful_only and self.child.isalive():
            self.send_key("C-c")
            return  # Airborne observation keeps publishing until pilot takeover/disarm.
        if self.child.isalive():
            try:
                self.child.send("\x03")
            except (OSError, ValueError):
                pass
            deadline = time.time() + wait
            while time.time() < deadline and self.child.isalive():
                time.sleep(0.2)
        if self.child.isalive():
            try:
                self.child.terminate(force=False)
            except Exception:
                pass
            deadline = time.time() + 4.0
            while time.time() < deadline and self.child.isalive():
                time.sleep(0.2)
        if self.child.isalive():
            try:
                self.child.terminate(force=True)
            except Exception:
                pass
        if self.state in ("running", "starting"):
            self.state = "exited"
            self.ended_at = time.time()
            self._emit_state()

    def snapshot(self):
        return {
            "id": self.id,
            "title": self.title,
            "command": self.command,
            "state": self.state,
            "exit_code": self.exit_code,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "seq": self.seq,
            "chars": self.chars,
        }

    def history_since(self, seq):
        with self._lock:
            return [(n, d) for n, d in self.history if n > seq]


class SessionManager(object):
    def __init__(self, log_dir, on_output=None, on_state=None, on_note=None):
        self.log_dir = log_dir
        self.sessions = {}
        self.on_output = on_output
        self.on_state = on_state
        self.on_note = on_note
        self._lock = threading.RLock()

    def get(self, sid):
        return self.sessions.get(sid)

    def open(self, sid, title, command, target, dimensions=None, keep_existing=True,
             graceful_only=False, allow_sudo_password=False):
        with self._lock:
            existing = self.sessions.get(sid)
            if existing is not None and existing.state in ("running", "starting"):
                if keep_existing:
                    return existing
                existing.close()
                if existing.state in ("running", "starting"):
                    raise ValueError("旧观察入口仍在收尾，不能替换会话")
            log_path = os.path.join(self.log_dir, "%s-%s.log" % (
                sid, time.strftime("%Y%m%d_%H%M%S")))
            session = Session(sid, title, command, target, log_path=log_path,
                              on_output=self.on_output, on_state=self.on_state,
                              on_note=self.on_note,
                              dimensions=dimensions or (36, 140), graceful_only=graceful_only,
                              allow_sudo_password=allow_sudo_password)
            self.sessions[sid] = session
            return session.start()

    def close(self, sid, wait=6.0):
        trial = self.sessions.get("trial")
        if (sid in ("roscore", "mavros", "lidar", "observation_localization")
                and trial is not None and getattr(trial, "graceful_only", False)
                and trial.state in ("running", "starting")):
            raise ValueError("观察入口尚未退出，请先手动落地上锁并等待 OBSERVATION_CLOSED；保留定位与飞控链路")
        session = self.sessions.get(sid)
        if session is None:
            return None
        session.close(wait=wait)
        return session

    def close_all(self):
        # Keep ROS/MAVROS available until the supervisor closes children/bag.
        order = (["trial"] if "trial" in self.sessions else [])
        order += [sid for sid in reversed(list(self.sessions)) if sid != "trial"]
        for sid in order:
            if sid != "trial":
                trial = self.sessions.get("trial")
                if trial is not None and getattr(trial, "graceful_only", False) and trial.state in ("running", "starting"):
                    raise ValueError("观察入口仍在保持/收尾，设备会话已保留；落地上锁并等待退出后再停止全部")
            try:
                self.close(sid, wait=120.0 if sid == "trial" else 3.0)
            except Exception:
                pass

    def snapshots(self):
        return {sid: session.snapshot() for sid, session in self.sessions.items()}


def run_once(target, command, timeout=30.0, log_prefix=None):
    """执行一次性远程命令，返回 (exit_code, 输出文本)。"""
    if os.name == "nt":
        from wb_native_ssh import run_bytes as native_run_bytes
        code, output = native_run_bytes(target, command, timeout)
        text = output.decode("utf-8", "replace")
        if target.password:
            text = text.replace(target.password, "[口令已隐藏]")
        return code, text
    argv = target.remote_argv(command, force_tty=False)
    child = pexpect.spawn(argv[0], argv[1:], encoding="utf-8", codec_errors="replace",
                          timeout=timeout, env=dict(os.environ))
    chunks = []
    tail = ""
    responder = PromptResponder(target, child)  # One-off checks never authorize sudo.
    deadline = time.time() + float(timeout)
    try:
        while True:
            try:
                data = child.read_nonblocking(size=4096, timeout=1.0)
            except pexpect.TIMEOUT:
                if not child.isalive():
                    break
                if time.time() > deadline:
                    break
                continue
            except pexpect.EOF:
                break
            if not data:
                if not child.isalive():
                    break
                continue
            chunks.append(data)
            tail = (tail + data)[-600:]
            if time.time() < deadline and responder.answer(tail):
                tail = ""
    finally:
        code = None
        try:
            if child.isalive():
                child.close(force=True)
            code = child.exitstatus
            if code is None and child.signalstatus is not None:
                code = 128 + int(child.signalstatus)
        except Exception:
            code = None
    output = "".join(chunks)
    for secret in sorted(responder.secrets, key=len, reverse=True):
        output = output.replace(secret, "[口令已隐藏]")
    return (0 if code is None else code), output


def run_bytes(target, command, timeout=60.0):
    """Download through pipes, with SSH auth on a separate askpass channel.

    Never use a PTY for binary stdout (it merges prompts/stderr and maps LF to
    CRLF). The helper contains no credential: a bounded local Unix socket
    supplies only SSH login answers from Target's in-memory/profile password.
    Unknown exit status, failure and timeout never return partial file bytes.
    """
    if os.name == "nt":
        from wb_native_ssh import run_bytes as native_run_bytes
        return native_run_bytes(target, command, timeout)
    import socket
    import subprocess
    import sys
    import tempfile

    argv = target.remote_argv(command, force_tty=False)
    env = dict(os.environ)
    env.pop("SSH_ASKPASS", None)
    env["SSH_ASKPASS_REQUIRE"] = "never"
    with tempfile.TemporaryDirectory(prefix="wb-auth-") as directory:
        listener = None
        worker = None
        stop = threading.Event()
        if target.transport == "ssh" and target.password and target.auto_password:
            address = os.path.join(directory, "answer.sock")
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            listener.bind(address)
            os.chmod(address, 0o600)
            listener.listen(1)
            listener.settimeout(.2)

            def answer():
                attempts = 0
                while not stop.is_set():
                    try:
                        connection, _ = listener.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    with connection:
                        connection.settimeout(1)
                        try:
                            prompt = connection.recv(4096).decode("utf-8", "replace")
                            # No sudo, application passwords or private-key passphrases.
                            if PROMPT_PATTERNS[1].search(prompt) and attempts < 3:
                                attempts += 1
                                connection.sendall(target.password.encode("utf-8"))
                        except (OSError, UnicodeError):
                            pass

            helper = os.path.join(directory, "askpass")
            with open(helper, "w", encoding="utf-8") as handle:
                handle.write("#!%s\n" % sys.executable +
                             "import os, socket, sys\n"
                             "with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:\n"
                             "    s.settimeout(2)\n"
                             "    s.connect(os.environ['WB_ASKPASS_SOCKET'])\n"
                             "    s.sendall(sys.argv[1].encode('utf-8'))\n"
                             "    chunks = []\n"
                             "    while True:\n"
                             "        data = s.recv(4096)\n"
                             "        if not data: break\n"
                             "        chunks.append(data)\n"
                             "    if not chunks: sys.exit(1)\n"
                             "    sys.stdout.buffer.write(b''.join(chunks) + b'\\n')\n")
            os.chmod(helper, 0o700)
            env.update(SSH_ASKPASS=helper, SSH_ASKPASS_REQUIRE="force",
                       DISPLAY=env.get("DISPLAY") or ":wb", WB_ASKPASS_SOCKET=address)
            worker = threading.Thread(target=answer, daemon=True)
            worker.start()
        child = None
        try:
            child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, env=env, start_new_session=True)
            try:
                output, errors = child.communicate(timeout=float(timeout))
            except subprocess.TimeoutExpired:
                child.kill()  # Only this download's local SSH process, no ROS nodes.
                child.communicate()
                return 124, b"Download timed out; incomplete data discarded"
            code = child.returncode
            if code == 0:
                return 0, output
            error = errors or b"Download failed; incomplete data discarded"
            if target.password:
                error = error.replace(target.password.encode("utf-8"), b"[redacted]")
            return (code if code is not None else 1), error[-2000:]
        except OSError:
            return 1, b"Cannot start download transport"
        finally:
            stop.set()
            if worker:
                worker.join(timeout=2)
            if listener:
                listener.close()


def quote(text):
    return shlex.quote(text)


def now():
    return time.time()
