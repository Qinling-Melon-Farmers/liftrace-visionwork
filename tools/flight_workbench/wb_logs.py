"""Explicit log operations; reuse existing recorder/FC transfer, never flight APIs."""
import base64
import json
import re
import threading
import uuid
from pathlib import Path

from wb_ssh import quote, run_bytes

# This stdlib-only code runs through the existing SSH client, not an installed
# board daemon. Status/download are read-only; only clicked actions create files.
REMOTE = r'''
import base64
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}

def inside(root, path):
    path = path.resolve()
    if root != path and root not in path.parents:
        raise ValueError("Path outside configured logs directory")
    return path

def processes():
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = [v.decode("utf-8", "replace") for v in (entry / "cmdline").read_bytes().split(b"\0") if v]
            names = {Path(arg).name for arg in args}
            kind = None
            if names & {"wb_log_worker", "record_diagnostics.py", "trial_bag.py", "record_shadow.py", "record", "rosbag_record"}:
                kind = "bag"
            elif "rosbag" in names and "record" in args:
                kind = "bag"
            elif (any(name.startswith("fetch_px4_ulog") and name.endswith(".py") for name in names)
                  or "--workbench-index-marker" in args):
                kind = "ulog"
            if kind:
                found.append({"pid": int(entry.name), "kind": kind})
        except OSError:
            continue
    return found

def identity(pid):
    try:
        base = Path("/proc") / str(pid)
        # Field 22, after comm (which may contain spaces or parentheses).
        fields = (base / "stat").read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return {"start": fields[19], "argv": (base / "cmdline").read_bytes().decode("utf-8").split("\0")[:-1]}
    except (OSError, ValueError, IndexError):
        return None

def owned(meta):
    if not isinstance(meta.get("pid"), int) or meta["pid"] <= 1:
        return False
    current = identity(meta["pid"])
    return bool(current and current == meta.get("identity"))

def jobs(directory):
    result = []
    if directory.is_dir():
        for run in sorted(directory.iterdir(), reverse=True)[:100]:
            if run.is_symlink() or not run.is_dir():
                continue
            meta = read_json(run / "job.json")
            if meta.get("kind") not in ("bag", "ulog"):
                continue
            live = owned(meta)
            summary = read_json(run / "recording" / "summary.json")
            ready = read_json(run / "recording" / "recording_ready.json")
            complete = (summary.get("bag_closed") is True and not summary.get("error")) if meta["kind"] == "bag" else (run / "flight.ulg.download.json").is_file()
            state = ("recording" if ready.get("ready") else "starting") if live and meta["kind"] == "bag" else "transferring" if live else "complete" if complete else "failed"
            try:
                with (run / "operation.log").open("rb") as handle:
                    handle.seek(max(0, (run / "operation.log").stat().st_size - 3000))
                    tail = handle.read().decode("utf-8", "replace")
            except OSError:
                tail = ""
            result.append({"id": run.name, "kind": meta["kind"], "pid": meta.get("pid"),
                           "active": live, "state": state, "started_at": meta.get("started_at"),
                           "duration": meta.get("duration"), "log_id": meta.get("log_id"), "profile": meta.get("profile"),
                           "summary": summary, "tail": tail})
    return result

def file_closed(root, path, current_jobs):
    relative = path.relative_to(root)
    if relative.parts[0] == "flight_workbench_recordings":
        run = root / relative.parts[0] / relative.parts[1]
        if any(j["id"] == run.name and j["active"] for j in current_jobs):
            return False
        if path.suffix == ".bag":
            summary = read_json(run / "recording" / "summary.json")
            return summary.get("bag_closed") is True and not summary.get("error")
        if path.suffix == ".ulg":
            return path.with_name(path.name + ".download.json").is_file()
    return not path.name.endswith((".active", ".part"))

def catalog(root, current_jobs):
    files = []
    visited = 0
    for directory, dirs, names in os.walk(str(root), followlinks=False):
        visited += 1
        base = Path(directory)
        dirs[:] = sorted([d for d in dirs if not d.startswith(".") and not (base / d).is_symlink()], reverse=True)
        if len(base.relative_to(root).parts) >= 5 or visited >= 600:
            dirs[:] = []
        for name in names:
            path = base / name
            if path.is_symlink() or not path.is_file() or path.suffix not in {".bag", ".ulg", ".log", ".txt", ".json", ".yaml", ".csv", ".active", ".part"}:
                continue
            stat = path.stat()
            files.append({"path": path.relative_to(root).as_posix(), "size": stat.st_size,
                          "mtime": stat.st_mtime, "downloadable": file_closed(root, path, current_jobs)})
        if visited >= 600:
            break
    return sorted(files, key=lambda f: f["mtime"], reverse=True)[:150]

def action(request):
    board = Path(request["root"]).resolve()
    root = inside(board, board / "logs")
    directory = inside(root, root / "flight_workbench_recordings")
    operation = request["operation"]
    if operation == "download":
        path = inside(root, root / request["path"])
        if not path.is_file() or not file_closed(root, path, jobs(directory)):
            raise ValueError("File missing, recording still active, or file not finalized")
        before = path.stat()
        with path.open("rb") as stream:
            # Reject bags still opened for writing, including legacy internal bag.
            for proc in Path("/proc").iterdir():
                if not proc.name.isdigit():
                    continue
                try:
                    descriptors = list((proc / "fd").iterdir())
                except OSError:
                    continue
                for fd in descriptors:
                    try:
                        if fd.resolve() == path:
                            info = (proc / "fdinfo" / fd.name).read_text()
                            flags = next(int(line.split()[1], 8) for line in info.splitlines() if line.startswith("flags:"))
                            if flags & os.O_ACCMODE:
                                raise ValueError("File is still open for writing")
                    except (OSError, StopIteration):
                        continue
            import shutil
            sys.stdout.buffer.write(("WB_BINARY_" + request["nonce"] + " " + str(before.st_size) + "\n").encode("ascii"))
            shutil.copyfileobj(stream, sys.stdout.buffer, length=1024*1024)
            sys.stdout.buffer.flush()
        after = path.stat()
        if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError("File changed during download; discard incomplete data")
        sys.stdout.buffer.write(("\nWB_END_" + request["nonce"] + "\n").encode("ascii"))
        return None
    if operation == "status":
        current = jobs(directory)
        return {"ok": True, "jobs": current, "recorders": processes(), "files": catalog(root, current),
                "available": {"bag": (board / request["bag_script"]).is_file() and (board / request["bag_config"]).is_file(),
                              "routine": (board / request["routine_script"]).is_file(),
                              "ulog": (board / request["ulog_script"]).is_file()}}
    if operation == "index":
        if any(p["kind"] == "ulog" for p in processes()):
            raise ValueError("ULog transfer in progress; index request refused")
        # Run the bundled index-only helper in memory. Older board fetch scripts
        # keep downloading unchanged; no deployment or board-file replacement.
        source = request.get("index_source")
        if not source:
            raise ValueError("Missing bundled FC index helper; no IDs inferred")
        child = subprocess.Popen([request["python"], "-c", source, "--namespace", request["namespace"],
                                  "--list", "--format", "json", "--list-timeout", "15", "--timeout", "20",
                                  "--workbench-index-marker"],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        try:
            output, _ = child.communicate(timeout=40)
        except subprocess.TimeoutExpired:
            child.send_signal(signal.SIGINT)  # Allow existing finally END, no forced kill.
            try:
                child.communicate(timeout=8)
            except subprocess.TimeoutExpired:
                pass  # Do not SIGKILL a transfer before END; process detection blocks retry.
            raise ValueError("Index timed out; check transfer state before retry")
        if child.returncode != 0:
            raise ValueError("FC index query failed")
        # finally END warnings are separate trailing lines in the supplied CLI.
        entries, _ = json.JSONDecoder().raw_decode(output.decode("utf-8").lstrip())
        if not isinstance(entries, list):
            raise ValueError("Invalid FC log index")
        return {"ok": True, "entries": entries}
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / ".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if operation == "stop":
            run = inside(directory, directory / request["id"])
            meta = read_json(run / "job.json")
            if not owned(meta):
                raise ValueError("No matching owned process; no signal sent")
            os.kill(meta["pid"], signal.SIGINT)  # Individual owned PID only; never killall/rosnode kill.
            deadline = time.monotonic() + 10
            while owned(meta) and time.monotonic() < deadline:
                time.sleep(.1)
            return {"ok": True, "stopping": owned(meta), "id": run.name}
        kind = request["kind"]
        if any(j["active"] for j in jobs(directory)) or any(p["kind"] == kind for p in processes()):
            raise ValueError("Recording/transfer already running; reuse existing log instead of duplicating")
        script_key = "routine_script" if kind == "bag" and request["profile"] == "routine" else "bag_script" if kind == "bag" else "ulog_script"
        script = inside(board, board / request[script_key])
        if not script.is_file():
            raise ValueError("Existing project script missing on board; not deployed by workbench")
        run = directory / (kind + "_" + time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8])
        if kind == "bag" and request["profile"] == "routine":
            worker = ROUTINE_WORKER + "\nmain(" + repr({"script": str(script), "output": str(run / "recording"),
                                                        "duration": request["duration"], "settings": request["routine_settings"]}) + ")\n"
            encoded = base64.b64encode(worker.encode("utf-8")).decode("ascii")
            args = [request["python"], "-c", "import base64; exec(base64.b64decode(%r))" % encoded, "wb_log_worker"]
        elif kind == "bag":
            config = inside(board, board / request["bag_config"])
            if not config.is_file():
                raise ValueError("Existing recording config missing on board")
            args = [request["python"], str(script), "--output", str(run / "recording"),
                    "--config", str(config), "--duration", str(request["duration"])]
        else:
            args = [request["python"], str(script), "--namespace", request["namespace"],
                    "--log-id", str(request["log_id"]), "--output", str(run / "flight.ulg"), "--timeout", "600"]
        run.mkdir()
        with (run / "operation.log").open("ab") as output:
            child = subprocess.Popen(args, cwd=str(board), stdin=subprocess.DEVNULL,
                                     stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        meta = {"kind": kind, "pid": child.pid, "identity": identity(child.pid),
                "started_at": time.time(), "duration": request.get("duration"), "log_id": request.get("log_id"),
                "profile": request.get("profile")}
        (run / "job.json").write_text(json.dumps(meta), encoding="utf-8")
        return {"ok": True, "id": run.name, "state": "starting"}
'''

ROUTINE_WORKER = r'''
import importlib.util
import json
import os
from pathlib import Path
import signal
import threading
import time

def main(request):
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    spec = importlib.util.spec_from_file_location("project_trial_bag", request["script"])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = Path(request["output"])
    output.mkdir()
    recorder = module.TrialBag(str(output), request["settings"], dict(os.environ))
    ready = output / "recording_ready.json"
    started = time.monotonic()
    try:
        recorder.start()
        ready.write_text(json.dumps({"ready": True, "pid": os.getpid()}))
        print("READY: routine lightweight bag only", flush=True)
        while not stop.wait(.2) and time.monotonic() - started < request["duration"]:
            recorder.check()
            if recorder.closed or recorder.process.poll() is not None:
                break
    finally:
        recorder.close()  # Existing module closes only its own rosbag/throttle.
        report = json.loads((output / "bag_recording.json").read_text())
        (output / "summary.json").write_text(json.dumps({
            "bag_closed": bool(report.get("bags")) and not report.get("active") and not report.get("errors"),
            "error": "; ".join(report.get("errors", [])) or None,
            "recording_status": report.get("status"), "bag_recording": report,
            "elapsed_s": time.monotonic() - started}))
        ready.write_text(json.dumps({"ready": False, "pid": os.getpid()}))
'''


def relative_path(value):
    if (not isinstance(value, str) or len(value) > 512 or not value
            or any(not re.fullmatch(r"[A-Za-z0-9_.-]+", part) or part.startswith(".")
                   for part in value.split("/"))):
        raise ValueError("非法日志相对路径")
    return value


class LogsClient:
    def __init__(self, board):
        self.board = board
        self.lock = threading.RLock()
        self.cached = None

    def payload(self, operation, **values):
        settings = self.board.config.get("logging", {})
        result = {"operation": operation, "root": self.board.root,
                  "python": self.board.config["connection"].get("board_python", "/usr/bin/python3"),
                  "bag_script": relative_path(settings.get("bag_script", "deployment/low_hover_observation/record_diagnostics.py")),
                  "bag_config": relative_path(settings.get("bag_config", "deployment/low_hover_observation/recording.yaml")),
                  "routine_script": relative_path(settings.get("routine_script", "deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_bag.py")),
                  "routine_settings": settings.get("routine_settings", {"bag_image_hz": 5, "record_map_clouds": False, "record_inflated_cloud": True}),
                  "ulog_script": relative_path(settings.get("ulog_script", "tools/flight_logs/fetch_px4_ulog.py")),
                  "namespace": settings.get("mavros_namespace", "/mavros")}
        if not re.fullmatch(r"/[A-Za-z][A-Za-z0-9_/]*", result["namespace"]):
            raise ValueError("非法 MAVROS namespace")
        result.update(values)
        if operation == "index":
            tool = Path(__file__).resolve().parent
            helper = tool / "ulog_viewer" / "px4_log_index.py"
            if not helper.is_file():
                helper = tool.parent / "flight_logs" / "px4_log_index.py"
            result["index_source"] = helper.read_text(encoding="utf-8")
        return result

    def command(self, payload):
        source = REMOTE + "\nROUTINE_WORKER = " + repr(ROUTINE_WORKER) + "\nrequest = " + repr(payload) + "\n"
        if payload["operation"] == "download":
            source += "action(request)\n"
        else:
            source += ("try:\n    print('WB_LOGS_JSON=' + json.dumps(action(request)))\n"
                       "except Exception as error:\n"
                       "    print('WB_LOGS_JSON=' + json.dumps({'ok': False, 'error': str(error)}))\n"
                       "    sys.exit(1)\n")
        encoded = base64.b64encode(source.encode("utf-8")).decode("ascii")
        code = "import base64; exec(base64.b64decode(%r))" % encoded
        # Suppress environment-script chatter, including for binary download.
        env = self.board.abs_path(self.board.config["connection"].get("env_script", ""))
        return "cd %s && source %s >/dev/null 2>&1 && %s -c %s" % (
            quote(self.board.root), quote(env), quote(payload["python"]), quote(code))

    def execute(self, operation, **values):
        code, output = self.board.run(self.command(self.payload(operation, **values)), timeout=58 if operation == "index" else 25)
        for line in output.splitlines():
            if line.startswith("WB_LOGS_JSON="):
                result = json.loads(line.split("=", 1)[1])
                if code == 0 and result.get("ok") is True:
                    return result
                if not result.get("ok"):
                    raise ValueError("日志操作失败：" + str(result.get("error", "unknown")))
        # Do not return arbitrary SSH output or credentials in an HTTP response.
        raise ValueError("日志操作失败或超时；检查连接认证、板端脚本与日志状态（exit=%s）" % code)

    def status(self):
        with self.lock:
            self.cached = self.execute("status")
            return self.cached

    def index(self, body):
        if body.get("confirm") != "查询飞控日志索引":
            raise ValueError("需要明确点击查询飞控日志索引")
        with self.lock:
            return self.execute("index")

    def start(self, body):
        kind = body.get("kind")
        if kind not in ("bag", "ulog"):
            raise ValueError("只支持轻量 bag 或已封闭 ULog 下载；不支持 streaming 录制")
        expected = "开始轻量录制" if kind == "bag" else "取回飞控日志"
        if body.get("confirm") != expected:
            raise ValueError("需要明确点击确认：" + expected)
        values = {"kind": kind}
        if kind == "bag":
            profile = body.get("profile", "routine")
            if profile not in ("routine", "diagnostic"):
                raise ValueError("非法 bag 档位")
            values["profile"] = profile
            duration = body.get("duration", 300)
            if type(duration) is not int or not 1 <= duration <= 900:
                raise ValueError("录制时长必须为 1–900 秒的整数")
            values["duration"] = duration
        else:
            log_id = body.get("log_id")
            if type(log_id) is not int or not 0 <= log_id <= 65534:
                raise ValueError("必须填写实际日志 ID（0–65534），不能假定最新编号")
            values["log_id"] = log_id
        with self.lock:
            result = self.execute("start", **values)
            self.cached = {"jobs": [{"active": True, "id": result["id"]}]}
            return result

    def stop(self, body):
        job = relative_path(body.get("id"))
        if "/" in job or not re.fullmatch(r"(?:bag|ulog)_[0-9]{8}_[0-9]{6}_[0-9a-f]{8}", job):
            raise ValueError("非法工作台录制 ID")
        if body.get("confirm") != "停止日志操作":
            raise ValueError("需要明确点击停止日志操作")
        with self.lock:
            return self.execute("stop", id=job)

    def download(self, path):
        path = relative_path(path)
        with self.lock:
            nonce = uuid.uuid4().hex
            code, output = run_bytes(self.board.target, self.command(self.payload("download", path=path, nonce=nonce)), timeout=120)
            if code:
                return code, output
            # Frame the file separately from any shell startup stdout. A length
            # and end marker prove we received the whole unchanged file.
            marker = ("WB_BINARY_" + nonce + " ").encode("ascii")
            start = output.find(marker)
            try:
                if start < 0:
                    raise ValueError("No binary header")
                line = output.index(b"\n", start)
                size = int(output[start + len(marker):line])
                end = line + 1 + size
                if size < 0 or output[end:] != ("\nWB_END_" + nonce + "\n").encode("ascii"):
                    raise ValueError("Incomplete binary frame")
                return 0, output[line + 1:end]
            except (ValueError, IndexError):
                return 1, b"Invalid or incomplete binary transfer; data discarded"

    def active(self):
        return any(job.get("active") for job in (self.cached or {}).get("jobs", []))
