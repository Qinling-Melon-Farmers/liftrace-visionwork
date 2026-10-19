#!/usr/bin/env python3
"""Read-only, bounded ROS1 recorder; imports/connects to ROS only without --preview.

Parent contract: --output RUN/recording --config recording.yaml [--duration SEC].
Poll recording_ready.json and process returncode; SIGINT/SIGTERM stops only us.
READY means writer alive, NOT ground/flight/localization readiness. No publishers,
flight services, parameter changes, roslaunch, or child rosbag/flight processes.
"""
import argparse
import io
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import shutil
import sys
import threading
import time
import xmlrpc.client

import yaml


SAFE_TYPES = frozenset({
    "nav_msgs/Odometry", "geometry_msgs/PoseStamped", "geometry_msgs/TwistStamped",
    "geometry_msgs/PoseWithCovarianceStamped", "geometry_msgs/Vector3Stamped",
    "sensor_msgs/Imu", "sensor_msgs/BatteryState", "std_msgs/String",
    "std_msgs/Bool", "std_msgs/Float32", "std_msgs/Float64",
    "diagnostic_msgs/DiagnosticArray", "rosgraph_msgs/Log", "mavros_msgs/State", "mavros_msgs/ExtendedState",
    "mavros_msgs/RCIn", "mavros_msgs/RCOut", "mavros_msgs/ESCTelemetry",
    "mavros_msgs/ESCInfo", "mavros_msgs/ESCStatus", "mavros_msgs/TimesyncStatus",
    "mavros_msgs/StatusText", "mavros_msgs/AttitudeTarget", "mavros_msgs/PositionTarget",
    "mavros_msgs/Thrust", "mavros_msgs/VFR_HUD",
})
FORBIDDEN_NAME = re.compile(
    r"image|compressed|jpe?g|png|point.?cloud|cloud|lidar|livox|camera|depth|scan|video", re.I)
DEFAULTS = dict(max_duration_s=900, diagnostic_hz=1, discovery_interval_s=2,
                queue_messages=1024, queue_bytes=16777216, max_message_bytes=65536,
                subscriber_queue_size=10, snapshot_timeout_s=3,
                min_free_space_bytes=1073741824)


def validate_topic(name, types):
    if (not isinstance(name, str) or not re.fullmatch(r"/[A-Za-z][A-Za-z0-9_/]*", name)
            or "//" in name or name.endswith("/") or FORBIDDEN_NAME.search(name)):
        raise ValueError("unsafe/non-exact topic name: %r" % name)
    if not isinstance(types, list) or not types or any(t not in SAFE_TYPES for t in types):
        raise ValueError("topic %s has non-allowlisted wire type(s): %r" % (name, types))


def load_config(path):
    cfg = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(cfg, dict) or cfg.get("version") != 1:
        raise ValueError("recording config requires version: 1")
    unknown = set(cfg) - set(DEFAULTS) - {"version", "topics", "status_topic"}
    if unknown:
        raise ValueError("unknown recording options: %s" % sorted(unknown))
    cfg = {**DEFAULTS, **cfg}
    limits = dict(max_duration_s=(0, 900), diagnostic_hz=(0.1, 1),
                  discovery_interval_s=(1, 30), queue_messages=(1, 4096),
                  queue_bytes=(1024, 67108864), max_message_bytes=(256, 262144),
                  subscriber_queue_size=(1, 100), snapshot_timeout_s=(0.1, 10),
                  min_free_space_bytes=(0, 1099511627776))
    for key, (low, high) in limits.items():
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("%s must be finite numeric" % key)
        if not low <= value <= high:
            raise ValueError("%s outside [%s, %s]" % (key, low, high))
    for key in ("queue_messages", "queue_bytes", "max_message_bytes", "subscriber_queue_size", "min_free_space_bytes"):
        if int(cfg[key]) != cfg[key]:
            raise ValueError("%s must be integer" % key)
        cfg[key] = int(cfg[key])
    if not isinstance(cfg.get("topics"), list) or not cfg["topics"]:
        raise ValueError("topics must be a nonempty list")
    seen = set()
    for spec in cfg["topics"]:
        if not isinstance(spec, dict) or set(spec) != {"name", "types"}:
            raise ValueError("each topic requires only name and types")
        validate_topic(spec["name"], spec["types"])
        if spec["name"] in seen:
            raise ValueError("duplicate topic: %s" % spec["name"])
        seen.add(spec["name"])
    cfg["status_topic"] = cfg.get("status_topic", "/low_hover_observation/status")
    validate_topic(cfg["status_topic"], ["std_msgs/String"])
    if cfg["status_topic"] not in seen:
        cfg["topics"].append(dict(name=cfg["status_topic"], types=["std_msgs/String"]))
    else:
        status = next(s for s in cfg["topics"] if s["name"] == cfg["status_topic"])
        if "std_msgs/String" not in status["types"]:
            raise ValueError("status_topic must permit std_msgs/String")
    return cfg


def effective_duration(duration, ceiling):
    if not math.isfinite(duration) or duration < 0 or duration > 900:
        raise ValueError("duration must be finite and in [0, 900]")
    return min(duration, ceiling) if duration and ceiling else duration or ceiling


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


class TopicStats:
    def __init__(self):
        self.received = self.written = self.dropped = self.invalid_stamp = 0
        self.source_stamp = self.receive_ros = self.receive_wall = self.receive_mono = None
        self.source_age = self.received_gap = self.max_received_gap = None
        self.age_min = self.age_max = None
        self.source_regressions = 0

    def observe(self, message, ros, wall, mono):
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        source = None
        if stamp is not None:
            try:
                source = float(stamp.to_sec())
                if not math.isfinite(source) or source <= 0:
                    source = None
            except (ValueError, TypeError, AttributeError):
                pass
            if source is None:
                self.invalid_stamp += 1
        if source is not None and self.source_stamp is not None and source < self.source_stamp:
            self.source_regressions += 1
        self.received_gap = None if self.receive_mono is None else mono - self.receive_mono
        if self.received_gap is not None:
            self.max_received_gap = max(self.max_received_gap or 0, self.received_gap)
        self.source_stamp, self.receive_ros = source, ros
        self.receive_wall, self.receive_mono = wall, mono
        self.source_age = ros - source if source is not None and ros > 0 else None
        if self.source_age is not None:
            self.age_min = min(self.age_min, self.source_age) if self.age_min is not None else self.source_age
            self.age_max = max(self.age_max, self.source_age) if self.age_max is not None else self.source_age
        self.received += 1

    def snapshot(self, mono):
        return {**vars(self), "silent_s": None if self.receive_mono is None else mono - self.receive_mono}


class BoundedBuffer:
    """Nonblocking enqueue; cap both serialized bytes and message count."""
    def __init__(self, count, byte_limit):
        self.items = queue.Queue(count)
        self.byte_limit = byte_limit
        self.bytes = 0
        self.lock = threading.Lock()

    def put(self, item, size):
        with self.lock:
            if self.bytes + size > self.byte_limit:
                return False
            try:
                self.items.put_nowait((item, size))
            except queue.Full:
                return False
            self.bytes += size
            return True

    def get(self, timeout=0.1):
        item, size = self.items.get(timeout=timeout)
        with self.lock:
            self.bytes -= size
        return item


class BagWriter:
    """Owns only its bag; failures request recorder shutdown, never kill another PID."""
    def __init__(self, bag, buffer, on_written):
        self.bag, self.buffer, self.on_written = bag, buffer, on_written
        self.stop = threading.Event()
        self.started = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, name="diagnostic-bag-writer", daemon=True)

    def run(self):
        self.started.set()
        try:
            while not self.stop.is_set() or not self.buffer.items.empty():
                try:
                    name, message, receive_stamp = self.buffer.get()
                except queue.Empty:
                    continue
                self.bag.write(name, message, t=receive_stamp)
                self.on_written(name)
        except Exception as exc:
            self.error = str(exc)
            self.stop.set()
        finally:
            try:
                self.bag.close()
            except Exception as exc:
                self.error = self.error or str(exc)

    def finish(self):
        self.stop.set()
        self.thread.join()  # never terminate another node or leave an active writer


class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


class MasterReader:
    def __init__(self, timeout):
        self.server = xmlrpc.client.ServerProxy(
            os.environ.get("ROS_MASTER_URI", "http://localhost:11311"),
            transport=TimeoutTransport(timeout), allow_none=True)

    def call(self, method, *args):
        code, message, value = getattr(self.server, method)("/low_hover_diagnostics", *args)
        if code != 1:
            raise RuntimeError(message)
        return value


class ResourceSampler:
    def __init__(self):
        self.cpu_wall, self.cpu_time = time.monotonic(), time.process_time()
        self.previous_system = None
        self.maxima = {}

    def sample(self):
        wall, cpu = time.monotonic(), time.process_time()
        values = {"recorder_cpu_percent": 100 * (cpu - self.cpu_time) / max(wall - self.cpu_wall, 1e-9)}
        self.cpu_wall, self.cpu_time = wall, cpu
        try:
            fields = [int(x) for x in Path("/proc/stat").read_text().splitlines()[0].split()[1:9]]
            total, idle = sum(fields), fields[3] + fields[4]
            values["system_cpu_percent"] = None
            if self.previous_system:
                dt, di = total - self.previous_system[0], idle - self.previous_system[1]
                if dt > 0:
                    values["system_cpu_percent"] = 100 * (dt - di) / dt
            self.previous_system = (total, idle)
        except (OSError, ValueError, IndexError):
            values["system_cpu_percent"] = None
        temperatures = {}
        for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
            try:
                temperatures[path.parent.name] = float(path.read_text()) / 1000
            except (OSError, ValueError):
                pass
        values["temperature_c"] = temperatures
        for key, value in values.items():
            if isinstance(value, (int, float)):
                self.maxima[key] = max(self.maxima.get(key, value), value)
        for name, value in temperatures.items():
            key = "temperature_c/" + name
            self.maxima[key] = max(self.maxima.get(key, value), value)
        return values


class Recorder:
    def __init__(self, cfg, output, duration, rospy, rosbag, message_loader):
        self.cfg, self.output, self.duration = cfg, Path(output), duration
        self.rospy, self.rosbag, self.message_loader = rospy, rosbag, message_loader
        self.stats = {s["name"]: TopicStats() for s in cfg["topics"]}
        self.subscribers, self.types, self.rejected = {}, {}, {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.reason, self.error = "external_stop", None
        self.accepting = True
        self.status = None
        self.writer = None
        self.resources = ResourceSampler()
        self.master = MasterReader(cfg["snapshot_timeout_s"])
        self.metadata_errors = []
        self.ros_initialized = False
        self.disk_free_bytes = None

    def disk_space_ok(self):
        self.disk_free_bytes = shutil.disk_usage(self.output).free
        return self.disk_free_bytes >= self.cfg["min_free_space_bytes"]

    def stop_signal(self, signum, _frame):
        self.reason = "signal_%s" % signal.Signals(signum).name
        self.stop.set()

    def receive(self, name, message):
        stamp = self.rospy.Time.now()
        wall, mono = time.time(), time.monotonic()
        with self.lock:
            if not self.accepting:
                return
            stats = self.stats[name]
            stats.observe(message, stamp.to_sec(), wall, mono)
            try:
                data = io.BytesIO()
                message.serialize(data)
                size = data.tell()
                if size > self.cfg["max_message_bytes"]:
                    stats.dropped += 1
                    return
                if name == self.cfg["status_topic"] and getattr(message, "_type", None) == "std_msgs/String":
                    try:
                        self.status = json.loads(message.data)
                    except (ValueError, TypeError):
                        self.status = {"invalid_json": True}
                if not self.buffer.put((name, message, stamp), size):
                    stats.dropped += 1
            except Exception:
                stats.dropped += 1

    def written(self, name):
        with self.lock:
            self.stats[name].written += 1

    def discover(self):
        advertised = dict(self.master.call("getPublishedTopics", ""))
        for spec in self.cfg["topics"]:
            name, permitted = spec["name"], spec["types"]
            wire = advertised.get(name)
            if not wire:
                continue
            if wire not in permitted or wire not in SAFE_TYPES:
                self.rejected[name] = "advertised forbidden/unexpected type: " + wire
                if name in self.subscribers:
                    self.subscribers.pop(name).unregister()
                continue
            if name in self.subscribers and self.types[name] == wire:
                continue
            if name in self.subscribers:
                self.subscribers.pop(name).unregister()
            cls = self.message_loader(wire)
            if cls is None or getattr(cls, "_type", None) != wire:
                self.rejected[name] = "message class unavailable: " + wire
                continue
            self.types[name] = wire
            self.subscribers[name] = self.rospy.Subscriber(
                name, cls, lambda msg, topic=name: self.receive(topic, msg),
                queue_size=self.cfg["subscriber_queue_size"], buff_size=262144,
                tcp_nodelay=True)
            self.rejected.pop(name, None)
        self.topic_manifest()

    def topic_manifest(self):
        atomic_json(self.output / "bag_topics.json", {
            "requested": self.cfg["topics"], "subscribed": self.types,
            "active_subscriptions": sorted(self.subscribers), "rejected": self.rejected,
            "missing": [name for name in self.stats if name not in self.subscribers],
            "bag_time": "ROS receive time; header source stamp preserved in message",
            "diagnostics": "1Hz last source/receive plus full-session age/gap/count aggregates"})

    def metadata(self, suffix):
        for kind, method, args in (("rosparams", "getParam", ("/",)),
                                   ("rosgraph", "getSystemState", ())):
            try:
                value = self.master.call(method, *args)
                if kind == "rosparams":
                    (self.output / ("rosparam_%s.yaml" % suffix)).write_text(
                        yaml.safe_dump(value), encoding="utf-8")
                else:
                    atomic_json(self.output / ("rosgraph_%s.json" % suffix), value)
                    nodes = sorted({node for section in value for _, names in section for node in names})
                    (self.output / ("rosnode_%s.txt" % suffix)).write_text(
                        "\n".join(nodes) + "\n", encoding="utf-8")
            except Exception as exc:
                self.metadata_errors.append("%s/%s: %s" % (suffix, kind, exc))

    def snapshot(self):
        with self.lock:
            return {name: stats.snapshot(time.monotonic()) for name, stats in self.stats.items()}

    def run(self):
        # Fresh subdirectory only: stale readiness/bags must never pass parent's polling.
        self.output.mkdir(parents=True, exist_ok=True)
        if any(self.output.iterdir()):
            raise ValueError("recording output must be empty (use a fresh run directory)")
        started_wall, started_mono = time.time(), time.monotonic()
        self.buffer = BoundedBuffer(self.cfg["queue_messages"], self.cfg["queue_bytes"])
        diagnostics = None
        try:
            if not self.disk_space_ok():
                self.reason = "startup_low_disk_space"
                raise RuntimeError("free disk %s bytes below configured floor %s" %
                                   (self.disk_free_bytes, self.cfg["min_free_space_bytes"]))
            (self.output / "recording_config.yaml").write_text(yaml.safe_dump(self.cfg), encoding="utf-8")
            self.rospy.init_node("low_hover_diagnostics", anonymous=True, disable_signals=True,
                                 disable_rosout=True, argv=["record_diagnostics.py"])
            self.ros_initialized = True
            diagnostics = (self.output / "diagnostics.jsonl").open("w", encoding="utf-8")
            bag = self.rosbag.Bag(str(self.output / "diagnostics.bag"), "w",
                                  compression="none", chunk_threshold=262144)
            self.writer = BagWriter(bag, self.buffer, self.written)
            self.writer.thread.start()
            if not self.writer.started.wait(3) or not self.writer.thread.is_alive():
                raise RuntimeError("bag writer failed to start")
            self.discover()  # metadata/master reachability; no requirement for missing optional topics
            if self.writer.error or not self.writer.thread.is_alive():
                raise RuntimeError(self.writer.error or "bag writer died")
            atomic_json(self.output / "recording_ready.json", {
                "ready": True, "pid": os.getpid(), "bag": "diagnostics.bag",
                "started_wall_s": started_wall, "effective_duration_s": self.duration,
                "meaning": "recording service only; no flight/ground gate"})
            print("READY " + str(self.output / "recording_ready.json"), flush=True)
            self.metadata("start")
            next_diagnostic, next_discover = 0, time.monotonic() + self.cfg["discovery_interval_s"]
            while not self.stop.is_set() and not self.rospy.is_shutdown():
                now = time.monotonic()
                if self.writer.error or not self.writer.thread.is_alive():
                    raise RuntimeError(self.writer.error or "bag writer stopped unexpectedly")
                if self.duration and now - started_mono >= self.duration:
                    self.reason = "duration_limit"
                    break
                if now >= next_diagnostic:
                    if not self.disk_space_ok():
                        self.reason = "low_disk_space"
                        break
                    diagnostics.write(json.dumps({"wall_s": time.time(), "elapsed_s": now - started_mono,
                        "ros_receive_now_s": self.rospy.Time.now().to_sec(), "topics": self.snapshot(),
                        "resources": self.resources.sample(), "disk_free_bytes": self.disk_free_bytes,
                        "status": self.status}, allow_nan=False) + "\n")
                    diagnostics.flush()
                    next_diagnostic = now + 1 / self.cfg["diagnostic_hz"]
                if now >= next_discover:
                    try:
                        self.discover()
                    except Exception as exc:
                        # Do not end acquisition for a temporary ROS master outage.
                        self.metadata_errors.append("discovery: " + str(exc))
                        self.metadata_errors = self.metadata_errors[-50:]
                    next_discover = time.monotonic() + self.cfg["discovery_interval_s"]
                self.stop.wait(0.05)
            if self.rospy.is_shutdown() and not self.stop.is_set():
                self.reason = "ros_shutdown"
        except Exception as exc:
            self.error = str(exc)
            if self.reason == "external_stop":
                self.reason = "recorder_error"
        finally:
            with self.lock:
                self.accepting = False
            for sub in self.subscribers.values():
                sub.unregister()
            if self.writer:
                self.writer.finish()
                self.error = self.error or self.writer.error
            if diagnostics:
                diagnostics.close()
            if self.ros_initialized:
                self.metadata("end")
            self.topic_manifest()
            snapshot = self.snapshot()
            atomic_json(self.output / "summary.json", {
                "reason": self.reason, "stop_reason": self.reason, "error": self.error,
                "disk_free_bytes": self.disk_free_bytes, "min_free_space_bytes": self.cfg["min_free_space_bytes"],
                "bag_closed": bool(self.writer and not self.writer.error),
                "started_wall_s": started_wall, "ended_wall_s": time.time(),
                "elapsed_s": time.monotonic() - started_mono, "topics": snapshot,
                "received_count": sum(s["received"] for s in snapshot.values()),
                "written_count": sum(s["written"] for s in snapshot.values()),
                "dropped_count": sum(s["dropped"] for s in snapshot.values()),
                "unwritten_count": sum(s["received"] - s["written"] - s["dropped"] for s in snapshot.values()),
                "missing": [n for n, s in snapshot.items() if not s["received"]],
                "rejected": self.rejected, "metadata_errors": self.metadata_errors,
                "resource_maxima": self.resources.maxima, "last_status": self.status})
            ready = self.output / "recording_ready.json"
            if ready.exists():
                value = json.loads(ready.read_text())
                atomic_json(ready, {**value, "ready": False, "stopped_wall_s": time.time()})
        return 1 if self.error else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="fresh run/recording directory; parents may exist")
    parser.add_argument("--config", default=str(Path(__file__).with_name("recording.yaml")))
    parser.add_argument("--duration", type=float, default=0, help="0: external stop, bounded by YAML max_duration_s")
    parser.add_argument("--preview", action="store_true", help="validate/print plan; no ROS imports, files or master access")
    args = parser.parse_args(argv)
    try:
        cfg = load_config(args.config)
        duration = effective_duration(args.duration, cfg["max_duration_s"])
        if args.preview:
            print(json.dumps({"preview": True, "output": str(Path(args.output).resolve()),
                "effective_duration_s": duration, "config": cfg}, indent=2))
            return 0
        import rospy
        import rosbag
        from roslib.message import get_message_class
        recorder = Recorder(cfg, args.output, duration, rospy, rosbag, get_message_class)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, recorder.stop_signal)
        return recorder.run()
    except Exception as exc:
        print("record_diagnostics: " + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
