#!/usr/bin/env python3
"""Offline tests: fake ROS/master/bag only; no ROS imports or sockets."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

import yaml

ROOT = Path(__file__).resolve().parent


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


record = import_file("record_diagnostics", ROOT / "record_diagnostics.py")
profile = import_file("prepare_profile", ROOT / "px4_logging/prepare_profile.py")


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class Message:
    _type = "geometry_msgs/PoseStamped"

    def __init__(self, stamp=98, data="", size=32):
        self.header = types.SimpleNamespace(stamp=Stamp(stamp))
        self.data, self.size = data, size

    def serialize(self, stream):
        stream.write(b"x" * self.size)


class StringMessage(Message):
    _type = "std_msgs/String"


class DiagnosticMessage(Message):
    _type = "diagnostic_msgs/DiagnosticArray"


class LogMessage(Message):
    _type = "rosgraph_msgs/Log"


class FakeBag:
    def __init__(self, fail_write=False, fail_close=False):
        self.writes, self.closed = [], False
        self.fail_write, self.fail_close = fail_write, fail_close

    def write(self, name, message, t):
        if self.fail_write:
            raise OSError("mock disk full")
        self.writes.append((name, message, t))

    def close(self):
        self.closed = True
        if self.fail_close:
            raise OSError("mock close failure")


class FakeROS:
    Time = types.SimpleNamespace(now=lambda: Stamp(100))

    def __init__(self, messages=None):
        self.messages = messages or {}
        self.subs, self.init_kwargs = {}, None

    def init_node(self, name, **kwargs):
        self.init_kwargs = kwargs

    def is_shutdown(self):
        return False

    def Subscriber(self, name, cls, callback, **kwargs):
        sub = types.SimpleNamespace(unregistered=False, cls=cls, callback=callback)
        sub.unregister = lambda: setattr(sub, "unregistered", True)
        self.subs[name] = sub
        for message in self.messages.get(name, []):
            callback(message)
        return sub


class FakeMaster:
    def __init__(self, advertised=None):
        self.advertised = advertised or {}

    def call(self, method, *args):
        if method == "getPublishedTopics":
            return list(self.advertised.items())
        if method == "getParam":
            return {"use_sim_time": False, "test_parameter": 1}
        if method == "getSystemState":
            return [[("/Odometry", ["/lio"])], [], []]
        raise AssertionError("non-read-only method: " + method)


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.cfg = record.load_config(ROOT / "recording.yaml")

    def write_config(self, cfg, path):
        path.write_text(yaml.safe_dump(cfg))
        return record.load_config(path)

    def test_defaults_and_status_contract(self):
        self.assertEqual(self.cfg["status_topic"], "/low_hover_observation/status")
        self.assertEqual(record.effective_duration(0, self.cfg["max_duration_s"]), 900)
        names = {s["name"] for s in self.cfg["topics"]}
        for name in ("/Odometry", "/laserMapping/realtime", "/mavros/rc/in",
                     "/mavros/esc_telemetry", "/mavros/setpoint_raw/target_attitude"):
            self.assertIn(name, names)

    def test_lio_realtime_contract_matches_production_source(self):
        production = (ROOT.parents[1] / "patrol_uav_ws-patrol_planner/src/FAST_LIO/src/laserMapping.cpp").read_text()
        self.assertIn('ros::init(argc, argv, "laserMapping")', production)
        self.assertRegex(production, r'private_nh\.advertise<diagnostic_msgs::DiagnosticArray>\("realtime",\s*1\)')
        topic = next(t for t in self.cfg["topics"] if t["name"] == "/laserMapping/realtime")
        self.assertEqual(topic["types"], ["diagnostic_msgs/DiagnosticArray"])

    def test_rosout_agg_warning_wire_type(self):
        topic = next(t for t in self.cfg["topics"] if t["name"] == "/rosout_agg")
        self.assertEqual(topic["types"], ["rosgraph_msgs/Log"])

    def test_forbidden_names_even_with_safe_types(self):
        for name in ("/image_raw", "/compressed", "/data/JPEG", "/cloud_registered",
                     "/livox/lidar", "/point_cloud", "/x/depth", "/camera/data",
                     "/png", "/scan", "/foo/*", "/foo//bar", "/foo/"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                record.validate_topic(name, ["std_msgs/String"])

    def test_forbidden_wire_types_even_on_safe_names(self):
        for wire in ("sensor_msgs/Image", "sensor_msgs/CompressedImage", "sensor_msgs/PointCloud",
                     "sensor_msgs/PointCloud2", "livox_ros_driver2/CustomMsg", "custom/HiddenImage"):
            with self.subTest(wire=wire), self.assertRaises(ValueError):
                record.validate_topic("/Odometry", [wire])

    def test_strict_config_options_and_ranges(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.yaml"
            for key, value in (("all_topics", True), ("queue_messages", 1.5),
                               ("max_duration_s", 901), ("diagnostic_hz", 2),
                               ("diagnostic_hz", float("nan")), ("queue_bytes", True)):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.write_config({**self.cfg, key: value}, path)

    def test_custom_status_is_in_whitelist(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = self.write_config({**self.cfg, "status_topic": "/my_observation/status"}, Path(tmp) / "c.yaml")
            self.assertIn("/my_observation/status", [s["name"] for s in cfg["topics"]])

    def test_duration_validation(self):
        self.assertEqual(record.effective_duration(300, 900), 300)
        self.assertEqual(record.effective_duration(300, 60), 60)
        self.assertEqual(record.effective_duration(0, 0), 0)
        for duration in (-1, 901, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                record.effective_duration(duration, 900)

    def test_preview_without_ros_or_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "not-created"
            # Intercept imports in this process; main preview must not touch ROS.
            original_import = __import__
            def guarded_import(name, *args, **kwargs):
                if name in ("rospy", "rosbag", "roslib") or name.startswith("roslib."):
                    raise AssertionError("preview imported ROS")
                return original_import(name, *args, **kwargs)
            with mock.patch("builtins.__import__", guarded_import), contextlib.redirect_stdout(io.StringIO()) as stdout:
                rc = record.main(["--output", str(output), "--config", str(ROOT / "recording.yaml"), "--preview"])
            self.assertEqual(rc, 0)
            self.assertTrue(json.loads(stdout.getvalue())["preview"])
            self.assertFalse(output.exists())


class BufferAndTimingTests(unittest.TestCase):
    def test_bounded_bytes_count_and_recovery(self):
        buffer = record.BoundedBuffer(2, 100)
        self.assertTrue(buffer.put("a", 60))
        self.assertFalse(buffer.put("b", 60))
        self.assertTrue(buffer.put("c", 40))
        self.assertFalse(buffer.put("d", 0))
        self.assertEqual(buffer.get(), "a")
        self.assertTrue(buffer.put("e", 60))
        self.assertEqual(buffer.bytes, 100)

    def test_writer_drains_and_closes_only_own_bag(self):
        buffer, bag, written = record.BoundedBuffer(10, 1024), FakeBag(), []
        for i in range(5):
            buffer.put(("/Odometry", Message(), Stamp(i)), 32)
        writer = record.BagWriter(bag, buffer, written.append)
        writer.thread.start()
        writer.finish()
        self.assertEqual(len(bag.writes), 5)
        self.assertEqual(len(written), 5)
        self.assertTrue(bag.closed)
        self.assertIsNone(writer.error)

    def test_writer_disk_error_closes_and_reports(self):
        buffer, bag = record.BoundedBuffer(1, 1024), FakeBag(fail_write=True)
        buffer.put(("/Odometry", Message(), Stamp(1)), 32)
        writer = record.BagWriter(bag, buffer, lambda name: None)
        writer.thread.start()
        writer.finish()
        self.assertTrue(bag.closed)
        self.assertIn("disk full", writer.error)

    def test_source_age_receive_gap_and_clock_reset(self):
        stats = record.TopicStats()
        stats.observe(Message(98), 100, 200, 10)
        self.assertEqual(stats.source_age, 2)
        stats.observe(Message(97), 96, 201, 10.5)
        self.assertEqual(stats.source_age, -1)  # negative ages remain visible
        self.assertEqual(stats.source_regressions, 1)
        self.assertEqual(stats.received_gap, 0.5)
        self.assertEqual(stats.snapshot(13)["silent_s"], 2.5)

    def test_missing_zero_source_is_not_receive_time(self):
        for message in (types.SimpleNamespace(), Message(0), Message(float("nan"))):
            stats = record.TopicStats()
            stats.observe(message, 100, 200, 10)
            self.assertIsNone(stats.source_stamp)
            self.assertIsNone(stats.source_age)


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.disk_patch = mock.patch.object(record.shutil, "disk_usage", return_value=types.SimpleNamespace(free=2**40))
        self.disk_patch.start()
        self.addCleanup(self.disk_patch.stop)

    def recorder(self, output, advertised=None, messages=None, bag=None):
        cfg = record.load_config(ROOT / "recording.yaml")
        ros, bag = FakeROS(messages), bag or FakeBag()
        recorder = record.Recorder(cfg, output, 0.06, ros,
            types.SimpleNamespace(Bag=lambda *args, **kwargs: bag),
            lambda wire: {"std_msgs/String": StringMessage, "diagnostic_msgs/DiagnosticArray": DiagnosticMessage,
                          "rosgraph_msgs/Log": LogMessage}.get(wire, Message))
        recorder.master = FakeMaster(advertised)
        return recorder, ros, bag

    def test_fresh_subdir_ready_then_summary_and_receive_bag_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "run/recording"
            output.parent.mkdir()
            msg = Message(98)
            status = StringMessage(data='{"phase":"HOVER","armed":false}')
            recorder, ros, bag = self.recorder(output,
                {"/mavros/local_position/pose": Message._type, "/low_hover_observation/status": "std_msgs/String"},
                {"/mavros/local_position/pose": [msg], "/low_hover_observation/status": [status]})
            original_metadata = recorder.metadata
            def metadata(suffix):
                if suffix == "start":
                    ready = json.loads((output / "recording_ready.json").read_text())
                    self.assertTrue(ready["ready"])
                    self.assertTrue(recorder.writer.thread.is_alive())
                    self.assertFalse(bag.closed)
                original_metadata(suffix)
            recorder.metadata = metadata
            with contextlib.redirect_stdout(io.StringIO()) as stdout:
                self.assertEqual(recorder.run(), 0)
            self.assertTrue(stdout.getvalue().startswith("READY "))
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["written_count"], 2)
            self.assertEqual(summary["reason"], "duration_limit")
            self.assertEqual(summary["last_status"]["phase"], "HOVER")
            self.assertTrue(summary["bag_closed"])
            self.assertIn("/Odometry", summary["missing"])
            self.assertEqual(bag.writes[0][2].to_sec(), 100)
            self.assertEqual(bag.writes[0][1].header.stamp.to_sec(), 98)
            self.assertFalse(json.loads((output / "recording_ready.json").read_text())["ready"])
            self.assertTrue(ros.init_kwargs["disable_rosout"])
            self.assertEqual(ros.init_kwargs["argv"], ["record_diagnostics.py"])
            self.assertTrue(all(sub.unregistered for sub in ros.subs.values()))
            self.assertEqual((output / "rosnode_start.txt").read_text(), "/lio\n")
            self.assertTrue((output / "rosparam_start.yaml").exists())
            self.assertTrue((output / "diagnostics.jsonl").exists())

    def test_runtime_image_injection_never_subscribes(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder, ros, bag = self.recorder(Path(tmp) / "r", {"/Odometry": "sensor_msgs/CompressedImage"})
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(recorder.run(), 0)
            self.assertNotIn("/Odometry", ros.subs)
            self.assertIn("/Odometry", recorder.rejected)

    def test_lio_diagnostic_and_rosout_actually_recorded(self):
        with tempfile.TemporaryDirectory() as tmp:
            messages = {"/laserMapping/realtime": [DiagnosticMessage()], "/rosout_agg": [LogMessage()]}
            recorder, ros, bag = self.recorder(Path(tmp) / "r",
                {"/laserMapping/realtime": DiagnosticMessage._type, "/rosout_agg": LogMessage._type}, messages)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(recorder.run(), 0)
            self.assertEqual({name for name, _, _ in bag.writes}, set(messages))

    def test_startup_low_disk_summary_no_ready_no_ros(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder, ros, bag = self.recorder(Path(tmp) / "r")
            with mock.patch.object(record.shutil, "disk_usage", return_value=types.SimpleNamespace(free=100)):
                self.assertEqual(recorder.run(), 1)
            self.assertIsNone(ros.init_kwargs)
            self.assertFalse((recorder.output / "recording_ready.json").exists())
            summary = json.loads((recorder.output / "summary.json").read_text())
            self.assertEqual(summary["stop_reason"], "startup_low_disk_space")

    def test_running_low_disk_drains_only_own_bag(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder, ros, bag = self.recorder(Path(tmp) / "r",
                {"/mavros/local_position/pose": Message._type}, {"/mavros/local_position/pose": [Message()]})
            with mock.patch.object(record.shutil, "disk_usage", side_effect=[types.SimpleNamespace(free=2**40), types.SimpleNamespace(free=100)]), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(recorder.run(), 0)
            summary = json.loads((recorder.output / "summary.json").read_text())
            self.assertEqual(summary["stop_reason"], "low_disk_space")
            self.assertTrue(summary["bag_closed"])
            self.assertEqual(summary["written_count"], 1)
            self.assertEqual(summary["disk_free_bytes"], 100)


    def test_late_topic_and_wire_change_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder, ros, bag = self.recorder(Path(tmp))
            recorder.discover()
            self.assertFalse(ros.subs)
            recorder.master.advertised["/mavros/local_position/pose"] = Message._type
            recorder.discover()
            sub = ros.subs["/mavros/local_position/pose"]
            recorder.master.advertised["/mavros/local_position/pose"] = "sensor_msgs/PointCloud2"
            recorder.discover()
            self.assertTrue(sub.unregistered)
            self.assertNotIn("/mavros/local_position/pose", recorder.subscribers)

    def test_sigint_and_sigterm_leave_control_alone(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signum=signum), tempfile.TemporaryDirectory() as tmp:
                recorder, ros, bag = self.recorder(Path(tmp) / "r")
                recorder.duration = 0
                timer = threading.Timer(0.03, recorder.stop_signal, args=(signum, None))
                timer.start()
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(recorder.run(), 0)
                timer.join()
                self.assertTrue(bag.closed)
                self.assertEqual(recorder.reason, "signal_" + signal.Signals(signum).name)
                self.assertEqual((recorder.output / "rosnode_end.txt").read_text(), "/lio\n")

    def test_disk_error_summary_nonzero_no_control_calls(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder, ros, bag = self.recorder(Path(tmp) / "r",
                {"/mavros/local_position/pose": Message._type},
                {"/mavros/local_position/pose": [Message()]}, FakeBag(fail_write=True))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(recorder.run(), 1)
            summary = json.loads((recorder.output / "summary.json").read_text())
            self.assertIn("disk full", summary["error"])
            self.assertFalse(summary["bag_closed"])

    def test_oversize_message_dropped_nonblocking(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder, ros, bag = self.recorder(Path(tmp) / "r",
                {"/mavros/local_position/pose": Message._type},
                {"/mavros/local_position/pose": [Message(size=65537)]})
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(recorder.run(), 0)
            self.assertEqual(recorder.stats["/mavros/local_position/pose"].dropped, 1)
            self.assertEqual(len(bag.writes), 0)

    def test_output_reuse_refused_before_ros_initialization(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "recording_ready.json").write_text('{"ready":true}')
            recorder, ros, bag = self.recorder(Path(tmp))
            with self.assertRaises(ValueError):
                recorder.run()
            self.assertIsNone(ros.init_kwargs)


class PX4ProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "px4_logging/default_topics_30e763b.cpp.txt").read_text()

    def test_complete_default_names_preserved_and_capacity(self):
        text, manifest = profile.build_profile(self.source)
        names = {line.split()[0] for line in text.splitlines() if line and not line.startswith("#")}
        self.assertTrue(set(manifest["default_names"]) <= names)
        self.assertEqual(manifest["subscription_count"], 232)
        self.assertLessEqual(manifest["subscription_count"] + 4, 255)
        for line in text.splitlines():
            if line and not line.startswith("#"):
                self.assertEqual(len(line.split()), 3)  # explicit instance is essential
                self.assertLess(len(line), 79)

    def test_height_yaw_reset_sources_and_primary_instance_one(self):
        text, _ = profile.build_profile(self.source)
        for name in ("estimator_aid_src_ev_yaw", "estimator_aid_src_ev_hgt",
                     "estimator_aid_src_baro_hgt", "estimator_aid_src_mag",
                     "estimator_status_flags", "estimator_innovations"):
            for instance in (0, 1):
                self.assertRegex(text, r"(?m)^" + name + r" \d+ " + str(instance) + r"$")
        self.assertIn("vehicle_attitude 20 0", text)
        self.assertIn("vehicle_local_position 20 0", text)
        self.assertIn("vehicle_visual_odometry 0 0", text)

    def test_checked_in_profile_reproducible(self):
        text, manifest = profile.build_profile(self.source)
        self.assertEqual((ROOT / "px4_logging/logger_topics.txt").read_text(), text)
        self.assertEqual(json.loads((ROOT / "px4_logging/profile_manifest.json").read_text()), manifest)

    def test_excess_estimator_slots_rejected_without_truncation(self):
        with self.assertRaises(ValueError):
            profile.build_profile(self.source, 6)

    def test_newer_px4_checkout_refused(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(profile.subprocess, "check_output", return_value="newer_revision\n"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                profile.main(["--source", tmp, "--output", str(Path(tmp) / "prepared")])
            self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
