#!/usr/bin/env python3
"""板端只读遥测探针（试飞工作台使用）。

边界（重要）：
  * 只订阅话题、读节点列表、读服务列表；
  * 不发布任何话题、不调用任何服务、不改飞控模式、不解锁、不碰舵机；
  * 进程名固定为 flight_workbench_probe，不在 run_trial.py 的冲突节点集合里。

每 interval 秒向 stdout 打印一行 JSON，由笔记本侧工作台解析。master 未起来时也保持
存活并周期性报告 {"master": false}，这样工作台可以在 roscore 之前就启动探针。

用法（板端，先 source 现场环境）：
    python3 board_probe.py --interval 1.0 --topics /mavros/state,/camera/image_raw
"""
import argparse
import importlib
import json
import math
import os
import socket
import sys
import time


OBSERVE_TYPES = {
    'fc_pose': ('geometry_msgs.msg', 'PoseStamped'),
    'lio_pose': ('nav_msgs.msg', 'Odometry'),
    'ev_pose': ('geometry_msgs.msg', 'PoseStamped'),
    'setpoint': ('geometry_msgs.msg', 'PoseStamped'),
    'battery': ('sensor_msgs.msg', 'BatteryState'),
    'rc_out': ('mavros_msgs.msg', 'RCOut'),
    'esc_status': ('mavros_msgs.msg', 'ESCStatus'),
    'esc_telemetry': ('mavros_msgs.msg', 'ESCTelemetry'),
    'low_hover': ('std_msgs.msg', 'String'),
}


def observe_topics(text):
    try:
        value = json.loads(text)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError('observe-topics must be a JSON mapping')
    if not isinstance(value, dict) or set(value) - set(OBSERVE_TYPES):
        raise argparse.ArgumentTypeError('unknown observe topic key')
    types = {}
    for key, topic in value.items():
        if not isinstance(topic, str) or not topic.startswith('/') or any(c.isspace() for c in topic):
            raise argparse.ArgumentTypeError('observe topics must be absolute topic names')
        if topic in types and types[topic] != OBSERVE_TYPES[key]:
            raise argparse.ArgumentTypeError('one observe topic cannot have conflicting wire types')
        types[topic] = OBSERVE_TYPES[key]
    return value


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topics", default="")
    parser.add_argument("--services", default="")
    parser.add_argument("--terminal-hover-topic", default="")
    parser.add_argument("--lio-realtime-topic", default="")
    parser.add_argument("--observe-topics", type=observe_topics, default={})
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--nodes-interval", type=float, default=5.0)
    parser.add_argument("--node-name", default="flight_workbench_probe")
    parser.add_argument("--once", action="store_true", help="打印一行后退出（自检用）")
    return parser.parse_args()


class Tracker(object):
    def __init__(self, topics):
        self.stats = {}
        for topic in topics:
            self.stats[topic] = {"count": 0, "window": 0, "last": None, "window_start": time.time()}

    def observe(self, topic):
        now = time.time()
        row = self.stats.setdefault(topic, {"count": 0, "window": 0, "last": None,
                                            "window_start": now})
        row["count"] += 1
        row["window"] += 1
        row["last"] = now

    def snapshot(self):
        now = time.time()
        out = {}
        for topic, row in self.stats.items():
            span = max(now - row["window_start"], 1e-6)
            out[topic] = {
                "count": row["count"],
                "hz": round(row["window"] / span, 2),
                "age": None if row["last"] is None else round(now - row["last"], 3),
            }
            row["window"] = 0
            row["window_start"] = now
        return out


def subscribe(rospy, topics, tracker, payloads, extractors):
    """已注册类型的话题解析成 dict/str；其余只统计新鲜度。"""
    for topic in topics:
        extractor = extractors.get(topic)

        def make_callback(name, extract):
            def callback(msg):
                tracker.observe(name)
                if extract is None:
                    return
                try:
                    payloads[name] = extract(msg)
                except Exception:
                    payloads[name] = None
            return callback

        message_type = getattr(extractor, "msg_type", None) or rospy.AnyMsg
        try:
            rospy.Subscriber(topic, message_type, make_callback(topic, extractor), queue_size=1)
        except Exception as error:  # 类型不可用时退回通用订阅
            try:
                rospy.Subscriber(topic, rospy.AnyMsg, make_callback(topic, None), queue_size=1)
            except Exception:
                sys.stderr.write("probe: cannot subscribe %s: %s\n" % (topic, error))


def read_payload(value):
    """把订阅到的值整理成可 JSON 序列化的结果（dict / JSON 字符串 / 短文本）。"""
    if value is None:
        return None
    if isinstance(value, (dict, list)):
        return value
    text = str(value).strip()
    if not text:
        return None
    if not text.startswith("{"):
        return text[:2000]
    try:
        return json.loads(text)
    except ValueError:
        return text[:2000]


def state_extractor():
    def extract(msg):
        return {"connected": bool(msg.connected), "armed": bool(msg.armed),
                "mode": str(msg.mode), "guided": bool(getattr(msg, "guided", False)),
                "system_status": int(getattr(msg, "system_status", 0))}
    return extract


def extended_extractor():
    def extract(msg):
        return {"landed_state": int(getattr(msg, "landed_state", 0)),
                "vtol_state": int(getattr(msg, "vtol_state", 0))}
    return extract


def string_extractor():
    def extract(msg):
        return getattr(msg, "data", None)
    return extract


def diagnostic_extractor():
    def extract(msg):
        return [{"name": str(s.name), "level": int(s.level), "message": str(s.message),
                 "values": {str(kv.key): str(kv.value) for kv in s.values}}
                for s in msg.status]
    return extract


def finite_number(value):
    try:
        if isinstance(value, bool) or value is None:
            return None
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError, OverflowError):
        return None


def pose_extractor(clock, odometry=False):
    def extract(msg):
        pose = msg.pose.pose if odometry else msg.pose
        p, q = pose.position, pose.orientation
        xyz = [finite_number(v) for v in (p.x, p.y, p.z)]
        quat = [finite_number(v) for v in (q.x, q.y, q.z, q.w)]
        stamp = finite_number(msg.header.stamp.to_sec())
        now = finite_number(clock())
        if None in xyz + quat or stamp is None or now is None or not math.isfinite(now-stamp):
            return None
        norm = math.hypot(*quat)
        if not math.isfinite(norm) or norm < 1e-12:
            return None
        x, y, z, w = [v / norm for v in quat]
        return dict(frame=str(msg.header.frame_id), stamp=stamp, source_age=now-stamp,
                    x=xyz[0], y=xyz[1], z=xyz[2],
                    roll_deg=math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
                    pitch_deg=math.degrees(math.asin(max(-1., min(1., 2*(w*y-z*x))))),
                    yaw_deg=math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))))
    return extract


def battery_extractor():
    def extract(msg):
        return {key: finite_number(getattr(msg, key, None))
                for key in ('voltage', 'current', 'percentage')}
    return extract


def rc_out_extractor():
    def extract(msg):
        channels = list(msg.channels)
        if any(isinstance(v, bool) or not isinstance(v, int) for v in channels):
            return None
        return {'channels': channels}
    return extract


def esc_extractor(field):
    def extract(msg):
        entries = []
        for slot, item in enumerate(getattr(msg, field)):
            index = getattr(item, 'index', None)
            entry = {'index': index if isinstance(index, int) and not isinstance(index, bool) else None,
                     'slot': slot}  # Array position only; never a physical motor/channel identity.
            entry.update({key: finite_number(getattr(item, key, None))
                          for key in ('rpm', 'voltage', 'current', 'temperature')})
            entries.append(entry)
        return {'entries': entries}
    return extract


def low_hover_extractor():
    def extract(msg):
        value = json.loads(msg.data)
        return value if isinstance(value, dict) else None
    return extract


def json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


def main():
    args = parse_args()
    topics = [t for t in args.topics.split(",") if t.strip()]
    services = [s for s in args.services.split(",") if s.strip()]
    payloads = {}
    extractors = {}

    import rospy  # noqa: E402  (延迟导入：无 master 时也可打印结构化错误)

    try:
        from mavros_msgs.msg import State, ExtendedState
        extractors["/mavros/state"] = state_extractor()
        extractors["/mavros/state"].msg_type = State
        extractors["/mavros/extended_state"] = extended_extractor()
        extractors["/mavros/extended_state"].msg_type = ExtendedState
    except Exception:
        pass
    try:
        from std_msgs.msg import String
        for topic in ("/navigation/mission_status", "/uav_high_view/probe_status",
                      "/board_trials/auto_land_status", args.terminal_hover_topic):
            if not topic:
                continue
            extractor = string_extractor()
            extractor.msg_type = String
            extractors[topic] = extractor
    except Exception:
        pass

    if args.lio_realtime_topic:
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            extractor = diagnostic_extractor()
            extractor.msg_type = DiagnosticArray
            extractors[args.lio_realtime_topic] = extractor
        except ImportError:
            pass

    observe_registered = set()
    for key, topic in args.observe_topics.items():
        if topic not in topics:
            topics.append(topic)
        try:
            module, name = OBSERVE_TYPES[key]
            msg_type = getattr(importlib.import_module(module), name)
            if key in ('fc_pose', 'lio_pose', 'ev_pose', 'setpoint'):
                extractor = pose_extractor(lambda: rospy.Time.now().to_sec(), key == 'lio_pose')
            elif key == 'battery':
                extractor = battery_extractor()
            elif key == 'rc_out':
                extractor = rc_out_extractor()
            elif key in ('esc_status', 'esc_telemetry'):
                extractor = esc_extractor(key)
            else:
                extractor = low_hover_extractor()
            extractor.msg_type = msg_type
            extractors[topic] = extractor
            observe_registered.add(key)
        except Exception:
            # Count available packets using AnyMsg; do not manufacture typed data.
            pass

    for topic in list(extractors):
        if topic not in topics:
            topics.append(topic)
    for topic in extractors:
        payloads[topic] = None

    hostname = socket.gethostname()
    tracker = Tracker(topics)
    nodes = []
    node_read = 0.0
    started = False

    def emitter(master, extra=None):
        row = {
            "t": round(time.time(), 3),
            "master": bool(master),
            "probe": {"node": "/" + args.node_name, "host": hostname, "pid": os.getpid()},
            "topics": tracker.snapshot(),
            "observe": {key: None for key in args.observe_topics},
        }
        if master:
            def fresh_payload(topic):
                age = (row["topics"].get(topic) or {}).get("age")
                if age is None or age < 0 or age > 2:
                    return None
                return read_payload(payloads.get(topic))

            row["nodes"] = list(nodes)
            row["services"] = service_types
            row["state"] = fresh_payload("/mavros/state")
            row["extended"] = fresh_payload("/mavros/extended_state")
            row["mission"] = fresh_payload("/navigation/mission_status")
            row["probe_status"] = fresh_payload("/uav_high_view/probe_status")
            row["auto_land"] = fresh_payload("/board_trials/auto_land_status")
            row["terminal_hover"] = fresh_payload(args.terminal_hover_topic)
            row["lio_realtime"] = fresh_payload(args.lio_realtime_topic)
            for key, topic in args.observe_topics.items():
                value = fresh_payload(topic) if key in observe_registered else None
                if key in ('fc_pose', 'lio_pose', 'ev_pose', 'setpoint') and isinstance(value, dict):
                    value = dict(value)
                    try:
                        value['source_age'] = finite_number(rospy.Time.now().to_sec() - value['stamp'])
                    except Exception:
                        value['source_age'] = None
                row['observe'][key] = value
        if extra:
            row.update(extra)
        try:
            line = json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError):
            # 板端不允许因为一个不可序列化的字段就停止上报
            line = json.dumps(json_safe(row), ensure_ascii=False, default=str, allow_nan=False)
        sys.stdout.write(line + "\n")
        sys.stdout.flush()

    service_types = {}
    interval = max(0.2, float(args.interval))
    while not rospy.is_shutdown():
        if not started:
            try:
                rospy.init_node(args.node_name, disable_signals=True, log_level=rospy.ERROR)
                subscribe(rospy, topics, tracker, payloads, extractors)
                started = True
            except Exception as error:
                emitter(False, {"error": "no ROS master: %s" % str(error)[:200]})
                if args.once:
                    return 1
                time.sleep(2.0)
                continue
        now = time.time()
        if now - node_read >= float(args.nodes_interval):
            node_read = now
            try:
                import rosnode
                nodes = sorted(rosnode.get_node_names())
            except Exception:
                pass
            try:
                import rosservice
                listed = set(rosservice.get_service_list())
                service_types = {}
                for service in services:
                    if service in listed:
                        try:
                            service_types[service] = rosservice.get_service_type(service)
                        except Exception:
                            service_types[service] = "unknown"
            except Exception:
                pass
        emitter(True)
        if args.once:
            return 0
        time.sleep(interval)
    return 0


if __name__ == "__main__":
    sys.exit(main())
