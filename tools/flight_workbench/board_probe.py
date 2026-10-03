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
import json
import os
import socket
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topics", default="")
    parser.add_argument("--services", default="")
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
                      "/board_trials/auto_land_status"):
            extractor = string_extractor()
            extractor.msg_type = String
            extractors[topic] = extractor
    except Exception:
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
        }
        if master:
            row["nodes"] = list(nodes)
            row["services"] = service_types
            row["state"] = read_payload(payloads.get("/mavros/state"))
            row["extended"] = read_payload(payloads.get("/mavros/extended_state"))
            row["mission"] = read_payload(payloads.get("/navigation/mission_status"))
            row["probe_status"] = read_payload(payloads.get("/uav_high_view/probe_status"))
            row["auto_land"] = read_payload(payloads.get("/board_trials/auto_land_status"))
        if extra:
            row.update(extra)
        try:
            line = json.dumps(row, ensure_ascii=False)
        except (TypeError, ValueError):
            # 板端不允许因为一个不可序列化的字段就停止上报
            line = json.dumps(row, ensure_ascii=False, default=str)
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
