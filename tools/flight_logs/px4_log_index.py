#!/usr/bin/env python3
"""Query real MAVLink LOG_ENTRY IDs; never infer IDs from FTP filenames."""
import argparse
from datetime import datetime, timedelta, timezone
import json
import math
import signal
import sys
import time


def collect_entries(rospy, namespace, timeout, require_ground, clock=time.monotonic, sleep=time.sleep):
    from mavros_msgs.msg import LogEntry
    from mavros_msgs.srv import LogRequestList, LogRequestEnd
    prefix = namespace.rstrip('/') + '/log_transfer/raw/'
    entries = {}
    counts = [None]
    def receive(msg):
        if not 0 <= msg.id <= 65534 or not 0 <= msg.num_logs <= 65535:
            return
        counts[0] = msg.num_logs
        if msg.num_logs and msg.size >= 16:
            entries[msg.id] = msg
    subscriber = rospy.Subscriber(prefix + 'log_entry', LogEntry, receive, queue_size=1024)
    end = None
    try:
        rospy.wait_for_service(prefix + 'log_request_end', timeout=5)
        end = rospy.ServiceProxy(prefix + 'log_request_end', LogRequestEnd)
        rospy.wait_for_service(prefix + 'log_request_list', timeout=5)
        request = rospy.ServiceProxy(prefix + 'log_request_list', LogRequestList)
        sleep(.3)
        require_ground()
        if not request(0, 65534).success:
            raise RuntimeError('LOG_REQUEST_LIST failed')
        deadline = clock() + timeout
        while clock() < deadline:
            require_ground()
            if rospy.is_shutdown():
                raise RuntimeError('ROS shutdown during FC index query')
            if counts[0] is not None and len(entries) >= counts[0]:
                break
            sleep(.05)
        if counts[0] is None:
            raise RuntimeError('No FC LOG_ENTRY response; no IDs inferred')
        complete = len(entries) >= counts[0]
        result = []
        for identifier, msg in sorted(entries.items()):
            stamp = msg.time_utc.to_sec()
            try:
                beijing = datetime.fromtimestamp(stamp, timezone(timedelta(hours=8))).isoformat() if stamp > 0 else None
            except (OverflowError, OSError, ValueError):
                beijing = None
            result.append({'log_id': identifier, 'bytes': int(msg.size), 'fcu_time': stamp,
                           'time_beijing': beijing, 'num_logs': msg.num_logs,
                           'last_log_num': msg.last_log_num, 'index_complete': complete})
        return result
    finally:
        # Only terminate our LOG_REQUEST mode; never stop a ROS/device process.
        try:
            if end is not None and not end().success:
                raise RuntimeError('LOG_REQUEST_END failed; check FC transfer state')
        finally:
            subscriber.unregister()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/mavros')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--format', choices=('json', 'text'), default='json')
    parser.add_argument('--list-timeout', type=float, default=15)
    parser.add_argument('--timeout', type=float, default=20)
    parser.add_argument('--workbench-index-marker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not math.isfinite(args.list_timeout) or not 0 < args.list_timeout <= 30:
        parser.error('--list-timeout must be 0..30 seconds')
    import rospy
    from mavros_msgs.msg import State
    rospy.init_node('workbench_px4_log_index', anonymous=True, disable_signals=True)
    state = [None, 0.]
    def receive(msg):
        state[:] = [msg, time.monotonic()]
    subscriber = rospy.Subscriber(args.namespace.rstrip('/') + '/state', State, receive, queue_size=1)
    def require_ground():
        msg, received_at = state
        if msg is None or time.monotonic() - received_at > 3 or not msg.connected or msg.armed:
            raise RuntimeError('Fresh connected and disarmed FCU state required')
    def interrupted(*_):
        raise KeyboardInterrupt('Index interrupted')
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        until = time.monotonic() + 6
        while state[0] is None and time.monotonic() < until:
            time.sleep(.05)
        require_ground()
        entries = collect_entries(rospy, args.namespace, args.list_timeout, require_ground)
        if args.format == 'json':
            print(json.dumps(entries, allow_nan=False), flush=True)
        else:
            for entry in entries:
                print('ID {log_id} | {bytes} bytes | {time_beijing}'.format(**entry))
    finally:
        subscriber.unregister()


if __name__ == '__main__':
    main()
