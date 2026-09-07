#!/usr/bin/env python3
"""Read-only compact simulation records. No publishers or flight services."""
import csv
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import rosgraph
import roslib.message
import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import ParamGet
from sensor_msgs.msg import CameraInfo, PointCloud2
from std_msgs.msg import String


def plain(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (str, int, float, bool, type(None))):
        return value
    if isinstance(value, (list, tuple)):
        return [plain(x) for x in value]
    if hasattr(value, 'to_nsec'):
        return {'stamp_ns': value.to_nsec()}
    if hasattr(value, '__slots__'):
        return {name: plain(getattr(value, name)) for name in value.__slots__}
    return str(value)


class Recorder:
    def __init__(self):
        run_dir = rospy.get_param('~run_dir')
        if not run_dir:
            raise ValueError('run_dir must be supplied by the simulation wrapper')
        self.run = Path(run_dir)
        self.run.mkdir(parents=True, exist_ok=True)
        self.topics = rospy.get_param('~topics')
        self.model = rospy.get_param('~truth_model')
        self.offset = np.array(rospy.get_param('~truth_world_offset'), dtype=float)
        self.period = 1.0 / float(rospy.get_param('~pose_rate', 10.0))
        self.snapshot_radius = float(rospy.get_param('~snapshot_radius', 1.5))
        self.max_snapshots = int(rospy.get_param('~max_failure_snapshots', 3))
        self.lock = threading.RLock()
        self.events = (self.run / 'key_events.jsonl').open('w', buffering=1)
        self.files = {}
        self.writers = {}
        self.last = {}
        self.types = {}
        self.latest = {'phase': 'STARTUP'}
        self.goal = None
        self.mission_key = None
        self.snapshot_goals = set()
        self.snapshots = 0
        self.closed = False
        self.graph_saved = False
        for name in ('mavros_pose', 'truth_pose', 'lio_pose',
                     'planner_setpoint', 'mavros_setpoint'):
            f = (self.run / (name + '.csv')).open('w', buffering=1)
            self.files[name] = f
            self.writers[name] = csv.writer(f)
            self.writers[name].writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.subscribers = [
            rospy.Subscriber(self.topics['pose'], PoseStamped, self.pose, queue_size=1),
            rospy.Subscriber(self.topics['lio'], Odometry, self.lio, queue_size=1),
            rospy.Subscriber(self.topics['truth'], ModelStates, self.truth, queue_size=1),
            rospy.Subscriber(self.topics['camera_info'], CameraInfo, self.camera, queue_size=1),
            rospy.Subscriber(self.topics['mission'], String, self.mission, queue_size=1),
        ]
        for name in ('planner_setpoint', 'mavros_setpoint'):
            self.subscribers.append(rospy.Subscriber(
                self.topics[name], PoseStamped,
                lambda msg, kind=name: self.row(kind, msg.header.stamp.to_sec(), msg.pose),
                queue_size=1))
        for key in ('decision', 'result', 'planner', 'release', 'state', 'extended_state', 'bspline'):
            self.subscribers.append(rospy.Subscriber(
                self.topics[key], rospy.AnyMsg,
                lambda msg, kind=key: self.message(kind, msg), queue_size=20))
        self.timer = rospy.Timer(rospy.Duration(1.0), self.flush)
        rospy.on_shutdown(self.close)

    def event(self, kind, value):
        with self.lock:
            if not self.closed:
                self.events.write(json.dumps({'ros_sec': rospy.get_time(),
                                             'kind': kind, 'data': value}) + '\n')

    def row(self, name, stamp, pose):
        with self.lock:
            if self.closed or stamp - self.last.get(name, -1) < self.period:
                return
            self.last[name] = stamp
            p, q = pose.position, pose.orientation
            xyz = np.array([p.x, p.y, p.z], dtype=float)
            if name == 'truth_pose':
                xyz -= self.offset
            self.writers[name].writerow([stamp, *xyz, q.x, q.y, q.z, q.w])
            self.latest[name] = {'stamp': stamp, 'xyz': xyz.tolist()}

    def pose(self, msg):
        self.row('mavros_pose', msg.header.stamp.to_sec(), msg.pose)

    def lio(self, msg):
        self.row('lio_pose', msg.header.stamp.to_sec(), msg.pose.pose)

    def truth(self, msg):
        if self.model in msg.name:
            self.row('truth_pose', rospy.get_time(), msg.pose[msg.name.index(self.model)])

    def camera(self, msg):
        path = self.run / 'actual_camera_info.json'
        if not path.exists():
            path.write_text(json.dumps(plain(msg), indent=2))

    def mission(self, msg):
        value = json.loads(msg.data)
        key = tuple(value.get(k) for k in ('phase', 'committed_slots',
                    'post_delivery_route_index', 'active_decision_seq'))
        with self.lock:
            self.latest['mission'] = value
            self.latest['phase'] = value.get('phase')
            if key != self.mission_key:
                self.mission_key = key
                self.event('mission', value)
                rospy.loginfo('key record: phase=%s slots=%s return_index=%s', *key[:3])

    def message(self, kind, raw):
        name = raw._connection_header['type']
        if name not in self.types:
            self.types[name] = roslib.message.get_message_class(name)
        msg = self.types[name]().deserialize(raw._buff)
        value = plain(msg)
        if name == 'std_msgs/String':
            try:
                value = json.loads(msg.data)
            except ValueError:
                pass
        if kind in ('state', 'extended_state'):
            # Header timestamps alone should not create repeated state events.
            compare = {k: v for k, v in value.items() if k != 'header'}
            if self.latest.get(kind) == compare:
                return
            self.latest[kind] = compare
        self.event(kind, value)
        if kind == 'decision' and getattr(msg, 'has_goal', False):
            g = msg.goal.pose.position
            self.goal = np.array([g.x, g.y, g.z])
        if (kind == 'planner' and hasattr(msg, 'FAILED_ATTEMPT') and
                msg.status == msg.FAILED_ATTEMPT):
            goal_id = msg.goal_seq
            p = msg.requested_goal.pose.position
            snapshot_goal = np.array([p.x, p.y, p.z])
            if goal_id not in self.snapshot_goals and self.snapshots < self.max_snapshots:
                self.snapshot_goals.add(goal_id)
                self.snapshots += 1
                threading.Thread(target=self.snapshot, args=(self.snapshots, snapshot_goal), daemon=True).start()

    def snapshot(self, number, goal):
        result = {'goal': goal.tolist(), 'ros_sec': rospy.get_time(), 'clouds': {}}
        for key in ('static_map', 'inflated_map'):
            try:
                msg = rospy.wait_for_message(self.topics[key], PointCloud2, timeout=3.0)
                fields = {f.name: f.offset for f in msg.fields}
                dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                    'offsets': [fields[n] for n in ('x', 'y', 'z')], 'itemsize': msg.point_step})
                p = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
                xyz = np.column_stack([p[n] for n in ('x', 'y', 'z')])
                nearby = xyz[np.all(np.abs(xyz - goal) < self.snapshot_radius, axis=1)]
                result['clouds'][key] = {'stamp': msg.header.stamp.to_sec(),
                                        'frame': msg.header.frame_id, 'points': nearby.tolist()}
            except Exception as exc:
                result['clouds'][key] = {'error': str(exc)}
        (self.run / ('local_map_failure_%d.json' % number)).write_text(json.dumps(result))

    def flush(self, _event):
        with self.lock:
            if self.closed:
                return
            result = dict(self.latest, ros_sec=rospy.get_time())
        path = self.run / 'live_progress.json'
        temp = path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(result, indent=2))
        temp.replace(path)
        if not self.graph_saved and self.latest.get('phase') not in ('IDLE', 'STARTUP'):
            try:
                pub, sub, srv = rosgraph.Master(rospy.get_name()).getSystemState()
                (self.run / 'ros_system_state.json').write_text(json.dumps(
                    {'publishers': dict(pub), 'subscribers': dict(sub), 'services': dict(srv)}, indent=2))
                (self.run / 'rosparams.yaml').write_text(yaml.safe_dump(
                    rosgraph.Master(rospy.get_name()).getParam('/'),
                    allow_unicode=True, sort_keys=True))
                readback = {}
                service = rospy.get_param('~parameter_get_service')
                for name in rospy.get_param('~parameter_names'):
                    try:
                        rospy.wait_for_service(service, timeout=1.0)
                        response = rospy.ServiceProxy(service, ParamGet)(param_id=name)
                        readback[name] = plain(response)
                    except Exception as exc:
                        readback[name] = {'error': str(exc)}
                (self.run / 'px4_parameters_readback.json').write_text(json.dumps(readback, indent=2))
                self.graph_saved = True
            except Exception as exc:
                self.event('graph_capture_error', str(exc))

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.events.close()
            for f in self.files.values():
                f.close()


if __name__ == '__main__':
    rospy.init_node('competition_key_recorder')
    Recorder()
    rospy.spin()
