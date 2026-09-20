#!/usr/bin/python3
"""Read-only ROS 1 monitor: small state subscriptions + periodic raw cloud samples."""
import json
import math
import struct
import threading
import time

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from plan_manage.msg import Bspline, PlannerStatus
from std_msgs.msg import Bool, String


def xyz(point):
    return '(%.3f, %.3f, %.3f)m' % (point.x, point.y, point.z)


def pose_text(pose):
    q = pose.orientation
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    yaw = 'invalid'
    if math.isfinite(norm) and norm > 1e-12:
        yaw = '%.1fdeg' % math.degrees(math.atan2(
            2*(q.w*q.z + q.x*q.y), norm - 2*(q.y*q.y + q.z*q.z)))
    return 'xyz=%s yaw=%s' % (xyz(pose.position), yaw)


def string_summary(msg, keys):
    try:
        data = json.loads(msg.data)
        return ' '.join('%s=%s' % (key, data.get(key, '?')) for key in keys)
    except (ValueError, AttributeError):
        return repr(msg.data[:180])


def cloud_summary(msg):
    """Decode just Header/height/width; never iterate or copy point records."""
    data = msg._buff
    if len(data) < 16:
        raise ValueError('short cloud header')
    _, sec, nsec, frame_len = struct.unpack_from('<IIII', data, 0)
    if frame_len > len(data) - 24:
        raise ValueError('invalid cloud frame length')
    frame = data[16:16+frame_len].decode('utf-8', errors='replace')
    height, width = struct.unpack_from('<II', data, 16+frame_len)
    age = rospy.Time.now().to_sec() - (sec + nsec * 1e-9) if sec or nsec else None
    return '%s points=%d frame=%s stamp_age=%s' % (
        'NONEMPTY' if height*width else 'EMPTY', height*width, frame,
        'unset' if age is None else '%.2fs' % age)


def _de_boor(control_points, knots, degree, value):
    """Evaluate a clamped/non-uniform B-spline without third-party modules."""
    count = len(control_points)
    if degree < 1 or count <= degree or len(knots) != count + degree + 1:
        raise ValueError('invalid B-spline dimensions')
    low, high = knots[degree], knots[count]
    if not all(math.isfinite(item) for item in knots) or high <= low:
        raise ValueError('invalid B-spline knots')
    value = max(low, min(high, value))
    if value >= high:
        span = count - 1
    else:
        span = degree
        while span + 1 < count and value >= knots[span + 1]:
            span += 1
    work = [list(control_points[span - degree + index])
            for index in range(degree + 1)]
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            knot_index = span - degree + index
            denominator = knots[knot_index + degree - level + 1] - knots[knot_index]
            alpha = 0.0 if abs(denominator) < 1e-12 else (
                value - knots[knot_index]) / denominator
            work[index] = [
                (1.0 - alpha) * work[index - 1][axis] + alpha * work[index][axis]
                for axis in range(3)]
    return tuple(work[degree])


def trajectory_summary(message, sample_count):
    points = [(point.x, point.y, point.z) for point in message.pos_pts]
    knots = list(message.knots)
    degree = int(message.order)
    sample_count = max(2, sample_count)
    start = knots[degree]
    end = knots[len(points)]
    samples = [_de_boor(points, knots, degree,
                        start + (end - start) * index / (sample_count - 1))
               for index in range(sample_count)]
    length = sum(math.sqrt(sum(
        (samples[index][axis] - samples[index - 1][axis]) ** 2
        for axis in range(3))) for index in range(1, len(samples)))
    path = ' -> '.join('(%.2f,%.2f,%.2f)' % point for point in samples)
    return ('traj_id=%d degree=%d control_points=%d duration=%.2fs '
            'sampled_length=%.2fm\n  path[%d]=%s' %
            (message.traj_id, degree, len(points), end - start,
             length, sample_count, path))


class Monitor:
    def __init__(self):
        self.interval = float(rospy.get_param('~interval', 5.0))
        if not math.isfinite(self.interval) or self.interval < 1.0:
            raise ValueError('interval must be finite and >= 1 second')
        self.window = min(2.0, self.interval * 0.8)
        self.stale = max(2.0, self.interval)
        self.path_samples = int(rospy.get_param('~path_samples', 12))
        if self.path_samples < 2 or self.path_samples > 50:
            raise ValueError('path_samples must be within [2, 50]')
        self.lock = threading.Lock()
        self.values = {}
        self.cloud_values = {}
        self.clouds = {}
        self.master = rosgraph.Master(rospy.get_name())
        self.subs = []
        self.cloud_topics = [
            rospy.get_param('~scan_topic', '/cloud_registered'),
            rospy.get_param('~map_topic', '/freedom/static_pointcloud'),
            rospy.get_param('~lio_map_topic', '/Laser_map')]
        if rospy.get_param('~sample_occupancy', False):
            self.cloud_topics.append(rospy.get_param('~occupancy_topic', '/sdf_map/occupancy_inflate'))
        self.cloud_topics = list(dict.fromkeys(rospy.resolve_name(t) for t in self.cloud_topics))
        self.specs = [
            ('lio', '~lio_odom_topic', '/Odometry', Odometry,
             lambda m: 'frame=%s child=%s %s' % (m.header.frame_id, m.child_frame_id, pose_text(m.pose.pose))),
            ('fcu_pose', '~fcu_pose_topic', '/mavros/local_position/pose', PoseStamped,
             lambda m: 'frame=%s %s' % (m.header.frame_id, pose_text(m.pose))),
            ('fcu', '~fcu_state_topic', '/mavros/state', State,
             lambda m: 'connected=%s armed=%s mode=%s' % (m.connected, m.armed, m.mode)),
            ('mission', '~mission_topic', '/navigation/mission_status', String,
             lambda m: string_summary(m, ['phase', 'last_reason', 'manual_start_required'])),
            ('bridge', '~bridge_topic', '/navigation/planner_bridge_status', String,
             lambda m: string_summary(m, ['adapter_faulted', 'last_reason', 'gate_reason'])),
            ('control_ready', '~control_ready_topic', '/mission/control_ready', Bool,
             lambda m: str(m.data)),
            ('goal', '~goal_topic', '/fastplanner/goal', PoseStamped,
             lambda m: 'frame=%s xyz=%s' % (m.header.frame_id, xyz(m.pose.position))),
            ('planner', '~planner_status_topic', '/planning/goal_status', PlannerStatus,
             self.planner_summary),
            ('trajectory', '~trajectory_topic', '/planning/bspline', Bspline,
             lambda m: trajectory_summary(m, self.path_samples)),
        ]
        self.topics = {}
        for key, param, default, cls, render in self.specs:
            topic = rospy.resolve_name(rospy.get_param(param, default))
            self.topics[key] = topic
            self.subs.append(rospy.Subscriber(topic, cls, self.on_state,
                                             callback_args=(key, render), queue_size=1))
        rospy.on_shutdown(self.close)

    @staticmethod
    def planner_summary(msg):
        names = ['ACCEPTED', 'PLANNING', 'TRAJECTORY_READY', 'REPLANNING',
                 'TRAJECTORY_FINISHED', 'FAILED_ATTEMPT', 'CANCELLED']
        status = names[msg.status] if msg.status < len(names) else str(msg.status)
        return 'status=%s goal_seq=%d distance=%.3fm reason=%s effective_goal=%s' % (
            status, msg.goal_seq, msg.distance_to_goal, msg.reason[:120], xyz(msg.effective_goal.pose.position))

    def on_state(self, msg, context):
        key, render = context
        value = render(msg)
        now = time.monotonic()
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        stamp_sec = stamp.to_sec() if stamp is not None else 0.0
        with self.lock:
            self.values[key] = (now, value, stamp_sec)

    def on_cloud(self, msg, topic):
        with self.lock:
            if topic in self.cloud_values:
                return
            try:
                actual_type = msg._connection_header.get('type', '')
                if actual_type != 'sensor_msgs/PointCloud2':
                    raise ValueError('expected PointCloud2, got ' + actual_type)
                self.cloud_values[topic] = cloud_summary(msg)
            except (ValueError, struct.error) as exc:
                self.cloud_values[topic] = 'ERROR ' + str(exc)

    def graph(self):
        try:
            pubs, _, _ = self.master.getSystemState()
            return {topic: nodes for topic, nodes in pubs}, None
        except Exception as exc:
            return {}, str(exc)

    def close(self):
        for sub in list(self.clouds.values()) + self.subs:
            sub.unregister()
        self.clouds.clear()

    def run(self):
        print('Read-only monitor; positions are in the printed frames, not GPS/AGL. '
              'Clouds are sampled intermittently; event age does not prove a fault. '
              'Occupancy: graph-only unless sample_occupancy:=true.', flush=True)
        while not rospy.is_shutdown():
            start = time.monotonic()
            publishers, graph_error = self.graph()
            with self.lock:
                self.cloud_values = {}
            for topic in self.cloud_topics:
                if publishers.get(topic):
                    self.clouds[topic] = rospy.Subscriber(
                        topic, rospy.AnyMsg, self.on_cloud, callback_args=topic,
                        queue_size=1, buff_size=2**20)
            while not rospy.is_shutdown() and self.clouds and time.monotonic() - start < self.window:
                with self.lock:
                    received = set(self.cloud_values)
                for topic in received:
                    sub = self.clouds.pop(topic, None)
                    if sub:
                        sub.unregister()
                time.sleep(0.1)
            for sub in list(self.clouds.values()):
                sub.unregister()
            self.clouds.clear()
            if rospy.is_shutdown():
                break
            now = time.monotonic()
            with self.lock:
                values, clouds = dict(self.values), dict(self.cloud_values)
            lines = ['\n[%s] navigation status' % time.strftime('%H:%M:%S')]
            if graph_error:
                lines.append('ROS MASTER ERROR: ' + graph_error)
            for key, _, _, _, _ in self.specs:
                topic = self.topics[key]
                if not publishers.get(topic):
                    text = 'NO_PUBLISHER' if not graph_error else 'GRAPH_UNAVAILABLE'
                elif key not in values:
                    text = 'NO_MESSAGE_SINCE_MONITOR_START (may be idle/event-only)'
                else:
                    arrived, value, stamp = values[key]
                    age = now - arrived
                    live = key in ('lio', 'fcu_pose', 'fcu')
                    state = ('STALE' if age > self.stale else 'RECENT') if live else 'LAST_EVENT'
                    stamp_age = ' stamp_age=%.2fs' % (rospy.Time.now().to_sec()-stamp) if live and stamp else ''
                    text = '%s received_age=%.1fs%s %s' % (state, age, stamp_age, value)
                lines.append('%-13s %s' % (key + ':', text))
            for topic in self.cloud_topics:
                lines.append('%s: %s' % (topic, clouds.get(topic,
                    'NO_SAMPLE_IN_WINDOW' if publishers.get(topic) else 'NO_PUBLISHER')))
            for topic in ['/sdf_map/occupancy_inflate', '/planning_vis/trajectory']:
                lines.append('%s publishers=%s (registration only)' % (topic, publishers.get(topic, [])))
            print('\n'.join(lines), flush=True)
            while not rospy.is_shutdown() and time.monotonic() - start < self.interval:
                time.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('navigation_status_monitor')
    try:
        Monitor().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
