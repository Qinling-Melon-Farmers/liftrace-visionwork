#!/usr/bin/env python3
"""Convert measured feedback and setpoints across the MAVROS/LIO boundary.

Odometry twist remains in child_frame_id per nav_msgs/Odometry. No new TF
authority, timestamp rewriting, or latest-transform fallback is introduced.
"""
import copy
import math
import threading
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_matrix, quaternion_multiply, euler_from_quaternion, quaternion_from_euler


def convert_pose(pose, transform):
    result = copy.deepcopy(pose)
    t, q = transform.translation, transform.rotation
    rotation = np.array([q.x, q.y, q.z, q.w], dtype=float)
    original = np.array([pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w], dtype=float)
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    translation = np.array([t.x, t.y, t.z])
    if not all(np.all(np.isfinite(v)) for v in
               (rotation, original, position, translation)):
        raise ValueError('nonfinite pose/transform')
    if np.linalg.norm(rotation) < 1e-6 or np.linalg.norm(original) < 1e-6:
        raise ValueError('invalid quaternion')
    rotation /= np.linalg.norm(rotation)
    original /= np.linalg.norm(original)
    matrix = quaternion_matrix(rotation)[:3, :3]
    xyz = matrix.dot(position) + translation
    xyzw = quaternion_multiply(rotation, original)
    result.position.x, result.position.y, result.position.z = xyz.tolist()
    (result.orientation.x, result.orientation.y,
     result.orientation.z, result.orientation.w) = xyzw.tolist()
    return result, matrix


class FrameAdapter:
    def __init__(self):
        self.local = rospy.get_param('~local_frame', 'map')
        self.mission = rospy.get_param('~mission_frame', 'camera_init')
        self.max_age = float(rospy.get_param('~max_age', 0.3))
        self.future = float(rospy.get_param('~future_tolerance', 0.05))
        if (not self.local or not self.mission or self.local == self.mission
                or not math.isfinite(self.max_age) or self.max_age <= 0
                or not math.isfinite(self.future) or self.future < 0):
            raise ValueError('invalid adapter parameters')
        self.wait = float(rospy.get_param('~transform_wait', 0.08))
        if not 0 <= self.wait <= self.max_age:
            raise ValueError('invalid transform wait')
        self.buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.subscribers = []
        self.command_lock = threading.Lock()
        self.last_command_stamp = rospy.Time(0)
        for name, kind, source, target, input_default, output_default in (
            ('pose', PoseStamped, self.local, self.mission,
             '/mavros/local_position/pose', '/navigation/local_pose'),
            ('odom', Odometry, self.local, self.mission,
             '/mavros/local_position/odom', '/navigation/local_odom'),
            ('setpoint', PoseStamped, self.mission, self.local,
             '/navigation/setpoint_mission', '/mavros/setpoint_position/local'),
            ('landing_vertical', PoseStamped, self.mission, self.local,
             '/navigation/landing_setpoint_vertical', '/mavros/setpoint_raw/local'),
            ('landing_touchdown', PositionTarget, self.mission, self.local,
             '/navigation/landing_setpoint_touchdown', '/mavros/setpoint_raw/local'),
        ):
            # Feedback-only mode has no flight-command publisher or subscriber.
            if name in ('setpoint', 'landing_vertical', 'landing_touchdown') and not rospy.get_param('~enable_setpoints', False):
                continue
            input_topic = rospy.get_param('~landing_touchdown_setpoint_topic' if name == 'landing_touchdown'
                                         else '~' + name + '_input', input_default)
            output_topic = rospy.get_param('~' + name + '_output', output_default)
            if rospy.resolve_name(input_topic) == rospy.resolve_name(output_topic):
                raise ValueError('adapter input/output loop')
            output_kind = PositionTarget if name == 'landing_vertical' else kind
            pub = rospy.Publisher(output_topic, output_kind, queue_size=1)
            self.subscribers.append(rospy.Subscriber(
                input_topic, kind, self.callback,
                callback_args=(source, target, pub, name), queue_size=1))

    def callback(self, msg, args):
        if len(args) > 3 and args[3] in ('setpoint', 'landing_vertical', 'landing_touchdown'):
            # Serialize both output lanes: an old queued XYZ callback must never
            # override a newer vertical-only command (nor a later abort/hold).
            with self.command_lock:
                if msg.header.stamp <= self.last_command_stamp:
                    return
                if self.convert_and_publish(msg, args):
                    self.last_command_stamp = msg.header.stamp
        else:
            self.convert_and_publish(msg, args)

    def convert_and_publish(self, msg, args):
        source, target, pub = args[:3]
        vertical = len(args) > 3 and args[3] == 'landing_vertical'
        touchdown = len(args) > 3 and args[3] == 'landing_touchdown'
        try:
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if (msg.header.frame_id != source or msg.header.stamp.to_sec() <= 0
                    or not -self.future <= age <= self.max_age):
                raise ValueError('source frame/timestamp rejected')
            transform = self.buffer.lookup_transform(
                target, source, msg.header.stamp, rospy.Duration(self.wait))
            # A zero TF stamp denotes a static transform.
            if (transform.header.stamp.to_sec() > 0 and
                    abs((msg.header.stamp-transform.header.stamp).to_sec()) > self.max_age):
                raise ValueError('transform timestamp rejected')
            out = copy.deepcopy(msg)
            is_odom = isinstance(msg, Odometry)
            source_pose = touchdown_pose(msg) if touchdown else (msg.pose.pose if is_odom else msg.pose)
            pose, rotation = convert_pose(source_pose,
                                          transform.transform)
            if is_odom:
                if not msg.child_frame_id:
                    raise ValueError('odometry child frame missing')
                out.pose.pose = pose
                jacobian = np.zeros((6, 6))
                jacobian[:3, :3] = jacobian[3:, 3:] = rotation
                covariance = np.array(msg.pose.covariance).reshape(6, 6)
                if not np.all(np.isfinite(covariance)):
                    raise ValueError('invalid covariance')
                out.pose.covariance = (jacobian.dot(covariance).dot(jacobian.T)).ravel().tolist()
            elif not touchdown:
                out.pose = pose
            if vertical:
                out = vertical_target(msg.header, pose, rotation)
            if touchdown:
                out = touchdown_target(msg, pose, rotation)
            out.header.frame_id = target
            pub.publish(out)
            return True
        except (ValueError, tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(5.0, 'Navigation frame conversion rejected: %s', error)
            return False


TOUCHDOWN_MASK = (PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX |
                  PositionTarget.IGNORE_VY | PositionTarget.IGNORE_AFX |
                  PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                  PositionTarget.IGNORE_YAW_RATE)


def touchdown_pose(msg):
    """Validate mode 4's fixed XY / downward VZ contract; Z is only a TF anchor."""
    if (not isinstance(msg, PositionTarget)
            or msg.coordinate_frame != PositionTarget.FRAME_LOCAL_NED
            or msg.type_mask != TOUCHDOWN_MASK):
        raise ValueError('invalid touchdown coordinate frame/mask')
    if (not all(math.isfinite(v) for v in (msg.position.x, msg.position.y,
                                         msg.position.z, msg.velocity.z, msg.yaw))
            or msg.velocity.z >= 0):
        raise ValueError('touchdown requires finite pose and negative vertical velocity')
    pose = Pose()
    pose.position = copy.deepcopy(msg.position)
    q = quaternion_from_euler(0., 0., msg.yaw)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def touchdown_target(msg, pose, rotation):
    if not np.allclose(rotation[:, 2], [0., 0., 1.], atol=1e-6, rtol=0.):
        raise ValueError('touchdown requires gravity-aligned frames')
    out = PositionTarget()
    out.header = copy.deepcopy(msg.header)
    # ENU data; MAVROS performs ENU->NED. Never enable Z position control.
    out.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    out.type_mask = TOUCHDOWN_MASK
    out.position = copy.deepcopy(pose.position)
    out.velocity.z = float(rotation.dot([0., 0., msg.velocity.z])[2])
    q = pose.orientation
    out.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    return out


def vertical_target(header, pose, rotation):
    """H mode 3: zero XY velocity plus independent Z position trajectory."""
    # A Z-only command is meaningful only between gravity-aligned frames.
    # Do not project a tilted mission vertical into uncontrolled local XY.
    if not np.allclose(rotation[:, 2], [0., 0., 1.], atol=1e-6, rtol=0.):
        raise ValueError('vertical landing requires gravity-aligned frames')
    out = PositionTarget()
    out.header = copy.deepcopy(header)
    # MAVROS accepts ENU data here and performs the ENU->NED conversion.
    out.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    out.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                     PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                     PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                     PositionTarget.IGNORE_YAW_RATE)
    out.position.z = pose.position.z
    # Brake horizontal motion without returning to a fixed XY position.
    out.velocity.x = out.velocity.y = 0.
    q = pose.orientation
    out.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    return out


if __name__ == '__main__':
    rospy.init_node('navigation_frame_adapter')
    FrameAdapter()
    rospy.spin()
