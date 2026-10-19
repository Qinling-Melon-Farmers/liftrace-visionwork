#!/usr/bin/env python3
"""Convert measured feedback and setpoints across the MAVROS/LIO boundary.

Odometry twist remains in child_frame_id per nav_msgs/Odometry. No new TF
authority, timestamp rewriting, or latest-transform fallback is introduced.
"""
import copy
import math
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_multiply


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
        for name, kind, source, target, input_default, output_default in (
            ('pose', PoseStamped, self.local, self.mission,
             '/mavros/local_position/pose', '/navigation/local_pose'),
            ('odom', Odometry, self.local, self.mission,
             '/mavros/local_position/odom', '/navigation/local_odom'),
            ('setpoint', PoseStamped, self.mission, self.local,
             '/navigation/setpoint_mission', '/mavros/setpoint_position/local'),
        ):
            # Feedback-only mode has no flight-command publisher or subscriber.
            if name == 'setpoint' and not rospy.get_param('~enable_setpoints', False):
                continue
            input_topic = rospy.get_param('~' + name + '_input', input_default)
            output_topic = rospy.get_param('~' + name + '_output', output_default)
            if rospy.resolve_name(input_topic) == rospy.resolve_name(output_topic):
                raise ValueError('adapter input/output loop')
            pub = rospy.Publisher(output_topic, kind, queue_size=1)
            self.subscribers.append(rospy.Subscriber(
                input_topic, kind, self.callback,
                callback_args=(source, target, pub), queue_size=1))

    def callback(self, msg, args):
        source, target, pub = args
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
            pose, rotation = convert_pose(msg.pose.pose if is_odom else msg.pose,
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
            else:
                out.pose = pose
            out.header.frame_id = target
            pub.publish(out)
        except (ValueError, tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(5.0, 'Navigation frame conversion rejected: %s', error)


if __name__ == '__main__':
    rospy.init_node('navigation_frame_adapter')
    FrameAdapter()
    rospy.spin()
