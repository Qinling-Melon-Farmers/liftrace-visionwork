#!/usr/bin/env python3
"""Independent, observation-only IMU prediction. Never publishes to MAVROS."""
import json
import os
import sys
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import String
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from fast_lio.msg import PredictionState
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from ev_predictor import Predictor, Limits, State, Imu


def vector(v):
    return np.array([v.x, v.y, v.z], dtype=float)


def snapshot(msg):
    q = msg.pose.orientation
    quat = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > 1e-3:
        raise ValueError('invalid_state_quaternion')
    return State(msg.header.stamp.to_sec(), vector(msg.pose.position),
                 quaternion_matrix(quat)[:3, :3], vector(msg.velocity),
                 vector(msg.gyro_bias), vector(msg.accel_bias), vector(msg.gravity),
                 np.array(msg.covariance).reshape(18, 18), np.array(msg.process_noise),
                 msg.accel_scale, msg.imu_time_offset, msg.epoch,
                 msg.header.frame_id, msg.imu_frame)


class Shadow:
    def __init__(self):
        settings = rospy.get_param('~limits', {})
        self.predictor = Predictor(Limits(**settings))
        self.lock = threading.Lock()
        self.last_stamp = None
        self.last_status = 0.
        self.rate = float(rospy.get_param('~output_hz', 50.))
        self.status_rate = float(rospy.get_param('~status_hz', 2.))
        if not np.isfinite(self.rate) or self.rate <= 0 or not np.isfinite(self.status_rate) or self.status_rate <= 0:
            raise ValueError('rates must be finite and positive')
        self.body_xyz = np.array(rospy.get_param('~imu_to_body_xyz', [0., 0., 0.]))
        if self.body_xyz.shape != (3,) or not np.isfinite(self.body_xyz).all():
            raise ValueError('invalid imu_to_body_xyz')
        self.pubs = []
        for name in ('~raw_pose', '~smooth_pose'):
            resolved = rospy.resolve_name(name)
            if not resolved.startswith('/ev_shadow/'):
                raise ValueError('shadow outputs must stay under /ev_shadow, cannot drive flight')
            self.pubs.append(rospy.Publisher(name, PoseStamped, queue_size=1))
        self.status = rospy.Publisher('~status', String, queue_size=1)
        self.imu_sub = rospy.Subscriber(rospy.get_param('~imu_topic'), ImuMsg, self.imu_cb,
                                       queue_size=500, tcp_nodelay=True)
        self.state_sub = rospy.Subscriber(rospy.get_param('~state_topic'), PredictionState,
                                         self.state_cb, queue_size=1, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(1/self.rate), self.tick)

    def imu_cb(self, msg):
        with self.lock:
            self.predictor.add_imu(Imu(msg.header.stamp.to_sec(), vector(msg.linear_acceleration),
                                      vector(msg.angular_velocity), msg.header.frame_id),
                                   rospy.Time.now().to_sec())

    def state_cb(self, msg):
        with self.lock:
            if not msg.valid:
                if self.predictor.state is None:
                    self.predictor.reason = 'waiting_for_valid_source_state'
                else:
                    self.predictor.fail('source_state_invalid')
                return
            try:
                self.predictor.correction(snapshot(msg), rospy.Time.now().to_sec())
            except (ValueError, TypeError) as exc:
                self.predictor.fail(str(exc))

    def tick(self, _event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            pair = self.predictor.output(now)
            if pair is not None and (self.last_stamp is None or pair[0].t > self.last_stamp+1e-7):
                self.last_stamp = pair[0].t
                for state, pub in zip(pair, self.pubs):
                    out = PoseStamped()
                    out.header.frame_id = state.frame
                    out.header.stamp = rospy.Time.from_sec(state.t)
                    # Transform only the reference point, as in existing EV bridge.
                    # Mounting rotation/world alignment are not guessed here.
                    p = state.p+state.r@self.body_xyz
                    out.pose.position.x, out.pose.position.y, out.pose.position.z = p
                    mat = np.eye(4)
                    mat[:3, :3] = state.r
                    q = quaternion_from_matrix(mat)
                    out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
                    pub.publish(out)
            if now-self.last_status >= 1/self.status_rate or now < self.last_status:
                self.last_status = now
                s, c = self.predictor.state, self.predictor.correction_t
                data = dict(reason=self.predictor.reason, fault=self.predictor.fault,
                            shadow_only=True, valid=pair is not None,
                            output_age=None if s is None else now-s.t,
                            correction_age=None if c is None else now-c,
                            imu_cache=len(self.predictor.imu), stats=self.predictor.stats)
                self.status.publish(String(data=json.dumps(data, allow_nan=False)))


if __name__ == '__main__':
    rospy.init_node('ev_shadow')
    Shadow()
    rospy.spin()
