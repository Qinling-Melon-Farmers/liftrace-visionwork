#!/usr/bin/env python3
"""Observation-only ROS adapter for the reset-aware task/FC boundary.

All outputs stay in an isolated candidate namespace. No MAVROS command output,
mode service, arming, TF authority or raw actuator call exists in this node.
FC reset and independent LIO health contracts must come from verified producers;
they are deliberately not inferred from local_position or a pose difference.
"""
import copy
import json
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from uav_mission.msg import ReleasePermission
from uav_mission.task_frame_continuity import (
    BoundaryRejected, FcReset, FcState, LioHealth, Limits, Pose,
    TaskFrameBoundary, Transform, rotate)


def required(mapping, key, kind):
    value = mapping[key]
    if type(value) is not kind:
        raise BoundaryRejected('invalid_contract_field:'+key)
    return value


def parse_reset(payload):
    data = json.loads(payload)
    if required(data, 'version', int) != 1:
        raise BoundaryRejected('unsupported_reset_contract')
    return FcReset(float(data['stamp']), required(data, 'fc_epoch', str),
                   required(data, 'previous_counter', int), required(data, 'counter', int),
                   required(data, 'frame_id', str),
                   Transform(data['translation_new_from_previous'],
                             data['quaternion_new_from_previous_xyzw']),
                   required(data, 'authoritative', bool))


def parse_health(payload):
    data = json.loads(payload)
    if required(data, 'version', int) != 1:
        raise BoundaryRejected('unsupported_lio_health_contract')
    return LioHealth(float(data['stamp']), required(data, 'lio_epoch', str),
                     required(data, 'frame_id', str), required(data, 'healthy', bool),
                     required(data, 'body_calibrated', bool))


def from_pose(msg, odom=False):
    pose = msg.pose.pose if odom else msg.pose
    p, q = pose.position, pose.orientation
    return Pose(msg.header.stamp.to_sec(), msg.header.frame_id,
                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def to_pose(value):
    out = PoseStamped()
    out.header.stamp = rospy.Time.from_sec(value.stamp)
    out.header.frame_id = value.frame
    out.pose.position.x, out.pose.position.y, out.pose.position.z = value.xyz
    (out.pose.orientation.x, out.pose.orientation.y,
     out.pose.orientation.z, out.pose.orientation.w) = value.xyzw
    return out


class EvTaskBoundary:
    def __init__(self):
        self.lock = threading.RLock()
        # Required calibration has no invented zero/default mounting or ground.
        config = rospy.get_param('~reference')
        self.core = TaskFrameBoundary(
            task_frame=config['task_frame'], fc_frame=config['fc_frame'],
            lio_frame=config['lio_frame'], fc_epoch=config['fc_epoch'],
            lio_epoch=config['lio_epoch'],
            task_from_fc=Transform(**config['task_from_fc']),
            task_from_lio=Transform(**config['task_from_lio']),
            body_to_camera=Transform(**config['body_to_camera']),
            ground_z=config['ground_z'],
            calibration_verified=config.get('calibration_verified', False),
            initial_reset_counter=config['initial_reset_counter'],
            limits=Limits(**rospy.get_param('~limits', {})))
        self.fc_msg = self.lio_msg = self.health = self.permission = None
        ns = rospy.get_param('~observer_namespace', '/ev_task_boundary').rstrip('/')
        if ns != '/ev_task_boundary' and not ns.startswith('/ev_task_boundary_'):
            raise BoundaryRejected('isolated_observer_namespace_required')
        self.pubs = {}
        for name, kind in (('task_pose', PoseStamped), ('task_odom', Odometry),
                           ('camera_pose', PoseStamped), ('fc_setpoint', PoseStamped),
                           ('fc_hold_request', PoseStamped), ('release_permission', ReleasePermission),
                           ('status', String)):
            topic = rospy.resolve_name(rospy.get_param('~'+name+'_output', ns+'/'+name))
            if not topic.startswith(ns+'/'):
                raise BoundaryRejected('observer_output_must_remain_isolated')
            self.pubs[name] = rospy.Publisher(topic, kind, queue_size=1)
        subscriptions = (
            ('fc_odom_input', '/mavros/local_position/odom', Odometry, self.on_fc),
            ('lio_body_pose_input', '/mavros/vision_pose/pose', PoseStamped, self.on_lio),
            ('state_input', '/mavros/state', State, self.on_state),
            ('reset_input', ns+'/fc_reset_input', String, self.on_reset),
            ('lio_health_input', ns+'/lio_health_input', String, self.on_health),
            ('task_setpoint_input', ns+'/task_setpoint_input', PoseStamped, self.on_setpoint),
            ('permission_input', ns+'/permission_input', ReleasePermission, self.on_permission))
        self.subscribers = [rospy.Subscriber(rospy.get_param('~'+name, default), kind, cb,
                                            queue_size=1)
                            for name, default, kind, cb in subscriptions]
        rospy.Timer(rospy.Duration(.02), self.on_timer)

    def on_state(self, msg):
        with self.lock:
            self.core.update_state(FcState(msg.header.stamp.to_sec(), msg.connected,
                                          msg.armed, msg.mode))

    def _observe(self):
        if self.fc_msg is None or self.lio_msg is None or self.health is None or self.core.state is None:
            return
        now = rospy.Time.now().to_sec()
        try:
            accepted = self.core.observe(from_pose(self.fc_msg, True),
                                         from_pose(self.lio_msg), self.health,
                                         self.core.state, now)
            if accepted:
                snap = self.core.snapshot(now)
                out = copy.deepcopy(self.fc_msg)
                out.header.frame_id = self.core.task_frame
                out.pose.pose = to_pose(snap['body']).pose
                # Child-frame twist is unchanged; pose covariance is in the
                # new header frame, transformed with the same reference.
                rotation = np.column_stack([rotate(self.core.task_from_fc.xyzw, v)
                                             for v in ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))])
                j = np.zeros((6,6)); j[:3,:3] = j[3:,3:] = rotation
                cov = np.array(out.pose.covariance).reshape(6,6)
                if not np.all(np.isfinite(cov)) or not out.child_frame_id:
                    raise BoundaryRejected('invalid_fc_odom_covariance_or_child')
                out.pose.covariance = (j @ cov @ j.T).ravel().tolist()
                self.pubs['task_pose'].publish(to_pose(snap['body']))
                self.pubs['task_odom'].publish(out)
                self.pubs['camera_pose'].publish(to_pose(snap['camera']))
        except BoundaryRejected as exc:
            self.core._block(str(exc))

    def on_fc(self, msg):
        with self.lock:
            self.fc_msg = msg
            self._observe()

    def on_lio(self, msg):
        with self.lock:
            self.lio_msg = msg
            self._observe()

    def on_health(self, msg):
        with self.lock:
            try:
                self.health = parse_health(msg.data)
                self._observe()
            except (ValueError, KeyError, TypeError) as exc:
                self.core._block('invalid_lio_health_contract', True)
                rospy.logwarn_throttle(2., 'EV task health rejected: %s', exc)

    def on_reset(self, msg):
        with self.lock:
            try:
                self.core.apply_reset(parse_reset(msg.data), rospy.Time.now().to_sec())
            except (ValueError, KeyError, TypeError) as exc:
                self.core._block('invalid_fc_reset_contract', True)
                rospy.logwarn_throttle(2., 'EV task reset rejected: %s', exc)

    def on_setpoint(self, msg):
        with self.lock:
            try:
                output = self.core.command(from_pose(msg), rospy.Time.now().to_sec())
                self.pubs['fc_setpoint'].publish(to_pose(output))
            except BoundaryRejected:
                pass  # The status exposes HOLD_REQUIRED; no guessed command.

    def on_permission(self, msg):
        with self.lock:
            self.permission = (copy.deepcopy(msg), self.core.generation)

    def on_timer(self, _event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            snap = None
            try:
                snap = self.core.snapshot(now)
            except BoundaryRejected:
                pass
            hold = self.core.hold_request(now)
            if hold is not None:
                self.pubs['fc_hold_request'].publish(to_pose(hold))
            if self.permission is not None:
                original, generation = self.permission
                result = copy.deepcopy(original)
                result.permitted = bool(original.permitted and self.core.release_allowed(
                    permission_stamp=original.header.stamp.to_sec(),
                    valid_until=original.valid_until.to_sec(), now=now, generation=generation,
                    evidence_stamp=original.evidence_stamp.to_sec()))
                if not result.permitted:
                    result.reason = 'task_reference_inhibited:'+self.core.reason
                # Keep the original evidence/permission timestamps and expiry.
                self.pubs['release_permission'].publish(result)
            status = {'version':1, 'observe_only':True, 'ready':self.core.ready,
                      'hold_required':not self.core.ready, 'reason':self.core.reason,
                      'latched_fault':self.core.fault, 'generation':self.core.generation,
                      'reset_counter':self.core.reset_counter,
                      'effective_stamp':self.core.effective_stamp,
                      'fc_epoch':self.core.fc_epoch, 'lio_epoch':self.core.lio_epoch,
                      'ground_z_task':self.core.ground_z}
            if snap:
                status.update(body_agl=snap['body_agl'], camera_agl=snap['camera_agl'])
            self.pubs['status'].publish(String(data=json.dumps(status, allow_nan=False)))


if __name__ == '__main__':
    rospy.init_node('ev_task_boundary')
    EvTaskBoundary()
    rospy.spin()
