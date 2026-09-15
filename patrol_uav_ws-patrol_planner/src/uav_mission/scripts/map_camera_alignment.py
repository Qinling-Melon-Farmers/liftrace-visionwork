#!/usr/bin/env python3
"""Estimate and broadcast camera_init -> map from two poses of one body."""
from collections import deque
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State


def _normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in q)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _inverse(q):
    return (-q[0], -q[1], -q[2], q[3])


def _rotate(q, vector):
    x, y, z, w = q
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _dot(a, b):
    return sum(left * right for left, right in zip(a, b))


def _angle(a, b):
    return 2.0 * math.acos(max(-1.0, min(1.0, abs(_dot(a, b)))))


def _blend_quaternion(old, new, alpha):
    if _dot(old, new) < 0.0:
        new = tuple(-value for value in new)
    return _normalize(tuple(
        (1.0 - alpha) * left + alpha * right
        for left, right in zip(old, new)))


def _mean(samples):
    translation = tuple(
        sum(sample[0][axis] for sample in samples) / len(samples)
        for axis in range(3))
    reference = samples[0][1]
    aligned = [
        q if _dot(reference, q) >= 0.0 else tuple(-value for value in q)
        for _, q in samples]
    rotation = _normalize(tuple(
        sum(q[axis] for q in aligned) for axis in range(4)))
    max_translation = max(math.sqrt(sum(
        (sample[0][axis] - translation[axis]) ** 2 for axis in range(3)))
        for sample in samples)
    max_angle = max(_angle(rotation, sample[1]) for sample in samples)
    return translation, rotation, max_translation, max_angle


class MapCameraAlignment:
    def __init__(self):
        self.parent = rospy.get_param("~parent_frame", "camera_init")
        self.child = rospy.get_param("~child_frame", "map")
        self.lio_frame = rospy.get_param("~lio_frame", self.parent)
        self.mavros_frame = rospy.get_param("~mavros_frame", self.child)
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.04))
        self.max_age = float(rospy.get_param("~max_age", 0.30))
        self.sample_count = int(rospy.get_param("~sample_count", 20))
        self.max_spread = float(rospy.get_param("~max_translation_spread", 0.10))
        self.max_angle_spread = math.radians(float(
            rospy.get_param("~max_rotation_spread_deg", 5.0)))
        self.update_alpha = float(rospy.get_param("~update_alpha", 0.10))
        self.jump_translation = float(rospy.get_param(
            "~reinitialize_translation_jump", 0.50))
        self.jump_angle = math.radians(float(rospy.get_param(
            "~reinitialize_rotation_jump_deg", 15.0)))
        self.require_disarmed = bool(rospy.get_param(
            "~require_disarmed_for_initialization", True))
        if (not self.parent or not self.child or self.parent == self.child or
                self.sample_count < 3 or self.sync_slop <= 0.0 or
                self.max_age <= 0.0 or not 0.0 < self.update_alpha <= 1.0):
            raise ValueError("invalid map/camera alignment parameters")

        self.lock = threading.RLock()
        self.lio = None
        self.mavros = None
        self.state = None
        self.samples = deque(maxlen=self.sample_count)
        self.jump_samples = deque(maxlen=self.sample_count)
        self.alignment = None
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(rospy.get_param(
            "~lio_pose_topic", "/mavros/vision_pose/pose"),
            PoseStamped, self._lio_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~mavros_pose_topic", "/mavros/local_position/pose"),
            PoseStamped, self._mavros_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~state_topic", "/mavros/state"),
                         State, self._state_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self._publish)
        rospy.loginfo(
            "[MapCameraAlignment] estimating %s -> %s from common body poses",
            self.parent, self.child)

    @staticmethod
    def _pose(message):
        p = message.pose.position
        q = message.pose.orientation
        translation = (p.x, p.y, p.z)
        if not all(math.isfinite(value) for value in translation):
            raise ValueError("invalid translation")
        return translation, _normalize((q.x, q.y, q.z, q.w))

    def _state_callback(self, message):
        with self.lock:
            self.state = message

    def _lio_callback(self, message):
        with self.lock:
            self.lio = message
            self._update()

    def _mavros_callback(self, message):
        with self.lock:
            self.mavros = message
            self._update()

    def _update(self):
        if self.lio is None or self.mavros is None:
            return
        if self.lio.header.frame_id != self.lio_frame:
            rospy.logwarn_throttle(2.0,
                "[MapCameraAlignment] expected LIO frame %s, got %s",
                self.lio_frame, self.lio.header.frame_id)
            return
        if self.mavros.header.frame_id != self.mavros_frame:
            rospy.logwarn_throttle(2.0,
                "[MapCameraAlignment] expected MAVROS frame %s, got %s",
                self.mavros_frame, self.mavros.header.frame_id)
            return
        now = rospy.Time.now()
        lio_stamp = self.lio.header.stamp
        mavros_stamp = self.mavros.header.stamp
        if (lio_stamp.is_zero() or mavros_stamp.is_zero() or
                abs((lio_stamp - mavros_stamp).to_sec()) > self.sync_slop or
                (now - lio_stamp).to_sec() > self.max_age or
                (now - mavros_stamp).to_sec() > self.max_age or
                (now - lio_stamp).to_sec() < -0.05 or
                (now - mavros_stamp).to_sec() < -0.05):
            return
        if self.state is None or not self.state.connected:
            return
        if (self.alignment is None and self.require_disarmed and
                self.state.armed):
            rospy.logwarn_throttle(2.0,
                "[MapCameraAlignment] initialization requires disarmed FCU")
            return
        try:
            camera_body_t, camera_body_q = self._pose(self.lio)
            map_body_t, map_body_q = self._pose(self.mavros)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "[MapCameraAlignment] %s", error)
            return
        camera_map_q = _normalize(_multiply(
            camera_body_q, _inverse(map_body_q)))
        rotated_map_body = _rotate(camera_map_q, map_body_t)
        camera_map_t = tuple(
            camera_body_t[index] - rotated_map_body[index]
            for index in range(3))
        candidate = (camera_map_t, camera_map_q)

        if self.alignment is None:
            self.samples.append(candidate)
            self._initialize(self.samples, "initial")
            return
        current_t, current_q = self.alignment
        distance = math.sqrt(sum(
            (camera_map_t[index] - current_t[index]) ** 2
            for index in range(3)))
        angle = _angle(camera_map_q, current_q)
        if distance <= self.jump_translation and angle <= self.jump_angle:
            self.jump_samples.clear()
            alpha = self.update_alpha
            self.alignment = (
                tuple((1.0 - alpha) * current_t[index] +
                      alpha * camera_map_t[index] for index in range(3)),
                _blend_quaternion(current_q, camera_map_q, alpha))
        else:
            self.jump_samples.append(candidate)
            if self._initialize(self.jump_samples, "pose reset"):
                self.jump_samples.clear()
            else:
                rospy.logwarn_throttle(2.0,
                    "[MapCameraAlignment] rejecting transform jump %.2fm %.1fdeg",
                    distance, math.degrees(angle))

    def _initialize(self, samples, reason):
        if len(samples) < self.sample_count:
            return False
        translation, rotation, spread, angle_spread = _mean(samples)
        if spread > self.max_spread or angle_spread > self.max_angle_spread:
            rospy.logwarn_throttle(2.0,
                "[MapCameraAlignment] unstable samples: %.3fm %.2fdeg",
                spread, math.degrees(angle_spread))
            return False
        self.alignment = (translation, rotation)
        rospy.loginfo(
            "[MapCameraAlignment] %s alignment ready: xyz=(%.3f %.3f %.3f), "
            "spread=%.3fm/%.2fdeg", reason,
            translation[0], translation[1], translation[2],
            spread, math.degrees(angle_spread))
        return True

    def _publish(self, _event):
        with self.lock:
            if self.alignment is None:
                return
            translation, rotation = self.alignment
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.parent
        transform.child_frame_id = self.child
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("map_camera_alignment")
    try:
        MapCameraAlignment()
        rospy.spin()
    except (ValueError, rospy.ROSInterruptException) as error:
        rospy.logfatal("[MapCameraAlignment] %s", error)
