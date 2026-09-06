#!/usr/bin/env python3
"""Publish the measured LIO body pose through MAVROS's standard EV input."""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def body_position(position, rotation, offset):
    """IMU-world position plus rotated IMU-to-body mounting translation."""
    x, y, z, w = rotation
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid LIO orientation')
    x, y, z, w = [v/norm for v in (x, y, z, w)]
    a, b, c = offset
    # q * offset * q^-1, avoiding an extra TF dependency or a latest-TF lookup.
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return [position[0]+a+w*tx+y*tz-z*ty,
            position[1]+b+w*ty+z*tx-x*tz,
            position[2]+c+w*tz+x*ty-y*tx], (x, y, z, w)


class ExternalPose:
    def __init__(self):
        self.frame = rospy.get_param('~frame', 'camera_init')
        self.offset = rospy.get_param('~imu_to_body_xyz', [0.0, 0.0, 0.0])
        self.max_age = rospy.get_param('~max_age', 0.3)
        self.future_tolerance = rospy.get_param('~future_tolerance', 0.05)
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', '/mavros/vision_pose/pose'), PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~odom_topic', '/Odometry'), Odometry, self.callback, queue_size=1)

    def callback(self, msg):
        age = (rospy.Time.now()-msg.header.stamp).to_sec()
        if msg.header.frame_id != self.frame or not -self.future_tolerance <= age <= self.max_age:
            rospy.logwarn_throttle(2.0, 'LIO external pose has an invalid frame or timestamp')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        try:
            xyz, xyzw = body_position((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.offset)
        except ValueError:
            return
        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x, out.pose.position.y, out.pose.position.z = xyz
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = xyzw
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('lio_external_pose')
    ExternalPose()
    rospy.spin()
