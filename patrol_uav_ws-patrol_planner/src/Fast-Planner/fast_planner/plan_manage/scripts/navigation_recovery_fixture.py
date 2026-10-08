#!/usr/bin/env python3
"""Directed ROS production-chain test, with an isolated synthetic plant.

Only main agent may launch through sim_run.sh. No real MAVROS/FC services.
The test requires actual recovery execution AND reaching the original goal.
"""
import csv
import json
import math
import os
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, Int8
from patrol_control.msg import MissionCommand
from plan_manage.msg import Bspline
from navigation_recovery_msgs.msg import NavigationRecoveryContext, NavigationRecoveryCommand


class Fixture:
    def __init__(self):
        self.out = Path(os.environ['SIM_RUN_DIR'])
        self.case = rospy.get_param('~case')
        if self.case not in ('column', 'buffer', 'height'):
            raise ValueError('case must be column, buffer or height')
        self.p = [float(rospy.get_param('~start_x')), 0., float(rospy.get_param('~start_z'))]
        self.v = [0., 0., 0.]
        self.goal = [-.5, 0., 2.7 if self.case == 'height' else self.p[2]]
        self.command = None
        self.mode = -1
        self.started = None
        self.identity = None
        self.deadline = None
        self.recovery_started = None
        self.recovery_count = self.execution_count = self.final_count = self.resumed_count = 0
        self.pose_pub = rospy.Publisher('/recovery_fixture/pose', PoseStamped, queue_size=1)
        self.odom_pub = rospy.Publisher('/recovery_fixture/odom', Odometry, queue_size=1)
        self.state_pub = rospy.Publisher('/recovery_fixture/state', State, queue_size=1)
        self.cloud_pub = rospy.Publisher('/recovery_fixture/cloud', PointCloud2, queue_size=1)
        self.goal_pub = rospy.Publisher('/fastplanner/goal', PoseStamped, queue_size=1, latch=True)
        self.context_pub = rospy.Publisher('/planning/recovery_context', NavigationRecoveryContext, queue_size=1, latch=True)
        self.mission_pub = rospy.Publisher('/mission/command', MissionCommand, queue_size=1, latch=True)
        rospy.Subscriber('/detect/point_class', Int8, lambda m: setattr(self, 'mode', m.data), queue_size=1)
        rospy.Subscriber('/recovery_fixture/setpoint', PoseStamped, self.setpoint, queue_size=1)
        rospy.Subscriber('/planning/recovery_command', NavigationRecoveryCommand, self.recovery, queue_size=10)
        rospy.Subscriber('/planning/recovery_execution', NavigationRecoveryCommand, self.execution, queue_size=10)
        rospy.Subscriber('/planning/bspline', Bspline, self.spline, queue_size=10)
        self.log = (self.out / 'production_recovery.csv').open('w')
        self.csv = csv.writer(self.log)
        self.csv.writerow(['stamp', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'fsm_commands', 'server_commands', 'controller_commands', 'resume_splines'])

    def setpoint(self, message):
        self.command = message
        if self.recovery_started is not None:
            self.final_count += 1

    def recovery(self, message):
        if message.active:
            self.recovery_count += 1
            if self.recovery_started is None:
                self.recovery_started = message.started_at
            if message.goal_stamp != self.identity or message.deadline > self.deadline:
                self.finish(False, 'recovery_changed_original_identity_or_deadline')

    def execution(self, message):
        if message.active:
            self.execution_count += 1

    def spline(self, message):
        if self.recovery_started is not None and message.start_time > self.recovery_started:
            if message.goal_stamp != self.identity:
                self.finish(False, 'resume_changed_original_goal')
            self.resumed_count += 1

    def finish(self, passed, reason):
        result = dict(status='PASS' if passed else 'FAIL', case=self.case, reason=reason,
                      scope='production_map_fsm_server_controller_with_synthetic_plant',
                      px4_sitl=False, hardware=False, original_goal=self.goal,
                      final_position=self.p, fsm_commands=self.recovery_count,
                      server_commands=self.execution_count, controller_commands=self.final_count,
                      resumed_splines=self.resumed_count, release_commands=0)
        (self.out / 'gate_status.json').write_text(json.dumps(result, indent=2))
        self.log.flush()
        rospy.signal_shutdown(reason)

    def run(self):
        wall_start = time.monotonic()
        previous = rospy.Time.now()
        rate = rospy.Rate(100)
        ticks = 0
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = min(.03, max(0., (now - previous).to_sec()))
            previous = now
            if self.started is not None and self.command is not None:
                cmd_age = (now - self.command.header.stamp).to_sec()
                if 0 <= cmd_age < .25:
                    target = self.command.pose.position
                    desired = [8. * (q - p) for q, p in zip((target.x, target.y, target.z), self.p)]
                    norm = math.sqrt(sum(a*a for a in desired))
                    if norm > .5:
                        desired = [a*.5/norm for a in desired]
                    dv = [q-v for q, v in zip(desired, self.v)]
                    norm = math.sqrt(sum(a*a for a in dv))
                    scale = min(1., .5*dt/norm) if norm else 1.
                    self.v = [v+a*scale for v, a in zip(self.v, dv)]
                    self.p = [p+v*dt for p, v in zip(self.p, self.v)]
            pose = PoseStamped()
            pose.header = Header(stamp=now, frame_id='camera_init')
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.p
            pose.pose.orientation.w = 1.
            self.pose_pub.publish(pose)
            odom = Odometry(header=pose.header, child_frame_id='body')
            odom.pose.pose = pose.pose
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = self.v
            self.odom_pub.publish(odom)
            if ticks % 10 == 0:
                state = State(header=pose.header, connected=True, armed=True, mode='OFFBOARD')
                self.state_pub.publish(state)
                points = [(-2.3, -2.3, .01), (2.3, 2.3, .01)]
                if self.case != 'height':
                    points += [(1., 0., .8), (1., 0., 1.), (1., 0., 1.2)]
                self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(pose.header, points))
            if self.started is None and self.mode == 1 and time.monotonic()-wall_start > 3:
                self.started = now
                self.identity = now
                self.deadline = now + rospy.Duration(35)
                goal = PoseStamped(header=Header(seq=41, stamp=now, frame_id='camera_init'))
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.goal
                goal.pose.orientation.w = 1.
                context = NavigationRecoveryContext(header=goal.header, deadline=self.deadline, active=True)
                mission = MissionCommand(header=goal.header, command=MissionCommand.SEARCH, goal=goal)
                self.mission_pub.publish(mission)
                self.context_pub.publish(context)
                self.goal_pub.publish(goal)
            if ticks % 5 == 0:
                self.csv.writerow([now.to_sec(), *self.p, *self.v, self.recovery_count,
                                   self.execution_count, self.final_count, self.resumed_count])
                self.log.flush()
            if self.case != 'height':
                # Full body envelope against the same synthetic point geometry.
                if abs(self.p[0]-1.) <= .30 and abs(self.p[1]) <= .30 and .60 <= self.p[2] <= 1.40:
                    self.finish(False, 'necessary_body_clearance_violated')
            if self.p[2] > 3.5 or self.p[2] < .05:
                self.finish(False, 'hard_height_violated')
            error = math.sqrt(sum((p-q)**2 for p, q in zip(self.p, self.goal)))
            if self.resumed_count and self.execution_count and self.final_count and error < .10:
                self.finish(True, 'recovered_and_reached_original_goal')
            if self.deadline is not None and now >= self.deadline:
                self.finish(False, 'original_action_deadline')
            if time.monotonic()-wall_start > 60:
                self.finish(False, 'startup_or_execution_wall_timeout')
            ticks += 1
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('navigation_recovery_fixture')
    fixture = Fixture()
    try:
        fixture.run()
    except rospy.ROSInterruptException:
        pass
