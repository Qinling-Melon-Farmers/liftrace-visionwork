#!/usr/bin/env python3
"""Turn the simulation-only bumper topic into durable collision facts."""

import json
import math
import os
import sys
import threading
import time

import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String

from uav_mission.contact_policy import (
    contact_episode_transition,
    relevant_contact_pairs,
)


class GazeboContactMonitor:
    def __init__(self):
        rospy.init_node("gazebo_contact_monitor")
        self._status_topic = rospy.get_param(
            "~status_topic", "/mission/gazebo_contact_status")
        self._raw_topic = rospy.get_param(
            "~raw_topic", "/mission/uav_contacts_raw")
        self._ignored = tuple(rospy.get_param(
            "~ignored_collision_patterns", ["ground_plane"]))
        self._status_path = rospy.get_param(
            "~status_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"),
                "gazebo_contact_status.json"))
        self._ready = False
        self._sample_count = 0
        self._actual_collision_count = 0
        self._active = False
        self._active_pairs = []
        self._events = []
        self._support_events=[]
        self._support_active=False
        self._support_count=0
        self._guard_xy=None
        self._last_message_wall = None
        self._lock = threading.RLock()
        self._publisher = rospy.Publisher(
            self._status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(
            self._raw_topic, ContactsState, self._on_contacts, queue_size=20)
        self._publish()

    @staticmethod
    def _contact_details(states):
        return [dict(pair=[s.collision1_name,s.collision2_name],info=s.info,
                     max_depth_m=max(s.depths) if s.depths else None,
                     total_force_norm_n=math.sqrt(s.total_wrench.force.x**2+s.total_wrench.force.y**2+s.total_wrench.force.z**2),
                     positions=[[v.x,v.y,v.z] for v in s.contact_positions[:8]],
                     normals=[[v.x,v.y,v.z] for v in s.contact_normals[:8]]) for s in states]

    def _on_contacts(self, message):
        with self._lock:
            for state in message.states:
                if state.info.startswith('guard_xy_m='):
                    try:self._guard_xy=float(state.info.split('=',1)[1])
                    except ValueError:pass
            support=[s for s in message.states if any(p in s.collision1_name or p in s.collision2_name for p in self._ignored)]
            if support:
                details=self._contact_details(support)
                force=max((v['total_force_norm_n'] for v in details),default=0.)
                if not self._support_active:
                    self._support_count+=1
                    self._support_events.append(dict(ros_stamp=message.header.stamp.to_sec(),last_ros_stamp=message.header.stamp.to_sec(),details=details,peak_sampled_force_n=force))
                    self._support_events=self._support_events[-64:]
                else:
                    event=self._support_events[-1];event['last_ros_stamp']=message.header.stamp.to_sec()
                    if force>event['peak_sampled_force_n']:event['peak_sampled_force_n']=force;event['details']=details
            self._support_active=bool(support)
            pairs = relevant_contact_pairs(
                ((state.collision1_name, state.collision2_name)
                 for state in message.states),
                self._ignored)
            active, increment = contact_episode_transition(
                self._active, pairs)
            self._ready = True
            self._sample_count += 1
            self._last_message_wall = time.monotonic()
            if increment:
                self._actual_collision_count += increment
                self._events.append({
                    "sequence": self._actual_collision_count,
                    "ros_stamp": message.header.stamp.to_sec(),
                    "wall_time": time.time(),
                    "pairs": [list(pair) for pair in pairs],
                    "details": self._contact_details([s for s in message.states if tuple(sorted((s.collision1_name,s.collision2_name))) in pairs]),
                })
            self._active = active
            self._active_pairs = pairs
            self._publish()

    def _payload(self):
        age = (None if self._last_message_wall is None else
               max(0.0, time.monotonic() - self._last_message_wall))
        return {
            "component": "gazebo_contact_monitor",
            "status": "READY" if self._ready else "WAITING",
            "ready": self._ready,
            "reason": ("contact_sensor_streaming" if self._ready else
                       "waiting_for_contact_sensor"),
            "raw_topic": self._raw_topic,
            "ignored_collision_patterns": list(self._ignored),
            "sample_count": self._sample_count,
            "last_sample_wall_age": age,
            "actual_collision_count": self._actual_collision_count,
            "contact_active": self._active,
            "active_pairs": [list(pair) for pair in self._active_pairs],
            "events": list(self._events),
            "guard_xy_m": self._guard_xy,
            "support_episode_count": self._support_count,
            "support_events": list(self._support_events),
        }

    def _publish(self):
        with self._lock:
            payload = self._payload()
            os.makedirs(
                os.path.dirname(self._status_path) or ".", exist_ok=True)
            temporary = self._status_path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.write("\n")
            os.replace(temporary, self._status_path)
            self._publisher.publish(String(
                data=json.dumps(payload, sort_keys=True)))

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self._publish()
            rate.sleep()


if __name__ == "__main__":
    GazeboContactMonitor().run()
    sys.exit(0)
