"""Pause class inference in H landing; invalidate in-flight mode transitions."""
import threading

import rospy
from std_msgs.msg import String


class DetectorStageGate:
    def __init__(self):
        self._lock = threading.Lock()
        self._enabled = bool(rospy.get_param("~pause_in_landing_mode", True))
        self._paused = (rospy.get_param("~default_align_mode", "disabled") == "landing")
        self._epoch = 0
        self._subscriber = rospy.Subscriber(
            rospy.get_param("~align_mode_topic", "/uav_vision/align_mode"),
            String, self._on_mode, queue_size=1)

    def _on_mode(self, msg):
        paused = msg.data.strip() == "landing"
        with self._lock:
            if paused != self._paused:
                self._paused = paused
                self._epoch += 1

    def begin(self):
        with self._lock:
            return None if self._enabled and self._paused else self._epoch

    def current(self, epoch):
        with self._lock:
            return epoch == self._epoch and not (self._enabled and self._paused)
