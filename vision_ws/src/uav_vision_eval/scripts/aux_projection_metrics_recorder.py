#!/usr/bin/env python3
"""把 AuxProjectionArray 的测距来源、有效率和不确定度写入 JSON。"""

from collections import Counter
import json
import os
import statistics
import threading

import rospy

from uav_vision_eval.msg import AuxProjectionArray


class Recorder:
    def __init__(self):
        self._topic = rospy.get_param(
            "~topic", "/uav_vision/aux/projection_diagnostics")
        self._output_path = os.path.abspath(rospy.get_param(
            "~output_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"),
                "projection_summary.json")))
        self._lock = threading.Lock()
        self._messages = 0
        self._observations = 0
        self._valid = 0
        self._sources = Counter()
        self._reasons = Counter()
        self._depth_fractions = []
        self._uncertainties = []
        self._ranges = []
        rospy.Subscriber(self._topic, AuxProjectionArray,
                         self._on_message, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(1.0), self._on_timer)
        rospy.on_shutdown(self._write)

    def _on_message(self, message):
        with self._lock:
            self._messages += 1
            for item in message.observations:
                self._observations += 1
                self._valid += int(item.valid)
                self._sources[item.range_source or "<empty>"] += 1
                if item.reason:
                    self._reasons[item.reason] += 1
                if item.depth_valid_fraction > 0.0:
                    self._depth_fractions.append(float(item.depth_valid_fraction))
                if item.position_uncertainty_m > 0.0:
                    self._uncertainties.append(float(item.position_uncertainty_m))
                if item.range_m > 0.0:
                    self._ranges.append(float(item.range_m))

    def _payload(self):
        return {
            "topic": self._topic,
            "messages": self._messages,
            "observations": self._observations,
            "valid_observations": self._valid,
            "valid_rate": (self._valid / float(self._observations)
                           if self._observations else None),
            "range_sources": dict(sorted(self._sources.items())),
            "reject_reasons": dict(sorted(self._reasons.items())),
            "mean_depth_valid_fraction": (
                statistics.mean(self._depth_fractions)
                if self._depth_fractions else None),
            "mean_position_uncertainty_m": (
                statistics.mean(self._uncertainties)
                if self._uncertainties else None),
            "mean_range_m": (
                statistics.mean(self._ranges) if self._ranges else None),
        }

    def _write(self):
        with self._lock:
            directory = os.path.dirname(self._output_path) or "."
            os.makedirs(directory, exist_ok=True)
            temporary = self._output_path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(self._payload(), stream, ensure_ascii=False,
                          indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temporary, self._output_path)

    def _on_timer(self, _event):
        self._write()


if __name__ == "__main__":
    rospy.init_node("aux_projection_metrics_recorder")
    Recorder()
    rospy.spin()
