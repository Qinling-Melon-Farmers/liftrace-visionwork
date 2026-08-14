#!/usr/bin/env python3
"""确认运行期派生机架同时保留下视 RGB，并新增斜下 RGB、CameraInfo 和深度。"""

import json
import os
import threading
import time

import rospy
from sensor_msgs.msg import CameraInfo, Image


class Assertion:
    def __init__(self):
        self._counts = {"down_image": 0, "aux_image": 0,
                        "aux_info": 0, "aux_depth": 0}
        self._start = time.monotonic()
        self._timeout = float(rospy.get_param("~timeout_sec", 25.0))
        self._report_path = rospy.get_param(
            "~report_path", os.path.join(
                os.environ.get("SIM_RUN_DIR", "/tmp"), "gate_status.json"))
        rospy.Subscriber("/downward_camera/image_raw", Image,
                         self._increment, callback_args="down_image", queue_size=1)
        rospy.Subscriber("/aux_camera/image_raw", Image,
                         self._increment, callback_args="aux_image", queue_size=1)
        rospy.Subscriber("/aux_camera/camera_info", CameraInfo,
                         self._increment, callback_args="aux_info", queue_size=1)
        rospy.Subscriber("/aux_camera/depth/image_raw", Image,
                         self._increment, callback_args="aux_depth", queue_size=1)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _increment(self, _message, key):
        self._counts[key] += 1

    def _write(self, status, reason):
        payload = {"gate": "oblique_derived_vehicle_camera_smoke",
                   "status": status, "reason": reason,
                   "message_counts": self._counts}
        os.makedirs(os.path.dirname(os.path.abspath(self._report_path)), exist_ok=True)
        temporary = self._report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, self._report_path)

    def _run(self):
        while not rospy.is_shutdown():
            if all(value >= 3 for value in self._counts.values()):
                self._write("PASS", "downward_and_auxiliary_streams_available")
                rospy.signal_shutdown("camera assertion passed")
                return
            if time.monotonic() - self._start >= self._timeout:
                self._write("FAIL", "camera_stream_timeout")
                rospy.signal_shutdown("camera assertion timeout")
                return
            time.sleep(0.10)


if __name__ == "__main__":
    rospy.init_node("oblique_vehicle_camera_assertion")
    Assertion()
    rospy.spin()
