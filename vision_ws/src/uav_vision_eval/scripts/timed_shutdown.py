#!/usr/bin/env python3
"""按墙钟时间结束一次评测 roslaunch，避免 /clock 暂停导致超时失效。"""

import time

import rospy


if __name__ == "__main__":
    rospy.init_node("timed_shutdown")
    timeout = float(rospy.get_param("~wall_timeout_sec", 40.0))
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.1)
    if not rospy.is_shutdown():
        rospy.loginfo("Evaluation wall timeout reached: %.1fs", timeout)
        rospy.signal_shutdown("evaluation complete")
