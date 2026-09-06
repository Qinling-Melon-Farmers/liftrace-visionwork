#!/usr/bin/env python3
"""Delay gazebo_ros/spawn_model for heavy toudi3 worlds.

PX4's stock posix_sitl.launch calls spawn_model as soon as the Gazebo ROS
service appears.  The toudi3 world may still be inserting static models at
that point, so the service call can time out.  This wrapper only delays the
call and then execs the stock ROS helper with all original arguments.
"""
import os
import sys
import time


def main():
    args = list(sys.argv[1:])
    delay = float(os.environ.get("TOUDI3_SPAWN_DELAY", "0.0"))
    retry_delay = float(os.environ.get("TOUDI3_SPAWN_RETRY_DELAY", "2.0"))
    retries = int(os.environ.get("TOUDI3_SPAWN_RETRIES", "4"))
    if len(args) >= 2 and args[0] == "--delay":
        delay = float(args[1])
        del args[:2]
    if len(args) >= 2 and args[0] == "--retry-delay":
        retry_delay = float(args[1])
        del args[:2]
    if len(args) >= 2 and args[0] == "--retries":
        retries = int(args[1])
        del args[:2]
    if delay > 0:
        time.sleep(delay)
    helper = "/opt/ros/noetic/lib/gazebo_ros/spawn_model"
    last_code = 1
    for attempt in range(max(1, retries)):
        last_code = os.spawnv(os.P_WAIT, helper, [helper] + args)
        if last_code == 0:
            return
        if attempt + 1 < max(1, retries):
            time.sleep(retry_delay)
    raise SystemExit(last_code)


if __name__ == "__main__":
    main()
