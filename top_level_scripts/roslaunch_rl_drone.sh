#!/usr/bin/env bash
# Laptop ROS launcher; invoke within sim_run.sh so locking/cleanup stay active.
set -e
research_python="${UAV_RESEARCH_PYTHON:-/home/xhj/miniconda3/envs/rl_drone/bin/python}"
# Select the installed environment directly after sim_run resets PATH. This
# avoids running conda's base interpreter with the ROS/Gazebo library overlay.
test -x "$research_python"
export PATH="${research_python%/*}:$PATH"
printf 'roslaunch interpreter: %s\n' "$research_python"
exec "$research_python" /opt/ros/noetic/bin/roslaunch "$@"
