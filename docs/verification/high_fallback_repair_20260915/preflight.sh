#!/usr/bin/env bash
set -e
task_dir=$(cd "${BASH_SOURCE[0]%/*}" && pwd)
task_root=$(cd "$task_dir/../../.." && pwd)
source /opt/ros/noetic/setup.bash
source "$task_root/vision_ws/devel/setup.bash"
source "$task_root/patrol_uav_ws-patrol_planner/devel/setup.bash" --extend
px4_root="${PX4_ROOT:-/home/xhj/PX4-Autopilot}"
export ROS_PACKAGE_PATH="$task_root/vision_ws/src:$task_root/patrol_uav_ws-patrol_planner/src:/opt/ros/noetic/share:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export SIM_RUN_DIR="$task_root/logs/high_five_preflight"
exec /home/xhj/miniconda3/envs/rl_drone/bin/python "$task_dir/preflight.py"
