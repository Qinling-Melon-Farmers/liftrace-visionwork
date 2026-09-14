#!/usr/bin/env bash
# Parameter expansion only: starts no ROS nodes or simulation.
set -e
task_dir=$(cd "${BASH_SOURCE[0]%/*}" && pwd)
task_root=$(cd "$task_dir/../../.." && pwd)
source /opt/ros/noetic/setup.bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export ROS_PACKAGE_PATH="$task_root/vision_ws/src:$task_root/patrol_uav_ws-patrol_planner/src:/opt/ros/noetic/share:/home/xhj/PX4-Autopilot:/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export SIM_RUN_DIR="$task_root/logs/fast_static"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python "$task_dir/static_check.py"
