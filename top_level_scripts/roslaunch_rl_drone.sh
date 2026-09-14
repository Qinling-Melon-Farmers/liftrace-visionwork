#!/usr/bin/env bash
# Laptop ROS launcher; invoke within sim_run.sh so locking/cleanup stay active.
set -e
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
printf 'roslaunch interpreter: %s\n' "$CONDA_PREFIX/bin/python"
exec "$CONDA_PREFIX/bin/python" /opt/ros/noetic/bin/roslaunch "$@"
