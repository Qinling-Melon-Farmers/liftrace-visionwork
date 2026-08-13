#!/usr/bin/env bash
# 旧文件名兼容入口；当前默认加载 toudi4_copy.world 与新下视相机机架。
# Run: bash /home/xhj/liftrace/top_level_scripts/launch_toudi3_full_sim.sh
set -e

export PX4_ROOT=/home/xhj/PX4-Autopilot
export PX4_GAZEBO=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export UAV_WS=/home/xhj/liftrace/patrol_uav_ws-patrol_planner
export VISION_WS=/home/xhj/liftrace/vision_ws
export ASTRA_LIB=/home/xhj/AstraDroneOpen/simulation/sim_workspace/devel/lib
export PX4_BUILD=$PX4_ROOT/build/px4_sitl_default/build_gazebo-classic

# ---- source workspaces ----
source /opt/ros/noetic/setup.bash
if [ -f "$VISION_WS/devel/setup.bash" ]; then
  source "$VISION_WS/devel/setup.bash"
fi
source "$UAV_WS/devel/setup.bash"

# ---- PX4 + Gazebo environment (must come AFTER workspace source) ----
export ROS_PACKAGE_PATH="/opt/ros/noetic/share:$PX4_ROOT:$PX4_GAZEBO:$UAV_WS/src:$VISION_WS/src:$ROS_PACKAGE_PATH"
export GAZEBO_MODEL_PATH="$PX4_GAZEBO/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="$ASTRA_LIB:$PX4_BUILD:$GAZEBO_PLUGIN_PATH"
export LD_LIBRARY_PATH="$ASTRA_LIB:$PX4_BUILD:$LD_LIBRARY_PATH"

for pkg in mavlink_sitl_gazebo px4 patrol_control; do
  rp=$(rospack find "$pkg" 2>&1) || {
    echo "FATAL: package '$pkg' not found: $rp"
    exit 1
  }
done

# ---- 统一 run 目录入口 ----
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
SIM_HELPERS=1 exec "$SCRIPT_DIR/sim_run.sh" toudi4_legacy_full \
  roslaunch patrol_control patrol_full_competition_sim.launch \
  world:="$PROJECT_ROOT/toudi4_copy.world" \
  gui:=false rviz:=false mapping_rviz:=false \
  start_visual:=true start_mapping:=true start_arming:=true \
  waypoint_mode:=true simulation_auto_land:=true px4_max_distance:=0.2
