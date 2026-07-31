#!/usr/bin/env bash
# Wrapper to launch toudi3 full competition simulation with correct environment.
# Run: bash /home/xhj/liftrace/top_level_scripts/launch_toudi3_full_sim.sh
set -e

export PX4_ROOT=/home/xhj/PX4-Autopilot
export PX4_GAZEBO=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export UAV_WS=/home/xhj/liftrace/patrol_uav_ws-patrol_planner
export VISION_WS=/home/xhj/liftrace/vision_ws
export ASTRA_LIB=/home/xhj/AstraDroneOpen/simulation/sim_workspace/devel/lib
export PX4_BUILD=$PX4_ROOT/build/px4_sitl_default/build_gazebo-classic

LOG=/tmp/toudi3_full_sim_$(date +%Y%m%d_%H%M%S).log

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

# ---- guard: verify package resolution ----
echo "===== TOUDI3 FULL COMPETITION SIMULATION =====" | tee "$LOG"
echo "Start: $(date)" | tee -a "$LOG"
echo "Log: $LOG" | tee -a "$LOG"
echo "" | tee -a "$LOG"

for pkg in mavlink_sitl_gazebo px4 patrol_control; do
  rp=$(rospack find "$pkg" 2>&1) || {
    echo "FATAL: package '$pkg' not found: $rp" | tee -a "$LOG"
    exit 1
  }
  echo "  [$pkg] -> $rp" | tee -a "$LOG"
done
echo "" | tee -a "$LOG"

# ---- launch main sim ----
roslaunch patrol_control patrol_full_competition_sim.launch \
  world:="$UAV_WS/toudi3.world" \
  gui:=false rviz:=false mapping_rviz:=false \
  start_visual:=true start_mapping:=true start_arming:=true \
  waypoint_mode:=true simulation_auto_land:=true px4_max_distance:=0.2 \
  >> "$LOG" 2>&1 &
MAIN_PID=$!
echo "Main sim PID: $MAIN_PID" | tee -a "$LOG"

# ---- wait for roscore, then launch sim helpers (replaces broken YOLO + missing Servo) ----
echo "Waiting for roscore..." | tee -a "$LOG"
for i in $(seq 1 60); do
  if rostopic list 2>/dev/null | grep -q "/rosout"; then
    echo "ROS ready after ${i}s" | tee -a "$LOG"
    break
  fi
  sleep 2
done

echo "Launching sim_helpers (mock YOLO + mock Servo)..." | tee -a "$LOG"
python3 /home/xhj/liftrace/top_level_scripts/sim_helpers.py \
  >> "$LOG" 2>&1 &
HELPER_PID=$!
echo "Helpers PID: $HELPER_PID" | tee -a "$LOG"

# wait for main sim to finish
wait $MAIN_PID
