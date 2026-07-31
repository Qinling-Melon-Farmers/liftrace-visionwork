#!/usr/bin/env bash

# Headless SITL entry point using the original visual/control chain.
# Run this inside WSL.  It intentionally keeps the terminal attached so the
# complete roslaunch lifetime and final shutdown status remain visible.
set -e
export TOUDI3_GUI=false
export TOUDI3_RVIZ=false
export TOUDI3_MAPPING_RVIZ=false
export TOUDI3_START_VISUAL=true
export TOUDI3_START_MAPPING=true
export TOUDI3_START_ARMING=true
export TOUDI3_WAYPOINT_MODE=true
export TOUDI3_SIMULATION_AUTO_LAND=true
export TOUDI3_PX4_MAX_DISTANCE=0.2

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
exec "${SCRIPT_DIR}/run_toudi3_full_competition_sim.sh"
