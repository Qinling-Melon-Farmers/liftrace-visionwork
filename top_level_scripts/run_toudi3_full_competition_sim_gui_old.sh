#!/usr/bin/env bash

# GUI SITL entry point using the original visual chain.
# Safe simulation only: no actuator_pwm and no hardware MAVROS endpoint.
set -e
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
export TOUDI3_GUI=true
export TOUDI3_RVIZ=true
export TOUDI3_MAPPING_RVIZ=false
export TOUDI3_START_VISUAL=true
exec "${SCRIPT_DIR}/run_toudi3_full_competition_sim.sh"
