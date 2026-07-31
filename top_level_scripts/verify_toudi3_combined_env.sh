#!/usr/bin/env bash

# Read-only V-SIM-00 verification.  It parses launch files but starts no nodes.
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/toudi3_combined_env.sh"

liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env

roslaunch --files uav_vision phase_d_map_mock.launch >/dev/null
roslaunch --files uav_vision phase_d_mock_patrol_regression.launch >/dev/null
roslaunch --files patrol_control toudi3_full_competition_sim_new_vision.launch >/dev/null

echo "V-SIM-00 PASS: combined packages, Python messages, and launch files are available."

