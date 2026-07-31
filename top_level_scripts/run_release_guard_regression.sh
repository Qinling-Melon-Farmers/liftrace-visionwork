#!/usr/bin/env bash

# Deterministic, hardware-free regression for:
# release_evidence -> release_permission -> guarded legacy Servo -> raw mock.
set -e

script_dir="${BASH_SOURCE[0]%/*}"
# shellcheck disable=SC1091
source "${script_dir}/toudi3_combined_env.sh"
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env

exec roslaunch uav_mission release_guard_regression.launch
