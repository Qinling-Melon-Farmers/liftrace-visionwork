#!/usr/bin/env bash
set -euo pipefail
report_dir="${BASH_SOURCE[0]%/*}"
root="$(cd "$report_dir/../../.." && pwd)"
case_name="${1:?Usage: run_production_case.sh column|buffer|height}"
case "$case_name" in column|buffer|height) ;; *) exit 64;; esac
[[ "${SIM_RUN_AUTHORIZED:-0}" == 1 ]] || { echo 'Main-agent current simulation authorization required' >&2;exit 64; }
export UAV_WS="$root/patrol_uav_ws-patrol_planner"
export VISION_WS="$root/vision_ws"
export SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1
exec bash "$root/top_level_scripts/sim_run.sh" "navigation_recovery_${case_name}" \
  roslaunch "$report_dir/production_recovery.launch" "case:=$case_name"
