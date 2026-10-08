#!/usr/bin/env bash
set -euo pipefail
report_dir="${BASH_SOURCE[0]%/*}"
root="$(cd "$report_dir/../../.." && pwd)"
out="${NAV_RECOVERY_TEST_DIR:-/tmp/navigation_recovery_20261008}/map"
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python "$report_dir/prepare_map_fixture.py" "$out"
p="$out/snapshot/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner"
g++ -std=c++17 -O1 -I"$out/stubs" -I/usr/include/eigen3 \
  -I"$p/plan_env/include" -I"$p/path_searching/include" -I"$p/bspline/include" \
  -I"$root/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/include" \
  "$out/production_map.cpp" "$out/map_recovery_cases.cpp" -o "$out/map_recovery_cases" \
  >"$out/compile.txt" 2>&1
"$out/map_recovery_cases" | tee "$out/results.txt"
