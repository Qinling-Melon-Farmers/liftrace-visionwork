#!/usr/bin/env bash
set -euo pipefail
report_dir="${BASH_SOURCE[0]%/*}"
root="$(cd "$report_dir/../../.." && pwd)"
planner="$root/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner"
out="${NAV_RECOVERY_TEST_DIR:-/tmp/navigation_recovery_20261008}"
mkdir -p "$out"
# Production spline math uses ROS only for one diagnostic log. Keep this
# standalone runner free of ROS initialisation and runtime dependencies.
mkdir -p "$out/stubs/ros"
printf '#pragma once\n#define ROS_ERROR_COND(...)\n' > "$out/stubs/ros/ros.h"
g++ -std=c++14 -O1 -Wall -Wextra -I/usr/include/eigen3 -I"$out/stubs" \
  -I"$planner/plan_manage/include" -I"$planner/bspline/include" \
  "$planner/plan_manage/test/navigation_recovery_test.cpp" \
  "$planner/bspline/src/non_uniform_bspline.cpp" \
  -lgtest -lpthread -o "$out/navigation_recovery_test" \
  >"$out/compile.txt" 2>&1
"$out/navigation_recovery_test" --gtest_output="xml:$out/results.xml" | tee "$out/results.txt"
