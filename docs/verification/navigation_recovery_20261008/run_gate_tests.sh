#!/usr/bin/env bash
set -euo pipefail
report_dir="${BASH_SOURCE[0]%/*}"
root="$(cd "$report_dir/../../.." && pwd)"
out="${NAV_RECOVERY_TEST_DIR:-/tmp/navigation_recovery_20261008}"
mkdir -p "$out"
set +u
source /opt/ros/noetic/setup.bash
set -u
g++ -std=c++14 -O1 -I/usr/include/eigen3 -I/opt/ros/noetic/include \
  -I"$root/patrol_uav_ws-patrol_planner/devel/include" \
  -I"$root/patrol_uav_ws-patrol_planner/src/navigation_recovery_msgs/include" \
  "$root/patrol_uav_ws-patrol_planner/src/navigation_recovery_msgs/test/recovery_gate_test.cpp" \
  -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lgtest -lpthread \
  -o "$out/recovery_gate_test"
"$out/recovery_gate_test" --gtest_output="xml:$out/gate_results.xml" | tee "$out/gate_results.txt"
