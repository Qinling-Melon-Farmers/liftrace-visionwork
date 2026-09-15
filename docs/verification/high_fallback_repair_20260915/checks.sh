#!/usr/bin/env bash
set -eo pipefail
cd "${BASH_SOURCE[0]%/*}/../../.."
source /opt/ros/noetic/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PY=/home/xhj/miniconda3/envs/rl_drone/bin/python
for test in test_high_view_full.py test_high_view_probe.py test_mission_runtime.py test_mission_core.py test_coverage_route.py test_navigation_manager_contract.py; do
  "$PY" -m unittest discover -s patrol_uav_ws-patrol_planner/src/uav_mission/test -p "$test"
done
"$PY" -m unittest discover -s vision_ws/src/uav_high_view/test
