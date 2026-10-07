#!/usr/bin/env bash
set -eo pipefail
cd "${BASH_SOURCE[0]%/*}/../../.."
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash
mission="$PWD/patrol_uav_ws-patrol_planner/src/uav_mission"
control="$PWD/patrol_uav_ws-patrol_planner/src/patrol_control"
export PYTHONPATH="$mission/test:$control/test:$PYTHONPATH"
/usr/bin/python3 -m unittest test_competition_hardware test_competition_release test_height_hold test_manual_offboard_start test_external_landing_handoff test_recovery_hold test_takeoff_xy_hold
for profile in deployment/competition/field.example.yaml deployment/competition/candidates/rectangle_motion.yaml deployment/competition/candidates/snake_motion.yaml deployment/competition/field_20261007_validated.yaml; do
  /usr/bin/python3 deployment/competition/check_wiring.py --profile "$profile"
done
bash deployment/competition/start.sh preview --check-config --site-config deployment/competition/field.example.yaml