#!/usr/bin/env bash
set -eo pipefail
cd "${BASH_SOURCE[0]%/*}/../../.."
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash
for profile in deployment/competition/field.example.yaml deployment/competition/candidates/rectangle_motion.yaml deployment/competition/candidates/snake_motion.yaml deployment/competition/candidates/snake3_motion.yaml; do
  /usr/bin/python3 deployment/competition/check_wiring.py --profile "$profile"
  bash deployment/competition/start.sh preview --check-config --site-config "$profile"
done
