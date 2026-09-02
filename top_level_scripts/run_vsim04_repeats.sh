#!/usr/bin/env bash
# Execute isolated V-SIM-04 repeats and aggregate them without weakening Gate semantics.
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export VISION_WS="${PROJECT_ROOT}/vision_ws"
ROS_SETUP="/opt/ros/noetic/setup.bash"
VISION_SETUP="${VISION_WS}/devel/setup.bash"

if [ ! -r "${ROS_SETUP}" ]; then
  echo "ROS setup is not readable: ${ROS_SETUP}" >&2
  exit 2
fi
if [ ! -r "${VISION_SETUP}" ]; then
  echo "vision overlay is not built; missing: ${VISION_SETUP}" >&2
  exit 2
fi

# Catkin setup scripts are not nounset-safe. Restore strict mode immediately.
set +u
source "${ROS_SETUP}"
source "${VISION_SETUP}"
set -u

export PYTHONPATH="${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/src${PYTHONPATH:+:${PYTHONPATH}}"

exec python3 "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/scripts/vsim04_repeat_runner.py" \
  --project-root "${PROJECT_ROOT}" "$@"
