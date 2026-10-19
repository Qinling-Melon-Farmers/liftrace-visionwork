#!/usr/bin/env bash
set -euo pipefail
TASK_DIR=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd)
B=${TASK_DIR%/deployment/actuator_startup_20261007}
set +u
source /opt/ros/noetic/setup.bash
source "$B/patrol_uav_ws-patrol_planner/devel/setup.bash"
set -u
# Direct CMake of just actuator_pwm; only consume existing generated Servo headers.
cmake -S "$TASK_DIR/source/actuator_pwm" -B "$TASK_DIR/build" \
  -DCATKIN_DEVEL_PREFIX="$TASK_DIR/devel" \
  -DCMAKE_INSTALL_PREFIX="$TASK_DIR/install" \
  -DCATKIN_ENABLE_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build "$TASK_DIR/build" --target pwm_node1 actuator_startup_tests -j2
ctest --test-dir "$TASK_DIR/build" --output-on-failure
