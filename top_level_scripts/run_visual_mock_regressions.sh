#!/usr/bin/env bash
# Deterministic visual-only L0 regressions. No PX4, MAVROS, control or actuator.
set -eo pipefail

PROJECT_ROOT=/home/xhj/liftrace
source /opt/ros/noetic/setup.bash
source "$PROJECT_ROOT/vision_ws/devel/setup.bash"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME || true
set -u

run_case() {
  local package=$1
  local launch_file=$2
  local pass_marker=$3
  local log_file
  log_file=$(mktemp "/tmp/${launch_file%.launch}.XXXXXX.log")

  echo "[L0] ${package}/${launch_file}"
  roslaunch "$package" "$launch_file" 2>&1 | tee "$log_file"
  # rospy INFO emitted immediately before signal_shutdown is reliably stored
  # in the per-node ROS log, but roslaunch does not always relay it to stdout.
  if ! grep -FRq "$pass_marker" /home/xhj/.ros/log/latest; then
    echo "[L0] FAIL: missing marker '${pass_marker}' (log: ${log_file})" >&2
    return 1
  fi
  echo "[L0] PASS: ${launch_file}"
}

run_case uav_vision circle_geometry_mock.launch \
  "[CircleGeometryAssertion] success"
run_case uav_vision_eval target_refiner_association_mock.launch \
  "V-ALG global ring association PASS"
run_case uav_vision_eval target_memory_freshness_mock.launch \
  "V-ALG target-memory freshness PASS"
run_case uav_vision target_memory_physical_mock.launch \
  "V-CL physical target memory PASS"
run_case uav_vision map_rejection_mock.launch \
  "V-CL invalid TF rejection PASS"
run_case uav_vision phase_d_map_mock.launch \
  "[PhaseDMapAssertion] success"

echo "V-L0 PASS: circle geometry, global association, memory freshness, physical map memory, invalid TF rejection, map/release evidence"
