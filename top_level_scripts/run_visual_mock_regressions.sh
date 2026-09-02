#!/usr/bin/env bash
# Deterministic visual-only L0 regressions. No PX4, MAVROS, control or actuator.
set -eo pipefail

SCRIPT_DIR=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd -P)
PROJECT_ROOT=${SCRIPT_DIR%/*}
ROS_LOG_ROOT=$(mktemp -d /tmp/uav_vision_l0_roslog.XXXXXX)

cleanup() {
  case "$ROS_LOG_ROOT" in
    /tmp/uav_vision_l0_roslog.*) rm -rf -- "$ROS_LOG_ROOT" ;;
  esac
}
trap cleanup EXIT

source /opt/ros/noetic/setup.bash
source "$PROJECT_ROOT/vision_ws/devel/setup.bash"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME || true
set -u

run_case() {
  local package=$1
  local launch_file=$2
  local pass_marker=$3
  shift 3
  local log_file
  local case_ros_log_dir
  log_file=$(mktemp "/tmp/${launch_file%.launch}.XXXXXX.log")
  case_ros_log_dir=$(mktemp -d "$ROS_LOG_ROOT/case.XXXXXX")

  echo "[L0] ${package}/${launch_file}"
  if ! ROS_LOG_DIR="$case_ros_log_dir" \
      roslaunch "$package" "$launch_file" "$@" 2>&1 | tee "$log_file"; then
    echo "[L0] FAIL: roslaunch failed (log: ${log_file})" >&2
    return 1
  fi
  # rospy INFO emitted immediately before signal_shutdown is reliably stored
  # in the per-node ROS log, but roslaunch does not always relay it to stdout.
  if ! rg -F -q -- "$pass_marker" "$case_ros_log_dir"; then
    echo "[L0] FAIL: missing marker '${pass_marker}' (log: ${log_file})" >&2
    return 1
  fi
  echo "[L0] PASS: ${launch_file}"
}

run_rostest() {
  local package=$1
  local test_file=$2
  echo "[L0] rostest ${package}/${test_file}"
  rostest "$package" "$test_file"
}

run_python_assertion() {
  local package=$1
  local executable=$2
  local installed_executable="$PROJECT_ROOT/vision_ws/devel/lib/$package/$executable"
  echo "[L0] ${package}/${executable}"
  if [[ ! -x "$installed_executable" ]]; then
    echo "[L0] FAIL: missing devel executable ${installed_executable}" >&2
    return 1
  fi
  "$installed_executable"
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
run_case uav_vision drop_aligner_freshness_mock.launch \
  "[DropAlignerFreshness] PASS fresh target selected"

# Exercise every operational align-mode without patrol_control/navigation.
run_case uav_vision phase_d_mode_mock.launch \
  "[PhaseDAssertion] success" \
  class_name:=panzer align_mode:=disabled \
  expect_target_class:=panzer \
  expect_align_mode:=disabled expect_yolo_detect:=panzer \
  expect_drop_ready:=false
run_case uav_vision phase_d_mode_mock.launch \
  "[PhaseDAssertion] success" \
  class_name:=circle align_mode:=drop_circle \
  expect_target_class:=circle expect_align_mode:=drop_circle \
  expect_yolo_detect:=Nothing expect_drop_ready:=true
run_case uav_vision phase_d_mode_mock.launch \
  "[PhaseDAssertion] success" \
  class_name:=red_cross align_mode:=drop_cross \
  expect_target_class:=red_cross \
  expect_align_mode:=drop_cross expect_yolo_detect:=Nothing \
  expect_drop_ready:=true expect_cross_status:=true
run_case uav_vision phase_d_mode_mock.launch \
  "[PhaseDAssertion] success" \
  class_name:=landing_pad align_mode:=landing \
  expect_target_class:=landing_pad expect_align_mode:=landing \
  expect_yolo_detect:=Nothing expect_drop_ready:=true

run_rostest uav_vision cross_geometry_regression.test
run_rostest uav_vision landing_mode_gate.test

run_python_assertion uav_vision target_selection_policy_assertion.py
run_python_assertion uav_vision detection_fusion_concurrency_assertion.py
run_python_assertion uav_vision_eval vsim04_metrics_assertion.py
run_python_assertion uav_vision_eval vsim04_failure_capture_assertion.py

echo "V-L0 PASS: geometry, fusion, four align modes, freshness, profile policy, map/release evidence, and V-SIM schema"
