#!/usr/bin/env bash
set -euo pipefail

# Bounded visual-only evaluation. No PX4, MAVROS, patrol_control or actuator
# is launched by any of the three supported scene launch files.

script_dir=$(cd -- "${BASH_SOURCE[0]%/*}" && pwd -P)
project_root="${script_dir%/*}"
export PROJECT_ROOT="${PROJECT_ROOT:-${project_root}}"
export VISION_WS="${VISION_WS:-${project_root}/vision_ws}"
if [[ -z "${UAV_WS:-}" ]]; then
  if [[ -f "${project_root}/patrol_uav_ws-patrol_planner/devel/setup.bash" ]]; then
    export UAV_WS="${project_root}/patrol_uav_ws-patrol_planner"
  else
    export UAV_WS="${LIFTRACE_INTEGRATION_WS:-/home/xhj/liftrace/patrol_uav_ws-patrol_planner}"
  fi
fi
scenario="${1:-standard}"
duration_sec="${2:-20}"
gate_profile="${4:-smoke}"
camera_x="${5:-}"
camera_y="${6:-}"
camera_z="${7:-}"
camera_yaw="${8:-0.0}"
evaluation_seed="${9:--1}"
extra_args=()

case "${scenario}" in
  standard)
    launch_file="standard_target_eval.launch"
    camera_x="${camera_x:-1.01558}"
    camera_y="${camera_y:-0.25625}"
    camera_z="${camera_z:-1.5}"
    ;;
  standard_pillbox)
    launch_file="standard_catalog_eval.launch"
    extra_args+=(target_id:=pillbox_1)
    camera_x="${camera_x:--0.602009}"
    camera_y="${camera_y:--1.04125}"
    camera_z="${camera_z:-1.5}"
    ;;
  standard_bridge)
    launch_file="standard_catalog_eval.launch"
    extra_args+=(target_id:=bridge_1)
    camera_x="${camera_x:--1.90310}"
    camera_y="${camera_y:--0.02273}"
    camera_z="${camera_z:-1.5}"
    ;;
  standard_tank)
    launch_file="standard_catalog_eval.launch"
    extra_args+=(target_id:=tank_1)
    camera_x="${camera_x:-0.282735}"
    camera_y="${camera_y:-3.85585}"
    camera_z="${camera_z:-1.5}"
    ;;
  standard_tent)
    launch_file="standard_catalog_eval.launch"
    extra_args+=(target_id:=tent_1)
    camera_x="${camera_x:-1.01558}"
    camera_y="${camera_y:-0.25625}"
    camera_z="${camera_z:-1.5}"
    ;;
  standard_panzer)
    launch_file="standard_catalog_eval.launch"
    extra_args+=(target_id:=panzer_1)
    camera_x="${camera_x:--1.58862}"
    camera_y="${camera_y:-3.02170}"
    camera_z="${camera_z:-1.5}"
    ;;
  red_cross)
    launch_file="red_cross_eval.launch"
    camera_x="${camera_x:--2.5}"
    camera_y="${camera_y:--2.5}"
    camera_z="${camera_z:-2.0}"
    ;;
  background)
    launch_file="background_eval.launch"
    camera_x="${camera_x:-4.2}"
    camera_y="${camera_y:--4.2}"
    camera_z="${camera_z:-2.0}"
    ;;
  landing_h)
    launch_file="landing_h_eval.launch"
    camera_x="${camera_x:--0.493412}"
    camera_y="${camera_y:--1.77269}"
    camera_z="${camera_z:-1.8}"
    ;;
  *)
    echo "usage: $0 SCENARIO [duration_sec] [output_dir] [smoke|formal] [camera_x camera_y camera_z camera_yaw seed]" >&2
    exit 64
    ;;
esac

if [[ ! "${duration_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "duration_sec must be a positive integer" >&2
  exit 64
fi
if [[ ! "${evaluation_seed}" =~ ^-?[0-9]+$ ]]; then
  echo "evaluation_seed must be an integer" >&2
  exit 64
fi
extra_args+=(
  camera_x:="${camera_x}" camera_y:="${camera_y}" camera_z:="${camera_z}"
  camera_yaw:="${camera_yaw}" evaluation_seed:="${evaluation_seed}"
)
if [[ "${gate_profile}" != "smoke" && "${gate_profile}" != "formal" ]]; then
  echo "gate_profile must be smoke or formal" >&2
  exit 64
fi

if [[ -n "${SIM_RUN_DIR:-}" ]]; then
  default_output_dir="${SIM_RUN_DIR}/visual_eval"
else
  default_output_dir="/tmp/uav_vision_eval/${scenario}_$(date +%Y%m%d_%H%M%S)"
fi
output_dir="${3:-${default_output_dir}}"
mkdir -p "${output_dir}/roslog"

# shellcheck disable=SC1091
source "${project_root}/top_level_scripts/toudi3_combined_env.sh"
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env

if [[ "${evaluation_seed}" -ge 0 ]]; then
  default_ros_port=$((11330 + evaluation_seed))
  default_gazebo_port=$((11400 + evaluation_seed))
else
  default_ros_port=11331
  default_gazebo_port=11445
fi
export ROS_MASTER_URI="${EVAL_ROS_MASTER_URI:-http://127.0.0.1:${default_ros_port}}"
export GAZEBO_MASTER_URI="${EVAL_GAZEBO_MASTER_URI:-http://127.0.0.1:${default_gazebo_port}}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME || true
export ROS_LOG_DIR="${output_dir}/roslog"

launch_pid=""
cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

roslaunch uav_vision_eval "${launch_file}" \
  gui:=false \
  start_vision:=true \
  run_truth_assertion:=false \
  record_metrics:=true \
  output_dir:="${output_dir}" \
  gate_profile:="${gate_profile}" \
  "${extra_args[@]}" \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!

deadline=$((SECONDS + duration_sec))
while (( SECONDS < deadline )); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    break
  fi
  sleep 1
done
cleanup
launch_pid=""
trap - EXIT INT TERM

if [[ ! -s "${output_dir}/summary.json" ]]; then
  echo "evaluation did not produce ${output_dir}/summary.json" >&2
  tail -n 120 "${output_dir}/launch.log" >&2 || true
  exit 3
fi
if ! grep -Fq "Spawn status: SpawnModel: Successfully spawned entity" \
    "${output_dir}/launch.log"; then
  echo "evaluation camera was not spawned successfully" >&2
  exit 3
fi
if grep -Eq '\[gazebo-[0-9]+\] process has died' \
    "${output_dir}/launch.log"; then
  echo "evaluation infrastructure process died unexpectedly" >&2
  exit 3
fi

set +e
/usr/bin/python3 "${VISION_WS}/src/uav_vision_eval/scripts/vision_metrics_report.py" \
  --summary "${output_dir}/summary.json" \
  --output "${output_dir}/report.md"
report_status=$?
set -e

echo "evaluation artifacts: ${output_dir}"
exit "${report_status}"
