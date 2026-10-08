#!/usr/bin/env bash
# Source from this deployment; never borrow another project's hardware_ws.
COMPETITION_ROOT="$(cd "${BASH_SOURCE[0]%/*}/../.." && pwd)"
export COMPETITION_ROOT
# 本机全部板端节点使用回环通信，切换Wi-Fi地址不影响节点互联。
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
# 默认模型必须来自当前完整工程；特殊模型仍可由入口 --model 显式指定。
export UAV_VISION_RKNN_MODEL_PATH="$COMPETITION_ROOT/runtime_models/flight_5cls_20260928_fp16.rknn"
# Bash source 会继承调用者的 $@；放入无参数函数，避免 --help/flight
# 被 catkin 的 setup_util 误当成自己的参数，甚至将 usage 当 shell 执行。
competition_load_environment() {
  source /opt/ros/noetic/setup.bash
  source "$COMPETITION_ROOT/vision_ws/devel/setup.bash"
  source "$COMPETITION_ROOT/patrol_uav_ws-patrol_planner/devel/setup.bash"
  for package in uav_mission uav_vision uav_high_view camera_sdk actuator_pwm; do
    resolved_package="$(rospack find "$package")" || return 1
    case "$resolved_package" in
      "$COMPETITION_ROOT"/*) ;;
      *) echo "Wrong overlay: $package resolved to $resolved_package" >&2; return 1 ;;
    esac
  done
}
competition_load_environment
competition_environment_status=$?
unset -f competition_load_environment
return "$competition_environment_status"
