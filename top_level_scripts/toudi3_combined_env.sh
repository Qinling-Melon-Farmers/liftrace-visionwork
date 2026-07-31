#!/usr/bin/env bash

# Source-only environment helper for the local toudi3 SITL stack.
# The patrol workspace must be built as a catkin overlay on top of vision_ws.
# This helper never starts ROS, PX4, Gazebo, a vehicle, or an actuator.

liftrace_setup_toudi3_combined_env() {
  local project_root uav_ws vision_ws px4_root sitl_gazebo
  local astra_sim_lib px4_plugin_lib vision_python had_nounset=0

  project_root="${PROJECT_ROOT:-/home/xhj/liftrace}"
  uav_ws="${UAV_WS:-${project_root}/patrol_uav_ws-patrol_planner}"
  vision_ws="${VISION_WS:-${project_root}/vision_ws}"
  px4_root="${PX4_ROOT:-/home/xhj/PX4-Autopilot}"
  sitl_gazebo="${SITL_GAZEBO:-${px4_root}/Tools/simulation/gazebo-classic/sitl_gazebo-classic}"
  astra_sim_lib="${ASTRA_SIM_LIB:-/home/xhj/AstraDroneOpen/simulation/sim_workspace/devel/lib}"
  px4_plugin_lib="${PX4_PLUGIN_LIB:-${px4_root}/build/px4_sitl_default/build_gazebo-classic}"
  vision_python="${VISION_PYTHON:-/home/xhj/miniconda3/envs/rl_drone/bin/python}"

  if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "[toudi3-env] missing /opt/ros/noetic/setup.bash" >&2
    return 1
  fi
  if [[ ! -f "${vision_ws}/devel/setup.bash" ]]; then
    echo "[toudi3-env] vision workspace is not built: ${vision_ws}" >&2
    return 1
  fi
  if [[ ! -f "${uav_ws}/devel/setup.bash" ]]; then
    echo "[toudi3-env] patrol workspace is not built: ${uav_ws}" >&2
    return 1
  fi
  if [[ ! -x "${vision_python}" ]]; then
    echo "[toudi3-env] vision Python is unavailable: ${vision_python}" >&2
    return 1
  fi

  case "$-" in
    *u*)
      had_nounset=1
      set +u
      ;;
  esac

  unset PYTHONHOME
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  # A correctly built patrol overlay records vision_ws as an underlay.  Source
  # only the top-level workspace so catkin composes all generated-message,
  # Python, library, pkg-config, and package paths consistently.
  # shellcheck disable=SC1090
  source "${uav_ws}/devel/setup.bash"

  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi

  export PROJECT_ROOT="${project_root}"
  export UAV_WS="${uav_ws}"
  export VISION_WS="${vision_ws}"
  export PX4_ROOT="${px4_root}"
  export SITL_GAZEBO="${sitl_gazebo}"
  export ASTRA_SIM_LIB="${astra_sim_lib}"
  export PX4_PLUGIN_LIB="${px4_plugin_lib}"
  export VISION_PYTHON="${vision_python}"
  # 仓库内仿真显式选择开发模型；uav_vision 运行包本身不再保存开发机绝对路径。
  export UAV_VISION_MODEL_PATH="${UAV_VISION_MODEL_PATH:-${vision_ws}/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt}"
  export TOUDI3_WORLD="${TOUDI3_WORLD:-${uav_ws}/toudi3.world}"

  export PATH="/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  export ROS_PACKAGE_PATH="${px4_root}:${sitl_gazebo}:${ROS_PACKAGE_PATH:-}"
  export GAZEBO_MODEL_PATH="${sitl_gazebo}/models:${GAZEBO_MODEL_PATH:-}"
  export GAZEBO_RESOURCE_PATH="${sitl_gazebo}/worlds:${GAZEBO_RESOURCE_PATH:-}"
  export GAZEBO_PLUGIN_PATH="${astra_sim_lib}:${px4_plugin_lib}:${GAZEBO_PLUGIN_PATH:-}"
  export LD_LIBRARY_PATH="${astra_sim_lib}:${px4_plugin_lib}:${LD_LIBRARY_PATH:-}"
}

liftrace_assert_toudi3_combined_env() {
  local package

  for package in uav_vision patrol_control px4 mavlink_sitl_gazebo; do
    if ! rospack find "${package}" >/dev/null 2>&1; then
      echo "[toudi3-env] ROS package unavailable: ${package}" >&2
      if [[ "${package}" == "uav_vision" ]]; then
        echo "[toudi3-env] rebuild patrol_uav_ws-patrol_planner after sourcing vision_ws/devel/setup.bash" >&2
      fi
      return 1
    fi
  done

  if ! /usr/bin/python3 -c 'from uav_vision.msg import TargetDetectionArray' >/dev/null 2>&1; then
    echo "[toudi3-env] system ROS Python cannot import uav_vision.msg" >&2
    return 1
  fi
  if ! "${VISION_PYTHON}" -c 'from uav_vision.msg import TargetDetectionArray' >/dev/null 2>&1; then
    echo "[toudi3-env] rl_drone Python cannot import uav_vision.msg" >&2
    return 1
  fi
}
