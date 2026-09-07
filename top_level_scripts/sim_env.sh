#!/bin/bash
# liftrace PX4+Gazebo simulation environment setup.
# Adapted from speedrace's sim_env.sh.

liftrace_strip_conda_from_path() {
    local path_entry cleaned_path=""
    local old_ifs="${IFS}"
    IFS=':'
    for path_entry in ${PATH}; do
        case "${path_entry}" in
            "${HOME}"/miniconda3/bin|"${HOME}"/miniconda3/condabin|"${HOME}"/anaconda3/bin|"${HOME}"/anaconda3/condabin)
                ;;
            *)
                if [ -n "${path_entry}" ]; then
                    if [ -n "${cleaned_path}" ]; then
                        cleaned_path="${cleaned_path}:${path_entry}"
                    else
                        cleaned_path="${path_entry}"
                    fi
                fi
                ;;
        esac
    done
    IFS="${old_ifs}"
    export PATH="${cleaned_path}"
}

liftrace_setup_sim_env() {
    local script_dir workspace_dir had_nounset=0

    script_dir="${BASH_SOURCE[0]%/*}"
    script_dir="$(cd "${script_dir}" && pwd)"
    workspace_dir="$(cd "${script_dir}/.." && pwd)"

    export LIFTRACE_WORKSPACE_DIR="${LIFTRACE_WORKSPACE_DIR:-${workspace_dir}}"
    export PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-${HOME}/PX4-Autopilot}"
    export PX4_GAZEBO_CLASSIC_DIR="${PX4_GAZEBO_CLASSIC_DIR:-${PX4_AUTOPILOT_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic}"
    export LIVOX_PLUGIN_DIR="${LIVOX_PLUGIN_DIR:-${HOME}/AstraDroneOpen/simulation/sim_workspace/devel/lib}"

    # Use speedrace's world files for now (shared simulation assets)
    export LIFTRACE_WORLD_DIR="${LIFTRACE_WORLD_DIR:-${HOME}/speedrace/robocup_map}"
    export LIFTRACE_PX4_WORLD_DIR="${LIFTRACE_PX4_WORLD_DIR:-${PX4_GAZEBO_CLASSIC_DIR}/worlds/robocup_map}"

    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

    if [ ! -f /opt/ros/noetic/setup.bash ]; then
        echo "[liftrace] missing /opt/ros/noetic/setup.bash" >&2
        return 1
    fi

    if [ ! -d "${PX4_AUTOPILOT_DIR}" ]; then
        echo "[liftrace] WARNING: PX4-Autopilot not found at ${PX4_AUTOPILOT_DIR}" >&2
    fi

    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE
    unset CONDA_EXE _CONDA_EXE _CE_M _CE_CONDA CONDA_SHLVL
    liftrace_strip_conda_from_path
    export PYTHONNOUSERSITE=1

    case "$-" in
        *u*)
            had_nounset=1
            set +u
            ;;
    esac

    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash

    if [ -f "${HOME}/AstraDroneOpen/simulation/sim_workspace/devel/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "${HOME}/AstraDroneOpen/simulation/sim_workspace/devel/setup.bash"
    fi

    if [ -f "${LIFTRACE_WORKSPACE_DIR}/patrol_uav_ws-patrol_planner/devel/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "${LIFTRACE_WORKSPACE_DIR}/patrol_uav_ws-patrol_planner/devel/setup.bash"
    fi

    if [ "${had_nounset}" -eq 1 ]; then
        set -u
    fi

    export ROS_PACKAGE_PATH="${PX4_AUTOPILOT_DIR}:${PX4_GAZEBO_CLASSIC_DIR}:${LIFTRACE_WORKSPACE_DIR}/patrol_uav_ws-patrol_planner/src${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
    export GAZEBO_MODEL_PATH="${PX4_GAZEBO_CLASSIC_DIR}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
    export GAZEBO_RESOURCE_PATH="${PX4_GAZEBO_CLASSIC_DIR}/worlds:${GAZEBO_RESOURCE_PATH:+:${GAZEBO_RESOURCE_PATH}}"

    if [ -d "${PX4_AUTOPILOT_DIR}/build/px4_sitl_default/build_gazebo-classic" ]; then
        export GAZEBO_PLUGIN_PATH="${PX4_AUTOPILOT_DIR}/build/px4_sitl_default/build_gazebo-classic:${LIVOX_PLUGIN_DIR}${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
        export LD_LIBRARY_PATH="${PX4_AUTOPILOT_DIR}/build/px4_sitl_default/build_gazebo-classic:${LIVOX_PLUGIN_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi

    return 0
}

liftrace_prepare_sim_assets() {
    if [ ! -d "${LIFTRACE_WORLD_DIR}" ]; then
        echo "[liftrace] missing world directory: ${LIFTRACE_WORLD_DIR}" >&2
        return 1
    fi

    rm -rf "${LIFTRACE_PX4_WORLD_DIR}"
    mkdir -p "${LIFTRACE_PX4_WORLD_DIR}"
    cp -a "${LIFTRACE_WORLD_DIR}/." "${LIFTRACE_PX4_WORLD_DIR}/"

    find "${LIFTRACE_PX4_WORLD_DIR}" -maxdepth 1 -name "*.world" -type f | while read -r world_file; do
        sed -i \
            "s|/home/smartdrone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/robocup_map|${LIFTRACE_PX4_WORLD_DIR}|g" \
            "${world_file}"
    done

    return 0
}

liftrace_assert_sim_env() {
    local package missing=0

    for package in px4 mavlink_sitl_gazebo fast_lio patrol_control; do
        if ! rospack find "${package}" >/dev/null 2>&1; then
            echo "[liftrace] missing ROS package: ${package}" >&2
            missing=1
        fi
    done

    return "${missing}"
}
