#!/bin/bash
# liftrace PX4+Gazebo full simulation launcher.
# Starts: roscore → PX4 SITL + Gazebo + MAVROS → FAST_LIO mapping

set -e

SESSION_NAME="liftrace_px4_sim"
WORKSPACE_DIR="${BASH_SOURCE[0]%/*}"
WORKSPACE_DIR="$(cd "${WORKSPACE_DIR}/.." && pwd)"
SIM_LAUNCH_FILE="${WORKSPACE_DIR}/mavros_posix_sitl_liftrace.launch"
SIM_ENV_SCRIPT="${WORKSPACE_DIR}/top_level_scripts/sim_env.sh"

source "${SIM_ENV_SCRIPT}"
liftrace_setup_sim_env
liftrace_prepare_sim_assets
liftrace_assert_sim_env

tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
tmux new-session -d -s "${SESSION_NAME}"

# Pane 0: roscore
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "cd ${WORKSPACE_DIR}" C-m
tmux send-keys "source ${SIM_ENV_SCRIPT}" C-m
tmux send-keys "liftrace_setup_sim_env" C-m
tmux send-keys "roscore" C-m

# Pane 1: PX4 SITL + Gazebo + MAVROS
tmux select-pane -t 1
tmux send-keys "sleep 3s" C-m
tmux send-keys "cd ${WORKSPACE_DIR}" C-m
tmux send-keys "source ${SIM_ENV_SCRIPT}" C-m
tmux send-keys "liftrace_setup_sim_env" C-m
tmux send-keys "liftrace_prepare_sim_assets" C-m
tmux send-keys "liftrace_assert_sim_env" C-m
tmux send-keys "roslaunch ${SIM_LAUNCH_FILE}" C-m

# Pane 2: FAST_LIO mapping
tmux select-pane -t 2
tmux send-keys "sleep 8s" C-m
tmux send-keys "cd ${WORKSPACE_DIR}" C-m
tmux send-keys "source ${SIM_ENV_SCRIPT}" C-m
tmux send-keys "liftrace_setup_sim_env" C-m
tmux send-keys "liftrace_assert_sim_env" C-m
tmux send-keys "roslaunch fast_lio mapping_mid360_sim.launch" C-m

# Pane 3: attached to session
tmux select-pane -t 3
if [ -z "${NO_ATTACH:-}" ]; then
    tmux -2 attach-session -t "${SESSION_NAME}"
fi
