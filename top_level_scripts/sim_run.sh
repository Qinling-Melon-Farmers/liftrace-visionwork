#!/usr/bin/env bash
# sim_run.sh — 统一仿真运行入口（阶段 0 日志基建）
#
# 用法：
#   SIM_RUN_AUTHORIZED=1 sim_run.sh <场景名> <roslaunch 命令...>
#   SIM_RUN_AUTHORIZED=1 sim_run.sh corridor_r1 roslaunch patrol_control patrol_full_competition_sim.launch world:=...
#
# 行为：
#   1. 生成 run 目录 logs/<场景名>_<YYYYMMDD>_<HHMMSS>/
#   2. 写 manifest.yaml 头部（时间、git HEAD、启动命令）
#   3. 可选启动 ffmpeg 录屏（后台，PID 记录）
#   4. 前台运行 roslaunch，输出到 run.log
#   5. 结束后调用 sim_finish.sh 优雅收尾（SIGINT 停 ffmpeg、归档、生成时间线）
set -u

# Sanitize tool lookup before the authorization/preflight checks themselves.
export PATH="/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME PYTHONPATH

# Gazebo/PX4 is a machine-wide resource.  An explicit one-command opt-in keeps
# resumed agent turns and read-only inspections from starting it accidentally.
if [ "${SIM_RUN_AUTHORIZED:-0}" != "1" ]; then
  echo "Refusing to start simulation: set SIM_RUN_AUTHORIZED=1 for an explicitly authorized run." >&2
  exit 64
fi

if [ "$#" -lt 2 ]; then
  echo "Usage: SIM_RUN_AUTHORIZED=1 sim_run.sh <scene> <roslaunch command...>" >&2
  exit 64
fi

SIM_RUN_LOCK_FILE="${SIM_RUN_LOCK_FILE:-/tmp/liftrace_sim_run.lock}"
exec 9>"${SIM_RUN_LOCK_FILE}"
if ! flock -n 9; then
  echo "Refusing to start simulation: another sim_run.sh owns ${SIM_RUN_LOCK_FILE}." >&2
  exit 73
fi

SIM_PROCESS_NAMES=(roscore rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rviz)
SIM_EXISTING_PROCESSES=""
for SIM_PROCESS_NAME in "${SIM_PROCESS_NAMES[@]}"; do
  SIM_PROCESS_PIDS="$(pgrep -x "${SIM_PROCESS_NAME}" 2>/dev/null | paste -sd, -)"
  if [ -n "${SIM_PROCESS_PIDS}" ]; then
    SIM_EXISTING_PROCESSES="${SIM_EXISTING_PROCESSES} ${SIM_PROCESS_NAME}=${SIM_PROCESS_PIDS}"
  fi
done
if [ -n "${SIM_EXISTING_PROCESSES}" ]; then
  echo "Refusing to start simulation: local SITL processes already exist:${SIM_EXISTING_PROCESSES}" >&2
  echo "Stop the owned run with top_level_scripts/stop_toudi3_sim.sh, then verify the process list." >&2
  exit 73
fi

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOGS_DIR="${PROJECT_ROOT}/logs"

# ---- source ROS 环境（可被调用者预先覆盖；setup 脚本与 nounset 不兼容，临时关闭） ----
set +u
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${PROJECT_ROOT}/vision_ws/devel/setup.bash" ]; then
  source "${PROJECT_ROOT}/vision_ws/devel/setup.bash"
fi
if [ -f "${PROJECT_ROOT}/patrol_uav_ws-patrol_planner/devel/setup.bash" ]; then
  source "${PROJECT_ROOT}/patrol_uav_ws-patrol_planner/devel/setup.bash"
fi
set -u

# ---- PX4 + Gazebo 环境（与 launch_toudi3_full_sim.sh 一致，可在调用前覆盖） ----
export PX4_ROOT="${PX4_ROOT:-/home/xhj/PX4-Autopilot}"
export PX4_GAZEBO="${PX4_GAZEBO:-${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic}"
export ASTRA_LIB="${ASTRA_LIB:-/home/xhj/AstraDroneOpen/simulation/sim_workspace/devel/lib}"
export PX4_BUILD="${PX4_BUILD:-${PX4_ROOT}/build/px4_sitl_default/build_gazebo-classic}"
export ROS_PACKAGE_PATH="/opt/ros/noetic/share:${PX4_ROOT}:${PX4_GAZEBO}:${PROJECT_ROOT}/patrol_uav_ws-patrol_planner/src:${PROJECT_ROOT}/vision_ws/src${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
export GAZEBO_MODEL_PATH="${PX4_GAZEBO}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
export GAZEBO_PLUGIN_PATH="${ASTRA_LIB}:${PX4_BUILD}${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export LD_LIBRARY_PATH="${ASTRA_LIB}:${PX4_BUILD}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

SCENE="${1:-sim}"
shift

TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${LOGS_DIR}/${SCENE}_${TS}"
mkdir -p "${RUN_DIR}"
# Allow launch files to place machine-readable reports beside run.log.
export SIM_RUN_DIR="${RUN_DIR}"

MANIFEST="${RUN_DIR}/manifest.yaml"
RUN_LOG="${RUN_DIR}/run.log"
RECORD_MP4="${RUN_DIR}/screenrecord.mp4"
HELPER_PID=""
FFMPEG_PID=""
MAIN_PID=""
SIM_CLEANUP_DONE=0

final_sim_cleanup() {
  local original_status=$?
  local cleanup_status=0
  local residual_processes=""

  trap - EXIT HUP INT TERM
  if [ "${SIM_CLEANUP_DONE}" = "1" ]; then
    exit "${original_status}"
  fi
  SIM_CLEANUP_DONE=1

  if [ -n "${MAIN_PID}" ] && kill -0 "${MAIN_PID}" 2>/dev/null; then
    kill -TERM "${MAIN_PID}" 2>/dev/null || true
  fi
  if [ -n "${HELPER_PID}" ] && kill -0 "${HELPER_PID}" 2>/dev/null; then
    kill -TERM "${HELPER_PID}" 2>/dev/null || true
  fi
  if [ -n "${FFMPEG_PID}" ] && kill -0 "${FFMPEG_PID}" 2>/dev/null; then
    kill -INT "${FFMPEG_PID}" 2>/dev/null || true
  fi

  if [ -x "${SCRIPT_DIR}/stop_toudi3_sim.sh" ]; then
    "${SCRIPT_DIR}/stop_toudi3_sim.sh" >> "${RUN_LOG}" 2>&1 || cleanup_status=1
  else
    echo "SITL cleanup helper missing: ${SCRIPT_DIR}/stop_toudi3_sim.sh" >> "${RUN_LOG}"
    cleanup_status=1
  fi

  for SIM_PROCESS_NAME in "${SIM_PROCESS_NAMES[@]}"; do
    SIM_PROCESS_PIDS="$(pgrep -x "${SIM_PROCESS_NAME}" 2>/dev/null | paste -sd, -)"
    if [ -n "${SIM_PROCESS_PIDS}" ]; then
      residual_processes="${residual_processes} ${SIM_PROCESS_NAME}=${SIM_PROCESS_PIDS}"
    fi
  done
  if [ -n "${residual_processes}" ]; then
    echo "SITL cleanup verification: FAIL:${residual_processes}" | tee -a "${RUN_LOG}" >&2
    cleanup_status=1
  else
    echo "SITL cleanup verification: PASS (no local ROS/Gazebo/PX4/RViz process remains)" | tee -a "${RUN_LOG}"
  fi

  if [ "${cleanup_status}" != "0" ]; then
    original_status=1
  fi
  exit "${original_status}"
}

trap final_sim_cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

# ---- manifest 头部 ----
{
  echo "# sim_run manifest — 自动生成，勿手改"
  echo "scene: ${SCENE}"
  echo "start_time: $(date -Iseconds)"
  echo "git_head: $(cd "${PROJECT_ROOT}" && git rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_status:"
  (cd "${PROJECT_ROOT}" && git status --short) | sed 's/^/  /'
  echo "launch_cmd:"
  printf '  - %s\n' "$@" | sed 's/ - /"\n  - "/g'
  echo "roslaunch_args:"
  printf '  %s\n' "$@" | sed 's/ /=/; s/ / /' | head -20
} > "${MANIFEST}"

# ---- sim_helpers（可选：SIM_HELPERS=1 启动，旧链 mock 用） ----
if [ "${SIM_HELPERS:-0}" = "1" ]; then
  python3 "${SCRIPT_DIR}/sim_helpers.py" >> "${RUN_LOG}" 2>&1 &
  HELPER_PID=$!
  echo "helper_pid: ${HELPER_PID}" >> "${MANIFEST}"
  sleep 2
fi

# ---- ffmpeg 录屏（可选：SIM_NO_RECORD=1 关闭） ----
if [ "${SIM_NO_RECORD:-0}" != "1" ]; then
  ffmpeg -f x11grab -framerate 10 -video_size 1280x720 -i :0 \
    -c:v libx264 -preset ultrafast -crf 28 "${RECORD_MP4}" \
    &>/tmp/sim_run_ffmpeg_$$.log &
  FFMPEG_PID=$!
  echo "ffmpeg_pid: ${FFMPEG_PID}" >> "${MANIFEST}"
fi

echo "===== SIM RUN: ${SCENE} =====" | tee "${RUN_LOG}"
echo "Run dir: ${RUN_DIR}" | tee -a "${RUN_LOG}"
echo "Start: $(date)" | tee -a "${RUN_LOG}"
echo "" | tee -a "${RUN_LOG}"

# ---- 运行 roslaunch；后台 wait 让信号 trap 能可靠回收整个 SITL 栈 ----
"$@" >> "${RUN_LOG}" 2>&1 &
MAIN_PID=$!
wait "${MAIN_PID}"
EXIT_CODE=$?
MAIN_PID=""

# A Gate assertion may stop roslaunch through required="true" while
# roslaunch itself still exits zero.  Prefer the structured Gate result when
# present so manifest.yaml reflects the actual acceptance outcome.
GATE_STATUS_FILE="${RUN_DIR}/gate_status.json"
if [ -f "${GATE_STATUS_FILE}" ]; then
  GATE_STATUS="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("status", ""))' "${GATE_STATUS_FILE}" 2>/dev/null || true)"
  echo "Gate status: ${GATE_STATUS:-UNKNOWN}" | tee -a "${RUN_LOG}"
  if [ "${GATE_STATUS}" != "PASS" ]; then
    EXIT_CODE=1
  fi
fi

echo "" | tee -a "${RUN_LOG}"
echo "Main cmd exited: ${EXIT_CODE}" | tee -a "${RUN_LOG}"

# ---- 优雅收尾 ----
echo "exit_code: ${EXIT_CODE}" >> "${MANIFEST}"
echo "end_time: $(date -Iseconds)" >> "${MANIFEST}"

# 停 sim_helpers
if [ -n "${HELPER_PID}" ]; then
  kill "${HELPER_PID}" 2>/dev/null
  sleep 1
  kill -9 "${HELPER_PID}" 2>/dev/null
  HELPER_PID=""
fi

if [ -n "${FFMPEG_PID}" ]; then
  # SIGINT 让 ffmpeg 写 moov atom
  kill -INT "${FFMPEG_PID}" 2>/dev/null
  for i in $(seq 1 10); do
    kill -0 "${FFMPEG_PID}" 2>/dev/null || break
    sleep 1
  done
  kill -9 "${FFMPEG_PID}" 2>/dev/null
  if ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1 "${RECORD_MP4}" &>/dev/null; then
    echo "recording: OK (moov present)" >> "${MANIFEST}"
  else
    echo "recording: INCOMPLETE (no moov)" >> "${MANIFEST}"
    # 尝试重封修复
    ffmpeg -i "${RECORD_MP4}" -c copy -movflags +faststart "${RECORD_MP4%.mp4}_fixed.mp4" \
      &>/tmp/sim_run_repack_$$.log && mv "${RECORD_MP4%.mp4}_fixed.mp4" "${RECORD_MP4}" \
      && echo "recording: REPACKED OK" >> "${MANIFEST}"
  fi
  FFMPEG_PID=""
fi

# ---- 归档 ros 日志 ----
if [ -d "${HOME}/.ros/log/latest" ]; then
  cp -aL "${HOME}/.ros/log/latest" "${RUN_DIR}/roslog" 2>/dev/null && \
    echo "roslog: archived" >> "${MANIFEST}"
fi

# ---- 事件时间线 ----
if [ -f "${SCRIPT_DIR}/sim_monitor.py" ]; then
  python3 "${SCRIPT_DIR}/sim_monitor.py" "${RUN_LOG}" > "${RUN_DIR}/timeline.txt" 2>/dev/null && \
    echo "timeline: generated" >> "${MANIFEST}"
fi

echo "===== SIM RUN DONE: ${SCENE} =====" | tee -a "${RUN_LOG}"
echo "Run dir: ${RUN_DIR}"
echo "Exit code: ${EXIT_CODE}"
exit "${EXIT_CODE}"
