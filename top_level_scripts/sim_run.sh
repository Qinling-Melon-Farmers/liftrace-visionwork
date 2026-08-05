#!/usr/bin/env bash
# sim_run.sh — 统一仿真运行入口（阶段 0 日志基建）
#
# 用法：
#   sim_run.sh <场景名> <roslaunch 命令...>
#   sim_run.sh corridor_r1 roslaunch patrol_control patrol_full_competition_sim.launch world:=...
#
# 行为：
#   1. 生成 run 目录 logs/<场景名>_<YYYYMMDD>_<HHMMSS>/
#   2. 写 manifest.yaml 头部（时间、git HEAD、启动命令）
#   3. 可选启动 ffmpeg 录屏（后台，PID 记录）
#   4. 前台运行 roslaunch，输出到 run.log
#   5. 结束后调用 sim_finish.sh 优雅收尾（SIGINT 停 ffmpeg、归档、生成时间线）
set -u

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOGS_DIR="${PROJECT_ROOT}/logs"

# WSL may inherit Windows Anaconda paths even when invoked non-interactively.
# Keep ROS command wrappers on the Ubuntu system Python before sourcing overlays.
export PATH="/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME PYTHONPATH

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

MANIFEST="${RUN_DIR}/manifest.yaml"
RUN_LOG="${RUN_DIR}/run.log"
RECORD_MP4="${RUN_DIR}/screenrecord.mp4"

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
HELPER_PID=""
if [ "${SIM_HELPERS:-0}" = "1" ]; then
  python3 "${SCRIPT_DIR}/sim_helpers.py" >> "${RUN_LOG}" 2>&1 &
  HELPER_PID=$!
  echo "helper_pid: ${HELPER_PID}" >> "${MANIFEST}"
  sleep 2
fi

# ---- ffmpeg 录屏（可选：SIM_NO_RECORD=1 关闭） ----
FFMPEG_PID=""
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

# ---- 前台运行 roslaunch ----
"$@" >> "${RUN_LOG}" 2>&1
EXIT_CODE=$?

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
