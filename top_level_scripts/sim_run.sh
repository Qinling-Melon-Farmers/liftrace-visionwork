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
echo "ffmpeg_pid: ${FFMPEG_PID}" >> "${MANIFEST}"
echo "exit_code: ${EXIT_CODE}" >> "${MANIFEST}"
echo "end_time: $(date -Iseconds)" >> "${MANIFEST}"

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
