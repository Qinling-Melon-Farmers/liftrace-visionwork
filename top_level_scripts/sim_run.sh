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
export PROJECT_ROOT
export VISION_WS="${VISION_WS:-${PROJECT_ROOT}/vision_ws}"
if [ -z "${UAV_WS:-}" ]; then
  if [ -f "${PROJECT_ROOT}/patrol_uav_ws-patrol_planner/devel/setup.bash" ]; then
    export UAV_WS="${PROJECT_ROOT}/patrol_uav_ws-patrol_planner"
  else
    # A git worktree normally has no copied build/devel products.  Reuse the
    # integration workspace only as a compiled underlay; source packages below
    # are still forced to this worktree.
    export UAV_WS="${LIFTRACE_INTEGRATION_WS:-/home/xhj/liftrace/patrol_uav_ws-patrol_planner}"
  fi
fi

# WSL may inherit Windows Anaconda paths even when invoked non-interactively.
# Keep ROS command wrappers on the Ubuntu system Python before sourcing overlays.
export PATH="/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME PYTHONPATH

# ---- source ROS 环境（可被调用者预先覆盖；setup 脚本与 nounset 不兼容，临时关闭） ----
set +u
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${UAV_WS}/devel/setup.bash" ]; then
  source "${UAV_WS}/devel/setup.bash"
fi
if [ -f "${VISION_WS}/devel/setup.bash" ]; then
  # Keep the compiled integration workspace available, but make the current
  # feature worktree the highest-priority visual overlay.
  source "${VISION_WS}/devel/setup.bash" --extend
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
REQUIRE_GATE="${SIM_REQUIRE_GATE:-0}"
VSIM04_CAPTURE_REQUESTED=0
VSIM04_CAPTURE_SUBDIR="failure_capture"
VSIM04_OUTPUT_OVERRIDE=0
for RUN_ARGUMENT in "$@"; do
  case "${RUN_ARGUMENT}" in
    enable_failure_capture:=*)
      VSIM04_CAPTURE_VALUE="${RUN_ARGUMENT#enable_failure_capture:=}"
      case "${VSIM04_CAPTURE_VALUE}" in
        true|True|TRUE|1) VSIM04_CAPTURE_REQUESTED=1 ;;
        false|False|FALSE|0) VSIM04_CAPTURE_REQUESTED=0 ;;
      esac
      ;;
    failure_capture_output_dir:=*)
      VSIM04_CAPTURE_SUBDIR="${RUN_ARGUMENT#failure_capture_output_dir:=}"
      ;;
    output_dir:=*) VSIM04_OUTPUT_OVERRIDE=1 ;;
  esac
done
case "${SCENE}" in
  vsim04*)
    if [ "${VSIM04_OUTPUT_OVERRIDE}" = "1" ]; then
      echo "sim_run owns V-SIM-04 output_dir inside SIM_RUN_DIR" >&2
      exit 2
    fi
    ;;
esac
if [ "${VSIM04_CAPTURE_REQUESTED}" = "1" ]; then
  VSIM04_CAPTURE_SUBDIR_VALID=1
  case "${VSIM04_CAPTURE_SUBDIR}" in
    ""|/*|*\\*) VSIM04_CAPTURE_SUBDIR_VALID=0 ;;
  esac
  OLD_IFS="${IFS}"
  IFS=/
  read -r -a VSIM04_CAPTURE_PARTS <<< "${VSIM04_CAPTURE_SUBDIR}"
  IFS="${OLD_IFS}"
  for VSIM04_CAPTURE_PART in "${VSIM04_CAPTURE_PARTS[@]}"; do
    if [ -z "${VSIM04_CAPTURE_PART}" ] || \
       [ "${VSIM04_CAPTURE_PART}" = "." ] || \
       [ "${VSIM04_CAPTURE_PART}" = ".." ] || \
       [[ ! "${VSIM04_CAPTURE_PART}" =~ ^[A-Za-z0-9._-]+$ ]]; then
      VSIM04_CAPTURE_SUBDIR_VALID=0
    fi
  done
  if [ "${VSIM04_CAPTURE_SUBDIR_VALID}" != "1" ]; then
    echo "invalid run-relative failure_capture_output_dir: ${VSIM04_CAPTURE_SUBDIR}" >&2
    exit 2
  fi
fi

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
  echo "require_gate: ${REQUIRE_GATE}"
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
elif [ "${REQUIRE_GATE}" = "1" ]; then
  echo "Gate status: MISSING (SIM_REQUIRE_GATE=1)" | tee -a "${RUN_LOG}"
  EXIT_CODE=1
fi

# V-SIM-04 的 required runner 失败会让 roslaunch 触发正常关停，roslaunch 本身仍可能
# 返回 0。对该场景必须以评测终态和六项产物为准，避免 INVALID 被误报为成功运行。
case "${SCENE}" in
  vsim04*)
    VSIM04_DIR="${RUN_DIR}/vsim04"
    VSIM04_SUMMARY="${VSIM04_DIR}/summary.json"
    VSIM04_STATUS=""
    VSIM04_PERFORMANCE_VERDICT=""
    VSIM04_PERFORMANCE_HARD_FAILURE=""
    VSIM04_QUALIFICATION=""
    VSIM04_SOAK_600S_PASS=""
    VSIM04_IS_SOAK=0
    VSIM04_EXPECTED_STATUS="MEASURED"
    case "${SCENE}" in
      vsim04_diag*) VSIM04_EXPECTED_STATUS="DIAGNOSTIC" ;;
      vsim04_soak*)
        VSIM04_EXPECTED_STATUS="SOAK_MEASURED"
        VSIM04_IS_SOAK=1
        ;;
    esac
    VSIM04_ARTIFACTS="manifest.json frames.csv events.csv summary.json report.md vision_search_performance.csv"
    VSIM04_MISSING=""
    for artifact in ${VSIM04_ARTIFACTS}; do
      if [ ! -f "${VSIM04_DIR}/${artifact}" ]; then
        VSIM04_MISSING="${VSIM04_MISSING} ${artifact}"
      fi
    done
    if [ -f "${VSIM04_SUMMARY}" ]; then
      VSIM04_STATUS="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("status", ""))' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_PERFORMANCE_VERDICT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("performance_verdict", {}).get("status", ""))' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_PERFORMANCE_HARD_FAILURE="$(python3 -c 'import json,sys; print(str(json.load(open(sys.argv[1], encoding="utf-8")).get("performance_verdict", {}).get("hard_failure", False)).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_QUALIFICATION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("qualification_status", ""))' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_SOAK_600S_PASS="$(python3 -c 'import json,sys; print(str(json.load(open(sys.argv[1], encoding="utf-8")).get("soak_600s_pass", False)).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
    fi
    echo "V-SIM-04 status: ${VSIM04_STATUS:-MISSING} (expected ${VSIM04_EXPECTED_STATUS})" | tee -a "${RUN_LOG}"
    echo "V-SIM-04 performance verdict: ${VSIM04_PERFORMANCE_VERDICT:-MISSING} (hard_failure=${VSIM04_PERFORMANCE_HARD_FAILURE:-unknown})" | tee -a "${RUN_LOG}"
    if [ "${VSIM04_IS_SOAK}" = "1" ]; then
      echo "V-SIM-04 soak qualification: ${VSIM04_QUALIFICATION:-MISSING} (600s_pass=${VSIM04_SOAK_600S_PASS:-false})" | tee -a "${RUN_LOG}"
    fi
    if [ -n "${VSIM04_MISSING}" ]; then
      echo "V-SIM-04 missing artifacts:${VSIM04_MISSING}" | tee -a "${RUN_LOG}"
    fi
    if [ "${VSIM04_STATUS}" != "${VSIM04_EXPECTED_STATUS}" ] || [ -n "${VSIM04_MISSING}" ]; then
      EXIT_CODE=1
    fi
    if [ "${VSIM04_IS_SOAK}" != "1" ] && \
       [ "${VSIM04_PERFORMANCE_HARD_FAILURE}" = "true" ]; then
      EXIT_CODE=1
    fi
    if [ "${VSIM04_IS_SOAK}" = "1" ] && \
       [ "${VSIM04_QUALIFICATION}" != "SMOKE_ONLY" ] && \
       [ "${VSIM04_QUALIFICATION}" != "SOAK_600S_MEASURED" ]; then
      EXIT_CODE=1
    fi
    if [ "${VSIM04_IS_SOAK}" != "1" ] && \
       [ "${VSIM04_EXPECTED_STATUS}" = "MEASURED" ] && \
       [ "${VSIM04_PERFORMANCE_VERDICT}" != "PASS" ]; then
      EXIT_CODE=1
    fi
    if [ "${VSIM04_IS_SOAK}" != "1" ] && \
       [ "${VSIM04_EXPECTED_STATUS}" = "DIAGNOSTIC" ] && \
       [ "${VSIM04_PERFORMANCE_VERDICT}" != "DIAGNOSTIC_ONLY" ]; then
      EXIT_CODE=1
    fi
    if [ "${VSIM04_CAPTURE_REQUESTED}" = "1" ]; then
      VSIM04_CAPTURE_DIR="${VSIM04_DIR}/${VSIM04_CAPTURE_SUBDIR}"
      VSIM04_CAPTURE_MANIFEST="${VSIM04_CAPTURE_DIR}/dataset_manifest.json"
      VSIM04_CAPTURE_STATUS=""
      if [ -f "${VSIM04_CAPTURE_MANIFEST}" ]; then
        VSIM04_CAPTURE_STATUS="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("status", ""))' "${VSIM04_CAPTURE_MANIFEST}" 2>/dev/null || true)"
      fi
      echo "V-SIM-04 capture status: ${VSIM04_CAPTURE_STATUS:-MISSING} (expected DIAGNOSTIC)" | tee -a "${RUN_LOG}"
      if [ "${VSIM04_CAPTURE_STATUS}" != "DIAGNOSTIC" ]; then
        EXIT_CODE=1
      fi
      VSIM04_CAPTURE_CHECKER="${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/scripts/vsim04_failure_capture_manifest_check.py"
      if ! PYTHONPATH="${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/src${PYTHONPATH:+:${PYTHONPATH}}" \
          python3 "${VSIM04_CAPTURE_CHECKER}" "${VSIM04_CAPTURE_MANIFEST}"; then
        echo "V-SIM-04 capture manifest/files validation: FAIL" | tee -a "${RUN_LOG}"
        EXIT_CODE=1
      else
        echo "V-SIM-04 capture manifest/files validation: PASS" | tee -a "${RUN_LOG}"
      fi
    fi
    ;;
esac

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
