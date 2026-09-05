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

# Starting Gazebo/PX4 consumes a machine-wide ROS/Gazebo resource.  Require an
# explicit per-command opt-in so a resumed agent turn, documentation check or
# launch-file inspection cannot accidentally start a simulator.
if [ "${SIM_RUN_AUTHORIZED:-0}" != "1" ]; then
  echo "Refusing to start simulation: set SIM_RUN_AUTHORIZED=1 for an explicitly authorized run." >&2
  exit 64
fi

if [ "$#" -lt 2 ]; then
  echo "Usage: SIM_RUN_AUTHORIZED=1 sim_run.sh <scene> <roslaunch command...>" >&2
  exit 64
fi

# Only one local SITL stack may own ROS/Gazebo at a time.  flock releases the
# lock automatically even if this wrapper is interrupted.
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
export PROJECT_ROOT
export VISION_WS="${VISION_WS:-${PROJECT_ROOT}/vision_ws}"
INTEGRATION_WS="${LIFTRACE_INTEGRATION_WS:-/home/xhj/liftrace/patrol_uav_ws-patrol_planner}"
if [ -z "${UAV_WS:-}" ]; then
  if [ -f "${PROJECT_ROOT}/patrol_uav_ws-patrol_planner/devel/setup.bash" ]; then
    export UAV_WS="${PROJECT_ROOT}/patrol_uav_ws-patrol_planner"
  else
    # A git worktree normally has no copied build/devel products.  Reuse the
    # integration workspace only as a compiled underlay; source packages below
    # are still forced to this worktree.
    export UAV_WS="${INTEGRATION_WS}"
  fi
fi

# ---- source ROS 环境（可被调用者预先覆盖；setup 脚本与 nounset 不兼容，临时关闭） ----
set +u
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi
INTEGRATION_UNDERLAY_SOURCED=0
if [ "${UAV_WS}" = "${PROJECT_ROOT}/patrol_uav_ws-patrol_planner" ] && \
   [ "${UAV_WS}" != "${INTEGRATION_WS}" ] && \
   [ -f "${INTEGRATION_WS}/devel/setup.bash" ]; then
  # A feature worktree may contain only a targeted catkin build.  Keep the
  # integration workspace underneath it so unchanged runtime executables and
  # generated interfaces remain available without copying build products.
  source "${INTEGRATION_WS}/devel/setup.bash"
  INTEGRATION_UNDERLAY_SOURCED=1
fi
if [ -f "${UAV_WS}/devel/setup.bash" ]; then
  if [ "${INTEGRATION_UNDERLAY_SOURCED}" = "1" ]; then
    source "${UAV_WS}/devel/setup.bash" --extend
  else
    source "${UAV_WS}/devel/setup.bash"
  fi
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
  python3 - "$@" <<'PY'
import json
import sys

arguments = sys.argv[1:]
print("launch_cmd:")
for argument in arguments:
    # A JSON string is also a valid YAML scalar and preserves spaces, quotes
    # and launch substitutions without handwritten escaping.
    print("  - " + json.dumps(argument, ensure_ascii=False))
print("roslaunch_args:")
for argument in arguments[:20]:
    print("  - " + json.dumps(argument, ensure_ascii=False))
PY
  echo "require_gate: ${REQUIRE_GATE}"
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
    VSIM04_ARTIFACT_SET_COMPLETE=""
    VSIM04_ERROR_COUNT=""
    VSIM04_P_INTERRUPT=""
    VSIM04_REQUESTED_GE_600=""
    VSIM04_REQUESTED_LT_600=""
    VSIM04_ACTUAL_GE_600=""
    VSIM04_EXPECTED_QUALIFICATION=""
    VSIM04_IS_SOAK=0
    VSIM04_EXPECTED_STATUS="MEASURED"
    case "${SCENE}" in
      vsim04_diag*) VSIM04_EXPECTED_STATUS="DIAGNOSTIC" ;;
      vsim04_soak_smoke*)
        VSIM04_EXPECTED_STATUS="SOAK_MEASURED"
        VSIM04_IS_SOAK=1
        VSIM04_EXPECTED_QUALIFICATION="SMOKE_ONLY"
        ;;
      vsim04_soak600*)
        VSIM04_EXPECTED_STATUS="SOAK_MEASURED"
        VSIM04_IS_SOAK=1
        VSIM04_EXPECTED_QUALIFICATION="SOAK_600S_MEASURED"
        ;;
      vsim04_soak*)
        VSIM04_EXPECTED_STATUS="SOAK_MEASURED"
        VSIM04_IS_SOAK=1
        VSIM04_EXPECTED_QUALIFICATION="INVALID_SCENE_NAME"
        ;;
    esac
    VSIM04_ARTIFACTS="manifest.json frames.csv events.csv summary.json report.md vision_search_performance.csv"
    VSIM04_MISSING=""
    for artifact in ${VSIM04_ARTIFACTS}; do
      if [ ! -s "${VSIM04_DIR}/${artifact}" ]; then
        VSIM04_MISSING="${VSIM04_MISSING} ${artifact}"
      fi
    done
    if [ -f "${VSIM04_SUMMARY}" ]; then
      VSIM04_STATUS="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("status", ""))' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_PERFORMANCE_VERDICT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("performance_verdict", {}).get("status", ""))' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_PERFORMANCE_HARD_FAILURE="$(python3 -c 'import json,sys; print(str(json.load(open(sys.argv[1], encoding="utf-8")).get("performance_verdict", {}).get("hard_failure", False)).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_QUALIFICATION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("qualification_status", ""))' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_SOAK_600S_PASS="$(python3 -c 'import json,sys; print(str(json.load(open(sys.argv[1], encoding="utf-8")).get("soak_600s_pass", False)).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_ARTIFACT_SET_COMPLETE="$(python3 -c 'import json,sys; print(str(json.load(open(sys.argv[1], encoding="utf-8")).get("artifact_set_complete", False)).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_ERROR_COUNT="$(python3 -c 'import json,sys; print(len(json.load(open(sys.argv[1], encoding="utf-8")).get("errors", [])))' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_P_INTERRUPT="$(python3 -c 'import json,sys; value=json.load(open(sys.argv[1], encoding="utf-8")).get("p_interrupt", "missing"); print("null" if value is None else value)' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_REQUESTED_GE_600="$(python3 -c 'import json,sys; print(str(float(json.load(open(sys.argv[1], encoding="utf-8")).get("requested_duration_sec", -1)) >= 600.0).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_REQUESTED_LT_600="$(python3 -c 'import json,sys; print(str(0.0 < float(json.load(open(sys.argv[1], encoding="utf-8")).get("requested_duration_sec", -1)) < 600.0).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
      VSIM04_ACTUAL_GE_600="$(python3 -c 'import json,sys; print(str(float(json.load(open(sys.argv[1], encoding="utf-8")).get("actual_wall_duration_sec", -1)) >= 600.0).lower())' "${VSIM04_SUMMARY}" 2>/dev/null || true)"
    fi
    echo "V-SIM-04 status: ${VSIM04_STATUS:-MISSING} (expected ${VSIM04_EXPECTED_STATUS})" | tee -a "${RUN_LOG}"
    echo "V-SIM-04 performance verdict: ${VSIM04_PERFORMANCE_VERDICT:-MISSING} (hard_failure=${VSIM04_PERFORMANCE_HARD_FAILURE:-unknown})" | tee -a "${RUN_LOG}"
    if [ "${VSIM04_IS_SOAK}" = "1" ]; then
      echo "V-SIM-04 soak qualification: ${VSIM04_QUALIFICATION:-MISSING} (expected ${VSIM04_EXPECTED_QUALIFICATION:-MISSING}, 600s_pass=${VSIM04_SOAK_600S_PASS:-false}, artifacts=${VSIM04_ARTIFACT_SET_COMPLETE:-false}, errors=${VSIM04_ERROR_COUNT:-unknown}, P_interrupt=${VSIM04_P_INTERRUPT:-missing})" | tee -a "${RUN_LOG}"
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
    if [ "${VSIM04_IS_SOAK}" = "1" ]; then
      if [ "${VSIM04_ARTIFACT_SET_COMPLETE}" != "true" ] || \
         [ "${VSIM04_ERROR_COUNT}" != "0" ] || \
         [ "${VSIM04_P_INTERRUPT}" != "null" ] || \
         [ "${VSIM04_QUALIFICATION}" != "${VSIM04_EXPECTED_QUALIFICATION}" ]; then
        EXIT_CODE=1
      fi
      if [ "${VSIM04_EXPECTED_QUALIFICATION}" = "SMOKE_ONLY" ] && \
         { [ "${VSIM04_SOAK_600S_PASS}" != "false" ] || \
           [ "${VSIM04_REQUESTED_LT_600}" != "true" ]; }; then
        EXIT_CODE=1
      fi
      if [ "${VSIM04_EXPECTED_QUALIFICATION}" = "SOAK_600S_MEASURED" ] && \
         { [ "${VSIM04_SOAK_600S_PASS}" != "true" ] || \
           [ "${VSIM04_REQUESTED_GE_600}" != "true" ] || \
           [ "${VSIM04_ACTUAL_GE_600}" != "true" ]; }; then
        EXIT_CODE=1
      fi
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
