#!/usr/bin/env bash

# Verify package boundary, manifest, standalone uav_vision build and launches.
set -euo pipefail

script_dir="${BASH_SOURCE[0]%/*}"
project_root="${PROJECT_ROOT:-$(cd "${script_dir}/.." && pwd)}"
zip_path="${1:-${project_root}/deliverables/uav_vision_navigation_handoff_20260807_beta1.zip}"

if [[ ! -f "${zip_path}" ]]; then
  echo "[导航交付验证] 找不到 ZIP：${zip_path}" >&2
  exit 2
fi

zip_dir="${zip_path%/*}"
zip_file="${zip_path##*/}"
if [[ "${zip_dir}" == "${zip_path}" ]]; then
  zip_dir="."
fi
if [[ -f "${zip_path}.sha256" ]]; then
  (cd "${zip_dir}" && sha256sum -c "${zip_file}.sha256")
fi

tmp_root="$(mktemp -d /tmp/uav_vision_navigation_verify.XXXXXX)"
cleanup() {
  case "${tmp_root}" in
    /tmp/uav_vision_navigation_verify.*) rm -rf -- "${tmp_root}" ;;
    *) echo "[导航交付验证] 拒绝清理异常路径：${tmp_root}" >&2 ;;
  esac
}
trap cleanup EXIT

unzip -q "${zip_path}" -d "${tmp_root}"
mapfile -t roots < <(find "${tmp_root}" -mindepth 1 -maxdepth 1 -type d -print)
if [[ "${#roots[@]}" -ne 1 ]]; then
  echo "[导航交付验证] ZIP 必须只有一个根目录" >&2
  exit 3
fi
bundle_root="${roots[0]}"
package_root="${bundle_root}/vision_ws/src/uav_vision"
reference_root="${bundle_root}/reference_integration"

required=(
  "README_FIRST.md"
  "INSTALL_AND_SIMULATION.md"
  "SOURCE_REVISION.txt"
  "MANIFEST.sha256"
  "vision_ws/src/uav_vision/package.xml"
  "vision_ws/src/uav_vision/launch/control_handoff_dev.launch"
  "vision_ws/src/uav_vision/launch/control_handoff_board.launch"
  "vision_ws/src/uav_vision/docs/NAVIGATION_GROUP_HANDOFF.md"
  "vision_ws/src/uav_vision/models/merged_standard_best.pt"
  "vision_ws/src/uav_vision/models/merged_standard_fp32.rknn"
  "reference_integration/README.md"
  "reference_integration/msg/MissionCommand.msg"
  "reference_integration/msg/ReleaseResult.msg"
  "reference_integration/scripts/coverage_policy.py"
  "reference_integration/scripts/coverage_search_manager.py"
  "reference_integration/config/coverage_toudi3.yaml"
  "reference_integration/launch/coverage_navigation.launch"
  "evidence/target_area_navigation_gate_status.json"
)
for relative_path in "${required[@]}"; do
  if [[ ! -f "${bundle_root}/${relative_path}" ]]; then
    echo "[导航交付验证] 缺少文件：${relative_path}" >&2
    exit 4
  fi
done

if find "${bundle_root}" -type f \( -name '*.pyc' -o -name '*.pyo' \) -print -quit | grep -q .; then
  echo "[导航交付验证] ZIP 含 Python 缓存" >&2
  exit 5
fi
if find "${bundle_root}" -type d -name __pycache__ -print -quit | grep -q .; then
  echo "[导航交付验证] ZIP 含 __pycache__" >&2
  exit 6
fi
if find "${reference_root}" -type f \( -name package.xml -o -name CMakeLists.txt \) -print -quit | grep -q .; then
  echo "[导航交付验证] 参考代码不得伪装成可编译 ROS 包" >&2
  exit 7
fi
if rg -n -e '<depend>patrol_control</depend>' -e '<depend>uav_mission</depend>' \
    "${package_root}/package.xml" "${package_root}/CMakeLists.txt"; then
  echo "[导航交付验证] 纯视觉包出现任务/控制依赖" >&2
  exit 8
fi
if rg -n '/home/xhj/' "${package_root}/config" "${package_root}/launch" "${package_root}/scripts"; then
  echo "[导航交付验证] 运行文件含主机绝对路径" >&2
  exit 9
fi
if ! rg -q '仅供参考|参考实现' "${reference_root}/README.md" "${bundle_root}/README_FIRST.md"; then
  echo "[导航交付验证] 缺少参考实现边界说明" >&2
  exit 10
fi
if ! rg -q '完整 toudi3.*ZIP.*不支持|ZIP 单独不支持' \
    "${bundle_root}/README_FIRST.md" "${bundle_root}/INSTALL_AND_SIMULATION.md"; then
  echo "[导航交付验证] 缺少完整仿真的额外依赖边界" >&2
  exit 11
fi

(cd "${bundle_root}" && sha256sum -c MANIFEST.sha256)
python3 -m py_compile \
  "${reference_root}/scripts/coverage_policy.py" \
  "${reference_root}/scripts/coverage_search_manager.py" \
  "${reference_root}/scripts/coverage_navigation_assertion.py"

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
  cd "${bundle_root}/vision_ws"
  catkin_make --pkg uav_vision -j1
  set +u
  source devel/setup.bash
  set -u
  roslaunch uav_vision control_handoff_dev.launch --nodes >/dev/null
  roslaunch uav_vision control_handoff_board.launch --nodes >/dev/null
fi

echo "VISION_NAVIGATION_HANDOFF_VERIFY PASS: ${zip_path}"
