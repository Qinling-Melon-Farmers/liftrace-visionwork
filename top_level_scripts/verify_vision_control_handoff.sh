#!/usr/bin/env bash

# Verify that a vision-to-control handoff archive is self-contained, contains
# no control implementation, and can build as an independent Catkin package.
set -euo pipefail

project_root="${PROJECT_ROOT:-/home/xhj/liftrace}"
zip_path="${1:-${project_root}/deliverables/uav_vision_control_handoff_20260717_alpha1.zip}"

if [[ ! -f "${zip_path}" ]]; then
  echo "[handoff-verify] archive not found: ${zip_path}" >&2
  exit 2
fi

tmp_root="$(mktemp -d /tmp/uav_vision_handoff_verify.XXXXXX)"
cleanup() {
  case "${tmp_root}" in
    /tmp/uav_vision_handoff_verify.*) rm -rf -- "${tmp_root}" ;;
    *) echo "[handoff-verify] refusing unsafe cleanup: ${tmp_root}" >&2 ;;
  esac
}
trap cleanup EXIT

unzip -q "${zip_path}" -d "${tmp_root}"
mapfile -t bundle_dirs < <(find "${tmp_root}" -mindepth 1 -maxdepth 1 -type d -print)
if [[ "${#bundle_dirs[@]}" -ne 1 ]]; then
  echo "[handoff-verify] expected one archive root, found ${#bundle_dirs[@]}" >&2
  exit 3
fi
bundle_root="${bundle_dirs[0]}"
package_root="${bundle_root}/vision_ws/src/uav_vision"

required=(
  "README_FIRST.md"
  "MANIFEST.sha256"
  "vision_ws/src/uav_vision/package.xml"
  "vision_ws/src/uav_vision/launch/control_handoff_dev.launch"
  "vision_ws/src/uav_vision/launch/control_handoff_board.launch"
  "vision_ws/src/uav_vision/docs/CONTROL_GROUP_HANDOFF.md"
  "vision_ws/src/uav_vision/models/merged_standard_best.pt"
  "vision_ws/src/uav_vision/models/merged_standard_fp32.rknn"
  "vision_ws/src/uav_vision/models/merged_standard_6cls_metadata.yaml"
)
for relative_path in "${required[@]}"; do
  if [[ ! -f "${bundle_root}/${relative_path}" ]]; then
    echo "[handoff-verify] missing required file: ${relative_path}" >&2
    exit 4
  fi
done

if find "${bundle_root}" -type f \( -name '*.pyc' -o -name '*.pyo' \) -print -quit | grep -q .; then
  echo "[handoff-verify] generated Python bytecode is present" >&2
  exit 5
fi
if find "${bundle_root}" -type d \( -name __pycache__ -o -name patrol_control -o -name uav_mission -o -name actuator_pwm \) -print -quit | grep -q .; then
  echo "[handoff-verify] generated cache or control package is present" >&2
  exit 6
fi
if rg -n -e '<depend>patrol_control</depend>' -e '<depend>uav_mission</depend>' \
    -e 'find_package\(.*patrol_control' "${package_root}/package.xml" "${package_root}/CMakeLists.txt"; then
  echo "[handoff-verify] visual package declares a control dependency" >&2
  exit 7
fi
if rg -n '/home/xhj/' "${package_root}/config" "${package_root}/launch" "${package_root}/scripts"; then
  echo "[handoff-verify] runtime files contain host-specific /home/xhj paths" >&2
  exit 8
fi

(cd "${bundle_root}" && sha256sum -c MANIFEST.sha256)

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  set -u
  cd "${bundle_root}/vision_ws"
  catkin_make --pkg uav_vision -j1
  set +u
  # shellcheck disable=SC1091
  source devel/setup.bash
  set -u
  roslaunch uav_vision control_handoff_dev.launch --nodes >/dev/null
  roslaunch uav_vision control_handoff_board.launch --nodes >/dev/null
fi

echo "VISION_CONTROL_HANDOFF_VERIFY PASS: ${zip_path}"
