#!/usr/bin/env bash
# Build the camera + RKNN visual-only OrangePi staging archive.
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/deliverables}"
BUNDLE_NAME="orangepi_vision_camera_20260902"
BOARD_MODEL="${UAV_VISION_RKNN_MODEL_PATH:?set UAV_VISION_RKNN_MODEL_PATH to merged_standard_fp32.rknn}"

if [[ ! -f "${BOARD_MODEL}" ]]; then
  echo "RKNN model is not a file: ${BOARD_MODEL}" >&2
  exit 2
fi
for REQUIRED in \
  "${PROJECT_ROOT}/vision_ws/src/CMakeLists.txt" \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision/package.xml" \
  "${PROJECT_ROOT}/vision_ws/src/camera_sdk/package.xml" \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision/config/merged_standard_6cls_metadata.yaml" \
  "${PROJECT_ROOT}/docs/ORANGEPI_CAMERA_VISION_LAB_CHECKLIST_20260902.md"; do
  if [[ ! -f "${REQUIRED}" ]]; then
    echo "required board-deploy input is missing: ${REQUIRED}" >&2
    exit 2
  fi
done
if [[ -n "$(git -C "${PROJECT_ROOT}" status --porcelain)" ]]; then
  echo "refusing to package a dirty vision tree; commit local changes first" >&2
  exit 2
fi
command -v zip >/dev/null 2>&1 || {
  echo "zip is not installed" >&2
  exit 3
}

mkdir -p "${OUTPUT_DIR}"
STAGE_PARENT="$(mktemp -d /tmp/orangepi_vision_bundle.XXXXXX)"
cleanup() {
  case "${STAGE_PARENT}" in
    /tmp/orangepi_vision_bundle.*) rm -rf -- "${STAGE_PARENT}" ;;
    *) echo "refusing to clean unexpected stage path: ${STAGE_PARENT}" >&2 ;;
  esac
}
trap cleanup EXIT

BUNDLE_ROOT="${STAGE_PARENT}/${BUNDLE_NAME}"
mkdir -p "${BUNDLE_ROOT}/vision_ws/src" \
  "${BUNDLE_ROOT}/top_level_scripts" \
  "${BUNDLE_ROOT}/vision_ws/src/uav_vision/models"
# Ubuntu 20.04 coreutils install(1) has no -L option.  Dereference the catkin
# workspace symlink explicitly so the bundle contains a regular file.
cp -L -- "${PROJECT_ROOT}/vision_ws/src/CMakeLists.txt" \
  "${BUNDLE_ROOT}/vision_ws/src/CMakeLists.txt"
chmod 0644 "${BUNDLE_ROOT}/vision_ws/src/CMakeLists.txt"
rsync -a --exclude __pycache__/ --exclude '*.pyc' --exclude '*.pyo' \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision/" \
  "${BUNDLE_ROOT}/vision_ws/src/uav_vision/"
rsync -a --exclude __pycache__/ --exclude '*.pyc' --exclude '*.pyo' \
  "${PROJECT_ROOT}/vision_ws/src/camera_sdk/" \
  "${BUNDLE_ROOT}/vision_ws/src/camera_sdk/"
install -m 0755 "${PROJECT_ROOT}/top_level_scripts/board_realtime_rknn_viewer.py" \
  "${BUNDLE_ROOT}/top_level_scripts/board_realtime_rknn_viewer.py"
install -m 0644 "${PROJECT_ROOT}/docs/ORANGEPI_CAMERA_VISION_LAB_CHECKLIST_20260902.md" \
  "${BUNDLE_ROOT}/README_FIRST.md"
install -m 0644 "${BOARD_MODEL}" \
  "${BUNDLE_ROOT}/vision_ws/src/uav_vision/models/merged_standard_fp32.rknn"
install -m 0644 \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision/config/merged_standard_6cls_metadata.yaml" \
  "${BUNDLE_ROOT}/vision_ws/src/uav_vision/models/merged_standard_6cls_metadata.yaml"

git -C "${PROJECT_ROOT}" rev-parse HEAD > "${BUNDLE_ROOT}/VISION_REVISION"
ZIP_PATH="${OUTPUT_DIR}/${BUNDLE_NAME}.zip"
TMP_ZIP="${STAGE_PARENT}/${BUNDLE_NAME}.zip"
(cd "${STAGE_PARENT}" && zip -qr "${TMP_ZIP}" "${BUNDLE_NAME}")
install -m 0644 "${TMP_ZIP}" "${ZIP_PATH}"
echo "ORANGEPI_VISION_BUNDLE_READY ${ZIP_PATH}"
