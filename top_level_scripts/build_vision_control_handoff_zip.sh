#!/usr/bin/env bash

# 构建只包含视觉运行链的控制组联调 ZIP。
# 不复制 patrol_control、uav_mission、actuator_pwm、PX4、规划器或评测大数据。
set -euo pipefail

project_root="${PROJECT_ROOT:-/home/xhj/liftrace}"
output_dir="${1:-${project_root}/deliverables}"
bundle_name="uav_vision_control_handoff_20260717_alpha1"
zip_path="${output_dir}/${bundle_name}.zip"
dev_model="${project_root}/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt"
board_model="${project_root}/vision_ws/test_data/board_eval_20260716/models/merged_standard_fp32.rknn"
metadata="${project_root}/vision_ws/src/uav_vision/config/merged_standard_6cls_metadata.yaml"

for required_file in "${dev_model}" "${board_model}" "${metadata}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "[视觉交付打包] 缺少必要文件：${required_file}" >&2
    exit 2
  fi
done
if ! command -v zip >/dev/null 2>&1; then
  echo "[视觉交付打包] 系统未安装 zip" >&2
  exit 3
fi

mkdir -p "${output_dir}"
stage_parent="$(mktemp -d /tmp/uav_vision_handoff_build.XXXXXX)"
cleanup() {
  case "${stage_parent}" in
    /tmp/uav_vision_handoff_build.*) rm -rf -- "${stage_parent}" ;;
    *) echo "[视觉交付打包] 拒绝清理异常路径：${stage_parent}" >&2 ;;
  esac
}
trap cleanup EXIT

bundle_root="${stage_parent}/${bundle_name}"
package_root="${bundle_root}/vision_ws/src/uav_vision"
mkdir -p "${package_root}" "${bundle_root}/evidence"

rsync -a \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '*.pyo' \
  --exclude 'launch/phase_d_mock_patrol.launch' \
  --exclude 'launch/phase_d_mock_patrol_regression.launch' \
  --exclude 'launch/phase_d_patrol_internal.launch' \
  "${project_root}/vision_ws/src/uav_vision/" "${package_root}/"

install -m 0644 "${dev_model}" "${package_root}/models/merged_standard_best.pt"
install -m 0644 "${board_model}" "${package_root}/models/merged_standard_fp32.rknn"
install -m 0644 "${metadata}" "${package_root}/models/merged_standard_6cls_metadata.yaml"
install -m 0644 "${project_root}/docs/VISION_CONTROL_HANDOFF_20260717.md" \
  "${bundle_root}/README_FIRST.md"
install -m 0644 "${project_root}/docs/BOARD_MODEL_COMPLETE_EVALUATION_20260716.md" \
  "${bundle_root}/evidence/板端模型完整评测_20260716.md"
install -m 0644 "${project_root}/docs/VISION_LAPTOP_SIM_BASELINE_20260715.md" \
  "${bundle_root}/evidence/笔记本仿真基线_20260715.md"

(cd "${bundle_root}" && \
  find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | \
  xargs -0 sha256sum > MANIFEST.sha256)

tmp_zip="${stage_parent}/${bundle_name}.zip"
(cd "${stage_parent}" && zip -qr "${tmp_zip}" "${bundle_name}")
install -m 0644 "${tmp_zip}" "${zip_path}"
sha256sum "${zip_path}" > "${zip_path}.sha256"

echo "[视觉交付打包] 已生成：${zip_path}"
echo "[视觉交付打包] ZIP 校验：${zip_path}.sha256"
