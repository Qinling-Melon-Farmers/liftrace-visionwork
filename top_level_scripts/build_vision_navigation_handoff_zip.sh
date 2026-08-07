#!/usr/bin/env bash

# Build the pure-vision navigation handoff plus non-buildable integration refs.
set -euo pipefail

script_dir="${BASH_SOURCE[0]%/*}"
project_root="${PROJECT_ROOT:-$(cd "${script_dir}/.." && pwd)}"
output_dir="${1:-${project_root}/deliverables}"
bundle_name="uav_vision_navigation_handoff_20260807_beta1"
zip_path="${output_dir}/${bundle_name}.zip"
dev_model="${project_root}/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt"
board_model="${project_root}/vision_ws/test_data/board_eval_20260716/models/merged_standard_fp32.rknn"
metadata="${project_root}/vision_ws/src/uav_vision/config/merged_standard_6cls_metadata.yaml"
evidence_run="${EVIDENCE_RUN_DIR:-${project_root}/logs/target_area_navigation_20260807_190817}"

required_files=(
  "${dev_model}"
  "${board_model}"
  "${metadata}"
  "${evidence_run}/gate_status.json"
  "${evidence_run}/coverage_status.json"
  "${evidence_run}/manifest.yaml"
)
for required_file in "${required_files[@]}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "[导航交付打包] 缺少必要文件：${required_file}" >&2
    exit 2
  fi
done
for required_command in zip rsync sha256sum; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "[导航交付打包] 缺少命令：${required_command}" >&2
    exit 3
  fi
done

mkdir -p "${output_dir}"
stage_parent="$(mktemp -d /tmp/uav_vision_navigation_handoff.XXXXXX)"
cleanup() {
  case "${stage_parent}" in
    /tmp/uav_vision_navigation_handoff.*) rm -rf -- "${stage_parent}" ;;
    *) echo "[导航交付打包] 拒绝清理异常路径：${stage_parent}" >&2 ;;
  esac
}
trap cleanup EXIT

bundle_root="${stage_parent}/${bundle_name}"
package_root="${bundle_root}/vision_ws/src/uav_vision"
reference_root="${bundle_root}/reference_integration"
mkdir -p "${package_root}" "${reference_root}" "${bundle_root}/evidence"

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
install -m 0644 "${project_root}/docs/VISION_NAVIGATION_HANDOFF_20260807.md" \
  "${bundle_root}/README_FIRST.md"

mkdir -p "${reference_root}/msg" "${reference_root}/config" \
  "${reference_root}/launch" "${reference_root}/scripts"
install -m 0644 "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/msg/MissionCommand.msg" \
  "${reference_root}/msg/MissionCommand.msg"
install -m 0644 "${project_root}/patrol_uav_ws-patrol_planner/src/uav_mission/msg/ReleaseResult.msg" \
  "${reference_root}/msg/ReleaseResult.msg"
install -m 0644 "${project_root}/patrol_uav_ws-patrol_planner/src/uav_mission/config/coverage_toudi3.yaml" \
  "${reference_root}/config/coverage_toudi3.yaml"
install -m 0644 "${project_root}/patrol_uav_ws-patrol_planner/src/uav_mission/launch/coverage_navigation.launch" \
  "${reference_root}/launch/coverage_navigation.launch"
install -m 0644 "${project_root}/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/coverage_policy.py" \
  "${reference_root}/scripts/coverage_policy.py"
install -m 0644 "${project_root}/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/coverage_search_manager.py" \
  "${reference_root}/scripts/coverage_search_manager.py"
install -m 0644 "${project_root}/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/coverage_navigation_assertion.py" \
  "${reference_root}/scripts/coverage_navigation_assertion.py"
install -m 0644 "${project_root}/vision_ws/src/uav_vision/docs/NAVIGATION_GROUP_HANDOFF.md" \
  "${reference_root}/README.md"

install -m 0644 "${project_root}/docs/VISION_TARGET_AREA_COVERAGE_BASELINE_20260807.md" \
  "${bundle_root}/evidence/靶标区域覆盖视觉候选基线_20260807.md"
install -m 0644 "${project_root}/docs/BOARD_MODEL_COMPLETE_EVALUATION_20260716.md" \
  "${bundle_root}/evidence/板端模型完整评测_20260716.md"
install -m 0644 "${evidence_run}/gate_status.json" \
  "${bundle_root}/evidence/target_area_navigation_gate_status.json"
install -m 0644 "${evidence_run}/coverage_status.json" \
  "${bundle_root}/evidence/target_area_navigation_coverage_status.json"
install -m 0644 "${evidence_run}/manifest.yaml" \
  "${bundle_root}/evidence/target_area_navigation_manifest.yaml"

head_revision="$(git -C "${project_root}" rev-parse HEAD)"
visual_tree_state="clean"
if ! git -C "${project_root}" diff --quiet HEAD -- vision_ws/src/uav_vision; then
  visual_tree_state="modified"
fi
{
  printf 'bundle=%s\n' "${bundle_name}"
  printf 'source_git_head=%s\n' "${head_revision}"
  printf 'packaged_visual_tree=%s\n' "${visual_tree_state}"
  printf 'stage4_evidence=target_area_navigation_20260807_190817\n'
  printf 'reference_integration=non_buildable_optional_reference\n'
} > "${bundle_root}/SOURCE_REVISION.txt"

(cd "${bundle_root}" && \
  find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | \
  xargs -0 sha256sum > MANIFEST.sha256)

tmp_zip="${stage_parent}/${bundle_name}.zip"
(cd "${stage_parent}" && zip -qr "${tmp_zip}" "${bundle_name}")
install -m 0644 "${tmp_zip}" "${zip_path}"
(cd "${output_dir}" && sha256sum "${bundle_name}.zip" > "${bundle_name}.zip.sha256")

echo "[导航交付打包] 已生成：${zip_path}"
echo "[导航交付打包] ZIP 校验：${zip_path}.sha256"
