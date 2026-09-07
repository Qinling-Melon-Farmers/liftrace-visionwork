#!/usr/bin/env bash
# Build the Git-external vision-to-navigation handoff evidence bundle.
set -euo pipefail

SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEFAULT_OUTPUT="${PROJECT_ROOT}/deliverables/liftrace_vision_to_navigation_handoff_20260904_v4.zip"

OUTPUT="${DEFAULT_OUTPUT}"
STATIC25_RUN="${PROJECT_ROOT}/logs/vsim04_diag_static25_seed11_20260904_140501"
B100_RUN="${PROJECT_ROOT}/logs/vsim04_diag_b100_seed11_20260904_140800"
C_FIXED_RUN="${PROJECT_ROOT}/logs/vsim04_c25_seed11_20260904_135946"
D_SUPPORTED_RUN="${PROJECT_ROOT}/logs/vsim04_diag_d50_supported_seed11_20260904_144351"
D_DESIGN_DIR=""
MODEL_PATH="/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt"
VCL06_GATE_RUN="/home/xhj/liftrace/logs/vcl06_p0_seed11_r11_20260904_030218"
PYTHON_BIN="/home/xhj/miniconda3/envs/rl_drone/bin/python"
FORCE=0

usage() {
  cat <<'EOF'
Usage: build_vision_nav_submission_bundle.sh [options]

Options:
  --output PATH                 Output ZIP path.
  --static25-run DIR            A static-height evidence run.
  --b100-run DIR                B full 5x5x4 height/speed evidence run.
  --c-fixed-run DIR             Current C25 run.
  --d-design-dir DIR            Existing D50 dry-run artifact directory.
                                 Omit to regenerate it deterministically.
  --d-supported-run DIR         D50 supported-slice Gazebo run.
  --model PATH                  Dev/sim model copied into the bundle.
  --vcl06-gate-run DIR          Latest joint Gate context (informational).
  --python PATH                 Python with PyYAML for validation/dry-run.
  --force                       Replace an existing ZIP and outer checksum.
  -h, --help                    Show this help.

The bundle intentionally excludes roslog/, screen recordings, bags, build
products, caches and large videos. Every included ROS run must contain
the six V-SIM-04 artifacts. Invalid historical outer manifests are preserved
as manifest_legacy.txt and accompanied by a parseable manifest.yaml wrapper.
EOF
}

need_value() {
  if [ "$#" -lt 2 ] || [ -z "${2}" ]; then
    echo "missing value for ${1}" >&2
    exit 2
  fi
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --output)
      need_value "$@"
      OUTPUT="$2"
      shift 2
      ;;
    --static25-run)
      need_value "$@"
      STATIC25_RUN="$2"
      shift 2
      ;;
    --b100-run)
      need_value "$@"
      B100_RUN="$2"
      shift 2
      ;;
    --c-fixed-run)
      need_value "$@"
      C_FIXED_RUN="$2"
      shift 2
      ;;
    --d-design-dir)
      need_value "$@"
      D_DESIGN_DIR="$2"
      shift 2
      ;;
    --d-supported-run)
      need_value "$@"
      D_SUPPORTED_RUN="$2"
      shift 2
      ;;
    --model)
      need_value "$@"
      MODEL_PATH="$2"
      shift 2
      ;;
    --vcl06-gate-run)
      need_value "$@"
      VCL06_GATE_RUN="$2"
      shift 2
      ;;
    --python)
      need_value "$@"
      PYTHON_BIN="$2"
      shift 2
      ;;
    --force)
      FORCE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

for command_name in awk find install mktemp realpath sha256sum sort xargs zip unzip; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "required command is unavailable: ${command_name}" >&2
    exit 2
  fi
done
if [ ! -x "${PYTHON_BIN}" ]; then
  echo "python is not executable: ${PYTHON_BIN}" >&2
  exit 2
fi
if ! "${PYTHON_BIN}" -c 'import yaml' >/dev/null 2>&1; then
  echo "python cannot import PyYAML: ${PYTHON_BIN}" >&2
  exit 2
fi

if [ -n "$(git -C "${PROJECT_ROOT}" status --porcelain --untracked-files=no)" ]; then
  echo "tracked worktree changes exist; commit them before building a handoff package" >&2
  exit 2
fi
SOURCE_REVISION="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"

case "${OUTPUT}" in
  *.zip) ;;
  *)
    echo "output must end in .zip: ${OUTPUT}" >&2
    exit 2
    ;;
esac
OUTPUT="$(realpath -m -- "${OUTPUT}")"
OUTPUT_DIR="${OUTPUT%/*}"
OUTPUT_BASENAME="${OUTPUT##*/}"
BUNDLE_NAME="${OUTPUT_BASENAME%.zip}"
if [[ ! "${BUNDLE_NAME}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "bundle root must be ASCII-safe: ${BUNDLE_NAME}" >&2
  exit 2
fi
if [ "${FORCE}" != "1" ] && { [ -e "${OUTPUT}" ] || [ -e "${OUTPUT}.sha256" ]; }; then
  echo "output already exists; pass --force to replace it: ${OUTPUT}" >&2
  exit 2
fi

REQUIRED_ARTIFACTS=(
  manifest.json
  frames.csv
  events.csv
  summary.json
  report.md
  vision_search_performance.csv
)
D_DESIGN_ARTIFACTS=(
  d50_manifest.json
  d50_trials.csv
  d50_trajectory_samples.csv
  d50_coverage.json
  d50_association_contracts.json
  summary.json
)

require_file() {
  if [ ! -s "$1" ]; then
    echo "required non-empty file is missing: $1" >&2
    exit 2
  fi
}

run_missing_artifacts() {
  local source_run="$1"
  local artifact
  local missing=()
  if [ ! -d "${source_run}" ]; then
    printf '%s\n' "run_directory"
    return 0
  fi
  for artifact in "${REQUIRED_ARTIFACTS[@]}"; do
    if [ ! -s "${source_run}/vsim04/${artifact}" ]; then
      missing+=("${artifact}")
    fi
  done
  if [ ! -s "${source_run}/manifest.yaml" ]; then
    missing+=("outer_manifest.yaml")
  fi
  if [ "${#missing[@]}" -gt 0 ]; then
    printf '%s\n' "${missing[@]}"
  fi
}

run_is_complete() {
  [ -z "$(run_missing_artifacts "$1")" ]
}

validate_run_json() {
  "${PYTHON_BIN}" - "$1/vsim04/manifest.json" "$1/vsim04/summary.json" <<'PY'
import json
import sys

for path in sys.argv[1:]:
    with open(path, "r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise SystemExit("expected JSON object: " + path)
PY
}

manifest_is_valid_yaml() {
  "${PYTHON_BIN}" - "$1" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    value = yaml.safe_load(stream)
if not isinstance(value, dict):
    raise SystemExit(1)
if not value.get("scene") or not value.get("git_head"):
    raise SystemExit(1)
PY
}

copy_outer_manifest() {
  local source_run="$1"
  local destination="$2"
  local source_manifest="${source_run}/manifest.yaml"
  local legacy_sha
  if manifest_is_valid_yaml "${source_manifest}" >/dev/null 2>&1; then
    install -m 0644 "${source_manifest}" "${destination}/manifest.yaml"
    return
  fi

  install -m 0644 "${source_manifest}" "${destination}/manifest_legacy.txt"
  legacy_sha="$(sha256sum "${source_manifest}" | awk '{print $1}')"
  "${PYTHON_BIN}" - \
    "${source_manifest}" \
    "${destination}/manifest.yaml" \
    "${source_run##*/}" \
    "${legacy_sha}" <<'PY'
import json
import re
import sys

source_path, output_path, run_name, source_sha = sys.argv[1:]
with open(source_path, "r", encoding="utf-8", errors="replace") as stream:
    text = stream.read()

fields = {}
for key in ("scene", "start_time", "git_head", "require_gate", "exit_code",
            "end_time", "roslog", "timeline"):
    match = re.search(r"^{}:\s*(.*)$".format(re.escape(key)), text, re.MULTILINE)
    if match:
        value = match.group(1).strip()
        if key in ("require_gate", "exit_code"):
            try:
                value = int(value)
            except ValueError:
                pass
        fields[key] = value

wrapper = {
    "bundle_manifest_schema": 1,
    "source_run_dir": run_name,
    "source_manifest_file": "manifest_legacy.txt",
    "source_manifest_sha256": source_sha,
    "source_format_status": "legacy_invalid_yaml_launch_cmd",
    "note": (
        "The source manifest is preserved byte-for-byte as "
        "manifest_legacy.txt; this parseable JSON/YAML wrapper does not "
        "rewrite the source run."
    ),
    "source_fields": fields,
}
with open(output_path, "w", encoding="utf-8", newline="\n") as stream:
    json.dump(wrapper, stream, ensure_ascii=False, indent=2, sort_keys=True)
    stream.write("\n")
PY
}

copy_required_run() {
  local label="$1"
  local source_run="$2"
  local destination="$3"
  local missing
  local artifact

  missing="$(run_missing_artifacts "${source_run}")"
  if [ -n "${missing}" ]; then
    echo "${label} is incomplete at ${source_run}; missing:" >&2
    while IFS= read -r missing_artifact; do
      printf '  %s\n' "${missing_artifact}" >&2
    done <<< "${missing}"
    exit 2
  fi
  validate_run_json "${source_run}"
  mkdir -p "${destination}/vsim04"
  copy_outer_manifest "${source_run}" "${destination}"
  for artifact in "${REQUIRED_ARTIFACTS[@]}"; do
    install -m 0644 \
      "${source_run}/vsim04/${artifact}" \
      "${destination}/vsim04/${artifact}"
  done
  if [ -s "${source_run}/run.log" ]; then
    install -m 0644 "${source_run}/run.log" "${destination}/run.log"
  fi
  if [ -s "${source_run}/timeline.txt" ]; then
    install -m 0644 "${source_run}/timeline.txt" "${destination}/timeline.txt"
  fi
}

copy_required_file() {
  local source_path="$1"
  local destination_path="$2"
  require_file "${source_path}"
  install -D -m 0644 "${source_path}" "${destination_path}"
}

STAGE_PARENT="$(mktemp -d /tmp/liftrace_vision_nav_bundle.XXXXXX)"
cleanup() {
  case "${STAGE_PARENT:-}" in
    /tmp/liftrace_vision_nav_bundle.*)
      rm -rf -- "${STAGE_PARENT}"
      ;;
    "") ;;
    *)
      echo "refusing to remove unexpected staging path: ${STAGE_PARENT}" >&2
      ;;
  esac
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

BUNDLE_ROOT="${STAGE_PARENT}/${BUNDLE_NAME}"
mkdir -p \
  "${BUNDLE_ROOT}/00_README" \
  "${BUNDLE_ROOT}/01_docs" \
  "${BUNDLE_ROOT}/02_VSIM04" \
  "${BUNDLE_ROOT}/03_BOARD_EVIDENCE" \
  "${BUNDLE_ROOT}/05_reproduce/config" \
  "${BUNDLE_ROOT}/05_reproduce/launch" \
  "${BUNDLE_ROOT}/05_reproduce/scripts" \
  "${BUNDLE_ROOT}/05_reproduce/docs" \
  "${BUNDLE_ROOT}/06_camera" \
  "${BUNDLE_ROOT}/07_VCL06_CONTEXT" \
  "${BUNDLE_ROOT}/08_model"

printf '%s\n' "${SOURCE_REVISION}" > "${BUNDLE_ROOT}/SOURCE_REVISION.txt"

copy_required_file \
  "${PROJECT_ROOT}/docs/handoff/视觉组给导航组_HANDOFF_20260902.md" \
  "${BUNDLE_ROOT}/HANDOFF.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/handoff/original/视觉组需求.md" \
  "${BUNDLE_ROOT}/00_README/视觉组需求_导航原文.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/handoff/original/近期工作说明_2026-08-25.md" \
  "${BUNDLE_ROOT}/00_README/近期工作说明_导航原文_2026-08-25.md"
copy_required_file \
  "${PROJECT_ROOT}/VISION_2026_ROADMAP.md" \
  "${BUNDLE_ROOT}/00_README/VISION_2026_ROADMAP.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/视觉组对导航组V-SIM-04扩展需求阶段回复_20260829.md" \
  "${BUNDLE_ROOT}/01_docs/历史_视觉组对导航组V-SIM-04扩展需求阶段回复_20260829.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/V-SIM-04仿真实验数据与分支移交索引_20260831.md" \
  "${BUNDLE_ROOT}/01_docs/历史_V-SIM-04仿真实验数据与分支移交索引_20260831.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/KS2A543相机基线下ABCD重验证决策_20260904.md" \
  "${BUNDLE_ROOT}/01_docs/KS2A543相机基线下ABCD重验证决策_20260904.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/视觉工程精简与B100延迟复盘_20260904.md" \
  "${BUNDLE_ROOT}/01_docs/视觉工程精简与B100延迟复盘_20260904.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/导航组轻量策略仿真上游_20260902.md" \
  "${BUNDLE_ROOT}/01_docs/导航组轻量策略仿真上游_20260902.md"

copy_required_run "static25 A" "${STATIC25_RUN}" \
  "${BUNDLE_ROOT}/02_VSIM04/A_static25_seed11"
copy_required_run "B100 full surface" "${B100_RUN}" \
  "${BUNDLE_ROOT}/02_VSIM04/B_full100_seed11"
copy_required_run "C25 post-fix" "${C_FIXED_RUN}" \
  "${BUNDLE_ROOT}/02_VSIM04/C_full25_seed11"
copy_required_run "D50 supported slice" "${D_SUPPORTED_RUN}" \
  "${BUNDLE_ROOT}/02_VSIM04/D_supported11_seed11"

D_DESIGN_DEST="${BUNDLE_ROOT}/02_VSIM04/D_design"
mkdir -p "${D_DESIGN_DEST}"
if [ -n "${D_DESIGN_DIR}" ]; then
  for artifact in "${D_DESIGN_ARTIFACTS[@]}"; do
    copy_required_file \
      "${D_DESIGN_DIR}/${artifact}" \
      "${D_DESIGN_DEST}/${artifact}"
  done
else
  D50_PYTHONPATH="${PROJECT_ROOT}/vision_ws/src/uav_vision/src:${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/src"
  PYTHONPATH="${D50_PYTHONPATH}" "${PYTHON_BIN}" \
    "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/scripts/vsim04_d50_dry_run.py" \
    --matrix "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/config/vsim04_trajectory_d50_matrix.yaml" \
    --output-dir "${D_DESIGN_DEST}"
  for artifact in "${D_DESIGN_ARTIFACTS[@]}"; do
    require_file "${D_DESIGN_DEST}/${artifact}"
  done
fi

for config_name in \
  vsim04_trial_matrix.yaml \
  vsim04_operating_surface_matrix.yaml \
  vsim04_lateral_c25_matrix.yaml \
  vsim04_trajectory_d50_matrix.yaml; do
  copy_required_file \
    "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/config/${config_name}" \
    "${BUNDLE_ROOT}/05_reproduce/config/${config_name}"
done
copy_required_file \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/config/scenarios/vsim04_r2026_targets.yaml" \
  "${BUNDLE_ROOT}/05_reproduce/config/vsim04_r2026_targets.yaml"
for launch_name in vsim04_stability.launch vsim04_d50_single.launch; do
  copy_required_file \
    "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/launch/${launch_name}" \
    "${BUNDLE_ROOT}/05_reproduce/launch/${launch_name}"
done
for script_name in sim_run.sh run_vsim04_surface.sh run_vsim04_d50.sh; do
  copy_required_file \
    "${PROJECT_ROOT}/top_level_scripts/${script_name}" \
    "${BUNDLE_ROOT}/05_reproduce/scripts/${script_name}"
done
copy_required_file \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/README.md" \
  "${BUNDLE_ROOT}/05_reproduce/uav_vision_eval_README.md"
copy_required_file \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision_eval/docs/VSIM04_D50_TRAJECTORY_ASSOCIATION.md" \
  "${BUNDLE_ROOT}/05_reproduce/docs/VSIM04_D50_TRAJECTORY_ASSOCIATION.md"

copy_required_file \
  "${PROJECT_ROOT}/vision_ws/calibration.yaml" \
  "${BUNDLE_ROOT}/06_camera/calibration_source.yaml"
copy_required_file \
  "${PROJECT_ROOT}/vision_ws/src/camera_sdk/param/calibration_1280x720.yaml" \
  "${BUNDLE_ROOT}/06_camera/calibration_1280x720_camera_info.yaml"
copy_required_file \
  "${PROJECT_ROOT}/vision_ws/src/uav_vision/launch/board_camera_vision.launch" \
  "${BUNDLE_ROOT}/06_camera/board_camera_vision.launch"
copy_required_file \
  "${PROJECT_ROOT}/docs/ORANGEPI_CAMERA_VISION_LAB_CHECKLIST_20260902.md" \
  "${BUNDLE_ROOT}/06_camera/ORANGEPI_CAMERA_VISION_LAB_CHECKLIST_20260902.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/OrangePi板端视觉性能报告_20260902.md" \
  "${BUNDLE_ROOT}/03_BOARD_EVIDENCE/OrangePi板端视觉性能报告_20260902.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/当前问题与责任边界.md" \
  "${BUNDLE_ROOT}/07_VCL06_CONTEXT/当前问题与责任边界.md"
copy_required_file \
  "${PROJECT_ROOT}/docs/NAV_VCL06_CONTRACT.md" \
  "${BUNDLE_ROOT}/07_VCL06_CONTEXT/NAV_VCL06_CONTRACT.md"
copy_required_file \
  "${VCL06_GATE_RUN}/gate_status.json" \
  "${BUNDLE_ROOT}/07_VCL06_CONTEXT/latest_gate_status.json"
copy_required_file \
  "${VCL06_GATE_RUN}/manifest.yaml" \
  "${BUNDLE_ROOT}/07_VCL06_CONTEXT/latest_gate_manifest.yaml"

require_file "${MODEL_PATH}"
install -m 0644 "${MODEL_PATH}" "${BUNDLE_ROOT}/08_model/best.pt"

"${PYTHON_BIN}" - \
  "${STATIC25_RUN}" "${B100_RUN}" "${C_FIXED_RUN}" "${D_SUPPORTED_RUN}" \
  "${VCL06_GATE_RUN}/gate_status.json" \
  "${BUNDLE_ROOT}/evidence_index.csv" <<'PY'
import csv
import json
import os
import sys

a_run, b_run, c_run, d_run, gate_path, output = sys.argv[1:]
fields = [
    "evidence_id", "requirement", "domain", "source_run",
    "vision_revision", "navigation_revision", "profile", "seed",
    "planned_trials", "completed_trials", "measurement_status",
    "performance_status", "p_confirm", "p_selected",
    "p_confirm_visibility", "p_selected_visibility", "p_interrupt",
    "pipeline_p95_ms", "map_error_p95_m", "artifact_complete",
    "package_path", "boundary",
]

def load(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)

def run_row(evidence_id, requirement, run, package_path, boundary):
    manifest = load(os.path.join(run, "vsim04", "manifest.json"))
    summary = load(os.path.join(run, "vsim04", "summary.json"))
    metrics = summary.get("metrics", {})
    completeness = summary.get("completeness", {})
    verdict = summary.get("performance_verdict", {})
    artifact = summary.get("artifact_completeness", {})
    revisions = manifest.get("revisions", {})
    return {
        "evidence_id": evidence_id,
        "requirement": requirement,
        "domain": "GAZEBO_VISUAL_ONLY",
        "source_run": os.path.basename(run),
        "vision_revision": revisions.get("vision", ""),
        "navigation_revision": revisions.get("navigation", ""),
        "profile": manifest.get("class_profile", ""),
        "seed": manifest.get("seed", ""),
        "planned_trials": summary.get(
            "trial_count", summary.get("expected_trial_count", "")),
        "completed_trials": summary.get("completed_trial_count", ""),
        "measurement_status": completeness.get(
            "status", summary.get("status", "")),
        "performance_status": verdict.get("status", ""),
        "p_confirm": metrics.get("p_confirm", ""),
        "p_selected": metrics.get("p_selected", ""),
        "p_confirm_visibility": metrics.get("p_confirm_visibility", ""),
        "p_selected_visibility": metrics.get("p_selected_visibility", ""),
        "p_interrupt": metrics.get("p_interrupt", ""),
        "pipeline_p95_ms": metrics.get(
            "p95_confirmation_pipeline_ms", ""),
        "map_error_p95_m": metrics.get("p95_map_error_xy", ""),
        "artifact_complete": artifact.get(
            "complete", summary.get("artifact_set_complete", "")),
        "package_path": package_path,
        "boundary": boundary,
    }

rows = [
    run_row("A_STATIC25", "A", a_run,
            "02_VSIM04/A_static25_seed11",
            "single seed; P_interrupt=null"),
    run_row("B_FULL100", "B", b_run,
            "02_VSIM04/B_full100_seed11",
            "full 5x5x4 surface on one revision; diagnostic only; P_interrupt=null"),
    run_row("C_FULL25", "C", c_run,
            "02_VSIM04/C_full25_seed11",
            "full-frame and visibility denominators differ; P_interrupt=null"),
    run_row("D_SUPPORTED11", "D", d_run,
            "02_VSIM04/D_supported11_seed11",
            "only runner-supported center/quadrant single-target trials"),
]
rows.append({
    "evidence_id": "D39_NOT_RUN",
    "requirement": "D",
    "domain": "DESIGN_ONLY",
    "source_run": "vsim04_trajectory_d50_matrix.yaml",
    "profile": "r2026",
    "seed": 11,
    "planned_trials": 39,
    "completed_trials": 0,
    "measurement_status": "NOT_RUN",
    "performance_status": "NOT_RUN",
    "artifact_complete": False,
    "package_path": "02_VSIM04/D_design",
    "boundary": "20 clipped-window; 7 visibility-preflight; 2 arena; multi10 lacks second-target/H truth",
})
gate = load(gate_path)
rows.append({
    "evidence_id": "VCL06_LATEST_CONTEXT",
    "requirement": "V-CL-06",
    "domain": "JOINT_SIM",
    "source_run": os.path.basename(os.path.dirname(gate_path)),
    "navigation_revision": "919e438",
    "profile": "r2026",
    "seed": 11,
    "planned_trials": 1,
    "completed_trials": 1,
    "measurement_status": "MEASURED",
    "performance_status": gate.get("status", ""),
    "artifact_complete": True,
    "package_path": "07_VCL06_CONTEXT/latest_gate_status.json",
    "boundary": "three-delivery subchain PASS; full Gate FAIL at post-delivery route 2; top-level run manifest does not freeze both repo revisions; reason={}".format(
        gate.get("reason", "")),
})

with open(output, "w", encoding="utf-8", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    for row in rows:
        writer.writerow({key: row.get(key, "") for key in fields})
PY

cat > "${BUNDLE_ROOT}/00_README/README_FIRST.md" <<EOF
# Vision-to-navigation handoff package

Start at the package-root \`HANDOFF.md\`, then use \`evidence_index.csv\` to
locate the authoritative run. The package is derived from the navigation
group's \`视觉组需求.md\` and keeps historical/diagnostic runs separate from
current results.

\`SOURCE_REVISION.txt\` records the exact clean Git revision used to build
this archive. The builder refuses tracked worktree changes.

## Evidence mapping

- Requirement A: \`02_VSIM04/A_static25_seed11\`.
- Requirement B: \`02_VSIM04/B_full100_seed11\`, one revision, 100/100 cells.
  Historical formal23/sparse30 are intentionally excluded and must not be
  spliced into the current surface.
- Requirement C: \`02_VSIM04/C_full25_seed11\`, 25/25 current-camera trials.
- Requirement D: the frozen D50 design and completed supported 11/11
  single-target slice. The other 39 designs remain explicit \`NOT_RUN\`.
- Visual-only \`P_interrupt\` remains null unless independent navigation
  acceptance/APPROACH evidence exists; selected_target is not a substitute.

The older navigation map-profile baseline/a68925d A/B is a different test from
the visual requirement's A/B surface and is intentionally not relabelled here.

## Source paths

- static25: \`${STATIC25_RUN}\`
- B100: \`${B100_RUN}\`
- C25: \`${C_FIXED_RUN}\`
- D supported: \`${D_SUPPORTED_RUN}\`
- latest VCL06 context: \`${VCL06_GATE_RUN}\`
- model: \`${MODEL_PATH}\`

## Board evidence

The package includes the accepted OrangePi performance report, camera
calibration and launch configuration. Large annotated videos and their removed
run directories are not duplicated in this A/B/C/D update archive.

## Integrity and exclusions

Every included ROS run has the six standard files under \`vsim04/\`. Invalid
historical outer manifests are preserved byte-for-byte as
\`manifest_legacy.txt\` and accompanied by a parseable wrapper. The archive
excludes roslog, bags, screen recordings, caches and build/devel products.
\`MANIFEST.sha256\` covers every bundled file except itself.
EOF

INTERNAL_MANIFEST_TMP="${STAGE_PARENT}/MANIFEST.sha256.tmp"
(cd "${BUNDLE_ROOT}" && \
  find . -type f -print0 | \
  LC_ALL=C sort -z | xargs -0 sha256sum > "${INTERNAL_MANIFEST_TMP}")
install -m 0644 "${INTERNAL_MANIFEST_TMP}" "${BUNDLE_ROOT}/MANIFEST.sha256"
(cd "${BUNDLE_ROOT}" && sha256sum -c MANIFEST.sha256 >/dev/null)

TMP_ZIP="${STAGE_PARENT}/${OUTPUT_BASENAME}"
(cd "${STAGE_PARENT}" && zip -qr "${TMP_ZIP}" "${BUNDLE_NAME}")
unzip -tq "${TMP_ZIP}" >/dev/null

mkdir -p "${OUTPUT_DIR}"
install -m 0644 "${TMP_ZIP}" "${OUTPUT}"
(cd "${OUTPUT_DIR}" && sha256sum "${OUTPUT_BASENAME}" > "${OUTPUT_BASENAME}.sha256")

echo "VISION_NAV_SUBMISSION_BUNDLE PASS"
echo "zip: ${OUTPUT}"
echo "checksum: ${OUTPUT}.sha256"
echo "D supported: INCLUDED_11_OF_11"
echo "large annotated MP4s: excluded from this A/B/C/D update archive"
