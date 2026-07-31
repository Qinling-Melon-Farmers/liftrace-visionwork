#!/usr/bin/env bash
# Fixed-seed L1 visual-only matrix. No PX4, MAVROS, patrol_control or actuator.
set -uo pipefail

script_dir="${BASH_SOURCE[0]%/*}"
project_root="${script_dir%/*}"
duration_sec="${1:-18}"
output_dir="${2:-/tmp/uav_vision_eval/fixed_30seed_$(date +%Y%m%d_%H%M%S)}"
matrix_file="${3:-${project_root}/vision_ws/src/uav_vision_eval/config/fixed_30_seed_matrix.tsv}"
seed_selector="${4:-all}"
port_offset="${EVAL_PORT_OFFSET:-0}"

if [[ ! "${duration_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "duration_sec must be a positive integer" >&2
  exit 64
fi
if [[ ! "${port_offset}" =~ ^[0-9]+$ ]]; then
  echo "EVAL_PORT_OFFSET must be a non-negative integer" >&2
  exit 64
fi
if [[ ! -f "${matrix_file}" ]]; then
  echo "matrix file missing: ${matrix_file}" >&2
  exit 66
fi
mkdir -p "${output_dir}"
run_table="${output_dir}/matrix_runs.tsv"
printf 'seed\tscenario\tresult\treport\n' > "${run_table}"

failed=0
while IFS=$'\t' read -r seed scenario camera_x camera_y camera_z camera_yaw gate_profile; do
  [[ -z "${seed}" || "${seed}" == \#* || "${seed}" == "seed" ]] && continue
  if [[ "${seed_selector}" != "all" && ",${seed_selector}," != *",${seed},"* ]]; then
    continue
  fi
  seed_dir="${output_dir}/seed_$(printf '%02d' "${seed}")_${scenario}"
  if EVAL_ROS_MASTER_URI="http://127.0.0.1:$((11330 + port_offset + seed))" \
      EVAL_GAZEBO_MASTER_URI="http://127.0.0.1:$((11400 + port_offset + seed))" \
      "${script_dir}/run_toudi3_visual_eval.sh" \
      "${scenario}" "${duration_sec}" "${seed_dir}" "${gate_profile}" \
      "${camera_x}" "${camera_y}" "${camera_z}" "${camera_yaw}" "${seed}"; then
    result=PASS
  else
    result=FAIL
    failed=1
  fi
  printf '%s\t%s\t%s\t%s\n' \
    "${seed}" "${scenario}" "${result}" "${seed_dir}/report.md" >> "${run_table}"
done < "${matrix_file}"

set +e
/usr/bin/python3 \
  "${project_root}/vision_ws/src/uav_vision_eval/scripts/summarize_seed_matrix.py" \
  --matrix "${matrix_file}" --run-dir "${output_dir}"
summary_status=$?
set -e
if [[ "${summary_status}" -ne 0 ]]; then
  failed=1
fi

echo "30-seed artifacts: ${output_dir}"
exit "${failed}"
