#!/usr/bin/env bash
set -uo pipefail

# Laptop-only visual regression suite. It starts Gazebo static cameras and the
# dev/sim PyTorch detector; it never starts PX4, MAVROS, patrol_control,
# arming, actuator_pwm, or a release mechanism.

script_dir="${BASH_SOURCE[0]%/*}"
duration_sec="${1:-22}"
suite_dir="${2:-/tmp/uav_vision_eval/suite_$(date +%Y%m%d_%H%M%S)}"
gate_profile="${3:-smoke}"
if [[ "${gate_profile}" != "smoke" && "${gate_profile}" != "formal" ]]; then
  echo "gate_profile must be smoke or formal" >&2
  exit 64
fi
mkdir -p "${suite_dir}"

scenarios=(
  standard_pillbox
  standard_bridge
  standard_tank
  standard_tent
  standard_panzer
  red_cross
  landing_h
  background
)

summary="${suite_dir}/suite_summary.tsv"
printf 'scenario\tresult\treport\n' > "${summary}"
failed=0
for scenario in "${scenarios[@]}"; do
  output_dir="${suite_dir}/${scenario}"
  if "${script_dir}/run_toudi3_visual_eval.sh" \
      "${scenario}" "${duration_sec}" "${output_dir}" "${gate_profile}"; then
    result=PASS
  else
    result=FAIL
    failed=1
  fi
  printf '%s\t%s\t%s\n' \
    "${scenario}" "${result}" "${output_dir}/report.md" >> "${summary}"
done

echo "suite artifacts: ${suite_dir}"
exit "${failed}"
