#!/usr/bin/env bash
# 构建 toudi3/toudi4 及新机架的完整 Gazebo 模型 ZIP（可再生大文件，不入 git）。

set -euo pipefail

script_dir="${BASH_SOURCE[0]%/*}"
script_dir="$(cd "${script_dir}" && pwd -P)"
project_root="${PROJECT_ROOT:-${script_dir%/*}}"
px4_root="${PX4_ROOT:-/home/xhj/PX4-Autopilot}"
px4_models="${px4_root}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
package_name="toudi4_gazebo_assets_complete_20260813"
output="${project_root}/${package_name}.zip"
stage_root="$(mktemp -d "${TMPDIR:-/tmp}/liftrace_assets.XXXXXX")"
stage="${stage_root}/${package_name}"
trap 'rm -rf -- "${stage_root}"' EXIT

models=(
  big_box3 big_box4 juniper_Tree
  dibao qiaoliang tanke zhangpeng zhuangjiache
  red_cross landing_h iris gps mid360
)

mkdir -p "${stage}/models" "${stage}/world" "${stage}/route" \
  "${stage}/truth" "${stage}/launch"

for model in "${models[@]}"; do
  source_dir="${px4_models}/${model}"
  if [[ ! -d "${source_dir}" ]]; then
    echo "缺少 Gazebo 模型: ${source_dir}" >&2
    exit 1
  fi
  cp -a "${source_dir}" "${stage}/models/${model}"
done

cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/models/iris_mid360_downward_camera" \
  "${stage}/models/iris_mid360_downward_camera"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/toudi3.world" "${stage}/world/"
cp -a "${project_root}/toudi4_copy.world" "${stage}/world/"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/config/patrol_toudi3.yaml" \
  "${stage}/route/"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/config/patrol_toudi4.yaml" \
  "${stage}/route/"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/config/patrol_toudi4_new_vision.yaml" \
  "${stage}/route/"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/uav_mission/config/coverage_toudi4.yaml" \
  "${stage}/route/"
cp -a "${project_root}/vision_ws/src/uav_vision_eval/config/sim_target_catalog_toudi4.yaml" \
  "${stage}/truth/"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_world.launch" \
  "${stage}/launch/"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_full_competition_sim.launch" \
  "${stage}/launch/"
cp -a "${project_root}/patrol_uav_ws-patrol_planner/src/patrol_control/launch/toudi3_full_competition_sim_new_vision.launch" \
  "${stage}/launch/"
cp -a "${project_root}/docs/TOUDI4_GAZEBO_ASSET_PACKAGE_README.md" "${stage}/README.md"

required=(
  "models/landing_h/model.sdf"
  "models/landing_h/materials/scripts/landing_h.material"
  "models/landing_h/materials/textures/Hjiangluo.png"
  "models/iris_mid360_downward_camera/model.sdf"
  "world/toudi4_copy.world"
)
for relative in "${required[@]}"; do
  [[ -s "${stage}/${relative}" ]] || {
    echo "资产不完整: ${relative}" >&2
    exit 1
  }
done

rm -f "${output}"
(
  cd "${stage_root}"
  zip -qr "${output}" "${package_name}"
)

unzip -tq "${output}"
echo "已生成: ${output}"
du -h "${output}"
