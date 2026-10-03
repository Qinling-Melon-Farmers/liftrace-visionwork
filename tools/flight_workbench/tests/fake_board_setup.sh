#!/usr/bin/env bash
# 生成纸板工程里可再生的部分（模块入口、占位文件），供本机自检使用。
# 用法：bash tests/fake_board_setup.sh
set -euo pipefail
fake="$(cd "${BASH_SOURCE[0]%/*}/fake_board" && pwd)"

folders="01_visual_interrupt 02_high_view_revisit 03_h_landing 04_corridor_landing 05_low_multi
        06_high_priority 07_memory_only 08_full_mission 09_high_speed_capture"

mkdir -p "$fake/runtime_models" "$fake/vision_ws/src/uav_vision/config" "$fake/logs"
: >"$fake/runtime_models/fake.rknn"
: >"$fake/vision_ws/src/uav_vision/config/fake_metadata.yaml"
: >"$fake/deployment/site_20260928/test_area.yaml"
: >"$fake/deployment/site_20260928/h_landing_test_area.yaml"

for folder in $folders; do
  dir="$fake/deployment/board_trials_4x4/$folder"
  mkdir -p "$dir"
  for entry in start start_real; do
    cat >"$dir/$entry.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
script_dir="\$(cd "\${BASH_SOURCE[0]%/*}" && pwd)"
mode="\${1:-preview}"
exec bash "\$script_dir/../../../emit_transcript.sh" "$folder" "\$mode"
EOF
    chmod +x "$dir/$entry.sh"
  done
  : >"$dir/settings.yaml"
done

chmod +x "$fake/emit_transcript.sh" "$fake/deployment/site_20260928/start_test.sh"
echo "fake board ready: $fake"
