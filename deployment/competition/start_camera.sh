#!/usr/bin/env bash
# 正赛与试飞共用已标定相机入口；只在操作者点击启动后运行。
# 不依赖 board_trials 目录，不改变曝光、分辨率或内参。
set -eo pipefail
SCRIPT_DIR="${BASH_SOURCE[0]%/*}"
source "$SCRIPT_DIR/environment.sh"
exec roslaunch camera_sdk camera_calibrated_1280x720.launch "video_devices:=${1:-/dev/video0}"
