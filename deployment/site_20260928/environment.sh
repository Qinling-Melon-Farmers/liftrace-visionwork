#!/usr/bin/env bash
# Source this file; it only selects the deployed workspace, never starts nodes.
BOARD_TRIAL_ROOT="$(cd "${BASH_SOURCE[0]%/*}/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$BOARD_TRIAL_ROOT/vision_ws/devel/setup.bash"
source "$BOARD_TRIAL_ROOT/patrol_uav_ws-patrol_planner/devel/setup.bash" --extend
export UAV_VISION_RKNN_MODEL_PATH="$BOARD_TRIAL_ROOT/runtime_models/flight_5cls_new_video_v8_20261011/flight_5cls_new_video_v8_20261011_fp16.rknn"
export UAV_VISION_RKNN_METADATA_PATH="$BOARD_TRIAL_ROOT/vision_ws/src/uav_vision/config/flight_5cls_new_video_v8_20261011_metadata.yaml"

# All ROS nodes run on this board; WiFi address changes must not break node XMLRPC.
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
