#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT=/home/xhj/liftrace
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
cd "$PROJECT_ROOT"

exec python vision_ws/scripts/augment_yolo_region_focus_dataset.py \
  --input-dataset vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713 \
  --output-dataset vision_ws/test_data/yolo_dataset_v5_region_focus_aug_20260714 \
  --variants-per-image 3 \
  --overwrite
