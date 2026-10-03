# 四轮对照复现入口

飞行源码冻结：`5667bed8fb83daacf9e10a9addbf5ba56bca927a`。研究分支，不用于机载部署。
本批为 seed32、seed34，各 baseline/strategy 一轮。完整运行命令及当轮环境也记录在各 run 的 `manifest.yaml`。

已有工作区须完成构建；模型权重使用本机既有文件，不纳入本报告目录。只在明确安排新仿真时执行下面的命令，统一包装器会检查单实例与存储并完成收尾。

在 PowerShell 中执行，选择 `seed` 和 `mode`：

```powershell
wsl -e bash -c '
set -e
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
seed=32
mode=strategy
export VISION_WS=$PWD/vision_ws
export UAV_WS=$PWD/patrol_uav_ws-patrol_planner
export ASTRA_MODEL_ROOT=$PWD/simulation_assets/models
export GAZEBO_MODEL_PATH=$PWD/vision_ws/src/uav_vision_eval/models:$ASTRA_MODEL_ROOT
if [ "$mode" = baseline ]; then
  pkg=uav_mission
  entry=navigation_horizontal_search_vcl06.launch
else
  pkg=uav_high_view
  entry=full_strategy.launch
fi
SIM_STORAGE_GUARD_PATH=/mnt/f SIM_NO_RECORD=1 SIM_REQUIRE_GATE=1 SIM_RUN_AUTHORIZED=1 \
timeout --signal=TERM --kill-after=30s 2850s bash top_level_scripts/sim_run.sh \
  high_view_reproduce_${mode}_seed${seed} roslaunch "$pkg" "$entry" \
  target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt \
  world:=$PWD/docs/verification/high_view_render_20260913/seed_${seed}/field.world \
  field_config:=$PWD/docs/verification/high_view_render_20260913/seed_${seed}/field_config.yaml \
  runtime_config:=$PWD/docs/verification/high_view_full_20260914/seed_${seed}/runtime.yaml \
  gate_geometry_config:=$PWD/docs/verification/high_view_full_20260914/seed_${seed}/gate_geometry.yaml \
  field_seed:=$seed
'
```

随机门的 `runtime_config` 和 `gate_geometry_config` 必须与 world 成套使用。本批沿用中性墙前后导航点，由原规划器寻找实际门洞；没有把靶真值送入飞行目标。

离线重建图表和报告（不启动仿真）：

```powershell
wsl -e bash -c '
set -e
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MPLBACKEND=Agg
export MPLCONFIGDIR=$PWD/logs/mpl_report_cache
mkdir -p "$MPLCONFIGDIR"
python docs/verification/high_view_full_20260914/analyze.py \
  --runs docs/verification/high_view_full_20260914/runs.json \
  --out docs/verification/high_view_full_20260914
python docs/verification/high_view_full_20260914/validate.py
'
```

`runs.json` 指向原始 run。绘图只读这些记录，不能补造缺失的完整任务时间。归档到其他目录后，需要先更新该文件内的路径。
