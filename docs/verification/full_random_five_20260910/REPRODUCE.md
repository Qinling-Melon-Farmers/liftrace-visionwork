# 数据与重现

本次已完成，不自动续跑。飞行源为7eb9446，PX4沿用R64补丁；运行依赖、权重与模型范围见项目README和deployment/px4_patches。

## 只重画已有数据

本机已有conda rl_drone，以下脚本只读既有run产物并重写派生报告，不启动ROS或飞行：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
cd /home/xhj/liftrace-worktrees/r2026-competition-integrated
python docs/verification/full_random_five_20260910/analysis_scripts/analyze.py
python docs/verification/full_random_five_20260910/analysis_scripts/details.py
python docs/verification/full_random_five_20260910/analysis_scripts/map_diagnostics.py
python docs/verification/full_random_five_20260910/analysis_scripts/stall_plots.py
```

脚本读取logs/_artifacts/full_random_five_20260910/matrix/matrix_status.json，要求COMPLETE且恰好五组；原始日志路径见archive_manifest.json。移机分析需先提供该索引指向的原始数据并调整路径。没有视频，因此不能重建未记录的真实机载画面。

## 未来获得新授权后的SITL复现

每轮`seed_NN/scenario_inputs`保存实际使用的world、field_config、runtime、gate_geometry、控制、飞机模型、源码版本等。不是用今天重新生成的地图代替旧布局。例：

```bash
# 仅在收到新的明确仿真启动请求后使用；本段文字本身不是授权。
CASE="$PWD/docs/verification/full_random_five_20260910/seed_31/scenario_inputs"
SIM_STORAGE_GUARD_PATH=/mnt/f SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 \
UAV_VISION_MODEL_PATH=/absolute/path/to/the/same/best.pt \
bash top_level_scripts/run_competition_sim.sh \
  field_seed:=31 world:="$CASE/field.world" \
  field_config:="$CASE/field_config.yaml" \
  runtime_config:="$CASE/runtime.yaml" \
  gate_geometry_config:="$CASE/gate_geometry.yaml" \
  observe_full_trial:=true gui:=false rviz:=false \
  record_debug:=false record_camera_video:=false record_overview_video:=false
```

批次编排使用run_seed_matrix.py的可选`--scenario-index`传入这四个文件；索引格式在matrix_status.json的scenario_index字段。新输出目录必须与原始五组分开；仍须统一包装器、单实例、正常/FAIL/异常都收尾。相同seed也可能因仿真调度、传感器和规划时序产生差异，不能保证一次复現相同Gate。
