# 2026 无人机竞赛整机工程

**R60先导完整Gate PASS 37/37；冻结版本十seed仅2/10完整通过、8/10完成三投。当前不能称为稳定通关，未合入main。** main保留R56历史验收31d0b2a；所有仿真已收尾，不追加轮次。

[完整报告与PASS飞行记录](docs/verification/r60_full_matrix/REPORT.md) · [十seed失败分析](docs/verification/r60_full_matrix/FAILURE_ANALYSIS.md) · [R56后修订说明](docs/verification/r60_full_matrix/REVISION_NOTES.md) · [rqt三版Nodes only图](docs/verification/r60_full_matrix/topology/index.html) · [成功记录Release](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/releases/tag/sim/r2026-r60-low-corridor-pass)

本精简分支包含今年实际使用的导航、视觉、任务、控制和仿真代码，不含机械组PWM实现或原始参考工作区；保留Servo服务定义和仿真mock。原始资产与旧包快照位于来源开发分支。

| 目录 | 用途 |
|---|---|
| vision_ws/src/uav_vision | 检测、几何精修、目标记忆、投递对齐 |
| vision_ws/src/uav_vision_eval | 仿真场景、独立评测与记录 |
| patrol_uav_ws-patrol_planner/src | LIO、FreeDOM、Planner、任务和控制 |
| top_level_scripts | 构建、统一仿真启动/收尾、记录图形工具 |

Ubuntu20.04/ROS Noetic/Gazebo Classic，Windows侧命令须用`wsl -e bash -c '...'`。获明确仿真启动授权后使用：

```bash
top_level_scripts/build_competition.sh
SIM_STORAGE_GUARD_PATH=/mnt/f UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 top_level_scripts/run_competition_sim.sh field_seed:=11
```

当前SITL默认是R60实验配置：搜索1.40m、投递取景1.60m，三投后廊外下降，再按0.45m走廊/0.50m H取景运行。它有真实完整PASS，但十seed结果不支持比赛级鲁棒性。硬件入口仍保留此前配置，未自动切成此候选；本轮未执行实机动作。

所有日志留在WSL项目`logs/`；默认不录全场bag/录屏，保留TXT/JSON/CSV、参数、故障局部地图及PX4 ULog。包装器单实例运行，成功、失败和中断均收尾。

[环境](docs/ENVIRONMENT.md) · [相机与参数](docs/CAMERA_AND_FLIGHT.md) · [接口](docs/INTERFACES.md) · [实机待验收](docs/HARDWARE.md) · [源码版本](docs/SOURCE_REVISIONS.md) · [任务优先级](VISION_2026_ROADMAP.md)

历史R56和R57/0.85m完整PASS继续保留，不能直接视作本次1.50m总宽/严格0.80m低空场景验收。[R4x至R56完整历史](docs/verification/r56_final/REPORT.md)。`docs/verification/rXX`中的旧报告按该轮日期/场景解释，当前状态以本页及ROADMAP为准。
