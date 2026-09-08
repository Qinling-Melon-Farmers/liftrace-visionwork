# 2026无人机竞赛整机工程

当前活动目标已更新：持续修复并验证seed11建图、起飞、搜索巡航和投递，不追求整场PASS；暂不启动矩阵。外围仅追加简单几何提供点云背景。下方R62前两次失败是历史记录，并非本目标停止。

**R62当前状态：两次seed11均在任务启动前FAIL，未解锁、0投递，未开始seed1–10。** 仿真已收尾；当前仿真模型尚不可作为可飞交付。[失败根因与视频索引](docs/verification/r62_seed11_gate/REPORT.md)。

[后续搜索/返航方案与规则原页](docs/competition/SEARCH_RETURN_PLAN_20260909.md)已登记；顺序为seed11通过→场景随机化→高度/覆盖/返航优化。当前飞行代码与420/180秒配置保持冻结。

最新复核：[R60尚未解决的问题、机头/相机行为和轻量仓更新](docs/verification/r61_layout_search/attitude_review/REVIEW.md)。先固定seed11全场基线，再开展门与树箱随机化；本次没有新的飞行PASS。

当前为R61候选：已修复外部投递丢标回旧航点，统一16/21/6cm实测安装关系，完成新9.6m地图、机架装配与seed11静态Gazebo截图。整机构建与240项任务/Gate回归通过；**没有新全场飞行PASS**。R60先导37/37 PASS、十seed2/10完整通过仍是历史结果，main保持R56验收。

[R61完整报告和地图/飞机图片](docs/verification/r61_layout_search/REPORT.md) · [seed1–11历史实际布局](docs/verification/r61_layout_search/layouts.html) · [会议规则校正](docs/competition/RULES_20260906.md) · [两份本地部署/仿真包](docs/BUNDLES.md)

本main集成来源分支保留原始工程/机械源码及保护快照，不等于精简交付包。完整打包源使用feat/r2026-competition-integrated；本分支不重新打包旧资产。机载包用于联调准备，不代表已完成实机比赛验收。

| 目录 | 用途 |
|---|---|
| vision_ws/src/uav_vision、camera_sdk | 视觉闭环与实机相机 |
| vision_ws/src/uav_vision_eval | 当前机架/world、仿真、独立评测与记录 |
| patrol_uav_ws-patrol_planner/src | LIO、地图、Planner、任务、控制 |
| simulation_assets | 当前场景依赖的12个外部模型、网格/材质及来源 |
| deployment、top_level_scripts | 两类包使用说明、构建与统一运行/收尾 |

Ubuntu20.04/ROS Noetic/Gazebo Classic；Windows侧WSL命令使用`wsl -e bash -c '...'`。编译用`bash top_level_scripts/build_competition.sh`。仿真获当轮明确授权后才执行：

```bash
UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 bash top_level_scripts/run_competition_sim.sh field_seed:=11
```

当前SITL候选按落地FC为local0，ground_z=−0.22；搜索local1.18即AGL1.40m，取景AGL1.60m，廊外下降后按走廊AGL0.45m/H0.50m引导。硬件入口保留此前任务配置，必须整体复核高度/坐标后联调，不能直接把SITL参数视为实机已验收参数。

日志统一留WSL本项目logs，不录全场bag/录屏，单实例强制收尾。本次只运行静态预览，未执行实际飞行、投递或新十seed批次。

[环境](docs/ENVIRONMENT.md) · [相机与飞行参数](docs/CAMERA_AND_FLIGHT.md) · [接口](docs/INTERFACES.md) · [实机缺口](docs/HARDWARE.md) · [版本](docs/SOURCE_REVISIONS.md) · [验收](docs/VALIDATION.md) · [任务优先级](VISION_2026_ROADMAP.md)

历史资料：[R60完整记录与十seed](docs/verification/r60_full_matrix/REPORT.md)、[失败分析](docs/verification/r60_full_matrix/FAILURE_ANALYSIS.md)、[R56后修订](docs/verification/r60_full_matrix/REVISION_NOTES.md)、[rqt三版Nodes only图](docs/verification/r60_full_matrix/topology/index.html)、[成功记录Release](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/releases/tag/sim/r2026-r60-low-corridor-pass)。R56/R57成功按当时模型与几何解释，不改写成新场景PASS。
