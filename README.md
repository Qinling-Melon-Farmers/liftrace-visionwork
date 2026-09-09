# 2026无人机竞赛主集成工程

**R64固定seed11完整37/37 PASS，任务422.712秒。** 三投、三恢复、9航点、两门、H对准、落地解除武装，零碰撞。最终中心距H中心6.5cm，保守55cm包络在名义黑圈内；未采用空中停机。[报告与视频索引](docs/verification/r64_seed11/REPORT.md)。随后十seed已完成，原始7/10完整PASS；5/7/8有靶板压墙，已修布设检查并保留原结果。近地落地仍有seed3失败，不能称实机已鲁棒。[11轮图表与分析](docs/verification/r64_matrix/REPORT.md)。

当前包含今年导航、视觉、任务、控制与仿真，机械组PWM实现另供。R63修复投后恢复/旧轨迹接管；R64修复PX4自动任务历史EKF重置重复应用，[固件补丁](deployment/px4_patches/README.md)是复现依赖。本主集成分支保留原始资产及保护快照；无旧参考的精简年度工程位于feat/r2026-competition-integrated。两者当前飞行核心同步。

场内9.6m内净、四组树箱、两处左右错列0.80m通口，外围简单几何补充点云。相机FC下16cm、IMU下21cm、落地镜头离支撑面6cm；保守包络55×55×40cm。相机刚性随完整机体姿态，无云台。

当前24点固定覆盖路线，搜索范围X[-4.3,4.3]/Y[0,7.1]，名义FC AGL1.4m；不是在线自适应覆盖。本批early_return_enabled=false，420秒提前返航禁用、600秒保留；硬件默认另按现场选配。

```bash
bash top_level_scripts/build_competition.sh
# 先按deployment/px4_patches说明构建对应PX4；只在明确授权后运行。
UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_STORAGE_GUARD_PATH=/mnt/f SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 bash top_level_scripts/run_competition_sim.sh field_seed:=11 record_camera_video:=true record_overview_video:=true
```

不启动Gazebo GUI也可录制服务端俯视相机和机载相机。视频/大日志留本地logs，默认0bag；坐标时序以CSV为准。[部署包](deployment/README_ONBOARD.md)、[仿真包](deployment/README_SIMULATION.md)、[任务](VISION_2026_ROADMAP.md)、[验收](docs/VALIDATION.md)、[环境](docs/ENVIRONMENT.md)、[规则](docs/competition/RULES_20260906.md)。

历史：[R60矩阵2/10](docs/verification/r60_full_matrix/REPORT.md)、[R62恢复碰靶](docs/verification/r62_full_seed11/REPORT.md)、[R63降落失败](docs/verification/r63_recovery/REPORT.md)。历史结果保持其源码/世界边界，不代替当前验收。main仍为R56历史基线，后续合入依照实跑和分支流程。

随机世界工具见[使用说明](docs/verification/r64_randomization/README.md)和simulation_tools；默认成功路线不变，随机门实际飞行尚未验收。
