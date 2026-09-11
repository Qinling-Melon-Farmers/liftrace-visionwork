# 2026无人机竞赛整机工程

2026-09-12四轮局部入口SITL对照已完成：seed32/34搜索时间分别增加6.2%/9.7%，航程增加3.1%/2.6%，当前入口策略继续默认关闭。8次入口中7次正常完成、1次被目标优先中断；seed34执行组37/37 PASS，其余3组在H区接近/降落碰撞。观察器整节点约0.50–0.55核，仍高于端侧目标；四份PX4日志无损压缩节省约292MiB。[完整报告、航迹、高度及时间分解](docs/verification/local_entry_sitl_20260912/REPORT.md)。正赛86e382d、CURRENT c369e6f8保持。

以下为各阶段历史记录，最新研究结论以上方报告和ROADMAP为准。

2026-09-12局部入口执行原型：研究开关启用后，建议可经原任务/桥接链生成单个入口目标；入口完成不推进名义游标，失败/超时恢复原目标且保留deadline，高权重投递优先。默认关闭，391项相关离线测试与两套构建通过，尚无新SITL收益。另清理95MiB可重建数组，保留原始对照数据。[报告、流程图与存储记录](docs/verification/local_entry_execution_20260912/REPORT.md)。

2026-09-12局部航段研究：已接通观测记忆→有界候选→原规划器膨胀栅格只读预检→建议输出。两个离线夹具验证“北侧已看则优先南侧、南侧阻塞则不强行推荐”；95组历史候选缺同刻SDF，未标成可达或节时。默认关闭，未改飞行目标。[报告与图表](docs/verification/local_search_proposals_20260912/REPORT.md)。

2026-09-12搜索研究继续推进端侧资源准备：完整点云分块、缓存字节上限、CPU软预算、32候选查询上限已实现；252份历史样本两环境零状态差异，ROS库核心P95 30.33→24.63ms。仅离线验证，未新增SITL或替换部署。[资源报告与图表](docs/verification/coverage_resources_20260911/REPORT.md)。

> **搜索效率研究分支：历史布局验证与后续只读模块已完成，禁止用于正赛部署。** 3轮完整SITL均37/37通过，另保留1轮诊断中止。seed32同图0.70→0.80m任务时间523.659→461.978s；seed34候选未表现出普适节时。见[完整研究报告](docs/verification/coverage_stage2_20260911/REPORT.md)。只读查询/任务历史账本未驱动自动改航，正赛整机86e382d和交付c369e6f8保持。纸箱仅用于树下垫高，无独立纸箱。

2026-09-11 驱动迁移：当前源码统一使用 **livox_ros_driver2 + Livox SDK2**；仿真仍由 Gazebo 发布 PointCloud2。两类源码包均需 SDK2 才能编译完整导航工作区，旧版本压缩包不会自动更新。[构建与实机接线说明](docs/deployment/LIVOX_DRIVER2.md)。


2026-09-10全随机五seed已完成：2/5完整PASS、4/5三投、3/5两门和9点投后路线；五组靶板布设合法，seed31降落碰墙、33第二门前规划失败、34搜索耗时导致600秒截尾。算法和参数冻结，未补跑。[完整报告与20张图](docs/verification/full_random_five_20260910/REPORT.md)。

**R64固定seed11完整37/37 PASS，任务422.712秒。** 三投、三恢复、9航点、两门、H对准、落地解除武装，零碰撞。最终中心距H中心6.5cm，保守55cm包络在名义黑圈内；未采用空中停机。[报告与视频索引](docs/verification/r64_seed11/REPORT.md)。随后十seed已完成，原始7/10完整PASS；5/7/8有靶板压墙，已修布设检查并保留原结果。近地落地仍有seed3失败，不能称实机已鲁棒。[11轮图表与分析](docs/verification/r64_matrix/REPORT.md)。

当前包含今年导航、视觉、任务、控制与仿真，机械组PWM实现另供。R63修复投后恢复/旧轨迹接管；R64修复PX4自动任务历史EKF重置重复应用，[固件补丁](deployment/px4_patches/README.md)是复现依赖。原始参考与旧快照保留在来源分支，精简分支不重复收录。

场内9.6m内净、四组树箱、两处左右错列0.80m通口，外围简单几何补充点云。相机FC下16cm、IMU下21cm、落地镜头离支撑面6cm；保守包络55×55×40cm。相机刚性随完整机体姿态，无云台。

当前24点固定覆盖路线，搜索范围X[-4.3,4.3]/Y[0,7.1]，名义FC AGL1.4m；不是在线自适应覆盖。本批early_return_enabled=false，420秒提前返航禁用、600秒保留；硬件默认另按现场选配。

```bash
bash top_level_scripts/build_competition.sh
# 先按deployment/px4_patches说明构建对应PX4；只在明确授权后运行。
UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_STORAGE_GUARD_PATH=/mnt/f SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 bash top_level_scripts/run_competition_sim.sh field_seed:=11 gui:=false rviz:=false record_camera_video:=false record_overview_video:=false
```

不启动Gazebo GUI也可录制服务端俯视相机和机载相机。视频/大日志留本地logs，默认0bag；坐标时序以CSV为准。[部署包](deployment/README_ONBOARD.md)、[仿真包](deployment/README_SIMULATION.md)、[任务](VISION_2026_ROADMAP.md)、[验收](docs/VALIDATION.md)、[环境](docs/ENVIRONMENT.md)、[规则](docs/competition/RULES_20260906.md)。

历史：[R60矩阵2/10](docs/verification/r60_full_matrix/REPORT.md)、[R62恢复碰靶](docs/verification/r62_full_seed11/REPORT.md)、[R63降落失败](docs/verification/r63_recovery/REPORT.md)。历史结果保持其源码/世界边界，不代替当前验收。远端main已保留分支合入R64默认验收基线，标签gate/r64-seed11-full；仍保留矩阵失败及未验证范围。

随机世界工具见[使用说明](docs/verification/r64_randomization/README.md)和simulation_tools；默认成功路线不变，随机门首次五seed已完成2/5整场PASS，尚未全组合鲁棒验收。

当前[R64全图/核心/仅飞行rqt拓扑](docs/topology/r64/README.md)已按PASS运行注册快照离线更新。[时间优化、轻量仓复用与辅助相机计划](docs/planning/r64_time_camera/PLAN.md)附10种策略×11布局几何比较；这些优化尚未接入飞行，安全返航仍关闭。


2026-09-09最新记录策略：后续无头运行只留日志/关键数据，关闭机载录像、俯视录像、桌面录屏和全场bag，在线机载图像仍供视觉算法使用。已有R64验收录像保留，未删除。
