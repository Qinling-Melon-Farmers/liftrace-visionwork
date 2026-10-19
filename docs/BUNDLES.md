# 本地机载与仿真包

2026-09-11 驱动迁移：当前源码统一使用 **livox_ros_driver2 + Livox SDK2**；仿真仍由 Gazebo 发布 PointCloud2。两类源码包均需 SDK2 才能编译完整导航工作区，旧版本压缩包不会自动更新。[构建与实机接线说明](deployment/LIVOX_DRIVER2.md)。


R64最新：seed11完整37/37 PASS，未实机验收；必须同步PX4补丁。R64新包替代旧R62运行源码，十seed原始7/10完整PASS，5/7/8有墙重叠布设，修复后的新布设未重跑矩阵。见[本轮报告](verification/r64_seed11/REPORT.md)和[固件依赖](../deployment/px4_patches/README.md)。下文较早状态按历史记录阅读。

R62当前源码已完成seed11建图、自动起飞、搜索、panzer第1槽投递确认并恢复搜索；完整比赛/实机未验收。两类包从精简分支干净提交导出，文件名liftrace_r62_onboard_<版本>.tar.gz与liftrace_r62_simulation_<版本>.tar.gz，位于本地deliverables。BUNDLE_MANIFEST记录精确源码与权重来源。

机载包：视觉/相机、导航/任务/控制、文档与RKNN权重；不含机械PWM或Gazebo运行包，需板端已有ROS/SDK/NPU/MAVROS等依赖。新起飞XY保持补丁包含在源码中，仍需实机验收。仿真包：整机源码、新地图/机架/依赖模型、外围简单几何、YOLO权重与报告；ROS/Gazebo/PX4及Livox插件环境另配。

[机载说明](../deployment/README_ONBOARD.md) · [仿真说明](../deployment/README_SIMULATION.md) · [本轮运行记录](verification/r62_operational/REPORT.md)。新的包只承诺这个阶段记录，不称稳定三投或整场PASS。旧fd147c72包保留作历史，不应继续当作当前仿真版本。

```bash
python top_level_scripts/build_competition_bundles.py --label r62 --onboard-rknn /path/to/merged_standard_fp32.rknn --sitl-weights /path/to/weights/best.pt --optional-model-root /path/to/PX4/sitl_gazebo-classic/models
```

使用已有Python环境；本机conda rl_drone。打包不启动ROS/飞控，不复制build/devel、凭据、bag或整场录屏。逐文件读取校验路径及压缩完整性，归档不进入git。相机视频另保存在本轮run目录，见报告索引。

完整集中分析与合法性标记见[11轮报告](verification/r64_matrix/REPORT.md)，新随机化工具范围见[说明](verification/r64_randomization/README.md)。旧包及旧验收按生成时的源码阅读，最新交付索引在deliverables/README.md。
