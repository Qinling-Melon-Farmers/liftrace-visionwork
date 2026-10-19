# 2026无人机整机运行工程传承包

本包面向下一届复现飞机运行链路，按运行源码和依赖选取，不是整仓历史备份。

## 包含什么

- 导航控制：MID360驱动、FAST-LIO、FreeDOM、Fast-Planner、任务调度、patrol_control、消息/服务及依赖。
- 视觉：相机SDK/驱动与标定参数、uav_vision、uav_high_view，最终v8五分类RKNN与metadata。
- 执行机构：当前actuator_pwm实体源码、引脚/PWM配置、投递服务链。
- 启动与场地：独立正赛、各试飞组和现场YAML、模型环境脚本与构建脚本。
- 工具：Windows/WSL工作台、回放、ULog、模型导出、诊断工具。distributions内另附可直接分发的工作台ZIP。
- 文档：运行上手指南、环境依赖、接口、外参、配置说明和已知验证边界。

## 不包含什么

旧Visual/detect_ws/Desktop工程副本、legacy_baseline及部署源码快照、旧Git历史、原始bag/视频、仿真世界、大批历史报告、训练数据、PT/ONNX和个人材料。原始竞赛历史仍在源码仓库和独立备份，训练相关资产另有模型数据包。

## 接收后从这里开始

1. **docs/closeout_20261019/ONBOARD_QUICKSTART.md**：本运行包的最短上手路径及入口表。
2. **docs/closeout_20261019/NEXT_TEAM_README.md**：详细工程交接背景及排错。其历史报告引用可在远端完整仓查询，本包不携带所有引用资源。
3. **docs/closeout_20261019/VALIDATION.md**：构建、检查与未验证边界；归档不等于所有研究候选实飞通过。
4. 版本和完整成员列表见ARCHIVE_MANIFEST.json。

Linux/WSL使用tar解压，保留软链接与执行位。在目标板重新构建；本包不是预编译系统镜像，不含Ubuntu/ROS、系统SDK安装包或飞控固件。仅运行Windows工作台时，另解压distributions中的ZIP，依其README_FIRST.md安装Python依赖即可。
