# 旧板端4×4避障参考镜像

2026-09-20从香橙派`10.231.47.193:/home/orangepi/liftrace_r64_onboard_405bda42`读取。保留板端实际源码和配置，未用本机d55da83覆盖；来源清单见`manifest.json`。

- `source/patrol_uav_ws-patrol_planner/src/uav_mission/launch/obstacle_4x4_hardware.launch`：现场旧4×4入口。
- `source/patrol_uav_ws-patrol_planner/src/uav_mission/config/obstacle_4x4/`：航高/控制/地图及显示配置。
- `source/top_level_scripts/start_obstacle_4x4.sh`：现场旧启动脚本。
- `source/`中相关LIO、地图、规划、控制和视觉源码：用于对照现场修改，不参与当前构建。
- `site_notes/obstacle_4x4/`：原板端说明、路线图和参数展开。
- `site_notes/`其他笔记保留现场4×5红十字设计和早期坐标修复材料，不代表全部已试飞。
- `flight_parameters/`：9月19日茶几四轮和地图补录的实际启动参数，不能用launch默认值替代。

本镜像仅包含所列文本源码、配置及小型说明资产，未复制build/devel、Git元数据、权重、完整模型资产、bag或视频，不能冒充可直接独立构建的整机交付包。旧源码包含driver1历史引用，而新分支活动代码保持driver2；不要把参考镜像加入新部署的ROS_PACKAGE_PATH。原始许可证和包清单一并保留。

`CATKIN_IGNORE`将本目录排除出Catkin包发现。新的四套入口见[板端部署总览](../BOARD_DEPLOYMENT.md)。
