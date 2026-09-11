# MID360：统一使用 Livox ROS Driver2

2026-09-11：当前整机源码中的 FAST_LIO 已迁移到 `livox_ros_driver2/CustomMsg`，随仓驱动替换为官方 driver2，依赖 Livox SDK2。旧 driver1/SDK1 副本已从当前集成源码移除；原始 2025 基线及 Git 历史保留。

| 环境 | 点云来源与类型 | FAST_LIO 配置 |
|---|---|---|
| 实机 MID360 | driver2，`livox_ros_driver2/CustomMsg` | `lidar_type: 1`，驱动 `xfer_format: 1` |
| Gazebo 仿真 | `liblivox_laser_simulation.so`，`sensor_msgs/PointCloud2` | `lidar_type: 4` |

`lidar_type` 是预处理类型，不是驱动版本。不可为了 driver2 改成 2；2 表示 Velodyne。仿真入口不会启动硬件驱动。此次迁移不改变飞行策略、点云滤波、外参或既有 seed 结论。

## 构建

两种源码包都编译同一份 FAST_LIO 和 driver2，因此都需要 **Livox SDK2** 的头文件与库。SDK2 需在目标主机安装，板端应使用 aarch64 原生构建；不能复制笔记本 x86 库。

- 官方 SDK2：<https://github.com/Livox-SDK/Livox-SDK2>
- 驱动来源：<https://github.com/Livox-SDK/livox_ros_driver2>，版本与本地调整见驱动目录 `INTEGRATION.md`。
- 使用 `top_level_scripts/build_competition.sh`；随仓 `package.xml` 已选择 ROS1，CMake 默认 ROS1。不要在集成工作区运行上游 `build.sh`。
- 默认查找 `/usr/local/include/livox_lidar_api.h` 和 SDK2 静态库；非标准安装可通过 CMake 的 `LIVOX_LIDAR_SDK_INCLUDE_DIR`、`LIVOX_LIDAR_SDK_LIBRARY` 指定。

## 实机接线

独立传感器入口为 `uav_mission/launch/mid360_driver2.launch`，必须传入 `user_config_path`，指向按真实雷达 IP、主机网卡 IP 配置的 SDK2 JSON。随仓 `livox_ros_driver2/config/MID360_config.json` 仅作为结构样例，不能当成已经适配本机的网络配置。

入口默认发布 `/livox/lidar`（CustomMsg）和 `/livox/imu`（sensor_msgs/Imu）；可通过 `lidar_topic`、`imu_topic`、`frame_id`、`publish_freq` 调整，话题应与 `mid360_hardware.yaml` 对齐。录包关闭。`competition_hardware.launch` 仍使用外部设备输入，需先单独建立此传感器链。

更换时应停止旧驱动并重新编译订阅端，不能让 driver1/driver2 同时占用同一设备与话题。旧 CustomMsg bag 的数据类型名仍是 driver1，不能把旧包或回放结果冒充 driver2 实机验证。

本次验证限于本机编译和离线接口测试；尚未验证 MID360 网络接收、时间同步、实机定位和飞行。历史 d1ab9208 等压缩包内容不会随源码变更，交付版本以 `deliverables/CURRENT.json` 为准。

## 本轮验证结果

- `build_competition.sh`：视觉、导航工作区实际构建通过。
- 仅含 driver2/FAST_LIO 的全新工作区、只叠加 `/opt/ros/noetic`：默认 ROS1 构建通过，使用 SDK2 静态库，无 driver1 依赖。
- `catkin_make preprocess_inputs` 后执行 `ctest -R fast_lio_preprocess_inputs -V`：两条输入检查通过，验证 CustomMsg 的点坐标、反射率、纳秒到毫秒转换、无效线号/回波过滤，以及 Gazebo XYZ-only 点云与盲区过滤。
- 硬件传感器 launch 静态展开通过：缺少设备 JSON 参数时拒绝展开，单一 driver2 节点、话题重映射和关闭录包符合配置；整机应用入口继续使用外部设备输入。
- 既有任务单元/契约回归 254 项通过。

本地日志：`logs/_artifacts/driver2_migration_20260911/`。测试初版遇到旧工程强制 Debug 与 gtest 库后缀冲突，最终使用不依赖 gtest 的离线 C++ 测试程序；未为此修改生产优化或飞行策略。
