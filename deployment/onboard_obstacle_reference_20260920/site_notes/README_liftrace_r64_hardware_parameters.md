# liftrace R64 实际飞行参数说明

工程根目录：`/home/orangepi/liftrace_r64_onboard_405bda42`

以下按实际飞行入口 `competition_hardware.launch` 整理。参数含义基于源码和 launch 加载关系；local 高度不等于 AGL。本文仅编辑说明文档，未修改工程源码或飞控参数。


说明：以下标记针对文件配置。未确认支持热更新的项统一建议重启对应节点，不表示所有节点都支持 rosparam set 即时生效。本文按实机默认启动链分析，未逐项读取当前飞控运行值。没有把普通可调参数标成绝对不可修改。

仅修改 launch/YAML 无需编译；修改 C++、头文件、消息/服务定义通常需重新编译；Python 修改通常重启即可。PX4 固件参数不由这些文件统一设置。

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/launch/competition_hardware.launch`

- `model_path`：RKNN 模型绝对路径，视觉节点从此文件加载模型。 【可修改；无需编译；重启对应节点生效】
- `metadata_path`：模型类别和输入输出元数据文件。 【可修改；无需编译；重启对应节点生效】
- `image_topic`：视觉订阅的原始图像话题，默认 `/camera/image_raw`。 【可修改；无需编译；重启对应节点生效】
- `camera_info_topic`：相机内参话题，默认 `/camera/camera_info`。 【可修改；无需编译；重启对应节点生效】
- `start_camera`：是否由本入口启动 UVC 相机；设备侧已有相机进程时应为 `false`。 【可修改；无需编译；重启对应节点生效】
- `video_devices`：UVC 摄像头设备路径。 【可修改；无需编译；重启对应节点生效】
- `camera_capture_fps`：相机采集帧率。 【可修改；无需编译；重启对应节点生效】
- `camera_publish_rate`：相机 ROS 图像发布频率。 【可修改；无需编译；重启对应节点生效】
- `start_yolo_live_view`：是否启动视觉调试图像发布。 【可修改；无需编译；重启对应节点生效】
- `show_yolo_window`：是否打开图形调试窗口。 【可修改；无需编译；重启对应节点生效】
- `yolo_debug_topic`：调试图像话题。 【可修改；无需编译；重启对应节点生效】
- `camera_optical_frame`：相机光学坐标系 TF 名称。 【可修改；无需编译；重启对应节点生效】
- `map_frame`：LIO、FreeDOM、视觉投影共用的地图坐标系，默认 `camera_init`。 【可修改；无需编译；重启对应节点生效】
- `ground_z`：地面在 local 坐标系中的 z 值，默认 `-0.22 m`。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `runtime_config`：任务运行 YAML 文件路径。 【可修改；无需编译；重启对应节点生效】
- `control_config`：控制、投递、降落 YAML 文件路径。 【可修改；无需编译；重启对应节点生效】
- `lio_config`：实机 MID360/FAST-LIO YAML 文件路径。 【可修改；无需编译；重启对应节点生效】
- `body_to_imu_xyz`：机体原点到 IMU 的安装平移，单位 m。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `imu_to_camera_z`：IMU 到相机的 z 向安装平移，单位 m。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `camera_quat_xyzw`：IMU 到相机光学坐标系的旋转四元数。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `planner_max_vel`：实机规划最大线速度，默认 `1.0 m/s`。 【可修改此 launch 内的固定值；不是顶层可传入的同名 arg；无需编译；重启生效】
- `planner_max_acc`：实机规划最大线加速度，默认 `1.0 m/s²`。 【可修改此 launch 内的固定值；不是顶层可传入的同名 arg；无需编译；重启生效】
- `planner_obstacles_inflation`：规划器障碍物安全膨胀距离，默认 `0.30 m`。 【可修改此 launch 内的固定值；不是顶层可传入的同名 arg；无需编译；重启生效】
- `planner_clearance_threshold`：轨迹与障碍物的最小间隙，默认 `0.025 m`。 【可修改此 launch 内的固定值；不是顶层可传入的同名 arg；无需编译；重启生效】
- `px4_max_distance`：相对当前位置的 setpoint 最大前导距离（不是每周期位移），默认 `0.40 m`。 【可修改此 launch 内的固定值；不是顶层可传入的同名 arg；无需编译；重启生效】
- `external_planner_max_command_z`：外部规划目标允许的 local z 上限，默认 `2.30 m`。 【可修改此 launch 内的固定值；不是顶层可传入的同名 arg；无需编译；重启生效】
- `search_altitude`：搜索航线 local z 高度，默认 `1.40 m`。 【可修改此 launch 内的固定值；不是顶层可传入的同名 arg；无需编译；重启生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_horizontal_field_runtime.yaml`

- `mission.frame`：任务坐标系。 【可修改；无需编译；重启对应节点生效】
- `mission.candidate_max_age`：目标候选消息最大允许延迟。 【可修改；无需编译；重启对应节点生效】
- `mission.transform_max_age`：TF 变换最大允许延迟。 【可修改；无需编译；重启对应节点生效】
- `mission.min_streak`：目标连续检测的最少帧数。 【可修改；无需编译；重启对应节点生效】
- `mission.max_target_z`：目标投影允许的最大 z。 【可修改；无需编译；重启对应节点生效】
- `mission.max_attempts`：单目标最大尝试次数。 【可修改；无需编译；重启对应节点生效】
- `mission.retry_cooldown`：失败后再次尝试的等待时间。 【可修改；无需编译；重启对应节点生效】
- `mission.timeout`：整场任务最长时间，`600 s`。 【可修改；无需编译；重启对应节点生效】
- `mission.forced_return_at`：到达该时间后强制返航，`420 s`。 【可修改；无需编译；重启对应节点生效】
- `mission.return_land_reserve`：返航和降落预留时间，`180 s`。 【可修改；无需编译；重启对应节点生效】
- `mission.delivery_reserve_per_slot`：每个投递槽预留时间，`60 s`。 【可修改；无需编译；重启对应节点生效】
- `mission.path_factor`：按路径长度估算耗时的倍率。 【可修改；无需编译；重启对应节点生效】
- `mission.nominal_speed`：任务预算采用的标称速度，`0.5 m/s`。 【可修改；无需编译；重启对应节点生效】
- `mission.decision_guard`：继续任务/返航决策的时间余量，`15 s`。 【可修改；无需编译；重启对应节点生效】
- `mission.approach_altitude`：接近目标或门的 local 高度，`1.60 m`。 【可修改；无需编译；重启对应节点生效】
- `mission.return_altitude`：返航航段 local 高度，`1.40 m`。 【可修改；无需编译；重启对应节点生效】
- `mission.target_action_timeout`：单次目标动作最长等待时间，`120 s`。 【可修改；无需编译；重启对应节点生效】
- `mission.motion_action_timeout`：单次移动动作最长时间，`90 s`。 【可修改；无需编译；重启对应节点生效】
- `mission.home_xy`：起点和返航点的平面坐标。 【可修改；无需编译；重启对应节点生效】
- `mission.landing_xy`：最终降落点的平面坐标。 【可修改；无需编译；重启对应节点生效】
- `mission.landing_anchor_tolerance`：最终降落水平误差，`0.05 m`。 【可修改；无需编译；重启对应节点生效】
- `search.min_x`：搜索区域 X 最小边界，`-2.007 m`。 【可修改；无需编译；重启对应节点生效】
- `search.max_x`：搜索区域 X 最大边界，`1.993 m`。 【可修改；无需编译；重启对应节点生效】
- `search.min_y`：搜索区域 Y 最小边界，`0.273 m`。 【可修改；无需编译；重启对应节点生效】
- `search.max_y`：搜索区域 Y 最大边界，`6.273 m`。 【可修改；无需编译；重启对应节点生效】
- `search.lane_spacing`：往返搜索线间距，`0.70 m`。 【可修改；无需编译；重启对应节点生效】
- `search.altitude`：搜索航线 local 高度，`1.40 m`。 【此处被实机 launch 的 search_altitude=1.40 覆盖；应改上层入口；无需编译；重启生效】
- `readiness.pose_max_age`：允许的最大位姿数据年龄，`0.5 s`。 【可修改；无需编译；重启对应节点生效】
- `readiness.map_max_age`：允许的最大地图数据年龄，`2.0 s`。 【可修改；无需编译；重启对应节点生效】
- `readiness.stamp_future_tolerance`：消息时间戳超前容差，`0.05 s`。 【可修改；无需编译；重启对应节点生效】
- `readiness.require_map`：是否必须有有效地图才能运行。 【可修改；无需编译；重启对应节点生效】
- `runtime.tick_hz`：任务主循环频率，`10 Hz`。 【可修改；无需编译；重启对应节点生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_horizontal_control.yaml`

- `waypoints[].x/y/z/yaw`：起飞和航点的位置、航向；z 为 local 高度。 【可修改；无需编译；重启对应节点生效】
- `waypoints[].hover_time`：到达航点后的悬停时间。 【可修改；无需编译；重启对应节点生效】
- `land_height`：进入降落控制的 local 高度，`0.30 m`。 【可修改；无需编译；重启对应节点生效】
- `align_height`：目标对准/捕获高度，`1.60 m`。 【可修改；无需编译；重启对应节点生效】
- `px4_max_distance`：控制层单步位置变化上限，`0.20 m`；实机总入口覆盖为 `0.40 m`。 【此处被实机 launch 的 0.40 覆盖；应改上层 launch；无需编译；重启生效】
- `max_yaw_change`：单控制周期航向变化上限，`0.20 rad`。 【可修改；无需编译；重启对应节点生效】
- `external_landing.capture_height`：H 目标捕获高度，`1.60 m`。 【可修改；无需编译；重启对应节点生效】
- `external_landing.auto_land_height`：满足视觉条件后触发降落的高度，`0.40 m`。 【可修改；无需编译；重启对应节点生效】
- `external_landing.alignment_tolerance`：目标水平对准容差，`0.08 m`。 【可修改；无需编译；重启对应节点生效】
- `external_landing.max_mark_offset`：视觉标记最大偏移，`0.60 m`。 【可修改；无需编译；重启对应节点生效】
- `external_landing.mark_max_age_sec`：视觉标记最大年龄，`0.50 s`。 【可修改；无需编译；重启对应节点生效】
- `external_landing.stable_frames`：触发降落所需连续稳定帧数，`10`。 【可修改；无需编译；重启对应节点生效】
- `external_landing.auto_land_retry_sec`：降落命令重试间隔，`1.0 s`。 【可修改；无需编译；重启对应节点生效】
- `threshould.takeoff_threshould`：起飞到位距离阈值，`0.30 m`。 【可修改；无需编译；重启对应节点生效】
- `threshould.waypoint_threshould`：航点到位距离阈值，`0.30 m`。 【可修改；无需编译；重启对应节点生效】
- `threshould.aligning_threshould`：对准到位距离阈值，`0.20 m`。 【此处被 patrol_control_px4_sim.launch 的 aligning_threshold 默认 0.05 覆盖；应改实际加载入口；无需编译；重启生效】
- `threshould.landing_threshould`：降落位置阈值，`0.15 m`。 【可修改；无需编译；重启对应节点生效】
- `threshould.arrive_yaw_threshould`：航向到位阈值，`0.30 rad`。 【可修改；无需编译；重启对应节点生效】
- `threshould.waypoint_adjust_max_second_threshould`：航点调整最长时间，`15 s`。 【可修改；无需编译；重启对应节点生效】
- `drop_system.height_threshold`：允许投递的高度门限，`0.20 m`。 【可修改；无需编译；重启对应节点生效】
- `drop_system.position_threshold`：允许投递的水平位置误差，`0.15 m`。 【可修改；无需编译；重启对应节点生效】
- `drop_system.release_setpoint_height`：投递 setpoint local 高度，`0.10 m`。 【可修改；无需编译；重启对应节点生效】
- `drop_system.descent_stable_duration`：投递前稳定保持时间，`2.0 s`。 【可修改；无需编译；重启对应节点生效】
- `uav_vision.recovery_height`：投递后恢复交接的 local 高度门限（不是全部回升动作的目标高度），`0.95 m`。 【可修改；无需编译；重启对应节点生效】
- `uav_vision.max_movement_distance`：视觉对准单次最大移动距离，`0.5 m`。 【可修改；无需编译；重启对应节点生效】
- `uav_vision.landing_pad_radius_m`：降落垫有效半径，`0.30 m`。 【可修改；无需编译；重启对应节点生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/config/horizontal_planner.yaml`

- `fsm/allow_goal_adjustment`：是否允许规划器调整目标点。 【可修改；无需编译；重启对应节点生效】
- `fsm/goal_adjustment_radius`：目标最大调整半径，`0.15 m`。 【可修改；无需编译；重启对应节点生效】
- `fsm/tracking_replan_distance`：偏离目标后触发重规划的距离，`0.45 m`。 【可修改；无需编译；重启对应节点生效】
- `fsm/min_replan_interval`：重规划最小间隔，`0.75 s`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/virtual_ceil_height`：SDF 虚拟天花板高度，`2.30 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/obstacles_inflation_up`：障碍物向上膨胀距离，`0.10 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/obstacles_inflation_down`：障碍物向下膨胀距离，`0.30 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/horizontal_avoidance/min_x/max_x`：水平避障 X 范围，`[-4.8,4.8] m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/horizontal_avoidance/min_y/max_y`：水平避障 Y 范围，`[-0.5,7.6] m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/horizontal_avoidance/obstacle_min_z`：参与避障的最低障碍 z，`0.18 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/horizontal_avoidance/floor_z`：规划器地板 z，`-0.12 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/resolution`：SDF 体素边长，`0.05 m`；越小越精细、内存越大。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/map_size_x`：SDF 地图 X 尺寸，`12.0 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/map_size_y`：SDF 地图 Y 尺寸，`22.0 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/map_size_z`：SDF 地图 Z 尺寸，`3.0 m`。 【可修改；无需编译；重启对应节点生效】
- `sdf_map/visualization_rate`：地图可视化刷新频率，`5 Hz`。 【可修改；无需编译；重启对应节点生效】
- `search/max_search_time`：单次搜索最长计算时间，`0.25 s`。 【可修改；无需编译；重启对应节点生效】
- `search/min_horizon_progress`：局部路径最小前进量，`0.20 m`。 【可修改；无需编译；重启对应节点生效】
- `search/direct_shot_distance`：允许直接连线到目标的距离，`2.0 m`。 【可修改；无需编译；重启对应节点生效】
- `manager/control_points_distance`：B-spline 控制点间距，`0.20 m`。 【可修改；无需编译；重启对应节点生效】
- `optimization/dist0`：优化基础距离尺度，`0.10 m`。 【可修改；无需编译；重启对应节点生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/config/competition_freedom.yaml`

- `sensor.max_range`：FreeDOM 接受的最大点云距离，`15 m`。 【可修改；无需编译；重启对应节点生效】
- `sensor.min_z/max_z`：接受点云的 z 范围，`[-3,5] m`。 【可修改；无需编译；重启对应节点生效】
- `map.voxel_depth`：体素层级深度；结合 sub_voxel_size 决定体素尺寸，增大深度会使自由空间体素更粗，实机为 `0`。 【可修改；无需编译；重启对应节点生效】
- `map.raycast_max_range`：空间射线最大距离，`15 m`。 【可修改；无需编译；重启对应节点生效】
- `map.raycast_min_z/max_z`：射线更新 z 范围，`[-3,5] m`。 【可修改；无需编译；重启对应节点生效】
- `raycast_enhancement.lidar_vertical_fov_upper_degree`：雷达垂直视场上边界，`52°`。 【可修改；无需编译；重启对应节点生效】
- `raycast_enhancement.lidar_vertical_fov_lower_degree`：雷达垂直视场下边界，`-7°`。 【可修改；无需编译；重启对应节点生效】
- `raycast_enhancement.max_raycast_enhancement_range`：射线增强最大距离，`12 m`。 【可修改；无需编译；重启对应节点生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/config/mid360_hardware.yaml`

- `common.lid_topic`：实机 LiDAR 输入话题 `/livox/lidar`。 【可修改；无需编译；重启对应节点生效】
- `common.imu_topic`：实机 IMU 输入话题 `/livox/imu`。 【可修改；无需编译；重启对应节点生效】
- `common.time_sync_en`：是否启用 FAST-LIO 内部时间偏移补偿逻辑（不是开启硬件同步），实机为 false。 【可修改；无需编译；重启对应节点生效】
- `common.time_offset_lidar_to_imu`：LiDAR 相对 IMU 时间偏移，`0.0 s`。 【可修改；无需编译；重启对应节点生效】
- `preprocess.lidar_type`：雷达类型，`1` 表示 Livox。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `preprocess.scan_line`：雷达线数配置，MID360 为 `4`。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `preprocess.blind`：近距离盲区半径，`0.5 m`。 【可修改；无需编译；重启对应节点生效】
- `mapping.acc_cov/gyr_cov`：加速度计/陀螺仪测量噪声协方差。 【可修改；无需编译；重启对应节点生效】
- `mapping.b_acc_cov/b_gyr_cov`：加速度计/陀螺仪偏置噪声协方差。 【可修改；无需编译；重启对应节点生效】
- `mapping.fov_degree`：水平视场角，`360°`。 【可修改；无需编译；重启对应节点生效】
- `mapping.det_range`：LIO 最大检测距离，`100 m`。 【可修改；无需编译；重启对应节点生效】
- `mapping.extrinsic_est_en`：是否在线估计 LiDAR-IMU 外参，实机为 false。 【可修改；无需编译；重启对应节点生效】
- `mapping.extrinsic_T`：LiDAR 到 IMU 平移外参，`[-0.011,-0.02329,0.04412] m`。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `mapping.extrinsic_R`：LiDAR 到 IMU 旋转矩阵。 【可按设备或标定结果修改，不可任意填写；无需编译；重启生效】
- `publish.map_publish_en`：是否发布内部 LIO 地图，实机为 true。 【可修改；无需编译；重启对应节点生效】
- `publish.map_publish_hz`：内部地图发布频率，`1 Hz`。 【可修改；无需编译；重启对应节点生效】
- `publish.path_en`：是否发布 LIO 路径，实机为 false。 【可修改；无需编译；重启对应节点生效】
- `publish.scan_publish_en`：是否发布扫描点云，实机为 true。 【可修改；无需编译；重启对应节点生效】
- `publish.dense_publish_en`：是否发布稠密点云，实机为 true。 【可修改；无需编译；重启对应节点生效】
- `publish.scan_bodyframe_pub_en`：是否发布机体坐标系点云，实机为 true。 【可修改；无需编译；重启对应节点生效】
- `pcd_save.pcd_save_en`：是否保存 PCD，实机为 false。 【可修改；无需编译；重启对应节点生效】
- `pcd_save.interval`：PCD 分片帧间隔；`-1` 表示单文件，可能占用大量内存。 【可修改；无需编译；重启对应节点生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/config/release_guard.yaml`

- `evidence_timeout`：释放证据最大年龄，`0.50 s`。 【可修改；无需编译；重启对应节点生效】
- `pose_timeout`：飞机位姿最大年龄，`0.50 s`。 【可修改；无需编译；重启对应节点生效】
- `control_state_timeout`：控制状态最大年龄，`0.50 s`。 【可修改；无需编译；重启对应节点生效】
- `permission_lifetime`：单条释放许可有效期，`0.25 s`。 【可修改；无需编译；重启对应节点生效】
- `publish_rate`：许可评估频率，`20 Hz`。 【可修改；无需编译；重启对应节点生效】
- `commitment_timeout`：旧兼容入口看门狗超时，`45 s`。 【可修改；无需编译；重启对应节点生效】
- `commitment_max_drift`：承诺锁定点允许水平漂移，`0.20 m`。 【可修改；无需编译；重启对应节点生效】
- `min_release_altitude`：允许释放的 local 高度下界，`-0.05 m`。 【可修改；无需编译；重启对应节点生效】
- `max_release_altitude`：允许释放的 local 高度上界，`0.30 m`。 【可修改；无需编译；重启对应节点生效】
- `payload_slots`：载荷槽数量，`3`。 【可修改；无需编译；重启对应节点生效】
- `first_payload_slot`：第一个使用的槽编号，`1`。 【可修改；无需编译；重启对应节点生效】
- `required_control_state`：允许释放的控制状态枚举值，`2`。 【必须匹配源码状态枚举，不可任意改编号；只改 YAML 无需编译，修改 C++ 枚举需编译】
- `pose_topic`：释放许可使用的位姿话题 `/mavros/local_position/pose`。 【可修改；无需编译；重启对应节点生效】
- `permission_topic`：释放许可输出话题 `/mission/release_permission`。 【可修改；无需编译；重启对应节点生效】
- `result_topic`：释放结果话题 `/mission/release_result`。 【可修改；无需编译；重启对应节点生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/vision_ws/src/uav_vision/config/target_detector_rknn.yaml`

- `conf_threshold`：目标检测置信度阈值，`0.5`。 【可修改；无需编译；重启对应节点生效】
- `iou_threshold`：NMS 的 IoU 阈值，`0.45`。 【可修改；无需编译；重启对应节点生效】
- `imgsz`：模型输入图像边长，`640` 像素。 【必须匹配 RKNN 模型约定，不可任意更改；参数修改无需 catkin 编译，改变模型输入可能需重新导出/转换模型；重启生效】
- `input_layout`：模型输入布局，`NHWC`。 【必须匹配 RKNN 模型约定，不可任意更改；参数修改无需 catkin 编译，改变模型输入可能需重新导出/转换模型；重启生效】
- `input_color_space`：模型输入颜色空间，`RGB`。 【必须匹配 RKNN 模型约定，不可任意更改；参数修改无需 catkin 编译，改变模型输入可能需重新导出/转换模型；重启生效】
- `input_dtype`：模型输入数据类型，`float32`。 【必须匹配 RKNN 模型约定，不可任意更改；参数修改无需 catkin 编译，改变模型输入可能需重新导出/转换模型；重启生效】
- `input_normalize`：是否对输入图像归一化，实机为 true。 【必须匹配 RKNN 模型约定，不可任意更改；参数修改无需 catkin 编译，改变模型输入可能需重新导出/转换模型；重启生效】
- `perf_topic`：推理性能统计话题 `/uav_vision/perf`。 【可修改；无需编译；重启对应节点生效】

## 参数文件：`/home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/src/uav_mission/config/aircraft_measurements_20260908.yaml`

- `camera_ground_clearance_when_landed`：落地时相机离地高度，`0.06 m`。 【测量记录，可按实测修订；无需编译；单改此文件不能改变实际安装或保证飞行参数生效】
- `aircraft_length_width_height`：机体长宽高，`[0.50,0.50,0.37] m`。 【测量记录，可按实测修订；无需编译；单改此文件不能改变实际安装或保证飞行参数生效】
- `fc_height_when_landed`：落地时飞控高度，`0.22 m`。 【测量记录，可按实测修订；无需编译；单改此文件不能改变实际安装或保证飞行参数生效】
- `lidar_imu_height_when_landed`：落地时 LiDAR/IMU 高度，`0.27 m`。 【测量记录，可按实测修订；无需编译；单改此文件不能改变实际安装或保证飞行参数生效】
- `conservative_length_width_height`：保守碰撞包络，`[0.55,0.55,0.40] m`。 【测量记录，可按实测修订；无需编译；单改此文件不能改变实际安装或保证飞行参数生效】
- `ground_z_in_landed_fc_frame`：落地 FC 坐标系中的地面 z，`-0.22 m`。 【测量记录，可按实测修订；无需编译；单改此文件不能改变实际安装或保证飞行参数生效】

## 实机覆盖规则

- `competition_hardware.launch` 的 `<param>` 会覆盖部分 YAML 值，实机应优先检查该文件。
- 实机 FAST-LIO 过滤覆盖值为 `point_filter_num=3`、`max_iteration=3`、`filter_size_surf=0.15`、`filter_size_map=0.15`、`cube_side_length=1000.0`。
- 实机规划覆盖值为 `planner_max_vel=1.0`、`planner_max_acc=1.0`、`planner_obstacles_inflation=0.30`、`planner_clearance_threshold=0.025`、`external_planner_max_command_z=2.30`。
- local 高度、`ground_z` 和 AGL 高度不是同一个量；调高度必须同时检查 runtime、control、launch 和地面零点。


