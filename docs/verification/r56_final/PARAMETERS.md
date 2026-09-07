# R56 完整 PASS 的参数与阈值

实际飞行源 cc899f2。本次完整 Gate 37/37 PASS。完整组合以 [运行时 ROS 参数全文](rosparams.yaml) 为准；当前表只提供关键数值索引。PX4 八项值在本次重跑 armed 后读取，全部 success=true。

| 组别 | 参数 |
|---|---|
| 搜索 | z 1.40 m，lane .70 m，x[-2.007,1.993]、y[.273,6.273] |
| 投递 | approach/align 1.60 m，静态/动态槽位补偿全 0；当前 mock Servo |
| 地图 | .05 m，12×22×3 m，origin[-6,-11,-.2]，6336000 格；XY .30、up .10、down .30 m |
| 规划 | max velocity 1.0 m/s，位置前视 .40 m，单次搜索预算 .25 s，净空 .025 m，曲线检查 .02 s |
| 任务 | 总 600 s，强制返程 420 s，返程预留 180 s，每槽 60 s，预算速度 .5 m/s，路径因子 1.5，余量 15 s |
| 事务 | motion 90 s、单靶 120 s、最多 2 次、冷却 20 s |
| 证据 | candidate/TF age .5 s、连续 3 帧；ready pose .5 s、map 2 s、future .05 s |
| 释放 | 高度开始 .20 m，位置 .15 m，释放 z .10 m，稳定 2 s，精定位 20 px；许可状态 .20 s；arbiter evidence/pose/control 各 .50 s |
| H 几何 | 半径 15—300 px、椭圆比≥.85、填充≥.70、轮廓点≥15；内部比 .78，S≤90，V≤110 |
| H 内部结构 | 面积 .10—.70、宽高比≥.55、solidity .25—.80、凹陷≥2、深度 .04、中心 .35 |
| H 对齐 | capture 1.60 m、XY .08 m、稳定 10 帧、age .50 s、anchor 最大位移 .60 m |
| H 下降 | 对准后锁存并下降；AUTO.LAND .40 m、land target .30 m、重试 1 s；原控制器 watchdog 120 s 仍存在 |
| Gate | 启动墙钟 180 s、整轮墙钟 2700 s、任务 ROS 600 s；门航点 .18 m、H .35 m |
| 边界 | 当前命令和地图顶界 2.30 m；比赛高度上限 4 m；Gazebo 原点 z 不等于护圈最高点 |
| CameraInfo | 1280×720，fx=fy=725.3510059644434，cx=631.6718631370258、cy=397.5663813311627 |
| 相机差异 | 实机标定 fy≈723.340，与 Gazebo fy 约 .28% 差异；D=[.00586686,.0179105,-.00100641,.00147156,-.0264851] |
| 安装 | body→IMU z=.12412，body→camera z=-.08588，IMU→camera (0,0,-.21) m |
| 雷达 | 正装机顶；body→ray (.011,.02329,.13)，IMU→ray (.011,.02329,.00588) m |
| PX4 | EV_CTRL9,HGT_REF0,GPS_CTRL0,BARO_CTRL1,BARO_NOISE.15,EV_DELAY0,EVP_NOISE.10,EVA_NOISE.05 |
| 记录 | 项目 logs 内 run；rosbag LZ4，单卷 1024 MB；SIM_STORAGE_GUARD_PATH=/mnt/f，启动至少 20 GiB |

## 11 个投后航点（camera_init，米）

```yaml
route:
  - [-2.386703, 4.672270, 1.40]
  - [-2.386703, 4.672270, 1.05]
  - [-2.386703, 5.172270, 0.75]
  - [-2.386703, 5.672270, 0.75]
  - [-2.386703, 6.672270, 0.75]
  - [-2.386703, 7.652690, 1.20]
  - [-0.512003, 8.053133, 1.20]
  - [ 0.287997, 8.053133, 1.20]
  - [ 1.723022, 8.009650, 1.20]
  - [ 2.523022, 8.009650, 1.20]
  - [ 3.274292, 8.057220, 1.60]
```

Wall15：y=6.07227、x[-2.786588,-1.986588]、z[.5,1.3]、第 5 段；Wall20：x=-.112003、y[7.60269,8.50269]、z[0,2]、第 8 段；Wall22：x=2.123022、y[7.55969,8.45969]、z[0,2]、第 10 段；H=(3.274292,8.057220)。

## 实际 PX4 读回

```json
{
  "EKF2_EV_CTRL": {
    "success": true,
    "integer": 9,
    "real": 0.0
  },
  "EKF2_HGT_REF": {
    "success": true,
    "integer": 0,
    "real": 0.0
  },
  "EKF2_GPS_CTRL": {
    "success": true,
    "integer": 0,
    "real": 0.0
  },
  "EKF2_BARO_CTRL": {
    "success": true,
    "integer": 1,
    "real": 0.0
  },
  "EKF2_BARO_NOISE": {
    "success": true,
    "integer": 0,
    "real": 0.15000000596046448
  },
  "EKF2_EV_DELAY": {
    "success": true,
    "integer": 0,
    "real": 0.0
  },
  "EKF2_EVP_NOISE": {
    "success": true,
    "integer": 0,
    "real": 0.10000000149011612
  },
  "EKF2_EVA_NOISE": {
    "success": true,
    "integer": 0,
    "real": 0.05000000074505806
  }
}
```

## 源配置全文

### vcl06_horizontal_control.yaml

```yaml
# External Mission Manager owns all search/target/return waypoints.
# The single takeoff seed below initializes the existing executor.
goal_list: ["bridge", "panzer", "pillbox", "tent", "tank"]

waypoints:
  - {x: 0.0, y: 0.0, z: 1.4, yaw: 0.0, pointmode: "Takeoff_point", hover_time: 0.0}

waypoint_skipping_index: 0
land_height: 0.30
align_height: 1.60
px4_max_distance: 0.20
max_yaw_change: 0.20

# 外部 Mission Manager 的 H 降落配置。
# 外部任务由 switch/auto_land=true 启用 AUTO.LAND；
# H 证据必须来自 camera_init、晚于 LAND 命令并持续满足新鲜度与稳定帧数。
external_landing:
  frame: camera_init
  # 正式链直接消费视觉 typed map detection；旧 PoseStamped 仅留 legacy 模式。
  detections_topic: /uav_vision/detections_mapped
  # 1 m H + installed camera: acquire the complete ring before descent.
  capture_height: 1.60
  auto_land_height: 0.40
  alignment_tolerance: 0.08
  max_mark_offset: 0.60
  mark_max_age_sec: 0.50
  stable_frames: 10
  auto_land_retry_sec: 1.0
  # 本项只处理控制进程失联；LAND 业务阶段只服从任务开始后 600 s
  # 的全场时限，不再另设“重捕”事务截止。
  watchdog_timeout_sec: 120.0

# The formal chain descends after drop_ready through patrol_control. The
# legacy alignment_control_converter is absent; its progressive_descent
# parameters do not control this run. The launch uses a 0.40 m position lead.

alignment:
  threshold: 20.0
  stable_count_threshold: 10
  timeout: 30.0

threshould:
  takeoff_threshould: 0.30
  waypoint_threshould: 0.30
  aligning_threshould: 0.20
  landing_threshould: 0.15
  arrive_yaw_threshould: 0.30
  times_detect_threshould: 40
  waypoint_adjust_max_second_threshould: 15
  land_adjust_max_second_threshould: 10
  planner_min_pub_threshould: 0.025

switch:
  flag_planner_px4: 0
  flag_landing_detect: 1
  # patrol_control reads this parameter as bool; use a YAML boolean so ROS
  # does not reject the XmlRpc integer and silently fall back to false.
  auto_land: true

drop_system:
  precision_threshold: 20.0
  height_threshold: 0.20
  position_threshold: 0.15
  release_setpoint_height: 0.10
  enable_drop: true
  descent_stable_duration: 2.0
  slot_offsets: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
  dynamic_slot_offsets: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]

uav_vision:
  update_goal_from_selected_target: false
  require_release_permission: true
  release_permission_state_topic: /mission/release_permission_active
  release_permission_timeout: 0.20
  recovery_height: 0.95
  selected_target_timeout: 1.0
  drop_offset_timeout: 1.0
  pixel_to_meter_ratio: 0.0015
  # 新机架安装方向：图像向右=-body_y，图像向下=-body_x。
  pixel_to_body_matrix: [0.0, -1.0, -1.0, 0.0]
  max_movement_distance: 0.5
  drop_circle_radius_m: 0.5
  drop_cross_radius_m: 0.5
  landing_pad_radius_m: 0.30
  enable_tank_interrupt: false

```

### vcl06_horizontal_field_runtime.yaml

```yaml
# VCL06 runtime for r2026 horizontal field.  This file changes only geometry and
# timing inputs; queue, retry and payload-slot behavior remains in MissionCore.

profile:
  name: r2026

mission:
  frame: camera_init
  candidate_max_age: 0.5
  transform_max_age: 0.5
  min_streak: 3
  max_target_z: 4.0
  max_attempts: 2
  retry_cooldown: 20.0
  timeout: 600.0
  # Reserve three-door transit plus H alignment inside the 10 min limit.
  forced_return_at: 420.0
  return_land_reserve: 180.0
  delivery_reserve_per_slot: 60.0
  path_factor: 1.5
  nominal_speed: 0.5
  decision_guard: 15.0
  approach_altitude: 1.60
  return_altitude: 1.40
  # r17 的 KS2A543 实跑中，复杂地图接近 + 低空视觉对准在 90 s 截止后约
  # 1 s 才满足释放高度；给完整目标事务保留适度调度余量，但不放宽证据或 Gate。
  target_action_timeout: 120.0
  # r33 的首个走廊航段已使用 55.8 s；r37b 从第三靶离场时因实体靶体
  # 绕障先爬升后下降，60 s 时仍在持续收敛。统一留 90 s 运动 lease，
  # 但继续服从 420 s 强制返航与 600 s 整场硬限。
  motion_action_timeout: 90.0
  # LAND 是最终阶段，只服从从任务开始计的 600 s 全场时限；不再额外
  # 设置会把正常 H 对齐/下降误判成“需要重捕”的短事务超时。
  result_future_tolerance: 0.1
  home_xy: [0.0, 0.0]
  # All points are camera_init coordinates derived from r2026 horizontal field.
  # The route first crosses the low Wall_15 opening south-to-north, then the
  # Wall_20 and Wall_22 scoring doors west-to-east, and ends above the final H.
  post_delivery_route_revision: r2026-horizontal-three-door-h-r3-h-capture
  landing_xy: [3.274292, 8.057220]
  landing_anchor_tolerance: 0.05
  post_delivery_route:
    # Adapt the successful centreline/brackets to the 1.4 m search profile.
    # Lower outside the wall inflation, then refresh the doorway view before
    # asking the planner to enter the narrow opening. All points use the same
    # planner and normal arrival criteria; no dwell or alternate command path.
    - [-2.386703, 4.672270, 1.40]  # south staging, horizontal tree avoidance
    - [-2.386703, 4.672270, 1.05]  # lower while clear of the entrance
    - [-2.386703, 5.172270, 0.75]  # low observation/centreline alignment
    - [-2.386703, 5.672270, 0.75]  # proven south bracket, new guard height
    - [-2.386703, 6.672270, 0.75]  # clear another 0.2 m before climbing
    - [-2.386703, 7.652690, 1.20]  # north corridor entry
    - [-0.512003, 8.053133, 1.20]  # Wall_20 west approach
    - [ 0.287997, 8.053133, 1.20]  # Wall_20 east clear
    - [ 1.723022, 8.009650, 1.20]  # Wall_22 west approach
    - [ 2.523022, 8.009650, 1.20]  # Wall_22 east clear
    - [ 3.274292, 8.057220, 1.60]  # final landing_h_clone capture

# Read-only Gate geometry.  route_indices are one-based and arm each crossing
# only for the center/clear legs, so repositioning from the third target does
# not count as an early or reverse door traversal.
post_delivery_gate:
  goal_tolerance: 0.18
  final_h_tolerance: 0.35
  doors:
    - name: Wall_15
      axis: y
      coordinate: 6.072270
      direction: positive
      route_indices: [5]
      lateral_min: -2.786588
      lateral_max: -1.986588
      z_min: 0.50
      z_max: 1.30
    - name: Wall_20
      axis: x
      coordinate: -0.112003
      direction: positive
      route_indices: [8]
      lateral_min: 7.602690
      lateral_max: 8.502690
      z_min: 0.00
      z_max: 2.00
    - name: Wall_22
      axis: x
      coordinate: 2.123022
      direction: positive
      route_indices: [10]
      lateral_min: 7.559690
      lateral_max: 8.459690
      z_min: 0.00
      z_max: 2.00

# Bounds match the static random-target spawn envelope declared in
# coverage_toudi3_random.yaml.  They are scenario geometry, not per-seed truth,
# and avoid spending most of the mission outside the only legal target region.
search:
  min_x: -2.007
  max_x: 1.993
  min_y: 0.273
  max_y: 6.273
  lane_spacing: 0.70
  altitude: 1.40
  route_revision: r2026-installed-camera-070m-lanes-r1
  max_failures_per_waypoint: 2

readiness:
  pose_max_age: 0.5
  map_max_age: 2.0
  # Full-chain load can make a producer stamp lead /clock by a few milliseconds.
  # This is clock-jitter tolerance, not extra allowance for stale telemetry.
  stamp_future_tolerance: 0.05
  require_map: true

runtime:
  tick_hz: 10.0
  mission_id_prefix: vcl06-random

```

## PX4 启动前配置脚本

```sh
#!/bin/sh
. /home/xhj/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.params
param set EKF2_BARO_CTRL 1
param set EKF2_BARO_NOISE 0.15
param set EKF2_EVA_NOISE 0.05
param set EKF2_EVP_NOISE 0.1
param set EKF2_EV_CTRL 9
param set EKF2_EV_DELAY 0.0
param set EKF2_GPS_CTRL 0
param set EKF2_HGT_REF 0

```

## 新竞赛 FreeDOM profile（本轮实际加载）

```yaml
# Competition field is 10 x 10 x 4 m. These overrides are shared by SITL
# and hardware; raw LiDAR/LIO ranges are unchanged.
sensor:
  max_range: 15.0              # m; covers the field diagonal
  min_z: -3.0                 # m relative to the sensor
  max_z: 5.0
map:
  # sub_voxel_size remains 0.10 m. Free-space voxels become 0.10 m rather
  # than 0.40 m; a 26-neighbour free test then spans 0.30 m, not 1.20 m.
  voxel_depth: 0
  raycast_max_range: 15.0
  raycast_min_z: -3.0
  raycast_max_z: 5.0
raycast_enhancement:
  # Match the recorded MID360 sector; half-bin padding covers scan noise.
  lidar_vertical_fov_upper_degree: 52.0
  lidar_vertical_fov_lower_degree: -7.0
  # The existing 8-bit depth image now has ~0.047 m bins, instead of 0.314 m.
  max_raycast_enhancement_range: 12.0
# counts_to_free=6, counts_to_revert=20 and the connectivity remain unchanged.

```

## 完整规划配置

```yaml
fsm/allow_goal_adjustment: false
fsm/tracking_replan_distance: 0.45
fsm/min_replan_interval: 0.75
sdf_map/virtual_ceil_height: 2.30
# Inflate an overhead obstacle DOWN for the guard above the body; inflate
# a floor obstacle UP for the undercarriage. These are not interchangeable.
sdf_map/obstacles_inflation_up: 0.10
sdf_map/obstacles_inflation_down: 0.30
sdf_map/horizontal_avoidance/enabled: true
sdf_map/horizontal_avoidance/min_x: -2.1
sdf_map/horizontal_avoidance/max_x: 2.1
sdf_map/horizontal_avoidance/min_y: 0.2
sdf_map/horizontal_avoidance/max_y: 5.5
sdf_map/horizontal_avoidance/obstacle_min_z: 0.40
sdf_map/horizontal_avoidance/floor_z: 0.10
# 5 cm voxels preserve the 0.30 m guard envelope at the 0.80 m doorway.
# The bounded map covers the whole field: X [-6,6], Y [-11,11], Z [-0.2,2.8].
# 6,336,000 cells (~355 MB of SDF buffers); physical ceiling remains 2.30 m.
sdf_map/resolution: 0.05
sdf_map/map_size_x: 12.0
sdf_map/map_size_y: 22.0
sdf_map/map_size_z: 3.0
sdf_map/visualization_rate: 5.0
# A short goal requires a complete path. Long partial paths must make progress.
search/max_search_time: 0.25
search/min_horizon_progress: 0.20
search/direct_shot_distance: 2.0
# Keep 20 cm spline controls, with dense 20 ms collision validation.
manager/control_points_distance: 0.20
optimization/dist0: 0.10

```

接触代理的网格、z=.20 m 和传感器保留；模型插件仅排除激光射线类别。SDF .05 m 规划网格与 FreeDOM .10 m 空闲网格属于不同用途。当前源快照见 vehicle_model.sdf。
