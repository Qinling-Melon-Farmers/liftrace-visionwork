# 投递像素偏差改用飞控测高尺度（2026-10-03）

标准圆环投递和红十字投递的横向修正改为相机离地高度与内参换算。识别到内环时，不再把该内环当成外径 1 m 的圆环来放大位移。

## 来源与范围

- 参考：导航仓 `sakelier/liftrace-controlwork` 的 `板载代码`，revision `d1fe025a8fb48d33e6db89b287053daa9c036bfb`。
- 参考分支的测高换算只作用于红十字，本次将其应用到当前标准圆环与红十字投递，两轴分别使用 `fx/fy`。
- 实现先落在导航仓 `feat/drop-height-pixel-scale`，基于 `板端参考分支` 的 `e22492b1a7d5094af66412ea6214af7e33acb846`，再以最小补丁同步视觉仓。没有整包覆盖导航或视觉代码。
- 导航来源补丁提交：`b9af5d09e3491dfb432b2e837da908ee513ab93b`。
- 修改前完整旧控制包快照在导航来源分支的 `legacy_baseline/20261003_drop_height_scale/`，含文件清单和规则要求的一次性 SHA256；精简集成分支不重复纳入快照。

## 换算与接线

```text
h_cam = camera_pose_in_mission_frame.z - ground_z
image_x_m = dx_px * h_cam / fx
image_y_m = dy_px * h_cam / fy
body_xy = 原 pixel_to_body_matrix × [image_x_m, image_y_m]
world_xy = 原 yaw 旋转(body_xy)
```

相机位姿来自现有 `/mavros/local_position/pose` → `vision_body` → 相机 TF，使用图像观测时间查询，包含现有安装平移。当前机架相机比飞控参考点低 0.16 m，不能直接把原始飞控 z 当成相机离地高度。地面高度沿用本轮启动测得的静止飞控 z 减去 0.22 m 支撑高度，与现有视觉投影共用同一基准。

例如取 `fx=725 px`、相机离地 `0.44 m`、横向误差 `40 px`：真实修正为约 `2.43 cm`，识别圆环半径是 30、100 或 250 px 都得到同一结果。此处是算例，不是新增实飞测量。

新增参数均位于 `uav_vision`：

| 参数 | 当前正式/试飞配置 | 用途 |
| --- | --- | --- |
| `drop_metric_scale_enabled` | `true` | 对标准圆环和红十字启用测高尺度 |
| `drop_camera_info_topic` | `/camera/camera_info`，由入口传入 | 标定内参 |
| `drop_camera_frame` | `downward_camera_optical_frame` | 标定与 TF 必须一致 |
| `drop_map_frame` | `camera_init`，由入口传入 | 高度所属坐标系 |
| `drop_ground_z` | 由试飞配置生成器和 launch 传入 | 本轮地面基准 |
| `drop_tf_max_age_sec` | `0.20` | 拒绝返回时间不匹配的 TF |

三个 VCL06 正式控制配置启用新尺度；未配置此开关的旧入口默认保留原行为。试飞配置生成器给每组写入本轮地面高度与相机话题，`application.launch` 支持实机/仿真相机话题覆盖。正式实机入口和搜索投递仿真入口同步传入光学 frame、地图 frame、地面高度与 CameraInfo 话题。

缺少有效 CameraInfo、观测时间或相应 TF，或相机离地高度非有限/不大于 5 cm 时，不更新这帧横向对准点，并使对应 mark 无效。有效数据恢复后继续接收；启用新尺度的投递路径不回退到圆环尺寸。标定是静态内参，不要求 CameraInfo 每帧重新发布。

以下保持各分支原样：目标识别与确认门槛（当前视觉试飞分支的圆环门槛为 0.75）、检测输出的实际 `radius_px`、`drop_ready`、任务层释放许可、Servo 代理、下降/释放高度、槽位、方向映射、yaw 旋转、最大横移步长及 H 降落路径。

## 验证

- 当前试飞工作树实际构建 `patrol_control` 和 `patrol_control-drop-action-test` 通过；只出现已有 CMake/PCL 配置警告。
- 23 项 Python 回归通过：7 项新增测试直接编译执行生产 C++ 三个方法（带 UBSan），覆盖半径独立性、离地高度、地面原点、不同 `fx/fy`、非对角方向映射、yaw、步长限制、缺失/无效标定和 TF、恢复及旧入口/H 行为；其余检查外部对准保持与原配置/释放许可接线。
- 11 项已有 DropAction C++ 测试通过，包含任务层许可与 Servo ACK 语义。
- 试飞应用实机、试飞应用仿真、正式实机三种 launch 的离线参数展开通过，实际覆盖自定义地面高度、地图 frame、相机 frame 与 CameraInfo 话题。没有启动节点或执行飞行动作。

## 尚未验证

本补丁没有部署到飞机，也没有重跑 SITL 或实飞。实飞投递精度待部署后的对比验证。

测高仍依赖飞控位姿。已发现的 EKF 高度重置会影响 `h_cam`，此补丁不修复该问题；姿态偏差、大倾角与相机标定误差也仍影响平面近似。飞控/雷达坐标和高度跳变诊断继续以既有报告为准。
