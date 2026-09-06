# 实机接线与应用入口

已确认 MID360 正装在飞机顶部，相机在雷达 IMU 正下方 21 cm。`competition_hardware.launch` 以 `mapping_imu → downward_camera_optical_frame` 的 -0.21 m 表达这项实测关系；相机转角单独配置。机体原点至 IMU 的默认 0.12412 m 来自仿真机架，实机应按飞控位置原点测量 `body_to_imu_xyz`，不要把这个默认值当作用户已测量的尺寸。

实机应用入口包含 LIO、FreeDOM、Fast-Planner、任务管理、RKNN 六分类视觉、圆环/红十字/H 精修、对准投递、H 自主降落。机械组提供 Servo 服务实现，本仓库只保留消息定义、释放代理及仿真 mock。输入设备通过自己的驱动启动，应用入口不设置串口/IP，也不更改飞控持久参数。

| 设备/来源 | 应用输入 | 说明 |
|---|---|---|
| MID360 驱动 | `/livox/lidar`、`/livox/imu` | 实机使用 CustomMsg 和雷达自身 IMU；仓库内 livox_ros_driver 消息包供 FAST_LIO 编译。MID360 设备驱动需使用机载已适配驱动，不能把旧 Livox SDK1 示例当作 MID360 启动配置 |
| PX4 + MAVROS | `/mavros/local_position/pose`、`odom`、`state`、`extended_state` | 飞控保留自己的 IMU 和气压计；LIO 提供标准外部视觉位置输入 |
| 已标定 USB 相机 | `/camera/image_raw`、`/camera/camera_info` | `camera_sdk/launch/camera_calibrated_1280x720.launch`，设备路径可配置，CameraInfo 与图像分辨率一致 |
| 机械组 | `/legacy/Servo_raw` | 类型 `patrol_control/Servo`；应用公开 `/Servo` 经现有释放许可代理转发 |

板端应用命令（仅记录操作方法，本次未执行实机启动）：

```bash
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash
roslaunch uav_mission competition_hardware.launch model_path:=/path/to/merged_standard_fp32.rknn
```

入口等待现有 `/navigation/start_mission` 服务触发。飞行前由现场人员完成飞控、遥控接管和输入链检查后启动任务；没有自动解锁节点。外部任务自主降落使用已有 `switch/auto_land` 开关，H 对齐后交给 PX4 AUTO.LAND，由 MAVROS 落地/解锁状态确认结束。

本机 SITL 的 PX4 外部视觉配置为水平位置+航向（EV_CTRL=9）、气压高度参考（HGT_REF=0）、BARO_CTRL=1、GPS_CTRL=0。这些值针对本机 PX4 版本；部署时按实机固件配置并重启确认。`EKF2_BARO_NOISE=0.15` 仅依据 Gazebo 1 Pa 噪声设置，不是实机气压计标定结果。

LiDAR 内部 IMU→扫描坐标外参与 IMU→相机安装外参是两件事。`mid360_hardware.yaml` 保留原 MID360 工程内部外参，使用 `/livox/imu`；Gazebo ray frame 使用独立仿真配置，不能相互覆盖。板端 ROS 实时链、实机噪声与现场飞行尚未验收。
