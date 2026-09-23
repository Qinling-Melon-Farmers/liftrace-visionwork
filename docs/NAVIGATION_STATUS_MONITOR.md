# 实机地图、位置与规划状态监视

先在已有终端启动 MAVROS、MID360 和 competition_hardware.launch，再打开新终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/liftrace_r64_onboard_405bda42/vision_ws/devel/setup.bash
source /home/orangepi/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/devel/setup.bash --extend
roslaunch uav_mission navigation_status_monitor.launch
```

默认每 5 秒打印一次；调低刷新频率：

```bash
roslaunch uav_mission navigation_status_monitor.launch interval:=10
```

可选采样占据体素（数据很大，默认只查询发布者注册状态）：

```bash
roslaunch uav_mission navigation_status_monitor.launch interval:=10 sample_occupancy:=true
```

脚本仅订阅和查询 ROS Master，不发布目标、不调用控制服务、不自动启动其他节点。
打印位置/航向及坐标系、飞控连接/解锁/模式、任务阶段、最近的目标与规划事件、轨迹
控制点数量，以及周期性采样得到的地图点数。不会逐点打印或反序列化 PointCloud2
点数组，但采样时仍须传输完整消息，不能称为零开销。地图无采样可能是没有新消息、
频率太低或传输过慢，不直接等同于节点退出。

`LAST_EVENT` 是历史最近事件，空闲任务不必持续发布；监视器启动前的非锁存规划事件
无法追溯，`NO_MESSAGE_SINCE_MONITOR_START` 不代表历史上从未规划。占据地图若 stamp
为零显示 `unset`，不会把它解释为巨大延迟。位置是对应 frame 下的局部位置，Z 不等于
离地高度；轨迹控制点不是精确的实际飞行路径，也不证明飞行已完成。

## RViz

Fixed Frame 使用 `camera_init`：

| 内容 | Topic/Frame | RViz 类型 |
|---|---|---|
| 当前配准扫描 | /cloud_registered | PointCloud2，Decay Time=0 |
| FreeDOM 地图 | /freedom/static_pointcloud | PointCloud2，Decay Time=0 |
| 膨胀占据体素 | /sdf_map/occupancy_inflate | PointCloud2，Boxes，Size(m)=0.05 |
| FAST-LIO 定位 | /Odometry | Odometry，Keep=1 |
| 机体位置坐标轴 | vision_body | TF，启用所需 frame |
| 规划轨迹 | /planning_vis/trajectory | Marker |
| 当前规划目标 | /fastplanner/goal | Pose |

不要启用所有重型显示项。图形显示建议先只用 FreeDOM 地图、位置和轨迹，PointCloud2
使用 Points 而非 Boxes，Queue Size=1。现有 FAST-LIO 示例配置有 Decay Time=1000 和
Queue Size=100000 的扫描累积项，会增加负担。

`/Laser_map` 已按用户要求恢复原 if(0)，目前为空；无需为本监视器启用它。
`/path` 是历史运动轨迹，硬件配置 path_en=false；不是规划路径。
`/planning/bspline` 是自定义轨迹消息，不能直接当 nav_msgs/Path 显示。
`/sdf_map/occupancy_inflate` 用体素中心点传输占据信息，不是二维 nav_msgs/OccupancyGrid。

2026-09-12 实测：FreeDOM 地图约 3.6 万点，当前扫描约 5000 点，膨胀占据体素约
187 万点。定位持续更新；任务 IDLE / waiting_for_manual_start，规划器 wait for goal，
未生成本次规划轨迹。MAVROS connected=true、armed=false、MANUAL。
camera_init 到 body、vision_body、mapping_imu 的 TF 可用，但到 map 没有连接；因此
frame=map 的 /mavros/local_position/pose 不能直接叠到 camera_init 地图中。独立查看
该 Pose 时用 Fixed Frame=map；本监视器始终明确打印 frame，不暗中混用坐标。

已在香橙派实际验证默认和 sample_occupancy=true 两种启动，按用户要求未编译。
本次未执行任何解锁、任务开始、模式切换或飞行指令。诊断结束后停止本次进程，
MID360 恢复 SDK WakeUp 模式（2），收到成功 ACK。
