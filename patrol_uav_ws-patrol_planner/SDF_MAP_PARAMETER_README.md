# 当前实机 SDF 地图参数说明

本文说明 `/home/orangepi/liftrace_r64_onboard_405bda42` 当前实机入口
`uav_mission/competition_hardware.launch` 使用的 SDF 地图参数来源。

## 参数来源

启动 `competition_hardware.launch` 时，规划器的参数通过以下链路加载：

```text
competition_hardware.launch
  └─ patrol_control_px4_sim.launch
       └─ plan_manage/launch/patrol_planner_px4_sim.launch
            └─ plan_manage/launch/patrol_planner_sim.xml
```

其中，`competition_hardware.launch` 第 70 行附近包含：

```xml
<rosparam ns="/fast_planner_node"
          file="$(find uav_mission)/config/horizontal_planner.yaml" />
```

因此当前实机最终参数应以以下文件为准：

```text
patrol_uav_ws-patrol_planner/src/uav_mission/config/horizontal_planner.yaml
```

代码 `sdf_map.cpp` 只负责读取这些 ROS 参数，并不决定它们的业务值。

## 当前实际值

| 参数 | 当前值 | 配置位置 |
|---|---:|---|
| SDF 分辨率 | `0.05 m` | `horizontal_planner.yaml:27` |
| 地图 X 尺寸 | `12.0 m` | `horizontal_planner.yaml:28` |
| 地图 Y 尺寸 | `12.0 m` | `horizontal_planner.yaml:29` |
| 地图 Z 尺寸 | `4.0 m` | `horizontal_planner.yaml:30` |
| 地图原点 X/Y | `-6.0 / -6.0 m` | 由 `sdf_map.cpp` 按尺寸计算 |
| 地图原点 Z | `-0.2 m` | `horizontal_planner.yaml` 未直接设置；当前 `sdf_map.cpp` 默认 `ground_height=1.0`，但 `competition_hardware.launch` 展开参数时实测为 `ground_height=-0.2` |
| 地图范围 X | `[-6.0, 6.0] m` | 尺寸 + 原点 |
| 地图范围 Y | `[-6.0, 6.0] m` | 尺寸 + 原点 |
| 地图范围 Z | `[-0.2, 3.8] m` | 尺寸 + 原点 |
| 局部更新范围 | `(8.5, 8.5, 4.5) m` | `horizontal_planner.yaml` 未设置；由 `patrol_planner_sim.xml`/默认参数提供，实机 launch 展开后为 `8.5, 8.5, 4.5` |
| 通用水平膨胀 | `0.30 m` | `horizontal_planner.yaml` 未直接设置；实机 launch 展开后为 `0.30` |
| 向上膨胀 | `0.10 m` | `horizontal_planner.yaml:11` |
| 向下膨胀 | `0.30 m` | `horizontal_planner.yaml:12` |
| 可视化截断高度 | `2.49 m` | 实机 launch 展开后的 `sdf_map/visualization_truncate_height` |

0.05 m 分辨率下，当前固定 SDF 网格理论单元数为：

```text
12 / 0.05 × 12 / 0.05 × 4 / 0.05 = 96 × 96 × 80 = 737,280 个体素
```

这里的“地图大小”是 SDF 规划网格的固定边界，不等于当前已经观测到的障碍物范围。
局部更新范围表示每次以当前位姿为中心更新的区域，超过传感器观测范围的体素不会凭空变成已知障碍。

## 两个容易混淆的文件

`patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_px4_sim.launch`
中的 `map_size_x/y/z` 默认值是 `20/20/5`，但它们是通用 include 的默认参数；在当前实机入口中，最终由 `horizontal_planner.yaml` 的 `/fast_planner_node/sdf_map/map_size_*` 覆盖，因此当前实际值是 `12/12/4`。

`patrol_uav_ws-patrol_planner/src/uav_mission/config/competition_freedom.yaml`
属于 FreeDOM 地图，不是 Fast-Planner SDF 网格。它的 `sensor.max_range=15 m`、`map.sub_voxel_size=0.10 m` 等参数决定 FreeDOM 点云地图的输入和体素处理范围，不设置 SDF 的 `map_size_x/y/z`。

## RViz 与命令行

查看 SDF 占据体素：

```text
Topic: /sdf_map/occupancy_inflate
Type: PointCloud2
Fixed Frame: camera_init
```

该消息是占据体素中心点，不是 `nav_msgs/OccupancyGrid`。膨胀后的体素表示规划器为无人机安全包络预留的不可通行区域。

低开销查看状态和消息点数：

```bash
roslaunch uav_mission navigation_status_monitor.launch
```

需要采样占据体素点数时：

```bash
roslaunch uav_mission navigation_status_monitor.launch interval:=10 sample_occupancy:=true
```

## 修改参数后的注意事项

修改 YAML 或 launch 后，必须重新启动 `competition_hardware.launch`；如果修改了 C++ 默认值或消息/节点代码，还必须重新编译工作空间并重新 `source devel/setup.bash`。建议启动后用以下命令核对最终参数：

```bash
rosparam get /fast_planner_node/sdf_map/resolution
rosparam get /fast_planner_node/sdf_map/map_size_x
rosparam get /fast_planner_node/sdf_map/map_size_y
rosparam get /fast_planner_node/sdf_map/map_size_z
rosparam get /fast_planner_node/sdf_map/local_update_range_x
rosparam get /fast_planner_node/sdf_map/local_update_range_y
rosparam get /fast_planner_node/sdf_map/local_update_range_z
rosparam get /fast_planner_node/sdf_map/obstacles_inflation
rosparam get /fast_planner_node/sdf_map/obstacles_inflation_up
rosparam get /fast_planner_node/sdf_map/obstacles_inflation_down
```

本说明只记录当前参数和查看方法，不修改任何参数、不启动任务、不解锁飞控。
