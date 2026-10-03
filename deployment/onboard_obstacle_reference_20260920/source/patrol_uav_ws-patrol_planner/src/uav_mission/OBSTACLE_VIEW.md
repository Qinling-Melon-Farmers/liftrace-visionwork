# 4×4 三维障碍地图与规划路径

在板端图形桌面的终端运行（SSH 需要可访问的 DISPLAY）。沿用设备驱动原启动命令，
仅追加 `rviz_enable:=false`，保留原来的 xfer_format 等设备参数。
已经有 Livox RViz 窗口时关闭该显示进程，避免多个 RViz 同时渲染。

应用仍按原来的 preview/flight 流程启动；本视图不启动驱动、应用、飞控输出或任务。

```bash
cd /home/orangepi/liftrace_r64_onboard_405bda42
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash --extend
roslaunch uav_mission obstacle_4x4_view.launch
```

蓝色方块：规划器占据图的外表面，含安全膨胀、水平避障柱和虚拟限高。
它不是原始点云，也不是物体真实外形。橙红色线：最近一次收到的最终 B 样条轨迹。
黄色箭头：`/navigation/local_pose`。网格表示 camera_init 的 Z=0，不代表实测地面高度。
显示坐标系统一为 camera_init。上下游坐标系不匹配时拒绝显示并输出错误，避免错位。

地图默认 0.10 m、1 Hz；只显示六邻域外表面，最多 12000 块，超限时依次改用
0.20、0.40 m 等显示精度。规划器仍使用原来的 0.05 m 分辨率。
轨迹最多 1000 点、0.03 m 线宽。RViz 10 FPS，不显示 PointCloud2、相机或控制点小球。
RViz 进程单独设置 LP_NUM_THREADS=2，限制 llvmpipe 软件渲染工作线程，不改变系统驱动。
这仍使用 OpenGL；如果 llvmpipe 负载较高，可使用：

```bash
roslaunch uav_mission obstacle_4x4_view.launch display_resolution:=0.20 max_blocks:=6000
```

`rviz:=false` 只启动转换节点，可在另一台已正确配置 ROS 网络的机器打开配套 view.rviz。
输出 `/navigation/obstacle_blocks` 和 `/navigation/planned_path` 均为锁存 Marker，
仅缓存转换节点启动后收到的结果；晚于规划事件启动转换节点时要等待下一次正常规划。
preview 默认不发布规划目标，所以地图可见、路径为空是正常情况。不要为显示路径切换 flight。
输入停止后保留最后画面；此视图不是数据新鲜度或飞行安全监控器。

只读排查：

```bash
rostopic info /sdf_map/occupancy_inflate
rostopic info /planning_vis/trajectory
rostopic echo -n 1 /navigation/local_pose/header
rostopic echo -n 1 /navigation/planner_bridge_status
```

构建：在已 source 两个工作空间的终端进入 patrol_uav_ws-patrol_planner，运行
`catkin_make obstacle_view_node -j2 -l2`。
单元测试：`catkin_make run_tests_uav_mission_gtest_test_obstacle_view -j2 -l2`。
隔离 ROS/图形测试脚本位于 `test/verify_obstacle_view.py`，不连接飞控控制链。
