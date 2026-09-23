查看内容话题或坐标系显示类型		
累积地图	/freedom/static_pointcloud	PointCloud2
当前扫描	/cloud_registered	PointCloud2
占据体素	/sdf_map/occupancy_inflate	PointCloud2，Style=Boxes
FAST-LIO 定位	/Odometry	Odometry
无人机机体位置和朝向	vision_body	TF
规划路径	/planning_vis/trajectory	Marker
当前规划目标	/fastplanner/goal	Pose


RViz 卡顿有明确的数据量因素：占据体素消息约含 187 万个体素中心点，全部画成 Boxes 开销很大。建议平时只显示 FreeDOM 地图、位置和路径；点云设为 Points、Decay Time=0、Queue Size=1。
新增的终端监视命令如下。先启动原有业务，在已加载项目环境的新终端运行：
roslaunch uav_mission navigation_status_monitor.launch
默认每 5 秒打印位置、航向、飞控状态、任务阶段、规划事件和地图点数。降低刷新频率：
roslaunch uav_mission navigation_status_monitor.launch interval:=10
需要额外检查占据体素点数时：
roslaunch uav_mission navigation_status_monitor.launch \
  interval:=10 sample_occupancy:=true
默认不订阅庞大的占据体素消息；其他地图也只周期性采样，不逐点打印。终端显示的是数值和状态摘要，不能替代地图形状的图形展示。

[16:52:14] navigation status
lio:          RECENT received_age=0.0s stamp_age=0.09s frame=camera_init child=body xyz=(-0.016, 0.005, 0.032)m yaw=0.1deg
fcu_pose:     RECENT received_age=0.0s stamp_age=0.02s frame=map xyz=(-0.015, 0.005, -0.018)m yaw=-0.7deg
fcu:          RECENT received_age=0.6s stamp_age=0.61s connected=True armed=False mode=STABILIZED
mission:      LAST_EVENT received_age=0.0s phase=SEARCH last_reason=search_continues manual_start_required=True
bridge:       LAST_EVENT received_age=0.0s adapter_faulted=False last_reason=executor_pending gate_reason=live_planner_output_enabled
control_ready: LAST_EVENT received_age=167.4s False
goal:         LAST_EVENT received_age=83.1s frame=camera_init xyz=(-2.007, 0.273, 1.400)m
planner:      LAST_EVENT received_age=0.1s status=FAILED_ATTEMPT goal_seq=2 distance=2.459m reason=new_trajectory_attempt_failed effective_goal=(-2.007, 0.273, 1.400)m
trajectory:   NO_MESSAGE_SINCE_MONITOR_START (may be idle/event-only)
/cloud_registered: NONEMPTY points=4923 frame=camera_init stamp_age=0.05s
/freedom/static_pointcloud: NONEMPTY points=26413 frame=camera_init stamp_age=0.00s
/Laser_map: EMPTY points=0 frame=camera_init stamp_age=0.05s
/sdf_map/occupancy_inflate publishers=['/fast_planner_node'] (registration only)
/planning_vis/trajectory publishers=['/fast_planner_node'] (registration only)