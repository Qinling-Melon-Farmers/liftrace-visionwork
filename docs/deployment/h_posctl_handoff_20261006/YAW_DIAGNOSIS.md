# 160434 H实飞 AUTO.LAND 目标生成诊断（ROS bag＋ULog）

## 结论

本轮直接证据更支持“无全球参考时 Navigator 的有限零经纬度目标，被 Auto 转成本地原点，导致水平回拉；初始化航向逻辑向该目标生成转向并受 45°/s 限速”这一首要根因链。不能将 62° 旋转归为直接重放一笔大航向 reset：今日保留的 delta_heading 只有 −1.13348°。历史 reset 计数初始化缺陷仍在该源码中，可能在激活时附带处理旧 reset；计数本轮不变不能单独排除它。现有日志不记录 Auto 内部锁/接受半径/未平滑目标，因此具体内部初始化路径属于源码推断，不能写成已实测读到的变量。

## 实际版本与时间

- ULog ver_sw = 30e763b6780061d70a14894e3e8b06e6a656f9b8，ver_hw = PX4_FMU_V6C，subtype V6C002001。
- 以下时刻是 FC 启动后秒数 timestamp/1e6，不是从该日志开头起算。日志约从 1525.63 s 开始。
- OFFBOARD→AUTO.LAND 在 1580.880316 s；POSCTL 在 1582.563299 s，AUTO 段约 1.682983 s。
- 与 bag 相对时间的既有模式对齐 offset = 1492.734063 s，散布约 ±0.37 s；不要用这一采样近似对齐推断毫秒级先后。下面两条 setpoint 的数值与 bag 精确吻合，支持同轮匹配。

| FC boot s | ENU 目标 X m（ULog NED y） | ENU yaw °（90−NED yaw） | ENU yawrate °/s（−NED rate） |
|---|---:|---:|---:|
| 1580.960645 | 1.9814918 | 3.583497 | 10.173843 |
| 1582.264401 | 1.1398687 | 62.259124 | 44.929568 |

ULog trajectory_setpoint 在该自动阶段的 yaw 线性拟合为 ENU +45.000053°/s；MPC_YAWRAUTO_MAX=45。后面的 62.26° 是限速转动过程中被采样的值，不是一个已知固定最终目标。

## 零 triplet 如何变成本地原点

1580.880364 s 的 position_setpoint_triplet：
current.valid=1、type=4(LAND)、lat=0、lon=0、alt=0、yaw=NaN、acceptance_radius=2 m；previous/next.valid 均为 0。

同刻最近 vehicle_local_position（1580.888538）：
NED position ≈ (−0.028479, 1.981535, −0.259460) m；
xy_global=0、z_global=0、ref_timestamp=0、ref_lat/lon/alt=NaN，但 xy_valid=z_valid=1。

精确 revision 的 [MissionBlock::set_land_item](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/navigator/mission_block.cpp#L814-L847) 直接取 global_position.lat/lon，没有“无效就设 NaN”的分支。它把 yaw 设 NaN、altitude 设 0。今日 ULog 没录 vehicle_global_position，不能读到上游原字段；但实际下游 triplet 的有限零值已明确。

[FlightTaskAuto::_evaluateGlobalReference 与 _evaluateTriplets](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp#L388-L451) 在 xy_global/z_global 无效时把 reference 纬经度/高度置 0；有限 triplet 纬经度走 project 分支，仅 NaN 纬经度才走锁当前位置分支。因此有限 (0,0,0) 的原始目标变成本地 (0,0,0)，不是切入位置 (−0.0285,1.9815)。

进入时距本地原点 1.981740 m，向原点的 NED 方位为 −89.176596°（ENU 179.176596°）。Auto land 平滑位置目标从 ENU X≈1.98 向 0 减小，已到 1.14 m，对应 y 速度指令约 −1.387265 m/s（vehicle_local_position_setpoint，含位置反馈后的速度）；同一时刻 trajectory_setpoint 的平滑 y 速度约 −1.015 m/s。这些量与“本地原点目标”一致，不需要把历史 −7.14 m 位置 delta 加给当前位置来解释。

## 航向与接受半径：初始化细节不能省略

MPC_YAW_MODE=0，源码生成向当前目标的航向；Land 初始化把这时的 _yaw_setpoint 保存为 _land_heading，再按 MPC_YAWRAUTO_MAX 限速。理论方向约 ENU 179°与实测正向转动相符。

但 triplet.acceptance_radius=2 m，实测原点距为 1.98174 m；若内部半径已正确装载为 2 m，仅靠“mode0 朝向目标”不能断言一定生成该转向。

源码还存在一个符合今日零目标条件的初始化链：[Auto 头文件](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.hpp#L110-L184) 中 _target_acceptance_radius 初值为 0，prev/next 有效记忆为 false，_triplet_target 矩阵默认值为零。默认 Vector3→Vector→[Matrix 的 _data{}](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/lib/matrix/matrix/Matrix.hpp#L19-L27) 是零初始化。首个 tmp_target=(0,0,0) 且 previous/next 均 false 时，会命中“target 未变”的比较，跳过 else 中接受半径赋值，使内部半径仍为 0。按此路径，距原点 1.98 m 大于内部 0 m，mode0 生成目标方向，再由 Land 保存。该内部字段没有记录，列为源码推断；不要误写“实测接受半径为0”。

## 历史 reset：可能附带重放，不能成为大角度直接解释

本日志全段 counter 都没有新增：
xy=61，z=8，vxy=58，vz=5，heading=5，vehicle_attitude.quat_reset_counter=5；
estimator primary_instance 恒为0，instance_changed_count 恒为5。

保留的最后一次 reset delta 全段不变：
NED delta_xy=(−11.170030,−7.141799) m；
delta_z=−0.017990 m；
delta_heading=−0.0197829 rad = −1.133477°；
delta_vxy=(0.001128,−0.000174) m/s、delta_vz=−0.011007 m/s。

[FlightModeManager::switchTask](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/FlightModeManager.cpp#L372-L410) 的无旧任务路径把 last_reset_counters 初始化为0，再交给新 Auto；[FlightTask 的 counter 比较](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/tasks/FlightTask/FlightTask.cpp#L47-L77) 因而可处理历史值。该问题仍需保留为修复候选，不能因 ULog counter 未增长就说没有重放。

不过 [Auto 的 reset handlers](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp#L719-L742) 中 XY/Z 不是把 delta 加到目标，而是把 smoothing 状态重置到当前估计位置；velocity 同样取当前速度。heading handler 才是 _yaw_sp_prev += delta。它在今日只能直接带来 −1.13° 的小偏置，无法单独产生 +62° 连续转动或约 0.84 m 的目标回拉。一次 handler 是否实际被调用未在 ULog记录；不能写“已排除任何二次影响”。

## 融合状态、磁力计与人工输入

实际 primary estimator0 在切入窗口：
cs_ev_pos=1、cs_ev_yaw=1、cs_ev_hgt=1、cs_ev_vel=0；
cs_mag_hdg=0、cs_mag_3d=0、cs_gps=0；
cs_ev_yaw_fault=0、reject_yaw=0。
cs_mag=1、cs_mag_dec=1，cs_mag_field_disturbed 在0/1间变化；因此不能写“磁力计所有逻辑都关闭”。但没有证据表明磁航向/三维磁融合切入接管 yaw。EKF2_EV_CTRL=15 是请求配置，不等于 EV velocity 运行融合为真。

MPC_LAND_RC_HELP=0、WV_EN=0、COM_OBS_AVOID=0。AUTO 窗口 manual_control_setpoint.roll/pitch≈0，yaw≈+0.0261～0.0284，valid=1、data_source=1、sticks_moving=0，低于 MPC_HOLD_DZ=0.1。偏航并非大幅打 yaw 杆；但 throttle≈−0.959，绝非“全部摇杆中立”，它在人工接管后对下降有意义。RC-help 禁用时这些杆量不通过 Auto land assistance 驱动目标。

1582.550680 s mode_slot 6→4、switch_changes41→42；1582.550692 action_request 记录 mode=2，随后1582.563299进入POSCTL。与明确的人工模式切换相符，不能写成已确认“杆量超过阈值触发自动override”。

## 产物与边界

logs/h_flight_review_20261006/analysis/ulog_window.json：切入前后完整选定字段窗口与全段 reset/flags 摘要。
logs/h_flight_review_20261006/analysis/ulog_diagnosis_summary.json：版本、零triplet、两组精确 bag 匹配值、原点方位、参数与缺失话题。

可用于当前文档写“首要根因链得到 ULog 与精确源码支持”；不能写“已经修复 AUTO”，不能仅刷历史 counter 修复就预期此零目标链自然消失。POSCTL 末端交接仍是人工落地操作策略；未修改飞控固件/参数，未完成新飞行验证。


## 数据取回与现有回放

原bag `logs/h_flight_review_20261006/board_landing_20261006_160434/flight_debug_0.bag`：85,348,842字节、101.411585秒。使用replaytool输出 `logs/h_flight_review_20261006/replay_160434/index.html`，四段相机/标注/轨迹/组合视频各101.5秒，完整解码及时长检查通过。

抽查83秒相机画面，完整H清晰且基本居中；90秒接管附近画面模糊、靶移出视野。在线有63条有效H投影覆盖约62.42–84.99秒。记录的唯一PointCloud2为规划膨胀占据云 `/sdf_map/occupancy_inflate`（134条），不是原始雷达云或LIO注册云；用户取消追加全高度点云回放，现有bag和四视频保留。

默认replaytool用FC odom显示轨迹，地面初始化第1.67秒有7.02m/13.26m两次大步，发生在READY和离地前，不能当作飞行了该距离；READY后35–96秒该显示轨迹X范围−0.065至2.035m、Y−0.093至0.077m。高度为局部Z，不能直接当AGL。运动分析应按时间范围并对照原生camera_init的LIO位姿。

现有MAVROS FTP先前只读read返回0字节；另一会话随后复用 `LOG_REQUEST_DATA` 成功完整取回日志ID36，文件 `logs/h_flight_review_20261006/ulog/h_quality_log36.ulg`，2,492,979字节。原SD命名日期滞后，航次以模式序列和两组精确setpoint值匹配，不以文件日期推定。

现场POSCTL策略已部署并通过ARM构建与43项板端离线测试，详情[DEPLOYMENT.md](DEPLOYMENT.md)。采用该策略绕开本轮AUTO目标路径，最后下降仍需飞手油门，尚无补丁后实飞验收。
