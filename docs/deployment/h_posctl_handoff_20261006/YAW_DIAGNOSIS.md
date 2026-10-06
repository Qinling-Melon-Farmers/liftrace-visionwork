# AUTO.LAND 末段偏航诊断（2026-10-06）

范围：160434 H 试飞 ROS bag、历史 103/70/71/72 ULog、与历史实机精确固件 revision 对应的 PX4 官方源码。没有修改工程源码、参数或共享文档，没有 SSH 实机动作、仿真或新飞行。今日 ULog 未能取回：专门代理使用现有 MAVROS1.20.1 FTP 成功 list/open/close，但对最新及前一份 .ulg 的 read 均返回 success=true/data_len=0；项目 logs 无缓存，未打开额外串口或改插件。记录在本地 logs/h_flight_review_20261006/ulog/ftp_retrieval_status.json。不能将下面的源码机制写成今日唯一根因。

## 160434 轮已经确认的事实

原始 bag：`logs/h_flight_review_20261006/board_landing_20261006_160434/flight_debug_0.bag`；当前工作树 HEAD 在任务开始为 `3defaac5`，原始包另以 run_metadata 为准。提取脚本、JSON 与诊断图保存在同级 ignored 日志目录 `logs/h_flight_review_20261006/analysis/`。

- 应用日志在北京时间 16:06:02.910 报告 AUTO.LAND mode enabled；1 Hz /mavros/state 在 16:06:03.261 首次记录 AUTO.LAND，在16:06:05.263 首次记录 POSCTL。后一时刻只是采样时刻；16:06:04.410 回传 target_local 已改为 type_mask=63，yawrate=0，控制内容变化比 state 消息早。
- PX4 回传 `/mavros/setpoint_raw/target_local` 目标 yaw 从16:06:03.011 的 +3.58° 连续增至16:06:04.311 的 +62.26°；回传 yawrate 从 +10.17°/s 增至 +44.93°/s。这里均为 MAVROS转换后的 ENU 角度。这不是从画面猜测旋转，而是飞控回传自动目标确实要求旋转。
- 同期 FC pose yaw 从近0°到 +46.94°峰值附近；LIO/EV pose也随之到约+48.24°，外部位置链没有单独保持原 yaw。这与实际转动相容，单靠这些姿态估计仍不是独立地面姿态真值。
- 自动目标 X 同期从1.981 m降到1.140 m，实际FC X从约1.981 m降到1.52 m；直到最终附近，LIO X约1.10 m。AUTO.LAND目标还发生水平运动，必须一起调查，不能只调低 yaw 速度掩盖。
- 外部 ROS outgoing setpoint 在自动目标开始转动时仍为 X约1.995 m、yaw0°，随后其 yaw 因旧控制插值跟随当前姿态而变化。因此本轮 leading 旋转目标在飞控回传自动链；不能把后续ROS setpoint yaw变化倒推成首因。
- 最终状态为 POSCTL落地/解除武装，result为INCOMPLETE，任务仍LAND/decision_pending。此轮不能记作自主降落验收通过。
- bag未记录 estimator reset counters、raw RC/manual_control、飞控完整参数；/mavros/statustext/recv 无消息。仅靠bag无法区分下面候选。

## 源码已经确认的机制与边界

1. 历史103/70/71/72日志都记录板端固件 `30e763b6780061d70a14894e3e8b06e6a656f9b8`（FMUv6C、v1.15），本机 `/home/xhj/PX4-Autopilot` 为99c4040-dirty，不能当同一固件。今日仍需ULog再次确认。
2. 精确历史固件 `FlightModeManager::switchTask()` 在没有旧 FlightTask 时让 `last_reset_counters{}` 保持0；新任务激活后交给它。OFFBOARD没有运行FlightTask，进入AUTO后 `_checkEkfResetCounters()`可能把早先的计数变化再次处理。Auto heading reset handler把历史delta加到 `_yaw_sp_prev`。该逻辑确实存在，仓库R64的既有补丁解决的是同一类初始化问题。今日是否触发仍需切换段ULog；持续XY目标变化也不能仅由heading handler解释。
   - [精确固件 FlightModeManager.cpp，372–409行](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/FlightModeManager.cpp#L372-L409)
   - [FlightTask重置处理，43–75行](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/tasks/FlightTask/FlightTask.cpp#L43-L75)
   - [Auto heading handler，739–742行](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp#L739-L742)
3. NAV Land本身没有设置朝北的yaw。navigator在当前地理位置有效时使用当前lat/lon，否则设NAN；land yaw始终NAN。Auto Land抓取进入时航向与位置，并可受RC帮助/估计器重置/自动目标影响。这支持逐层核查setpoint，不支持仅因没GPS就改磁偏角。
   - [MissionBlock::set_land_item](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/navigator/mission_block.cpp#L814-L848)
   - [Auto land setpoints](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp#L231-L307)
   - [官方Land模式：需要local position，无需global position](https://docs.px4.io/v1.15/en/flight_modes_mc/land)
4. 精确固件 EV yaw active 时，mag_hdg和mag_3D均要求 `!ev_yaw` 才生效。磁力计可能仍用于初始化/备用/场状态，但不能仅凭MAG_TYPE0和MAG_DECL参数把旋转归为磁航向；要查当时cs_ev_yaw/cs_mag_hdg/cs_mag_3D、EV创新与输入断流。
   - [mag控制](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/ekf2/EKF/mag_3d_control.cpp#L47-L73)
   - [EV yaw控制：input reset/融合超时可重置yaw](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/ekf2/EKF/ev_yaw_control.cpp#L42-L176)
5. POSCTL是人工控制的Position模式，中心油门保持高度，下降来自飞手压油门；不能写“切POSCTL自动下降”。零yaw杆可以锁当前航向，因此POSCTL后停止旋转与AUTO错误航向目标相容，但不独立证明估计器健康。
   - [官方POSCTL说明和人工落地步骤](https://docs.px4.io/v1.15/en/flight_modes_mc/position)

## 历史日志辅助证据

重新只读解析103/70/71/72：都没有AUTO.LAND nav_state18，仅OFFBOARD14→POSCTL2；不能用于今日AUTO问题复现。旧配置相同：EKF2_EV_CTRL15、GPS_CTRL0、MAG_TYPE0、MAG_DECL=-4.1、MPC_YAW_MODE0、MPC_YAWRAUTO_MAX45、MPC_LAND_RC_HELP0、WV_EN0、COM_RC_OVERRIDE1、stick_OV30。参数只证明配置请求，不能代替运行融合状态。各自日志有一次yaw reset；delta分别+82.26°/-112.78°/+74.28°/+1.06°，旧分析与落地标志指向地面段。这说明可能存在历史重置幅值，不是今日因果证据。

## 今日 ULog 到位后的判别顺序

| 候选 | 支持它所需的同段证据 | 处理候选 |
|---|---|---|
| AUTO激活重放历史reset | 切入前counter已非0；切入时counter未新增；yaw目标变化与既存delta相符；同一模式路径初始化为0 | 以实际30e固件基线移植既有计数初始化修复并按板型构建，独立验证；不能直接刷本机SITL产物 |
| 真正新EV yaw reset或断流重融合 | 切入时counter新增；EV输入跳变/reset/超时；cs_ev_yaw状态或创新变化 | 先修同步/坐标/输入质量；确认EV yaw契约再讨论磁备用参数 |
| Navigator全球坐标/目标位置异常 | triplet current lat/lon/xy_global与实际local/global位置不同步；目标X异常从triplet生成 | 查无GPS local/global参考与目标抓取路径，避免错用home/旧位置 |
| RC帮助或人为动作 | 今日MPC_LAND_RC_HELP=1并且yaw杆输入；或override及manual source变化 | 核对RC校准/trim/有效源和实际切换原因，不擅自禁用RC接管 |
| yaw估计稳定但控制器/机构跟随异常 | 目标不变而实测yaw角速度偏离；输出饱和或torque残差 | 再查姿态/角速度控制与执行器；当前bag已有目标变化，不能先归为电机唯一根因 |

优先录制/读取：vehicle_attitude/setpoint、trajectory_setpoint、position_setpoint_triplet、vehicle_local_position（heading/reset_counter/delta_heading/xy,z reset）、vehicle_status/control_mode、estimator flags/innovations/aid_src_ev_yaw、manual_control_setpoint/input_rc、vehicle_rates_setpoint/vehicle_angular_velocity、land_detected。板端FTP取回只由主代理/专门代理负责。

本轮建议作为操作策略：先采用用户已授权的POSCTL末端交接，报告人工落地边界；它绕过AUTO任务，不是修复AUTO.LAND根因。今天不换电、不追加飞行，仅按用户要求部署和地面检查。后续AUTO固件修复需基于今日ULog具体结论，另行安排验证。

## 已拉回的相机与轨迹回放

原始 bag 大小 85,348,842 字节，时长 101.411585 秒。使用 `tools/bag_replay/run.sh` 离线处理，输出 `logs/h_flight_review_20261006/replay_160434/index.html`，含原始相机、标注相机、轨迹与组合四个视频，10fps、每段101.5秒；FFmpeg完整解码和时长检查全部通过。

抽查组合回放83秒的相机画面：完整H靶清晰、靶心基本位于画面中央；90秒附近人工接管后画面明显模糊且靶移出视野。完整bag中63条有效投影H记录覆盖约62.42–84.99秒，不能把检测计数直接当作自主降落验收。

注意默认轨迹取FC odom，经记录TF显示为camera_init；初始化地面段包含很大的轨迹尖峰，不代表飞机飞出了该距离。实际飞行判断应限制在READY/离地后的时间范围，并对照原生camera_init的LIO位姿和FC原话题。组合回放的高度是局部坐标Z，不能直接当离地高度。

包内唯一PointCloud2为 `/sdf_map/occupancy_inflate`（134条，camera_init）；默认轨迹画面显示其当前高度切片。没有原始Livox、LIO注册点云或FreeDOM完整云。用户已取消追加全高度点云回放；现有四视频和原bag均保留。

本次POSCTL接口修改是末段人工降落方案，不是AUTO.LAND飞控根因修复；操作和身份门控约定见同目录 `INTERFACE_PROPOSAL.md`。飞控ULog仍需通过可用的现有链路或后续SD卡读取取得，当前不以零字节FTP文件伪造完成。
