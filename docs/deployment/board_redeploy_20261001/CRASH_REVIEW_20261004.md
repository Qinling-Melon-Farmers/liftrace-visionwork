# 2026-10-04 旧 r64 失事初诊与 LIO/EV 年龄门槛建议

更新依据：今日 ROS 文字日志、完整无过滤文件清单、用户指定的 72 号失事 ULog，以及 70/71 的本地对照摘要。本报告替代“尚无今日 ULog”的前期信息状态；Oct3 ULog 仅用于历史机制对照，不作为今天事故证据。

## 1. 当前判断与范围

**72 号记录支持：目标高度降低及相应减推力先发生，随后出现横滚跟踪失效、控制分配饱和、低比力和强冲击；EKF 实例切换与 RC Kill 均在冲击之后。没有证据将这次事故定为“LIO 超期引起 EV 停融并高度重置”，也不能仅凭一路高电机命令确定单电机损坏。**

已能定位控制交接和动力响应的异常窗口，尚不能区分电机/ESC/桨叶效率、动力配置/接线、供电支路、碰撞或其他外力等具体原因。降低高度目标会先降低总推力，这部分响应符合控制方向；“先于失控”不等于该指令单独足以造成事故，也不等于急降交接已经合理验收。

- 今日 ROS 实际入口根目录：`/home/orangepi/liftrace_r64_onboard_405bda42`，而非 `liftrace_board_trials_20260928`。证据为加载的 launch 及执行路径：[6d663ab0/roslaunch-orangepi5-7180.log:10](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/roslaunch-orangepi5-7180.log:10)、[6d663ab0/roslaunch-orangepi5-7180.log:110](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/roslaunch-orangepi5-7180.log:110)。
- **现场 19 点正在进行的 0928 工程部署仅为离线部署**，这是现场部署观测；它发生在旧 r64 失事运行之后，与此前失事无关。不能将部署包修改、构建或其默认参数写成这次失事时已运行的内容。

## 2. 数据确实存在什么

### 2.1 今日 ROS 文本与无旋转文件

采集工具的 fullget/fileinventory 提供了完整无过滤文件清单。因此不是搜索忽略规则造成漏检：三个主会话的 `rosout.log` 都存在且已取回，**清单及本地均无 `rosout.log.1` 等旋转文件，无已知旋转文件待补取**。

| UUID 前缀 | 本地 rosout 字节数 | 文本首末 CST | 边界 |
|---|---:|---|---|
| d1e5220e | 1,478,656 | 13:38:28–13:47:27 | 走廊及后续高位搜索；末尾半行 |
| 756cce96 | 2,179,072 | 17:46:25–17:50:59 | 走廊及后续高位搜索；末尾半行 |
| 6d663ab0 | 819,200 | 17:53:21–17:54:56.385 | 最后高位搜索；末尾半行 |
| 6ea5cae4 | 无 | 17:53:21 启动失败 | 只有 Livox roslaunch，run_id 冲突，不是第四个完整主会话 |

三份 rosout 大小与 [extra_inventory.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/extra_inventory.json:212) 对应，未发现本地副本短于清单字节数；这不保证写入了运行的全部结尾。每份 154 字节的 `rosout-1-stdout.log` 只含“logging to / re-publishing / subscribed”启动说明，不是旋转日志。`latest` 对应 6d，同一会话不重复计数。

19:15:51 CST 的 [clock 记录](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/extra_inventory.json:1092) 显示 NTP active、synchronized=yes；串口列表为空，USB 外设除 root hub 仅相机。这是取回时状态，不能反推失事时供电或连接情况。三个主会话按其内容和当前清单归入今天；NTP 当前同步不被用作所有历史时刻零偏差的证明。

### 2.2 今日 ULog、旧 bag 与观测缺口

本次新增原始记录 `试飞产物/log_70_2026-10-4-11-59-42.ulg`、71、72，其中 **72 号由用户明确指认为失事那次**。本次独立解析：

- 源文件：`试飞产物/log_72_2026-10-4-12-05-42.ulg`，1,066,808 字节；PX4 boot 120.923701–145.587645 s，约 24.664 s。
- 固件记录：`30e763b6780061d70a14894e3e8b06e6a656f9b8`，FMUv6C；无参数变更记录；ULog logger dropout 为 0，但不代表所有传感器/传输无丢包。
- 实际存在：姿态/目标、角速度/目标、actuator_motors、actuator_outputs、control_allocator_status、failure_detector_status、battery_status、system_power、vehicle_status、vehicle_command/action_request、双 EKF 状态/事件/位置和选择器。字段清单见 [datasets.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/datasets.json)。
- 姿态约 20 Hz，角速度约 50 Hz，输出约 10 Hz，分配/电池约 5 Hz，system_power 约 2 Hz；sensor_combined 共 5179 样本，平均约 210 Hz。各表不同频率，首次记录时刻不等于动作精确发生时刻。
- 缺少 `esc_status/esc_report`，没有逐电机实际 RPM/电流；缺少原始 `vehicle_visual_odometry`、`estimator_aid_src_ev_hgt`、`adc_report`。电源话题实际名是 `system_power`；没有名为 `power_status` 的表不意味着完全没有电源数据。
- [inventory.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/inventory.json) 中原有 42 个 bag，按 mtime 最新为 `board_high_priority_20261003_222005/flight_debug_0.bag`，5,334,310 字节。这些是旧 bag，不能当今日三会话的同步 ROS 测量。之前已归档 `px4_log781.ulg` 仍属于 Oct3 17:14，不属于今天。
- 本轮已取得今日 ULog，故不能再说“没有今日飞控日志”。尚缺的是与今日运行匹配的逐帧 ROS LIO/EV、原始 EV 和动力实测，而不是全部飞控观测。

### 2.3 ROS 按序定位及末段限制

| 会话 / CST | 已记录内容与来源 | 限定 |
|---|---|---|
| d1，13:42:15→13:42:55 | 起飞，后进入LAND并接受AUTO.LAND；[rosout.log:659](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/d1e5220e-bfb5-11f1-b455-0000a40bff7d/rosout.log:659)、[rosout.log:1153](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/d1e5220e-bfb5-11f1-b455-0000a40bff7d/rosout.log:1153) | 属于较早运行，不与72号拼接 |
| d1，13:47:12→13:47:27 | 后续启动出现LIO警告、SEARCH；末行截断；[rosout.log:1762](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/d1e5220e-bfb5-11f1-b455-0000a40bff7d/rosout.log:1762)、[rosout.log:1873](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/d1e5220e-bfb5-11f1-b455-0000a40bff7d/rosout.log:1873) | 警告不能直接定位为后续失事原因 |
| 756，17:46:57→17:48:43 | 起飞、走廊运行后出现clearance失败、命令距离超限、负高度/无效指令，最终SIGINT；[rosout.log:410](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:410)、[rosout.log:1293](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:1293)、[rosout.log:1319](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:1319)、[rosout.log:1349](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:1349)、[rosout.log:1792](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:1792) | 是独立运行中的异常，不自动对应72号 |
| 756，17:48:59→17:50:59 | 后续高位搜索启动有LIO警告，进入SEARCH、RESUME，末尾截断；[rosout.log:1802](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:1802)、[rosout.log:1911](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:1911)、[rosout.log:2653](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/756cce96-bfd8-11f1-a9ce-0000a40bff7d/rosout.log:2653) | UUID内可能有多轮launch，不能把一个目录当单次飞行 |
| 6d，17:54:07.518→17:54:08.450 | 起飞完成；accepted目标高度0.8；随后首次无效负高度指令；[rosout.log:530](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:530)、[rosout.log:531](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:531)、[rosout.log:543](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:543) | 通过第3节timesync及共同事件，与72号建立对应 |
| 6d，17:54:44.759→17:54:56.385 | RESUME；随后11条accepted，currentZ范围2.689–2.789、commandZ范围1.910–2.036；末次currentZ2.775→commandZ2.002，deltaZ=-0.773；[rosout.log:991](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:991)、[rosout.log:1010](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:1010)、[rosout.log:1130](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:1130)、[rosout.log:1135](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:1135) | 这些已晚于72号Kill/Disarm及ULog结束；只证明ROS还在处理/打印状态，不能证明飞机仍正常飞行，也不能把末条降高当失事起点 |

无效指令在6d中持续记录到17:54:45.549，共37条；随后恢复accepted的文字状态不等于实际运动恢复。819200字节尾部截断只能标出本地文字记录边界，不能把最后一行时间当失事、断电或进程退出时间。

## 3. 72 号时间轴：以相对 t 为主

定义 **t=0 对应 ULog start_timestamp，即 PX4 boot 120.923701 s**。以下 t 均为 boot 秒减该值；起始前缓存的 Armed 文本会有小负 t，不是时间回退。

ULog 中 24 条 `timesync_status` 的远端时间是板端 epoch。按
`ROS epoch = boot秒 − estimated_offset/1e6`，
对 offset 随时间插值；记录 RTT 为 0.525–2.545 ms。133.356944 s 的高度目标变化映射为 **17:54:07.551 CST**，与 6d 的 accepted 指令 17:54:07.549 相差约 **2.07 ms**；附近 NED→ENU 位姿约为 (-0.164,0.009,1.546)，与 ROS (-0.163,0.009,1.552) 相符。时间同步和共同目标变化支持对应到最新 6d 会话，未按 ULog 文件名“12点”强行定时。24条estimated_offset范围为 -1791107514198683～-1791107514189616 μs。上述2.07 ms是同一目标切换在两端日志中的事件对齐差，包含传输、回调及打印延迟，不能当成整段时钟的统计误差上界；没有据此把旧bag或其他UUID强行映射到72号。此转换不是独立 UTC 标定，毫秒级打印不表示全部变量都有毫秒级采样精度。详见 [summary.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/summary.json)、[timesync_status_0.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/timesync_status_0.csv)。

| 相对 t/s | boot/s | 已记录事件 | 具体来源 |
|---:|---:|---|---|
| -0.010 | 120.913593 | RC 开关解锁 | [key_events.csv:2](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/key_events.csv:2) |
| 4.099 | 125.022299 | nav_state=14，OFFBOARD | [key_events.csv:3](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/key_events.csv:3) |
| 4.968 | 125.891467 | Takeoff detected | [key_events.csv:4](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/key_events.csv:4) |
| 6.079–10.270 | 127.002–131.194 | 断电/低电量健康文本及三次 Failsafe activated 文本；见第 5 节的遥测限制 | [timeline.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/timeline.csv) |
| **12.433** | **133.356944** | trajectory_setpoint 的 NED z 从 -1.8 改为 -0.8，即向上坐标目标 1.8→0.8 m | [trajectory_setpoint_0.csv:47](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/trajectory_setpoint_0.csv:47)；[6d663ab0/rosout.log:531](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/board_snapshot/.ros/log/6d663ab0-bfd9-11f1-8555-0000a40bff7d/rosout.log:531) |
| 12.439 | 133.363076 | 位置控制 thrust[2] 从约 -0.635 降低幅值至 -0.372；此时飞机估计 Z 向上约 1.56 m | [vehicle_local_position_setpoint_0.csv:90](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/vehicle_local_position_setpoint_0.csv:90) |
| **12.761** | **133.684941** | 起飞后 Motor 1 归一化命令首次到 1.0 | [actuator_motors_0.csv:130](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/actuator_motors_0.csv:130) |
| **12.859** | **133.782359** | torque_setpoint_achieved、thrust_setpoint_achieved 均为 false；Motor 1 高端、Motor 2 低端饱和 | [control_allocator_status_0.csv:67](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/control_allocator_status_0.csv:67) |
| 12.911 / 13.010 | 133.834957 / 133.933860 | 倾角先超过 10°、再超过 20°；近水平姿态目标未随之增加 | [attitude_euler.csv:262](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/attitude_euler.csv:262) |
| 13.048–13.130 | 133.971957–134.053415 | RC mode-slot 请求接管；状态从 OFFBOARD→POSCTL→MANUAL | [key_events.csv:13](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/key_events.csv:13) |
| **13.145** | **134.068522** | 比力范数最低 0.557 m/s²，明显失重特征 | [sensor_combined_0.csv:2824](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/sensor_combined_0.csv:2824) |
| 13.307 | 134.230518 | 记录最大倾角 45.396°，最大 roll 45.168°；未记录到完整倒扣 | [attitude_euler.csv:268](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/attitude_euler.csv:268) |
| **13.329** | **134.252796** | 比力峰值 145.611 m/s²，强冲击特征；冲击对象需其他证据确认 | [sensor_combined_0.csv:2865](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/sensor_combined_0.csv:2865) |
| **14.337** | **135.260515** | 主 EKF 0→1 | [estimator_selector_status_0.csv:19](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/estimator_selector_status_0.csv:19) |
| 14.353 | 135.276877 | 融合输出 reset counter 增加，NED delta_z=-0.04433 m | [vehicle_local_position_0.csv:146](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/vehicle_local_position_0.csv:146) |
| 15.294 | 136.217946 | 已非主实例的 EKF0 出现新 baro-height reset 事件 | [estimator_event_flags_0.csv:19](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/estimator_event_flags_0.csv:19) |
| **18.652–18.663** | **139.575813–139.586399** | RC ACTION_KILL→Kill engaged→manual_lockdown=true | [key_events.csv:22](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/key_events.csv:22)、[actuator_armed_0.csv:61](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/actuator_armed_0.csv:61) |
| 18.764 | 139.687233 | 首个记录到四路驱动输出全零的样本 | [actuator_outputs_1.csv:190](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/actuator_outputs_1.csv:190) |
| 23.668 | 144.591695 | Disarmed by kill-switch；参数 COM_KILL_DISARM=5 s | [key_events.csv:26](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/key_events.csv:26) |

**Kill 比强冲击晚约 5.334 s，不是最初失控/冲击的触发事件。** 全零输出在 Kill 后首个约 10 Hz 记录样本出现；不能把“约 0.1 s 才在日志见到”当成精确执行延迟，也不能把 5 秒后 disarm 解释成期间驱动一直输出。Kill 后上游 `actuator_motors` 仍可能有控制请求，必须看下游 `actuator_outputs` 和 lockdown。

![72号事件窗口，横轴相对t](../../../logs/field_update_20261004/crash72/failure_window.png)

完整变量轨迹另见 [overview.png](../../../logs/field_update_20261004/crash72/overview.png)，事件表保留相对 t、boot 和 CST 三种时间。

## 4. 控制与动力：能证明失配，不能直接判坏电机

### 4.1 先降目标、再减推力，随后才失去姿态跟踪

t=12.433 s 时估计位置向上约 1.55 m，而新目标为 0.8 m。位置控制器将期望 NED vz 从约 -0.257 m/s 改为 +0.760 m/s，并降低向上推力幅值，方向上符合下降请求；这是已记录的控制响应，不是先发生了 EV 高度 reset 后控制器才错误下降。

不过，1 m 的目标阶跃、起飞完成即接较低巡航目标及其下降限幅值得独立复核。它提供事故前的具体触发背景，不能仅凭时序认定“下调目标必然导致事故”，也不能因此忽略动力跟踪裕量。

t≈13.010 s：

- measured roll=+19.534°，最近 target roll=-0.926°；
- measured roll rate=+1.602 rad/s，最近 target roll rate=-2.128 rad/s；
- 这几条样本时差约 1–9 ms，控制器在请求反方向纠正，而实际横滚仍向正方向增加。

来源：[attitude_euler.csv:262](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/attitude_euler.csv:262)、[attitude_setpoint_euler.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/attitude_setpoint_euler.csv)、[vehicle_angular_velocity_0.csv:650](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/vehicle_angular_velocity_0.csv:650)、[vehicle_rates_setpoint_0.csv:650](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/vehicle_rates_setpoint_0.csv:650)。

t=12.859 s 控制分配已报告无法满足 torque/thrust；它是基于模型和输出约束的分配结果，不是对电机实际推力的测量。RC 切模式发生在明显倾角偏离之后、冲击之前，后续低油门 MANUAL 输出也应与先前 OFFBOARD 阶段分开，不能统称自动控制先关电机。

### 4.2 电机命令与实物位置

真实统计窗口 **boot [127.000,133.300) s，即相对 t [6.076,12.376) s**，位于降目标之前：

| 软件电机 | 归一化命令中位数 | 窗口最小–最大 |
|---|---:|---:|
| Motor 1 / control[0] | **0.931** | 0.692–0.956 |
| Motor 2 / control[1] | 0.522 | 0.375–0.544 |
| Motor 3 / control[2] | 0.551 | 0.505–0.572 |
| Motor 4 / control[3] | 0.530 | 0.442–0.545 |

这是持续命令不对称的线索，提示应查动力效率、重心/载荷、混控/标定及接线等；不能把 0.931 写成“实际转速93.1%”，更不能由它单独宣布 Motor 1 损坏。[summary.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/summary.json) 中 `motor_pre_event_127_133p3` 的 extrema 已修为该真实窗口，所有 extrema 的时间均在窗口内；中位数亦按同一窗口计算。

参数记录 `PWM_AUX_FUNC1..4=[101,103,104,102]`，软件功能映射为：
Motor 1→AUX1，Motor 3→AUX2，Motor 4→AUX3，Motor 2→AUX4。
**这里只确认软件通道，不能据此命名左前/右后等实物电机位置，实际位置需要接线核验。**

真正持续记录输出的是 `actuator_outputs instance 1`；instance 0/2 只有启动前缓存的单条数据。该轮 `PWM_AUX_TIM0=-3`，输出数值按驱动原始单位报告，不写成实际 RPM，也不擅自写成 PWM 微秒。四路归一化命令、驱动输出、实际 RPM 是三层不同数据；最后一层此次缺失。

`failure_detector_status` 的 fd_motor/motor_failure_mask 等保持 0，`handled_motor_failure_mask=0`。这只表示没有被该检测链报告/处理的故障，不能排除电机或 ESC 故障；缺少 ESC 反馈尤其限制这种排除。

## 5. 电池、电源与告警：不要把遥测读数当独立电气实测

72 号记录：

| 项目 | 结果 | 可支持的结论及限制 |
|---|---|---|
| 电池 | cell_count=6，source=0（power-module 来源） | 电压/电流是电源测量链遥测，不是另接仪表的独立读数 |
| voltage_v | **17.996–28.809 V** | 跨度异常；6串电池的该高值需检查量程、标定、连接及瞬态，不能据此说真实电池必然升到28.8 V |
| connected | 三次短暂 false：boot127.984、132.184、132.984 s，均下一约0.2 s样本恢复 | 表明测量/状态链不稳定，不等价于动力母线三次物理断开 |
| current_a | **0–46.215 A**；t≈12.46 s附近记录0，之后又有31.4 A及低值 | 需结合降推力、负载和同源测量异常；不是已验证的真实电流突降，也不能从中确定哪路 ESC 失效 |
| scale | **1.0–1.1667** | 当轮 `MC_BAT_SCALE_EN=0`；不能将 scale 抖动直接归因为该速率控制路径的电压补偿动作 |
| warning / faults | warning采样恒0；faults在0/2间变化；另有低电/断连文本 | 不同状态/记录时刻须分别报告，不能用任一项替代其他项 |
| system_power | 5 V轨 **4.715–4.891 V**；brick_valid=1、servo_valid=1；periph/hipower过流标志=0 | 未记录到飞控供电失效；2 Hz采样无法排除短瞬态，也不能代表独立的电机大电流支路供电 |
| USB / ADC辅助项 | usb_connected=1而usb_valid=0；sensors3v3_valid=0；px4io voltage_v恒0 | “接着USB”不等于已由USB供电；无效/未测通道的0不能当作实际电压为0 |
| 原始ADC、ESC电流/RPM | 未记录 | 不能进一步分离ADC/模块/线束问题与真实动力供电异常 |

详细来源：[battery_status_0.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/battery_status_0.csv)、[system_power_0.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/system_power_0.csv)、[parameters.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/parameters.json)。

三条 Failsafe activated 文本在 boot127.013、129.994、130.984 s；但已记录的 `vehicle_status.failsafe` 全程 false，期间仍为 OFFBOARD，后面的模式变化有 RC mode-slot 请求对应。不能写成“电池failsafe自动切模式导致坠机”。`COM_LOW_BAT_ACT=0` 亦应与实际模式数据一并看待。

[ulog_compare.json](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/ulog_compare.json) 已有三轮ULog对照：70/71电池读数同样异常，最大分别35.983/29.345 V；70/71还出现brick_valid/usb_valid切换，而72没有记录到这种切换。因此异常电池遥测不是72独有，也不能把70/71的供电切换复制成72已发生事实。比较说明测量与供电链需要检查，不证明所有读数都是假值，更不证明真实动力供电无问题。

## 6. EV与EKF：今天不是Oct3那条已证实链

72 号的两个 EKF 实例中，已记录 `cs_ev_pos/cs_ev_hgt/cs_ev_yaw` 始终为1，`cs_ev_vel=0`，`vision_data_stopped=0`。参数 EV_CTRL=15 不等于实际融合了速度。原始 EV 未记录，故不能由这些状态算出 LIO/EV age 分布或保证每条输入新鲜；但没有重现Oct3可见的停融→恢复事件链。

- t=14.337 s 主实例0→1，晚于 t=13.329 s强冲击。
- t=14.353 s选择后的 vehicle_local_position 多个reset counter同时增加，Z 2→3，NED delta_z=-0.04433 m（向上+4.43 cm）；与实例切换相连，不能当成事前LIO超期造成的大幅EV高度重置。
- `reset_hgt_to_ev=true` 在两个实例最初缓存快照中已经存在，不能把这个持留事件位的每次出现都算新重置。
- 原实例0在失去主实例地位后，t=15.294 s出现新baro高度重置事件，其较低频位置记录随后出现z counter 2→4、delta_z约-4.219 m；实例1自身z counter保持2。这是冲击和切实例之后的非主实例行为，不能倒置为最初事故原因。
- 冲击后局部高度估计出现数米异常变化，不能用其曲线重建真实离地高度或实际“下穿地面”。先以姿态、角速度、比力、模式和输出时间顺序分析。

来源：[estimator_status_flags_0.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/estimator_status_flags_0.csv)、[estimator_status_flags_1.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/estimator_status_flags_1.csv)、[estimator_selector_status_0.csv:19](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/estimator_selector_status_0.csv:19)、[vehicle_local_position_0.csv:146](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/vehicle_local_position_0.csv:146)、[estimator_event_flags_0.csv:19](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/estimator_event_flags_0.csv:19)、[estimator_local_position_0.csv](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/estimator_local_position_0.csv)。

Oct3 17:14 的旧报告曾确认六次EV停融后高度重置、两次+37.64/+52.42 cm及姿态差旋转影响；它只解释那轮。见 [历史报告](PX4_JUMP_ULOG_171424_20261003.md)。本报告不拿该旧ULog填补今天任何缺失字段。

## 7. LIO/EV年龄门槛的分层与是否放宽

### 7.1 三个本地工作树的正式桥

| 工作树 | 检查HEAD | 正式桥 |
|---|---|---|
| r2026-board-vision-tests | daa1488e | [lio_external_pose.py:24](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/lio_external_pose.py:24) |
| r2026-ev-continuity | cc46ca00 | [lio_external_pose.py:24](/home/xhj/liftrace-worktrees/r2026-ev-continuity/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/lio_external_pose.py:24) |
| r2026-board-frame-fix | e5fb382d | [lio_external_pose.py:24](/home/xhj/liftrace-worktrees/r2026-board-frame-fix/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/lio_external_pose.py:24) |

这三份实现相同；检查到的相应硬件/定位launch只传frame/安装平移，没有覆盖桥的max_age/future_tolerance。board-vision-tests见 `competition_hardware.launch:35`，另外两树见 `competition_localization.launch:22`。这属于本地源码默认值和接线核查，**不是旧r64事故时参数已回读的证明**。今日快照内完整桥源码属于0928目录，旧r64此次源码快照主要是FAST-LIO；没有把它们混成同一运行版本。

### 7.2 年龄定义、阈值、超期行为

| 层级 | 时钟/指标与具体默认门槛 | 超期或中断后的行为 |
|---|---|---|
| 正式 `/Odometry→/mavros/vision_pose/pose` | 回调时 `ROS now−msg.header.stamp`，必须 **-0.05≤age≤0.30 s**；frame须匹配 | 拒收该条，return，不转发；frame/时间共用每2 s节流警告 |
| 正式桥接收年龄/断流层 | **没有独立last_receive年龄阈值，没有定时watchdog**；输入/输出queue_size=1 | 无回调则无输出，也不会由此主动产生上述警告。queue=1不是时间门槛，2 s是警告节流而不是允许断流2 s |
| 正式桥输出 | 保留源header；只补参考点平移，未重新打now | 不重发旧位置凑频率，不自动切模式、不发HOLD/ABORT；后续处置由飞控/任务层负责 |
| 导航frame适配（独立于EV桥） | 源消息age **-0.05～0.30 s**；动态TF与消息时间差≤0.30 s；查询等待0.08 s | 拒绝对应反馈/设定点转换，警告5 s节流；静态TF零stamp例外。不应把它当成EV输入门槛 |
| EV专项shadow校正 | `now−真实LIO校正时刻` ≤**0.30 s**，未来容差0.005 s | 迟到/未来校正拒收；已有状态的校正超时会锁存fault、停止输出，需显式重启 |
| EV专项shadow输出 | `now−最新积分IMU采样时刻` ≤**0.10 s**；IMU积分间隔≤**0.025 s**；未来容差0.005 s | 过旧、IMU断档等使输出停止/锁存故障；没有新采样时刻不重复发布。50 Hz是定时输出上限，不保证每tick有新观测 |
| PX4 EV融合 | 已归档的同revision代码诊断：`EV_MAX_INTERVAL=0.20 s`，相关停融条件使用2倍即**0.40 s** | 延迟融合时域内没有足够新有效样本时停融；重新满足条件可恢复。不是ROS接收间隔单阈值 |
| 飞控延迟参数 | 72号实际 `EKF2_EV_DELAY=0`、`EKF2_DELAY_MAX=200 ms` | 时间补偿/融合历史设置，不等于“允许200 ms旧输入”，更不等于桥接300 ms的替代门槛 |

正式桥定义/行为：[lio_external_pose.py:28](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/lio_external_pose.py:28)、[lio_external_pose.py:33](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/lio_external_pose.py:33)、[lio_external_pose.py:46](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/lio_external_pose.py:46)。
导航适配：[navigation_frame_adapter.py:45](/home/xhj/liftrace-worktrees/r2026-board-frame-fix/patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_frame_adapter.py:45)及其77行。
shadow参数：[ev_shadow.yaml:6](/home/xhj/liftrace-worktrees/r2026-ev-continuity/patrol_uav_ws-patrol_planner/src/FAST_LIO/config/ev_shadow.yaml:6)；
校正拒收/失援：[ev_predictor.py:204](/home/xhj/liftrace-worktrees/r2026-ev-continuity/patrol_uav_ws-patrol_planner/src/FAST_LIO/scripts/ev_predictor.py:204)；
输出超龄：同文件247行。shadow仅输出观察话题，未替换正式EV，也没有实飞验收其保护值。
PX4 200/400 ms依据：[同revision历史代码诊断](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/docs/deployment/board_redeploy_20261001/PX4_JUMP_ULOG_171424_20261003.md:63)；本机未取得该revision完整Git对象，本次未联网补取，也没有假称重新逐行验证固件源码。

bag记录时刻减源stamp、桥回调now减源stamp、当前时间减最后接收时刻、飞控timestamp_sample年龄，是四个不同指标。警告条数因节流不能换算拒收帧数；没有警告不能证明持续收到了有效EV。

### 7.3 当前不直接放宽的原因

**不建议把放宽300 ms作为这次失事修复。** 72号在失控前未记录到EV停融或高度reset，已有控制目标阶跃、动力命令不对称、纠正失败/饱和和冲击的直接证据。调整年龄门槛不能解释或修复已经观察到的动力跟踪问题。

独立处理LIO实时性时，有限放宽可减少桥端丢弃，但它同时允许更陈旧位置进入下游，不能保证飞控使用这些样本；更不能消除积压、真实断流或坐标/姿态不一致。将shadow校正期限放宽还会延长无雷达校正的惯性传播，增加漂移风险。不能同步放宽导航、释放许可或跳变保护来“消除报错”。

Oct3旧bag显示300 ms外拒收与输入中断有关，说明该问题值得单独研究，而非证明今天也应改阈值。后续如果专门验证门槛，应在同输入/同负载下分别量化桥源年龄、回调接收间隔、拒收原因、PX4有效融合样本年龄与创新；只有这些证据支持，才决定具体候选值。当前没有把500 ms或1 s确认为可用参数。

## 8. 当前可作出的工程结论与后续指标

1. **优先复核起飞→巡航高度交接与动力跟踪裕量。** 1.8→0.8 m目标阶跃及最初减推力是明确背景；命令不对称、反向纠正未奏效及分配饱和是明确异常。实际电机/ESC/桨叶、接线/旋向、动力参数/重心和供电支路的根因仍不能仅用此次日志区分。
2. **并行核查电源测量可信度。** 记录电压/电流异常与5 V轨/brick_valid的分别证据；未把current_a视作独立实测，也没有把connected=false视为物理掉电。70/71与72的差异保留。
3. **今天不能归因于旧Oct3 EV重置机制。** 72号主EKF切换在冲击之后，selected Z变化仅4.43 cm；非主EKF后续baro reset不倒置为原因。
4. **进一步区分硬件根因所需的数据：** ESC每路RPM/电流/错误与实际母线测量、接线位置确认、同步ROS源/桥/飞控时间戳、准确的最终设定点，以及视频中的触地/接管时刻。

复现脚本：[analyze72.py](/home/xhj/liftrace-worktrees/r2026-board-vision-tests/logs/field_update_20261004/crash72/analyze72.py)。它仅解析本地72号ULog，输出同目录。示例运行在已激活rl_drone后执行 `python -B logs/field_update_20261004/crash72/analyze72.py`；Windows宿主按仓库约定通过 `wsl -e bash -c '...'` 调用。CSV首行为字段名，报告链接行号对应本轮生成版本；原始日志未修改。

验证：ULog解码成功；关键事件/状态变化及原始样本导出；人工核对目标交接、时基、姿态/角速度纠正方向、输出关断、EKF主次实例和电源字段；图表已查看；窗口extrema时间已检查落在[127,133.3)内。保留缺失字段为未知，没有用旧ULog、软件电机命令或失效检测零位填充未知事实。


## 可随仓库查看的分析产物

- [全轮曲线](crash72_20261004/overview.png)
- [失控窗口曲线](crash72_20261004/failure_window.png)
- [关键事件表](crash72_20261004/key_events.csv)与[统计摘要](crash72_20261004/summary.json)

原始ULog、完整CSV和分析脚本保留于本机试飞产物及 `logs/field_update_20261004/crash72/`，不随Git提交。
