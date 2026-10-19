# 全场包红十字时序与末段异常分析

2026-10-12，只读评估。对象为“全场测试(这个才是).bag”的恢复索引副本。未改运行代码、原 bag、共享文档，未 commit，未启动 ROS/Gazebo/实机，未重跑模型。H 那轮 bag 用户确认尚未提供，本报告不分析该轮 H 根因。

## 结论

红十字首槽从 External ALIGN 到控制器正向 Servo ACK 共 **42.217 秒**：捕获/冻结 4.623 秒、串行补偿 5.547 秒、下降与释放等待 30.998 秒、请求至 ACK 1.049 秒。长等待主要发生在下降后，控制器反馈高度持续超过 0.45 m 释放上限；水平误差也反复超出 4 cm，并非一直只卡高度。

**0.40 m 目标已经贯穿任务设定、MAVROS 最终设定和 FC 回报目标。** 在下降后第 5 秒至请求前的 25.998 秒窗口内，三条目标链的 Z 恒定相同，反馈高度中位数约 0.473 m。因此可定位到“低位指令已发出，估计反馈仍偏高”，不能据此诊断真实离地高度、定位误差、地效、飞控参数或机械根因。

首次明确规划失败在 **270.371 秒，decision/goal 16**，位于第一道门所在走廊段。它是规划占据/可行性拒绝；日志中的 collision 计数不等于实物碰撞。包末仍 OFFBOARD、armed、IN_AIR，未记录完赛。原包最后 chunk 截断，后续 H 阶段缺失。

## 数据与版本边界

- 源路径按 sources.json 的 full 读取：/home/xhj/liftrace-deliverables/flight_review_20261012/full/source_reindexed.bag。恢复消息 196853 条、保留时长 272.272 秒；起点 epoch 1791730355.2809203，对应北京时间 **2026-10-11 22:52:35.281**。本文所有事件时间均为 bag 收录时间减起点，图像另注明曝光时间。
- 运行 metadata：high_view_full，低位 AGL 1.4、高位 2.0、投递目标 0.40、释放窗口 0.35–0.45 m，actuator_mode=real，raw 服务 /legacy/Servo_raw，takeoff_z_lead_m=0，运动优化和高位续扫开启。巡航前视 1.0、精细 0.4、走廊开放段 0.6、门段 0.4 m。
- 地面参考 ground_z=-0.215073719 m；本文“估计 AGL”统一等于任务/FC局部 Z 减这个记录值。投递目标局部 Z=0.184926281，控制器释放窗口 [0.134926281,0.234926281] m。MAVROS 原始局部 Z 不直接当离地高度；/Odometry 的 IMU 原点也不能直接当 FC 高度。
- 在线模型路径为 flight_5cls_new_video_v8_20261011_fp16.rknn，7545013 字节，五类含 red_cross、H 不在模型正类。metadata 的 candidate=true、hardware_validated=false 是模型声明，在线输出另按 bag 读取。
- deployment manifest 的 source_head=51f48fc…、date=2026-10-08 是继承记录，**不能当本轮二进制 HEAD**。运行日志实际具有 stable=5、红十字 4 cm/标准靶 6 cm、stage=compensate_xy、serial=true 和异步 Servo 特征。本地试飞工作树只用于解释这些匹配的代码路径，不证明现场可执行文件精确 revision。

## 红十字：捕获新帧、冻结、串行补偿、下降、许可、执行

| 节点 | 相对秒 | 可核对事实 |
|---|---:|---|
| APPROACH | 146.572 | target 2，red_cross，随后 ALIGN |
| 控制器进入 ALIGN | 147.482 | decision 8、attempt 1、slot 1 |
| 第一轮计数达到 4 | 149.700 | 149.460/149.563/149.604/149.700 的 stable_frames 为 1/2/3/4 |
| 新帧越界、计数清零 | 149.746 | offset_exceeds_limit；同帧 dx=27.246、dy=12.997，范数 **30.188 px**，超过 30 px |
| 第二轮达到 5 新帧 | 152.049 | evidence_valid=true；不是重复发布同一图片累积计数 |
| 冻结 | 152.105 | 中心约 (4.755,-1.638)，补偿 FC 目标 (4.874548,-1.649649)，source 曝光 151.938 秒 |
| 串行补偿完成、下降启动 | 157.652 | 先在捕获高度补偿约 12 cm 杆臂，XY <4 cm 后 serial=true 开始下降 |
| 三条指令链均已达到最终低位 | 160.264 前后 | mission 160.201、MAVROS 160.201、FC target echo 160.264 均 Z=0.184926286，即估计 AGL 0.400000006 |
| 许可首次放行 | 168.542 | permission_granted_from_commitment；随后多次放行/撤销 |
| 控制器 release_ready / RPC | 188.650 | FC误差 0.63 cm、投口误差 0.65 cm，FC局部Z=0.2344，估计AGL约 0.4495 m |
| raw_actuator_call_started | 188.658 | execution 1，slot 1，decision 8，terminal=false |
| PWM ACK / terminal结果 | 189.672 | raw_actuator_ack，success=true、terminal=true |
| 控制器 ACK | 189.699 | 正向 Servo ACK |
| ALIGN 完成后恢复 | 190.850 / 192.064 | 结束对准，任务重新交接运动 |

第二轮五张新帧的曝光时间约 **151.631、151.699、151.759、151.807、151.904 秒**；第一张合格至第五张仅 0.244 秒收录间隔。总捕获 4.623 秒包括先移动到像素范围内、第一轮四帧后越界清零、重新收敛，并非“识别五张图本身要四秒”。进入时像素偏差约109 px，冻结前最后 offset 范数约9.54 px。短暂 context_mismatch、feedback_missing 与 observation_out_of_order 均有记录，但不能拿某一句覆盖整个捕获过程。

冻结以后 XY 指令保持同一目标，低空图片放大和偏离相机中心没有触发新冻结/重复投递。152.45 秒后在线 evidence/drop_ready 多次 stale 或 feedback_missing；仍保留已建立的有界释放 commitment。这些消息不等于正在无限重采五帧。

下视原图 147.889、149.649、152.028、157.776 秒可见同一红十字贴在标准靶板中央，随后 160.237、163.919、188.739 秒靶体放大、偏右，符合补偿后下降的画面变化。图像只是几何变化旁证，不是人工靶心/落点真值。对应文件为 replay/frames/000675、000683、000693、000719、000730、000746、000858.jpg。只看原图和录下的在线输出，没有重推理。

另外，冻结控制中心与同源 mapped 中心约相差 7.0 cm，与记忆融合中心约相差 6.0 cm：同源 mapped 约 (4.755664,-1.567654)，selected/targets 约 (4.756382,-1.578401)。控制器采用像素偏差/尺度构建中心，记忆采用投影融合；二者不是同一字段。该差异应保留为精度评估项，不能直接当作物理投口误差，也没有证据将其认定为31秒等待的主因。

## 最终 Z 指令与反馈：已发低位，反馈超窗

核对窗口为 **162.652–188.650 秒**。完整控制过程还按同曝光/命令时间配对：824 对 mission 与 MAVROS 最终设定，Z 差全部为 0。

| 消息 | 窗口样本数 | 局部 Z 范围或中位数 | 按记录参考换算 |
|---|---:|---|---|
| /navigation/setpoint_mission | 520 | 全部 0.184926286 | 全部 0.400000006 m |
| /mavros/setpoint_position/local | 519 | 全部 0.184926286 | 全部 0.400000006 m |
| /mavros/setpoint_raw/target_local | 260 | 全部 0.184926286 | 全部 0.400000006 m |
| /navigation/local_pose | 780 | 中位 0.257620439 | 中位 **0.472694**；P95 0.488813 m |
| /mavros/local_position/pose | 779 | 中位 0.257604420 | 中位 **0.472678**；P95 0.488810 m |

FC 回报目标在这个窗口 coordinate_frame=1、type_mask=0；表格比较 bag 中 MAVROS 已发布数值，没有自行给 Z 反号或再次补安装偏置。刚启动下降时 FC target echo 有中间目标，例如157.764秒局部Z约0.8408；约160.264秒到达0.184926286，后续恒定。没有发现这段等待中重新指令爬回捕获高度的证据。

代表控制日志（下列AGL由日志局部Z换算，日志本身三位小数）：

| 相对秒 | XY | 估计 AGL | 控制器拒绝的实质 |
|---|---:|---:|---|
| 160.298 | 12.2 cm | 约0.742 m | 下降中，XY与高度均未合格 |
| 163.448 | 2.8 cm | 约0.494 m | XY合格，高度仍高 |
| 167.548 | 1.1 cm | 约0.474 m | XY合格，高度仍高 |
| 175.855 | 3.2 cm | 约0.468 m | XY合格，高度仍高 |
| 183.101 | 2.6 cm | 约0.474 m | XY合格，高度仍高 |
| 186.149 | 1.0 cm | 约0.460 m | XY合格，高度仍高 |
| 188.650 | 0.65 cm | 约0.4495 m | 首次满足最终释放条件并提交请求 |

下降后 FC 水平误差最高约12.8 cm；最终26秒窗口近似投口/FC最大误差中位4.39 cm、P95 6.82 cm。按收录时刻分段，约25.95秒高度高于上限，其中约11.53秒 XY已合格仍仅卡高度，约14.42秒 XY和高度同时不合格。投口重算使用日志三位小数冻结中心，4cm边界附近分类是近似统计，不能冒充控制器逐回调的精确判决。

与10月11日 (3) 的红十字 **38.518秒**相比，本包42.217秒；分段为4.623/5.547/30.998/1.049秒，历史为1.554/5.101/30.805/1.058秒。共同机制是低位设定后，估计反馈在释放上限附近偏高，XY合格也不能释放。本包低位估计常约0.46–0.49，历史约0.45–0.47 m；新增约3.70秒主要是捕获阶段更长。两轮布局、槽位等并非严格A/B，不推导固定提速收益。

## 许可与执行不是同一层门槛

152.090秒锁定 commitment，许可来源 evidence_stamp 相对152.062秒；45秒有界时限内，允许冻结后继续消费这份捕获依据。188.625秒请求前最后许可 permitted=true、decision8/attempt1/slot1/target2 正确；valid_until 相对188.874秒，请求时尚未过期。旧捕获依据距请求36.588秒，与“现在新图仍有效”是两件事。

许可层窗口比控制器宽：配置参考为AGL约0.27–0.47 m，日志只打印局部[0.05,0.25]的两位小数；本轮按地面参考对应局部约[0.054926,0.254926]。因此168.542秒许可层能放行，控制器自己的0.45m上限仍可拒绝。

下降至RPC阶段的许可消息占时约：release_altitude_invalid **17.91秒**，commitment_position_drift **2.94秒**，permission_granted_from_commitment **10.12秒**。首次位置漂移拒绝158.981–161.375秒，第二次178.825–179.375秒；已有commitment未因此形成新任务/新槽。172.174、175.577、176.375、177.925、183.374、187.524秒等再次放行，不能说31秒全是无许可等待。

最终是**真实服务链的成功软件/驱动回执**：raw服务实调用、PWM控制器ACK、release_result terminal success、控制器正向ACK四者吻合。PWM日志明确“physical release unverified”。没有独立载荷传感器、舵机角度/盒体脱离真值或落点标注，因此不能据此宣布实物已落到红十字靶心。

## 完整任务与首个明确规划异常

起飞完成102.556秒，任务SEARCH启动105.186秒，本轮没有历史起飞后十几秒仍不启动的停顿。三投顺序 red_cross / panzer / bridge，ALIGN至ACK分别42.217 / 16.862 / 18.876秒。三次软件回执后，246.678秒进入POST_DELIVERY_ROUTE，committed_slots=3。

投后decision13、14、15分别在253.528、259.516、263.577秒确认motion_arrival；decision16于263.656秒派发、263.713秒轨迹ready，目标约 **(7.693641,-0.707166,0.684926)**，记录AGL0.90m。270.371103秒首次status=5、trajectory_replan_attempt_failed；271.128604、271.895873秒两次新轨迹尝试继续失败。早于这三条，没有记录到planning/goal_status status=5。

首次失败时FC位姿约(7.780,-0.587,0.755)，距目标约0.164m；kinodynamic日志start_occ=1、goal_occ=0，初始40候选accepted=0，重试2187候选accepted=0（collision=1601、closed=586）。这证明规划查询认为起点占据，无法找到本次可行扩展；不证明机体真实碰门，也不单凭这条确定门位、地图或定位根因。

任务桥把失败作为planner_attempt_failed_nonterminal保留，trajectory_ready=false、faulted=false，没有新目标或完成回执。包截断前mission_failed仍false并非完赛通过：最后阶段POST_DELIVERY_ROUTE、index3/9、active decision16、route未完成。

末尾272.265秒FC/任务位姿约(7.589,-0.691,0.728)，估计AGL0.943m；最近state仍OFFBOARD/armed=true，extended_state仍IN_AIR。现场“险些撞门”须与上述时序联合解读，确认碰撞和点云/门位关系由主审/点云分析的几何核对补充。

## H证据：本包未进入LAND，不能解释另一轮H症状

全量检查：align_mode=landing **0**；原始/navigation/mission_command_raw LAND **0**；控制器/mission/command LAND **0**；raw、resolved、refined、mapped、targets 中H/landing类正输出均 **0**。raw来源仅target_detector3401、circle_detector4679、cross_detector3572条，没有landing_detector来源。72.439秒初始化日志确有LandingDetector ready、landing_mode_gate=true、active=false，不能写成节点没有启动。

运行预期H交接为POSCTL：初始化记录捕获约局部0.98m（AGL1.2），交接约局部0.15m（AGL0.37），低位目标约局部0.13m（AGL0.35），stable=10；它是配置，未见本包实际进入H捕获/下降/请求POSCTL。218.705与246.360秒泛化旧日志“WayPointDetectDone Landing completed”对应投递流程收尾；state仍OFFBOARD且后续继续飞行，不能当H着陆成功。

主审已人工查看末段拼图：现有全场包末尾为红色走廊障碍与地面，无H；仅靠无landing消息或走廊坐标不能推出画面中全程没有H。用户确认H反复横移、未切POSCTL那轮bag未发送，本报告不把本包门段问题套成该轮H原因。

原包尾部截断证据见tail_integrity.json：最后chunk声明733825字节，只剩420字节；expected_end=298033373 > 文件大小297299968。恢复索引仅保留可读内容，不能补出后续门/H/落地状态。缺失该轮H的实际LAND命令、H图像/几何、位姿/速度/高度及POSCTL请求与mode回报，无法评估其handoff拒绝条件。

## 可复核产物

- focus_findings.json：分段、完整Z链窗口统计、许可转移、代表消息、首个规划失败与任务尾状态。
- focus_telemetry.json：原包只读提取的结构化消息；不可用FC字段以null保留。
- focus_extract.py：ROS setup后以系统ROS Python读取恢复副本；仅输出本目录。
- focus_summarize.py：在现有rl_drone内生成定量结果；投口近似统计的边界已写入JSON。
- focus_h_evidence.json、focus_h_check.py：全量模式、命令、检测来源及H正输出核查。

未改前视1.0/0.6或其他现场配置。分析结果不是修改运行参数的授权。

## 追加：门段险撞的规划/控制时序（同一全场包）

本段只核对控制、轨迹、到达和占据拒绝时序；门位/点云/机体净空由点云分析核对。现场反馈为“险些撞门”，原图tail_271.5.jpg可见两侧红色障碍；现有包没有独立碰撞事实传感器，不把险撞升级为已碰撞。

### 保护航点保留，但实际路径不是固定X直线

投后已发的四个连续目标为：(6.493641,2.092834)、(7.693641,2.092834)、(7.693641,0.992834)、(7.693641,-0.707166)，全部局部Z=0.684926281，分别对应decision13/14/15/16。decision15在263.577秒motion_arrival_confirmed后才派发16，门前/门后相应航点仍在真实命令链里；没有证据说运动优化把这两个门保护端点删掉。到达确认是在容差内，不代表机体精确到达设定中心。

轨迹19于263.713秒ready。位姿跟踪显示：265.005秒约(7.749,0.543)，265.988秒约(7.866,0.172)，266.989秒约(7.963,-0.194)，267.323秒X峰值**7.973111**。与此同时规划位置指令266.356秒自身X达到**7.956403**，比航点名义X多约26.3cm；FC反馈峰值比名义X多约27.9cm。这说明侧向外弯同时出现在规划命令与执行反馈，不宜只归因为飞控把直线跟歪；该外弯是否侵入真实门边界，须结合点云/门位/机体包络核对。

SERVER progress在轨迹19开始时实际target_dist=0.6m，264.646秒首条记录已切到0.4m。manager约264.555秒选择DOOR，参数与后续实际SERVER取值吻合。264.240秒原始指令与位姿距离0.5846m、target_dist0.6，tracking_hold=false、interrupted=false。这是前视阶段切换的实录，不是本次修改建议，也不能单独宣称0.6m就是险撞根因。

### 270秒失败以后：保留轨迹19的旧末点，不是新轨迹或保护刹停

- 267.955秒起，/planning/pos_cmd已经输出轨迹19终点(7.693641,-0.707166,0.684926)，速度和加速度为0；即终点输出开始于首次规划失败之前。
- 首次失败前270.366秒的SERVER progress：goal16/traj19，projection_t=lookahead_t=duration=4.048420807，reason=no_lookahead，tracking_hold=false、interrupted=false。位姿到末点距离0.16356m。
- 270.371秒goal_status先trajectory_replan_attempt再FAILED_ATTEMPT；之后271.129和271.896秒重试也失败。后续没有新的TRAJECTORY_READY记录。
- **首次失败至包尾共190条/planning/pos_cmd，全部trajectory_id=19，全部同一末点，全部速度0。** 同时38条mission最终设定、38条MAVROS最终设定都等于这个旧末点，空间差为0。
- 对应SERVER progress持续同一traj19/traj_start，reason=no_lookahead、tracking_hold=false、interrupted=false；没有出现新轨迹号、另一条绕行曲线或局部保护hold。普通“维持旧终点”的位置保持，不能冒充系统已经触发保护刹停。
- 三次失败都为mission桥的planner_attempt_failed_nonterminal，未收到goal16到达确认，也没有派发decision17/下一道门目标。包截断时仍在追踪当前走廊段末点。

匹配的本地源码机制解释：kino_replan_fsm的普通REPLAN尝试失败回到新轨迹重试；checkCollisionCallback仅在EXEC_TRAJ状态对当前轨迹调用checkTrajCollision。traj_server的replanCallback会设置interrupted并保持当时位姿，但普通重规划不一定触发这个通知。本包未录/planning/replan，不能直接证明某条通知有没有发布；已录的SERVER interrupted=false与实际旧末点输出，是可直接验证的行为。不能仅凭“仍转发轨迹”日志宣布避障整体失效。

本段首次失败时start_occ=1、goal_occ=0和大量候选collision拒绝，说明从当时估计位置启动的新规划遇到占据约束；这不等于已证实原轨迹整段不安全、实体已撞门，或定位/膨胀/门口任一项独立失效。待点云分析将267秒外弯、270秒起点占据以及最近红色门障碍点云对齐后，才可判断具体净空问题。

辅助原始与统计见focus_door_findings.json（轨迹19 SERVER progress、190条失败后命令、goal_status、保护端点和前视转移）。
