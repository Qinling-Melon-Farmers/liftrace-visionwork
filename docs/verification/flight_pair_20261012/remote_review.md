# 2026-10-12 导航远端与前视参数只读核验

本报告聚焦远端实际差异、起飞/红十字投递和 1.0/0.6 m 前视影响。未修改运行源码或配置，未 merge、push、部署、启动仿真或实机；仅 fetch 与读取既有产物。报告在仓库外，不改共享联调台账。

## 1. 远端、工作树与跟踪关系（已确认）

- 来源：https://github.com/sakelier/liftrace-controlwork.git，分支 liftrace_board_trial_20260928。
- 工作树：/home/xhj/liftrace-controlwork-worktrees/board-trial-20260928。
- fetch 成功，无阻塞。命令：git fetch --no-tags origin refs/heads/liftrace_board_trial_20260928:refs/remotes/origin/liftrace_board_trial_20260928。
- 远端 HEAD 与本地 HEAD 均为 **421e6023d69e7026e3151770d8b647ec793004df**；左右差异计数 **0/0**。
- 已知基线：b19010c381a720565a94d3f64b2e36c7733e69d9。基线到远端只有一个提交：421e6023，2026-10-11 21:54:45 +0800，fix: 保留起飞完成终点并回退Z领先生产默认。
- origin 指向 sakelier；fork 指向 Qinling-Melon-Farmers/liftrace-controlwork，未操作 fork。
- 工作树无已跟踪修改；唯一未跟踪文件为 docs/verification/door_waypoint_protection_20261011/HANDOFF_JAMES.md，保持原样。
- branch.remote=origin、branch.merge 已设置，但 remote.origin.fetch 的选择性规则不包含本分支。因此普通 status/branch -vv 不显示 upstream，解析 @{upstream} 失败。显式 fetch 已取得远端引用，直接比较有效；本轮未修改 Git 跟踪配置。

**结论：没有本地尚未包含的远端提交；不能把既有红十字或前视逻辑描述为 fetch 后的新改动。**

## 2. 相对 b19010c3 的实际新增差异（已确认）

| 范围 | 421e6023 的改动 | 实际意义 |
| --- | --- | --- |
| 起飞完成回调 | 以 takeoff_point 和首航点 yaw 初始化 patrol_cmd、mavros_point_cmd、last_mavros_point_cmd；设置 external_waiting_for_motion_=true | 起飞完成后继续朝原始终点保持，避免把最后一次限距后的中间点永久当作终点；等待新任务释放接管 |
| 08 settings.yaml、competition/control_base.yaml | takeoff_z_lead_m 从 0.6 改为 0.0 | 默认恢复通用三维限距；显式 0.4～0.6 m 起飞试验能力仍保留 |
| 测试/文档 | 新增交接回归、调整既有夹具/参数断言、记录修复和 legacy 快照 | 不增加红十字或速度行为变化 |

核心路径（均相对导航工作树）：

- patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp:374（起飞完成），:1413（等待保持），:1698（Z领先与通用限距）。
- deployment/board_trials_4x4/08_full_mission/settings.yaml:48。
- patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/control_base.yaml:20。
- docs/verification/takeoff_handoff_20261011/REPORT.md。

完整 diff 为 71 个文件，其中 61 个属于不参与编译的 legacy_baseline/20261011/takeoff_handoff_hold；其余 10 个是源码、配置、测试与记录。未重新计算快照哈希。既有修复报告记录 30 项离线检查通过及构建成功，本轮未重跑编译或将其当作新实飞验收。

起飞完成仍按距终点小于配置阈值判断；421e6023 没有加入停稳/垂直速度判断，也不证明全部高度超差已消除。auto_start_after_arm 的 supervisor 另要求一秒内至少10个位姿样本、最新年龄<0.3 s、距起飞参考高度<0.15 m、各轴峰峰值<0.12 m，再连续满足一秒；控制 ready 与任务启动不同。

## 3. 已在基线内的红十字/门口变化（不是本轮新增）

| 提交 | 已存在的行为 |
| --- | --- |
| 38647c92 | 08 两档目标下降 AGL 0.40 m，释放窗口 0.35～0.45 m |
| b553b74c | 按绑定任务类别选择双 XY 误差门限：标准四类严格<0.06 m，red_cross 严格<0.04 m；未知类别不继承标准放宽 |
| fe890d5b | 默认冻结靶心后先完成 FC/投口补偿，再单向下降；下降后 XY 暂不满足会阻止释放，但不重新升回捕获高度 |
| e0c06431 | 粗 HINT 置信门槛同步为0.75；不是红十字最终释放门限 |
| 3bf76a82 | 优化投后路线时保留每次跨墙最近的门前/门后锚点，并保持 H 阶段计数 |
| b19010c3 | 起飞专用 Z 领先和终点步长保护；其0.6默认已由421e6023回退 |

释放还依赖绑定事务、freshMotion、OFFBOARD/armed、原许可与匹配 ACK。补偿投递中的 max_horizontal_speed、max_vertical_speed、stable_duration、min_samples 等兼容字段不构成该函数的停稳门槛；速度只检查有限值，H 使用独立停稳条件。路径：patrol_control.cpp:3721、:3732、:3796、:3815、:4282。

## 4. 现场参数与 ALIGN 是否绕过前视（已确认）

已读取主审的 bag6/timing.json、full/timing.json、full/analysis.json。两包均为 competition：规划1.2 m/s、加速度1.0 m/s²；motion 制动模型1.2 m/s与0.6 m/s²；following（巡航/精确/近墙/走廊）=1.0/0.4/0.2/0.4 m；takeoff_z_lead_m=0；相同现场配置、新模型 flight_5cls_new_video_v8_20261011。**1.0 是巡航前视距离，不是本两包巡航速度。**

元数据中的部署 manifest 仍为2026-10-08、51f48fc5970b10b5a8507f59874bd04ee8da7640，其模型字段也较旧。它不证明当前二进制 HEAD 是51f48或421e6023。实际日志包含 compensate_xy、serial=true、red_cross 的0.040门限，可确认这些行为已执行；仍不能据此唯一确定整包源码版本。

源码链路：

1. navigation_mission_manager.py:725 按动作选档，:761 同步写 /traj_server/traj_server/target_dist 与 /px4_max_distance。
2. Run_point 使用 planner 指令；traj_server.cpp:392 进行进度投影、:394 沿轨迹取有限前视点。
3. **Aligning 绕过 planner/轨迹前视**：patrol_control.cpp:1368 执行 externalMissionTick；:1617 清除 have_planner_cmd，直接使用 patrol_cmd。ALIGN 本身在 FollowingSpeed.select 中落入 PRECISION=0.4，而非近墙0.2档。
4. **Aligning 不绕过共享控制限距**：switch 之后，:1706 对所有适用模式执行三维限距。当前0.4 m会限制大幅下降开始时的输出，也会耦合同时存在的 XY/Z 误差；仅调轨迹 target_dist 对 ALIGN 没有直接作用，调共享 px4_max_distance 才会影响对准输出。
5. Z领先仅用于 Takeoff；已进入 Aligning 后不生效。motion.braking_accel 主要用于近边界策略预算，不是给 ALIGN/PX4 设定的垂直制动加速度。

### 红十字时序与持续偏高：不能归因于巡航前视

| 从 ALIGN 起算至 ACK 的阶段 | (6).bag | 全场测试.bag |
| --- | ---: | ---: |
| ALIGN→冻结 | 2.400 s | 4.623 s |
| 冻结→补偿完成/下降开始 | 3.352 s | 5.547 s |
| 下降开始→调用 | 7.584 s | 30.998 s |
| 调用→ACK | 1.060 s | 1.049 s |
| 合计 | 14.396 s | 42.217 s |

full日志在 t=146.551 s已选择 PRECISION lead=0.400m；t=147.482 s进入红十字 ALIGN，t=157.652 s下降开始。主审汇总下降后30个节流 waiting 样本：11个仅高度不满足、19个 XY与高度均不满足；没有仅XY不满足样本。它们是日志采样计数，不能当连续阻断时间占比。

本轮进一步配对 full/analysis.json 中 /navigation/setpoint_mission 与 /navigation/local_pose，使用同一 camera_init 坐标、源时间最近样本（间隔≤0.1 s）。源时间相对 bag 起点163～188.64 s，共512对，最大时间差0.0195 s：

- 所有512个发布高度均为 **AGL 0.400000 m**，已到原下降目标。
- 实测位置到发布指令的三维距离最大 **0.10963 m**，低于当前共享0.4 m限距。
- 实测 AGL 范围0.44949～0.50215 m，511/512对高于0.45 m。

**因此该持续偏高阶段并未被0.4 m共享限距截断。统一提高到0.6 m不会改变这些已到终点的指令，也不能作为解决高度稳态偏高的建议。** 起始下降瞬态可能受共享限距影响，但不能外推为后续约31秒的唯一根因。高度参考/坐标转换与飞控实际跟踪偏差仍待主审结合同步记录区分。

## 5. 1.0/0.6 m 两档的实际影响与2025对照

现有源码没有“全部阶段最低0.6 m”选项。08 competition的巡航1.0 m已启用；实测走廊几何由 trial_motion.py:38、:81生成**空段0.6/门口0.4 m**。仅修改 following.corridor_lead_m也会被生成器/走廊调度覆盖。门平面进入慢档距离0.75 m、退出0.95 m；距H≤0.8 m也选0.4 m（corridor_speed.py）。过期位姿还会以更高优先级把两参数压到≤0.2 m（manager:561）。

| 场景 | 改成巡航1.0/其余最低0.6 m的影响 | 判断边界 |
| --- | --- | --- |
| 开阔巡航 | 当前已有1.0；没有由0.6抬到1.0的新收益可认领 | 不改变1.2 m/s规划上限，也不保证实速 |
| 近墙导航 | 从0.2提高到0.6，会扩大沿轨迹指令距离与控制限距；可能缩短接近，但也增大弯道切角/跟踪误差需求 | 本两包该档有效导航区间短，不能认领整个ALIGN时间节省 |
| 门口 | 空段已是0.6；把门口0.4提高到0.6将取消已有局部减距，增加弯道两侧的指令跨度 | 80cm门配55cm名义包络仅每侧12.5cm几何余量；这不是本次实测净空，需实测门宽/姿态/轨迹误差 |
| ALIGN/投递 | 提高共享限距会改变大距离XY补偿及下降初段，可能同时加快XY/Z | 不改变4cm释放、高度窗口或许可；对已到0.4m指令却持续偏高无直接作用 |
| 控制末段/H | 距终点小于0.6时仍发布真实终点，0.6是上限而非强制保持0.6误差；大误差接近时可更激进 | 不绕过H独立停稳/POSCTL门槛；当前缺H录段，无H失效结论 |

轨迹算法同时约束前进弧长与距实测位置距离，并保留最多0.4 m进度投影窗口；提高 target_dist 到0.6不会扩大这个投影窗口。tracking_hold触发距离也从 L+0.05 随 L提高，可能推迟保护，不能作为已修复规划停滞的证据。控制器最终朝采样点的直线跟随不等于全程沿避障曲线的已验证轨迹。

主审有效导航统计（仅到ALIGN前）：bag6近边界0.2档 **5.145 s、净移0.278 m**；full **3.819 s、净移0.342 m**。净移不是路径长度，也不是慢档损失时间；不能把后续20～42秒对准投递计入该档的提速收益。

2025对照使用可定位旧集成源码提交 **0ddc845b8fe9672186d10d270a40f49679881498**：

| 配置 | 2025旧主集成基线 | 当前两包competition |
| --- | ---: | ---: |
| planner max_vel / max_acc | 3.0 m/s / 2.0 m/s² | 1.2 / 1.0 |
| traj_server target_dist | 0.6 m | 分档1.0/0.4/0.2及实测走廊0.6/0.4 |
| patrol_real px4_max_distance | 1.0 m | 同步跟随当前任务档 |

旧基线文件：patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_real.launch、src/patrol_control/config/patrol_real.yaml。当前保留的Desktop历史副本 target_dist 为0.4 m，与旧主集成0.6 m不同，不能混称同一个2025配置。旧算法也不等同当前弧长投影/保护链；3.0是规划上限，不是实测全程速度。

当前competition近边界模型距离 v²/(2a)+0.3v+0.15 = **1.71 m**（v=1.2、a=0.6），随后还考虑边界margin/滞回；它是策略假设，不是飞控实测停止距离。若另外提出巡航速度1.0 m/s，在相同a=0.6下为1.283 m；这是独立降速候选，不能与前视1.0 m混用。本轮只计算，未改配置。

## 6. 聚焦结论与待验证

**已确认：** fetch完成，远端与本地均421e6023；基线后唯一变更为起飞终点交接和Z领先默认回退。红十字4cm、先补偿再下降、巡航1m、走廊0.6/0.4均已在基线。ALIGN绕过规划前视，保留共享三维限距；full持续偏高时该限距并未截断下降目标。

**待验证：** 实机二进制来源；持续高度正误差的坐标参考/飞控跟踪原因；近墙及门口提高前视的收益与误差。主审/专项代理发现full末尾 start_occ=1、碰撞预报反复规划失败，此处应按地图占据/规划失败分析，不能先归因慢前视。原bag末chunk截断、当前无H录段；H失败分析等待补充记录，不以此报告推断。

本轮验证为Git差异、源码分支/参数传递核对、既有JSON/日志读取及rl_drone内纯Python档位/制动数值计算。未启动replay或节点，未重跑已有测试；运行源码、配置、Git索引及工作区既有文件均未改动。
