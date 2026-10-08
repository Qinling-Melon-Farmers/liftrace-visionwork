# 投递简化、末段净空与时序日志复核

日期：2026-10-08。工作树：F，`/home/xhj/liftrace-worktrees/r2026-board-frame-fix`。来源基线为 `b520ee66`；本文复核本轮尚待主代理统一提交的集成源码，不把历史 seed38 运行归为本轮动态验收。旧链保护快照由主代理统一保存于 `legacy_baseline/20261008/simplified_drop_h_recovery/`。

本次完成投递行为简化、局部下降净空准入、相关离线测试及少量现有日志补充。没有启动 ROS 主节点、SITL、实机服务或 SSH，没有部署、提交或跨仓同步。整包构建由主代理执行和记录。

## 投递行为

高位沿原可靠视觉流程取得当前事务的身份、类别、几何质量、精修中心、观测新鲜度与不同图像的连续确认，继续使用原配置的像素中心容差。控制器在高位跟随原靶心；保存补偿 FC 参考不等于先要求飞机到达该参考。

此前高位将释放级 XY 误差、速度、连续 0.30 s、不同里程计样本及高度停稳作为视觉计图/冻结的前置，本轮取消这些高位运动门槛。图像及反馈的上下文匹配、frame、时间戳、原有有界未来暂缓/缓存、质量和类别要求仍保留。未冻结图像不能因为控制反馈有效而跳过原像素中心要求。

视觉确认且末段净空许可后，冻结本事务靶心与完整姿态计算的补偿 XY：

```text
FC_ref_XY = target_center_XY - [R(q) × body_slot_arm]_XY
```

R(q) 包含完整姿态；三槽使用已确认的机体 FLU 后/右/左 12 cm 杆臂，不用地图固定轴偏移替代。既有圆环/红十字下降分支同时选择冻结的补偿 FC XY 和释放高度，允许剩余完整杆臂补偿与下降并行。后续图像缺失或残缺不重捕、不覆盖冻结参考，重复回调不累加杆臂。

真正机构 RPC 仍必须通过原最终释放窗：

| 条件 | 原门槛 |
| --- | --- |
| FC 对冻结 FC、实际投口对冻结靶心的 XY 误差 | 两者均 ≤ 0.04 m |
| FC/投口水平速度取较大值 | ≤ 0.05 m/s |
| 相对释放 FC 高度的误差 | ≤ 0.05 m |
| 垂向速度绝对值 | ≤ 0.10 m/s |
| 不同、新鲜的递增源时间戳 odom | 至少 3 个 |
| 合格样本连续跨度 | ≥ 0.30 s |
| odom/pose 新鲜度、样本间隔 | 原 0.20 s 上限 |
| 其他保护 | 原控制状态、任务绑定、有效释放许可、异步及防重复保护 |

实际投口和其速度仍用当前姿态、当前 FC 及角速度计算。因此 FC 到位不能掩盖旋转造成的投口偏移。单次位置到位、相同 odom 重复消费、清晰图像或净空许可都不能独立代替最终释放窗。

## 末段净空准入

复用现有 `/freedom/static_pointcloud` 的真实源时间戳和地图 frame；话题与参数通过 `drop_system/clearance/*` 配置。没有新节点、查询服务、消息契约或恢复状态机。没有接入实验投影。

选择原始地图的原因：当前 `/sdf_map/occupancy_inflate` 发布没有填写源时间戳，不能拿接收时间冒充新鲜源图。原始点云按实体包络及体素尺寸检查。

每个地图回调最多接受 250000 个输入点，并筛成当前 FC 周围默认 ±2 m 的局部 XY 缓存。控制/图像回调只扫描该缓存，不每 tick 扫全图。整个输入超限、空图、缺时间戳、frame 错误、格式或数据长度错误均拒绝，不能丢弃超限点后将其当净空。

检查当前 FC、冻结靶心、补偿 FC 的 XY 包络与当前到释放 Z 的组合棱柱。这一保守包络覆盖允许的水平补偿与下降插值，不仅检查两个端点。默认 XY 实体半径 0.39 m 覆盖 55 cm 正方形的任意 yaw，再加入既有 tracking reserve 和半个体素；Z 默认 FC 上 0.20 m、下 0.22 m，并加体素半格。FC 下 22 cm 沿既有静置测量，上方及水平使用现有保守包络，并非本轮新增实测。

| 参数 | 默认值 |
| --- | --- |
| map_topic | /freedom/static_pointcloud |
| map_max_age_sec | 2.0 s |
| local_radius_m | 2.0 m |
| voxel_size_m | 0.10 m |
| body_xy_radius_m | 0.39 m |
| body_up_m / body_down_m | 0.20 / 0.22 m |

冻结、冻结后的下降 tick、机构 RPC 准入三处都实际调用该检查。占据、过期、未来、未就绪或缓存覆盖不足时保持冻结目标，停止后续下降指令并重置最终释放窗；不会为等待而重追低位残圈。明确超界的补偿目标在生成时拒绝，清掉此次可用几何/就绪状态并保持当前位置。

拒绝沿原 action deadline 与任务换点流程处理，没有新增立即失败接口，不能声称已实现 fail-fast。外墙 bounds 拒绝沿现有 `boundary_policy.enabled`；使用入口必须提供当前场地的有效边界配置。局部净空并不代替原 APPROACH 的规划，也不授权障碍内斜向下降。

地图有效覆盖假设与现有 GridCost 一致：全局非空、frame 正确且新鲜的源图提供既有地图覆盖前提，局部无占据点才能通过；它不提供射线观测/未知空间证明。保守棱柱可能拒绝实际可行的窄路径，这轮没有增加绕行恢复。

## 日志与时间节点

只在现有状态变化及现有窗口返回处补充日志，复用当前变量，不增加新状态。既有日志前缀保留，便于继续使用历史解析入口。

| 时间节点 | 当前可读取的内容 | 解释边界 |
| --- | --- | --- |
| 投递视觉确认、冻结并准备下降 | `[DropGeometry] frozen` 增加 `stage=enter_descend`、decision、capture/release/当前 FC Z；原中心、FC、图像源戳保留 | 标识控制事务切换；实际首条降低高度的 setpoint 可能在后一个 tick，不能把日志时刻等同物理运动开始 |
| 最终释放窗成功 | `release_window_ready` 记录 FC/投口 XY 误差、横速、vz、实际 FC Z、释放/ceiling Z、span、samples、odom 源戳 | 证明这一评估通过最终物理窗；其后任务/许可仍须有效 |
| 机构请求排入异步执行 | 原 `async Servo submitted` 增加 `stage=release_rpc`、decision、实际/释放 Z、pose 源戳 | 排入 RPC 不等于原始舵机开始或 ACK；raw 执行仍需代理的 release_result 对齐 |
| 机构完成 | 原 positive Servo ACK、NOT_STARTED、超时/失败日志保留 | ACK 是服务执行结果，不能据此宣称真实包裹落点 |
| H 十帧锁点并准备下降 | 原 `fresh H alignment latched` 增加中心、decision、`stage=enter_descend`、capture/descent/ceiling/当前 FC Z | H 判断由 H 专项实现；本小结只复核时序可观测性 |
| H 低位窗口成功 | `low handoff window_ready` 记录 XY、横速、vz、当前/目标/ceiling Z、span、samples、odom 源戳 | 按当前配置和 controlled descent 规则评估，不能拿历史高位停稳规则替代 |
| POSCTL 模式请求已接受 | `handoff_request_accepted` 增加 mode、decision、Z、XY 和 pose 源戳；原 REQUESTED 事件保留 | 请求成功与实际进入模式分开 |
| 实际模式回执 | 原 OBSERVED 事件及 MAVROS mode observed 日志保留 | 进入 POSCTL 不等于触地或解除武装 |

现有最后拒绝原因仍可定位：最终投递窗 `waiting: <reason>` 含 XY、speed、zerr、samples；净空输出 `drop_map_not_ready/stale/future/frame_mismatch/local_coverage_missing` 或 `drop_path_occupied`、边界拒绝；H 低位窗口输出未通过 reason，handoff 状态/时间失败走原 failed closed 日志；任务许可缺失继续输出 Waiting for release authority。

少数动作绑定/许可检查本身仍是直接返回，完整事务原因还需结合既有 context/permission 消息。1 Hz 节流日志可能略过很短的拒绝原因切换，因此不能单凭最后一条文本重建每个控制 tick。需要精确实测阶段耗时的下一轮，应结合原 setpoint、pose/odom、release_result 与 H REQUESTED/OBSERVED 消息，不将本轮源码日志新增当成已经测得耗时。

## 本轮实际定向验证

均在 F 当前生产源码上执行，无仿真、主节点或硬件调用。

| 用例文件 | 实际运行项数 | 结果 |
| --- | ---: | --- |
| patrol_control/test/test_compensated_drop_geometry.py | 24 | PASS |
| uav_vision/test/test_compensated_drop_alignment.py | 30 | PASS |
| uav_vision/test/test_drop_observation_stamp.py | 5 | PASS |
| patrol_control/test/test_landing_motion_settlement.py | 10 | PASS |
| patrol_control/test/test_async_servo.py | 24 | PASS |

合计 93 个不同用例。投递/净空 24 项在日志修改后重新执行，仍通过，不把重复执行累加到项数。方法提取测试编译并运行当前控制生产方法、圆环/十字实际下降分支、真实几何/最终窗口/异步 helper；独立解析二进制使用实际 sensor_msgs::PointCloud2 C++ 类型和 ROS Time，无 ROS 节点。

覆盖三槽完整姿态杆臂、不累加、可靠视觉后未补偿/高位仍运动可开始下降、段内部占据拒绝、地图时效/frame/覆盖/超限/空图/格式拒绝、冻结保持、clear map 不绕过最终窗口、不同 odom/时长、角速度带来的投口速度、未来观测暂缓、缓存/重复图像、已排入异步动作不重复执行。

旧异步测试 double 只补上新净空接口默认 true 与窗口 reset，保留原 24 个 transport/ACK/撤销/NOT_STARTED 用例；真实净空判断由上述独立投递流程用例验证，不用这个 double 替代生产净空覆盖。

Python 命令采用既有 rl_drone；视觉测试先加载 F 自身导航/视觉 devel overlay。离线方法测试将 ROS 日志宏替代为 no-op，证明新增日志没有改变流程；实际 ROS 日志格式、频率与运行时耗时仍须同版运行核验。主代理另行执行最终增量构建与 catkin 收集验证。

## 未验证范围与下一步

没有本版三槽动态投递、整场飞行、板端稳定性、实物释放落点或本版 POSCTL 后触地验收。历史成功/失败 run 保持原版本结论，不引用它们作为这一轮已通过。

后续获得当前轮明确授权后，使用原单实例包装器定向检查：视觉冻结后首条降低 Z 的实际 setpoint、补偿/下降路径与地图拒绝、到释放高度后实际最终窗等待、每槽一次 raw 开始及 ACK；H 则核对低位窗口、REQUESTED、真实 POSCTL OBSERVED 和触地/解除武装。当前不自动开跑。
