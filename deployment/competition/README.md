# 独立正赛入口（本地候选）

field.example.yaml需按现场实测确认；缺门/H/墙面参数不能生成飞行配置。本轮不部署。

### 2026-10-08 本地整机候选回流

[本轮说明](../../docs/verification/flight_candidate_simplification_20261008/B_LOCAL_CANDIDATE.md)，整机来源 `87fb2726`。保留现场值与历史验收配置；默认运动优化/高位续扫开启，正确旋转补偿与最终释放保护保留，H高位等待简化与更低POSCTL交接、有界恢复独立候选已适配。实验投影不入正式链。本轮不上板，原执行机构资产不覆盖。


### 2026-10-08 正式高度、走廊前视与三扫描线选项

当前可选模板为 field.example.yaml、candidates/rectangle_motion.yaml、candidates/snake_motion.yaml 与 candidates/snake3_motion.yaml；入口必须显式指定 --site-config，不会自动替换路线。四份模板统一投递 **FC AGL 0.35m**、软件FC限高 **AGL 3.2m**、运动优化/高位续扫开启、navigation_recovery.enabled=false。规划速度上限仍为1.2m/s，加速度上限仍为1.0m/s²，原检测、确认、槽位补偿与释放门槛保持原配置。

走廊调度已显式写入基础模板，不再为null：开阔段前视0.6m，门前/入场下降/近H前视0.4m；进入慢档距离0.75m、退出慢档距离0.95m，近H半径0.8m。巡航前视1.0m、精密前视0.4m，走廊兜底与LAND/HOLD/ABORT共享0.4m。这些是跟随前视距离，不能称作飞行速度。墙平面Y=±1.6m为既有设计参考，须现场复测并与走廊航点/H一并确认；所有模板仍为site_confirmed=false，不自动填入现场通过点或H。

高位FC/镜头高度分别为：基础矩形2.60/2.44m、矩形与蛇两候选2.76/2.60m、蛇三候选2.16/2.00m。蛇三原始设计见 [三线来源](../../docs/planning/serpentine_20261004/PLAN.md)，六航点为(1.00,-3.95)、(1.00,4.10)、(3.70,4.10)、(3.70,-3.95)、(6.40,-3.95)、(6.40,4.10)。生成器按known_rig中0.16m垂直偏移校验镜头与FC高度，并将相对XY加上实际静止FC参考。三线候选可由工作台直接选择其确切文件路径，未改动工作台或控制代码。

离线入口可用 preview --check-config 核对未确认模板；实际离线生成须使用填好实测几何的配置，加 --output-dir /tmp/... --fc-reference X Y Z，不会启动ROS。check_wiring.py --profile <模板> 使用明确的测试几何，仅展开launch并构造运行对象，保留所选模板调度验证最终参数。历史field_20261007_validated.yaml和有限场地恢复候选均不推广/改写。本轮模板未实测，未启动仿真或板端。
