# 独立正赛入口

完整操作、版本来源和待验收范围见 [整机说明](../../docs/deployment/competition_integration_20261003/README.md)。本目录不依赖九组专项。field.example.yaml需按现场测量填写；默认缺走廊/H时不能飞行。

10月4日历史参数、规则/技术会差距、实测FOV覆盖及当时同步状态见 [10月4日正赛复核](../../docs/verification/competition_config_20261004/REPORT.md)。其中生成的runtime/control仅供离线参数分析，不可直接用于飞行。

2026-10-04新增[矩形/蛇形与速度衔接候选](../../docs/planning/motion_optimization_20261004/README.md)。候选位于 `candidates/`，默认模板不变，尚未SITL/实飞验收；按相机2.6m设计，对应当前rig的FC2.76m。

### 2026-10-06 H末段硬件推广

field.example及rectangle_motion/snake_motion候选显式使用landing_handoff_mode: POSCTL，复用已成功H专项的视觉对准下降交接，飞手完成最终降落。本轮未上板，原0.9m观察高度和速度参数不变。无人值守仿真应显式AUTO.LAND；POSCTL现场方案不能直接等同正赛自主落地验收。08专项与正式模板速度/运动开关差异见[核查报告](../../docs/deployment/h_promotion_20261006/REPORT.md)。

### 2026-10-06 本轮已补齐到0928工程

独立入口及依赖已部署192.168.3.126原0928根目录，配置检查及完整接线展开通过；没有启动ROS/飞行。模板仍未确认且门口/H留空，不可直接flight。定位入口采用与专项相同的4–7号大核OpenMP环境，保留原三线程编译。参见[部署与工作台报告](../../docs/deployment/survey_workbench_20261006/REPORT.md)。


### 2026-10-08 独立运行实现回收

比赛模板与两个 candidates 的全部原始数值保留，新增中文测量说明；有限空间试飞的2m搜索、0.5m/s、0.35m/s²、短前视和60s预算没有推广为比赛默认。10月7日成功测试配置单独保留为 field_20261007_validated.yaml，不自动选择。

已回收部署过的限高保持修复和实体PWM3左仓/后仓2100us释放、1700us锁止、被动启动。H仍为原比赛0.9m捕获及POSCTL交接，软件链验收不等同自主落地评分。三线程FAST-LIO、大核绑定和轻量bag保持现有实现。

显式开关与离线生成接口见[CLI契约](../../docs/verification/competition_release_20261008/CLI_CONTRACT.md)，文件清单、测试及本地存档边界见[交接报告](../../docs/verification/competition_release_20261008/REPORT.md)。


### 2026-10-08 默认运动优化与高位续扫

按本轮要求，正式 `field.example.yaml` 与两份候选默认开启运动优化和高位续扫。二者独立：`--motion-optimization on/off` 仅覆盖运动策略，`--resume-survey on/off` 仅覆盖 `survey_policy.resume_survey_enabled`；省略逐项继承所选 YAML。`field_20261007_validated.yaml` 保留历史实跑原值（运动优化开启、续扫关闭），不把新默认写成昨晚已验收。

高位续扫复用共有整场实现：最多一次、原任务总截止时间、当前位置安全竖直上升、回接未完成搜索段并保留重叠，仍要求新鲜位姿/地图和新的缺失类别证据。恢复原高位限高必须经过规划器限高 ACK；不修改释放门槛。当前新增正赛配置接线已离线验证，续扫未在昨晚整场实跑中开启，本轮未启动仿真或实机。

未确认模板允许 `preview --check-config` 展示继承或覆盖后的有效值，输出 `source=validated_settings` 和 `generation_ready=false`。缺实测门/H/墙面不能生成或飞行；不会填入假墙坐标。实际生成后的输出为 `CONFIG_EFFECTIVE`，包含 `runtime.yaml`/`overrides.yaml` 路径及生成配置中的开关值；工作台分别显示“离线配置检查”与“运行已生成配置”。改变 YAML 路径或开关后，旧结果不能确认新选择。

### 2026-10-08 高位等待简化与本地恢复候选

[本轮整机说明](../../docs/verification/flight_candidate_simplification_20261008/REPORT.md)：投递删除高位重复释放级停稳，保留固定靶心、完整旋转槽位补偿与最终4cm/0.05m/s/0.30秒释放检查。H恢复原高位确认，低位保留0.35m目标/0.37m请求上限，0.08m/s水平速度、0.10m/s垂直速度、0.15秒新鲜反馈允许缓降；微小未来状态改为有限暂缓。

三类恢复已接入独立 `candidates/recovery_validated_field_local.yaml`（10月7日有限场地几何），不改历史validated配置；仅该本地候选显式开启恢复，正式模板仍关闭。运动优化/高位续扫默认开启。实验投影未纳入；本轮仅本地构建与离线验证，暂不上板。


### 2026-10-08 正式高度、走廊前视与三扫描线选项

当前可选模板为 field.example.yaml、candidates/rectangle_motion.yaml、candidates/snake_motion.yaml 与 candidates/snake3_motion.yaml；入口必须显式指定 --site-config，不会自动替换路线。四份模板统一投递 **FC AGL 0.35m**、软件FC限高 **AGL 3.2m**、运动优化/高位续扫开启、navigation_recovery.enabled=false。规划速度上限仍为1.2m/s，加速度上限仍为1.0m/s²，原检测、确认、槽位补偿与释放门槛保持原配置。

走廊调度已显式写入基础模板，不再为null：开阔段前视0.6m，门前/入场下降/近H前视0.4m；进入慢档距离0.75m、退出慢档距离0.95m，近H半径0.8m。巡航前视1.0m、精密前视0.4m，走廊兜底与LAND/HOLD/ABORT共享0.4m。这些是跟随前视距离，不能称作飞行速度。墙平面Y=±1.6m为既有设计参考，须现场复测并与走廊航点/H一并确认；所有模板仍为site_confirmed=false，不自动填入现场通过点或H。

高位FC/镜头高度分别为：基础矩形2.60/2.44m、矩形与蛇两候选2.76/2.60m、蛇三候选2.16/2.00m。蛇三原始设计见 [三线来源](../../docs/planning/serpentine_20261004/PLAN.md)，六航点为(1.00,-3.95)、(1.00,4.10)、(3.70,4.10)、(3.70,-3.95)、(6.40,-3.95)、(6.40,4.10)。生成器按known_rig中0.16m垂直偏移校验镜头与FC高度，并将相对XY加上实际静止FC参考。三线候选可由工作台直接选择其确切文件路径，未改动工作台或控制代码。

离线入口可用 preview --check-config 核对未确认模板；实际离线生成须使用填好实测几何的配置，加 --output-dir /tmp/... --fc-reference X Y Z，不会启动ROS。check_wiring.py --profile <模板> 使用明确的测试几何，仅展开launch并构造运行对象，保留所选模板调度验证最终参数。历史field_20261007_validated.yaml和有限场地恢复候选均不推广/改写。本轮模板未实测，未启动仿真或板端。
