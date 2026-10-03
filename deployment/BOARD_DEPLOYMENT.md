# 板端部署与试飞分支

2026-09-29当前：[上板更新与bag-only录制](../docs/deployment/board_refresh_20260929/README.md)。SSH 192.168.43.59，部署目录liftrace_board_trials_20260928，含昨夜建图/视觉READY、记忆修复及高速入口；构建与离线检查通过，未启动试飞。

> 2026-09-29：八组共用入口已改为定位一致后才启动空白FreeDOM，完整视觉链持续输出后才READY；保留静态TF及25/20/10cm膨胀。[启动修复、高位门槛与昨晚记忆轮结论](../docs/deployment/mapping_startup_20260929/REPORT.md)。仅离线验证，尚未上板实测。

当晚追加：现场五组快捷入口的有投递flight已按用户指令选择真实舵机；preview、仅记忆和原八组默认不变。当前等待板端网络恢复后核实服务和坐标，再开始试飞。

2026-09-28现场五组已按前方6m、左右±1.5m准备并增加自动bag；本次不含H/走廊，[顺序、范围与启动命令](site_20260928/PREPARATION.md)。

2026-09-28最新现场部署：新IP **192.168.156.193**，独立目录 **/home/orangepi/liftrace_board_trials_20260928**。八组及五分类RKNN已完成板端构建、离线初始化与NPU样帧验证；现场命令见[部署操作说明](site_20260928/README.md)。默认模拟投递，尚未启动飞行。下文旧IP/旧目录为历史记录。

2026-09-28：[新增两轮高位巡航复盘与研究版适用性](../docs/deployment/flight_pair_20260928/REPORT.md)。真实装甲车先记成bridge，低位改类未更新冻结事务，后续真bridge已确认却被已投类别过滤；包含八份回放，在线代码未改。

2026-09-27晚：试飞组`high_view_priority_search.launch`实飞bag的[复盘与panzer/H优化计划](../docs/deployment/flight_review_20260927/REPORT.md)。该轮是高位航点中断链，未运行本八组的先建队列再重访策略。

2026-09-27当前：[导航仓“板端参考分支”交接说明](../docs/deployment/board_reference_20260927/README.md)。八组已对齐现场负载/建图档案，膨胀25/20/10cm；新配置只做离线验证与构建，未实跑。

**2026-09-27改参前的入口验收为[八组模块](board_trials_4x4/MODULES.md)：已完成同链SITL及全程录像；6组完整通过，走廊相关2组保留接触结果。没有更新实机或试飞组的“板载代码”分支。该历史轮膨胀27.5/20/10cm，静态TF与顶棚关闭继续继承。[报告](../docs/verification/board_modules_20260927/REPORT.md)**

2026-09-26候选更新：四套继承试飞组fa621262相机/槽位与参数化恢复；统一25/20/10cm膨胀，高位使用中部柱。已本机构建，未上传实机。[差异与操作边界](../docs/planning/obstacle_board_alignment_20260926/REPORT.md)。

分支：`feat/board-deployment-flight-20260920`。以当前板端专项成果为基础，专门保存部署入口、现场修复、试飞配置及结果摘要；不合入main、不替换正赛基线。

## 当前可维护入口

| 入口 | 目标 | 结束方式 |
|---|---|---|
| [视觉中断](board_trials_4x4/01_visual_interrupt/README.md) | 1.4m直飞，中断、对齐、一次模拟投递 | 原地降落 |
| [高位记忆重访](board_trials_4x4/02_high_view_revisit/README.md) | 2.6m完整一圈，重访已记忆的1–3目标 | 最后目标处降落 |
| [H降落](board_trials_4x4/03_h_landing/README.md) | 前方约2m H，接近、定点升高、对齐 | H上降落 |
| [走廊＋H](board_trials_4x4/04_corridor_landing/README.md) | 按实测点自主避障，末端H对齐 | H上降落；航点/H留空时拒绝运行 |

新增入口：[低位连续多投](board_trials_4x4/05_low_multi/README.md)、[高位提前中断](board_trials_4x4/06_high_priority/README.md)、[只记忆不投递](board_trials_4x4/07_memory_only/README.md)、[三投接走廊/H](board_trials_4x4/08_full_mission/README.md)。

八组统一默认：`alignment_mode=legacy_static`、`virtual_ceiling_enabled=false`、巡航上限0.5m/s。单位静态TF采用现场旧板端口径；已知相机/IMU外参和双向数值适配保留。不要同时启动实测对齐与单位静态TF。静态TF不证明两套估计器绝无误差。

共同修复：自动地面基准；float/double高度比较一致；READY同时要求视觉与控制设定点；只有经历已解锁IN_AIR后落地上锁才自动收尾；高位转低位也不重新开启顶棚；模拟投递服务独立隔离。目标高度、控制Z限幅与水平障碍柱仍有效。

板端现目录：`/home/orangepi/liftrace_board_trials_20260920`，当前SSH地址`10.231.47.193`（网络变更后重新确认）。[完整操作说明](board_trials_4x4/README.md)含构建、模型、初始化和任务启动。

## 设备与相机

MAVROS和MID360 driver2按现场既有配置先启动；视觉测试还必须有相机图像与CameraInfo。相机未启动时可在工程根目录另开终端运行：

```bash
bash deployment/board_trials_4x4/start_camera.sh /dev/video0
```

如果现场相机进程已在发布同名话题，复用该进程，不重复启动。四套测试入口不自动解锁或调用任务开始服务。应用READY只表示输入/控制输出就绪；起飞、控制就绪和航线启动是后续步骤，不能把它们混为一件事。

仓库不含RKNN权重、build/devel、原始bag/视频。模型使用板端已有`runtime_models/flight_5cls_20260928_fp16.rknn`，元数据在`vision_ws/src/uav_vision/config`。不能只复制某个settings.yaml到旧包而忽略匹配源码与消息版本。

## 现场旧工程留档

[旧板端4×4参考镜像](onboard_obstacle_reference_20260920/README.md)保存现场原始源码/配置、4×4说明、其他现场设计笔记及四轮茶几实跑参数快照。该目录有CATKIN_IGNORE，在活动工作区之外，供协同差异分析；不是新四套的运行overlay。

## 本轮实际飞行

[2026-09-20日志复盘](../docs/deployment/board_flight_20260920/REPORT.md)：首轮日志与录像已取回，0.4→1.2m约6.13s；任务全程IDLE、无模拟投递。第二轮任务已启动，第一点通过，第二个固定终点被膨胀地图占据，12秒后ABORT；详见[地图占据专项分析](../docs/deployment/board_flight_20260920/MAP_ABORT_ANALYSIS.md)。两轮均不能标作专项通过。


## 2026-09-26 试飞组长期维护来源
实际板载代码由试飞组维护在 https://github.com/sakelier/liftrace-controlwork/tree/板载代码 。本地部署分支与研究分支不自动覆盖该来源；分析每轮bag时先对照其实际revision和未提交修改。9月25日最简投递测试与fa621262的对照及提交时间限制见 [实飞报告](../docs/deployment/flight_review_20260925/REPORT.md)。


## 2026-09-26 渐进式试飞安排
已完成走廊避障与低空红十字中断的能力继续保留，下一步补齐真实释放及恢复，再到连续多投、高位记忆重访、走廊接H与整场。详见[阶段化方案](../docs/planning/staged_flight_20260926/PLAN.md)；该文档明确四套旧专项与最新板载入口的适配缺口，不表示可直接照旧配置启动。当前任务顺序以[ROADMAP](../VISION_2026_ROADMAP.md)为准。


## 2026-09-27 模块化专项更新

八组专项共用最新障碍柱、视觉记忆与坐标适配实现，原四组同步维护。请以 [八组说明](board_trials_4x4/MODULES.md) 为入口；默认模拟投递，已有板端舵机接线由显式实投入口继承。仿真入口独立，实机入口不自动解锁、不自动调用任务启动服务。


## 2026-09-28 五分类模型入口

八组共享 `flight_5cls_20260928_fp16.rknn` 与 `vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml`。默认模型名已更新，可用 `--model <路径> --metadata <匹配YAML>` 显式选择。仅换模型不要沿用六类表：red_cross现在是输出ID4；内部ROS消息仍使用类别名，任务/槽位接口不变。旧模型回退必须两个参数一起指定。

模型包单独交付；仓库只含配置、适配、工具与报告。默认模拟投递、显式实投入口、现场接线、静态TF、关闭虚拟顶棚、25/20/10cm膨胀保持原值。新增六组检查排除走廊两组；本次结果见 `docs/verification/model_five_class_20260928/REPORT.md`，不能沿用9月27日旧参数下“6组通过”的结论。板端NPU/实投仍需现场验收。


## 2026-09-29 新增高速拍摄计划

[高速飞行拍摄专项](../docs/planning/high_speed_capture_20260929/PLAN.md)用于五类靶与H负例在0.5/1m/s、正常/较暗光照下的对照。只采集不投递，板端录bag、本机合成四种回放。第09组入口现已完成离线检查与构建，尚未实飞；原八组速度和现场结束方式未改。


2026-09-29：[高速拍摄第09组操作](board_trials_4x4/09_high_speed_capture/README.md)。现场快捷入口capture/6，支持--capture-speed 0.5或1.0，仅采集不投递。
