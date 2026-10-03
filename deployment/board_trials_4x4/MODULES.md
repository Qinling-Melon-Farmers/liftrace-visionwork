# 八组板端专项与同链仿真（2026-09-27）

2026-09-28现场五组已按前方6m、左右±1.5m准备并增加自动bag；本次不含H/走廊，[顺序、范围与启动命令](../site_20260928/PREPARATION.md)。

2026-09-27晚：试飞组`high_view_priority_search.launch`实飞bag的[复盘与panzer/H优化计划](../../docs/deployment/flight_review_20260927/REPORT.md)。该轮是高位航点中断链，未运行本八组的先建队列再重访策略。

2026-09-27后续：[板端参考分支与最新负载/建图档案](../../docs/deployment/board_reference_20260927/README.md)。八组已继承现场FAST-LIO/FreeDOM配置，水平膨胀0.25m；本次改参后只做离线检查与构建，尚未重跑。下文6PASS/2INCOMPLETE是此前0.275m版本。

本次在 `feat/board-deployment-flight-20260920` 继续维护，原四组也同步更新。保留现场静态 TF、关闭虚拟顶棚、自动地面参考、已知相机外参和投递槽偏移。仿真结果见[八组验收报告](../../docs/verification/board_modules_20260927/REPORT.md)；配置能展开、单元测试通过不等于实飞通过。

| 目录 | 测试流程 | 结束条件 |
| --- | --- | --- |
| 01_visual_interrupt | 1.4m 直飞，视觉中断、接近、对齐、一次投递 | 恢复到低空高度后原地降落 |
| 02_high_view_revisit | 低位入场、2.6m 完整高位环线、记忆 1–3 个高权重目标、逐个重访 | 已记忆目标全部投完后在最后目标附近降落；不追加全场补搜 |
| 03_h_landing | 低位到前方约 2m 的 H、定点升高识别、视觉对齐 | 降落 H |
| 04_corridor_landing | 实测引导点、在线自主避障、H 前定点升高 | 降落 H；出厂航点留空，必须测量后填写 |
| 05_low_multi | 低位搜索、中断、投递、恢复搜索，默认两投 | 完成指定 1–3 投后原地降落 |
| 06_high_priority | 高位搜索，满足当前 TOP3 支持条件后提前中断、低位重访 | 已冻结目标全部投完后在最后目标附近降落 |
| 07_memory_only | 完整高位环线，只记录目标，规划下降 | 无投递指令，完成记忆后原地降落 |
| 08_full_mission | 高位搜索、低空重访三投、走廊、H | 完整任务；走廊引导点和 H 坐标必须实测填写 |

共同默认：4×4m 工作区；固定起飞坐标系 +X 向前、+Y 向左；巡航 0.5m/s、加速度 0.35m/s²；低位飞控中心离地 1.4m，高位 2.6m，投递高度由 `drop_agl` 单独配置。03低位接近使用1.0m，H识别升至1.8m；04/08走廊高度按各实测航点配置，本次仿真夹具为1.0m，04的H前transit默认仍1.4m。地面静置时采样飞控位置，利用已知起落架高度建立地面零点，不把 local Z=0 当成相机到地距离。

## 继承了哪些板端成果

- `map → camera_init` 使用现场已用的单位静态 TF；`navigation_frame_adapter` 同时转换位姿、里程计和任务设定点。静态 TF 只描述两坐标系的固定关系，不是静态点云地图。
- 虚拟顶棚始终关闭（`virtual_ceil_height=-0.1`），包括高位下降切低位、最后一投切走廊的参数阶段。控制高度上限仍保留。
- 正常三维膨胀使用水平 0.25m、上 0.20m、下 0.10m；0.10m 体素使有效离散边界需要按实际地图检查，不能等同于精确圆形半径。
- 四种高位模块（02/06/07/08）启用最新树冠中部障碍柱，低位/H/走廊专项使用真实三维地图。障碍柱采用分量约束和有限填充，不再把大型连通环带整体封死。
- 高位单帧类别框置信度 ≥0.60 可形成导航粗线索，不等于投递许可。提前中断仍要求三个高权重类别满足一致观测支持条件；2026-09-28已取消panzer额外精修特判。低空确认相邻有效观测可间隔 1s；对齐稳定帧与释放许可不放宽。
- 继承轨迹进度、目标身份和 FSM 恢复修复；共享恢复高度按 float32 对齐，恢复设定点额外高 0.10m，避免等于阈值却不能完成交接。
- 相机旋转与像素补偿继承 `known_rig.yaml`；槽位偏移继承板载源码中的三组不同偏移。这些偏移仍是旧控制的固定坐标补偿，不宣称已经完成机体系刚性外参标定。

来源 revision 和同步文件详见 `docs/deployment/modular_trials_20260927/sources.json`。

## 板端运行

先编译本分支两个 workspace，参考 `deployment/BOARD_DEPLOYMENT.md`。每组目录的 `start.sh preview` 检查定位、地图、视觉和bag录制；`start.sh flight` 启动控制输出，仍由现场执行解锁/OFFBOARD与任务启动。不要把仿真的自动解锁入口带到实机。

```bash
bash deployment/board_trials_4x4/01_visual_interrupt/start.sh preview --model /实际路径/model.rknn
bash deployment/board_trials_4x4/01_visual_interrupt/start.sh flight --model /实际路径/model.rknn
```

默认 **模拟投递**，独立 `/board_trials/mock_servo` 只返回软件 ACK，不连接 PWM。实投仅在 01/02/05/06/08 提供 `start_real.sh`，复用试飞组已有的 `/legacy/Servo_raw`，不重写舵机驱动。实投启动前检查该服务类型为 `patrol_control/Servo`；控制请求仍经 `/board_trials/Servo` 许可代理，不能绕过释放条件。

```bash
# 现场确认机构及实投条件后，才使用：
bash deployment/board_trials_4x4/01_visual_interrupt/start_real.sh --model /实际路径/model.rknn
```

03/07 不允许真实投递；04 同样没有投递阶段。04/08 的空走廊配置会明确拒绝启动，不自动使用仿真航点。每次保留下视原始与视觉叠加视频、坐标/候选/许可事件、飞行轨迹和结束结果；缺失候选、未投完、失败后降落均不能记为成功。

## 同链仿真

`common/uav_board_trials/launch/simulation.launch` 复用上述板端配置生成器、任务管理器、控制、Frame Adapter、释放代理和录像节点。区别仅为 Gazebo/PX4/LIO 模拟输入、笔记本 PyTorch 后端、模拟舵机及仿真自动启动/观察器。板端 `trial_manager.py` 仍拒绝模拟时钟，独立 `trial_sim_manager.py` 要求模拟时钟、统一 run 目录与 mock/none 执行器。

八个场景由 `prepare_simulation.py` 分别生成：直线靶、树与高位环线、前方 H、两扇 80cm 门、顺序两靶、提前中断三靶、只记忆两靶、三靶接双门/H。场景工作区为 4×4m，起飞边缘后方另留 0.6m 起飞缓冲区；该缓冲区只用于仿真起飞净空，不扩展任务航点。

仿真 MAVROS 输入使用 `map` 标签，适配后为 `camera_init`，与板端接线一致。真值坐标仅生成物理场景及验收图，不输入任务识别或投递控制。

```bash
cd /实际路径/本仓库
UAV_WS="$PWD/patrol_uav_ws-patrol_planner" VISION_WS="$PWD/vision_ws" \
SIM_STORAGE_GUARD_PATH=/mnt/f SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 \
bash top_level_scripts/sim_run.sh board8_01_visual_interrupt \
deployment/board_trials_4x4/common/uav_board_trials/scripts/run_simulation.sh \
visual_interrupt /实际路径/best.pt
```

`SIM_NO_RECORD=1` 仅关闭宿主屏幕录制；每组 Gazebo 俯视、相机原始、视觉叠加三路录像始终启动，保存在该 run 的 `generated/`。其他 trial 名称为表格目录去掉数字后的对应注册名：`high_view`、`landing`、`corridor_landing`、`low_multi`、`high_priority`、`memory_only`、`full_mission`。

## 本次验收与导出

八组均实际运行并录制；01/02/03/05/06/07完整通过，04和08在走廊门边记录55cm包络接触后停机，08此前完成3投。按用户意见不继续走廊专项修复，不把现场成功记录替代本次仿真结果。详见[报告与时间表](../../docs/verification/board_modules_20260927/REPORT.md)和[回放入口](../../docs/verification/board_modules_20260927/index.html)。

每组有原相机、视觉叠加、Gazebo俯视三份录制，以及根据记录生成的同屏航迹/高度回放。后者明确标注重建，不冒充Gazebo新录像；地图上红色十字为仿真场景真值，仅用于事后显示，绿色/黄色为新鲜视觉投影/粗线索。

工程根目录、rl_drone环境下：

```bash
python deployment/board_trials_4x4/common/uav_board_trials/scripts/report_simulation.py --finish-videos --dashboard
python deployment/board_trials_4x4/common/uav_board_trials/scripts/render_trial_replay.py <run目录>/generated
```

大视频保留在logs，不进入Git；index.html相对链接依赖同一工程下对应run目录。仅克隆源码不会同时得到视频，分享时需连同相应generated目录交付。


## 2026-09-28 五分类模型入口

八组共享 `flight_5cls_20260928_fp16.rknn` 与 `vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml`。默认模型名已更新，可用 `--model <路径> --metadata <匹配YAML>` 显式选择。仅换模型不要沿用六类表：red_cross现在是输出ID4；内部ROS消息仍使用类别名，任务/槽位接口不变。旧模型回退必须两个参数一起指定。

模型包单独交付；仓库只含配置、适配、工具与报告。默认模拟投递、显式实投入口、现场接线、静态TF、关闭虚拟顶棚、25/20/10cm膨胀保持原值。新增六组检查排除走廊两组；本次结果见 `docs/verification/model_five_class_20260928/REPORT.md`，不能沿用9月27日旧参数下“6组通过”的结论。板端NPU/实投仍需现场验收。

## 2026-09-28 后续：取消panzer特判

八组当前生成配置统一`interrupt_refined_classes: []`。06/08可凭两帧一致的panzer粗线索参与TOP3提前结束；02/07仍完成各自完整环线。低空复核与释放许可不变。本次仅离线验证，之前六组结果不代表取消后的动态验收。完整[策略、逐类退化和增强明细](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/high-view-search-research/docs/planning/panzer_five_class_20260928/VALIDATION_AND_AUGMENTATION.md)。


2026-09-29：板端专项已改为bag-only，不部署独立相机录像/转码/合成脚本；本页SITL录像入口仍为笔记本专用。[部署检查](../../docs/deployment/board_refresh_20260929/README.md)。
