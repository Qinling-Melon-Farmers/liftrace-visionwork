# uav_vision 导航组接口说明

交付版本：`20260807-beta1`
包版本：`uav_vision 0.2.1`
边界：视觉只输出观测、地图候选、记忆和像素对准证据，不输出飞行或执行机构命令。

> 版本阅读规则：本文保留 `20260807-beta1` 作为历史交付标识；对正式任务链、
> profile、实测结论或导航职责的表述如有冲突，**第 11 节 2026-08-31 合并态补充优先于
> 旧 beta1 与第 7～10 节阶段快照**。

接收方只有板端原始工程时，先阅读 ZIP 根目录 `INSTALL_AND_SIMULATION.md`。该文档说明
`vision_ws` 与原工程的位置关系、Catkin overlay、板端 RKNN 启动、话题/TF 检查、视觉
mock 以及完整 toudi3 仿真为什么需要额外开发机功能分支。

推荐导航组先在自己的笔记本使用 PT 入口完成整机仿真，冻结 ROS 接口后再上板切换 RKNN；
导航消费者不应直接依赖 PyTorch 或 RKNNLite。

## 1. 运行链

```text
Image + CameraInfo + TF + align_mode
  -> six-class detector
  -> geometry detectors + fusion + target refiner
  -> target map projector
  -> target memory
  -> /uav_vision/targets + /uav_vision/selected_target
  -> drop aligner outputs
```

标准目标必须完成同帧类别与蓝环一对一关联，且地图投影有效，才能进入 operational 候选。

正式 `r2026` 任务链不直接消费视觉原始排序，当前单一所有权数据流为：

```text
/uav_vision/targets
  -> profile_candidate_selector
  -> /mission/profile_selected_target
  -> target_search_manager_py
  -> /navigation/goal_raw
  -> navigation_visual_delivery_adapter
  -> /fastplanner/goal
```

manager 只发布 raw goal，adapter 是正式链唯一 `/fastplanner/goal` 发布者。
`/uav_vision/selected_target` 只用于视觉侧 `P_selected` 统计和“是否绕过 profile”审计，
不是正式 `r2026` 任务入口，不得绕过 `profile_candidate_selector` 直接触发 manager。

## 2. 输入接口

| 输入 | 类型 | 约束 |
| --- | --- | --- |
| 图像 | `sensor_msgs/Image` | queue 1；时间戳和光学 frame 必须真实 |
| CameraInfo | `sensor_msgs/CameraInfo` | 与图像分辨率、裁剪和去畸变状态一致 |
| TF | TF2 | 图像时间存在 `map_frame <- camera optical frame` |
| `/uav_vision/align_mode` | `std_msgs/String` | `disabled/drop_circle/drop_cross/landing` |
| `/uav_vision/reset_memory` | `std_srvs/Empty` | 任务开始/结束或地图重置时调用 |

## 3. 输出接口

### `/uav_vision/targets`

类型为 `uav_vision/TargetCandidateArray`，包含视觉记忆中的全部候选。用于动作前检查：

```text
state >= 2
map_valid == true
map_frame == expected mission frame
association_valid == true
reject_reason == ""
now - last_seen <= 0.5 s
```

`first_seen` 用于确定发现顺序，`last_seen` 是最后真实观测时间；长期地图记忆重发不会刷新
`last_seen`。`map_quality` 是工程质量分，不是定位协方差。

### `/uav_vision/selected_target`

单个视觉排序建议。标准类权重：

```text
tank=5, panzer=2.5, bridge=2, pillbox=1.5, tent=1
```

导航消费者可以结合可达性、任务终态、剩余时间和自身规则改选目标。该话题不是规划目标，
`uav_vision` 不发布 `/fastplanner/goal`。在正式 `r2026` 链中，该话题只作视觉统计与绕过
profile 审计；真正的立即中断提案是 selector 发布的 `/mission/profile_selected_target`。

### `/uav_vision/drop_offset`

输出目标精修中心相对 CameraInfo 主点的 `dx_px/dy_px`。它是像素偏差，不是地图 Pose。

### `/uav_vision/drop_ready` 与 `/uav_vision/release_evidence`

前者只表示视觉像素对准，后者提供目标身份、几何、观测年龄、稳定帧和拒绝原因；二者都
不是释放许可。

### `/uav_vision/alignment_target_context` 与上下文证据

导航 coordinator 在 ALIGN decision 内应以 `uav_vision/AlignmentTargetContext` 周期续租同一
冻结上下文。消息不依赖 `uav_mission`，但 `uint8 command` 与 `NavigationDecision v1` 同值；
默认只接受 `ALIGN=2`。必须填入 `source/header.stamp`、排他 `deadline`、profile、align mode、
`mission_id + decision_seq + semantic_target_id + first_seen + attempt + payload_slot`、目标观测时间
和地图 `target_pose`。`has_target=true` 时 `semantic_target_id=0` 完全合法。

正式链应给 `drop_aligner` 设置 `require_alignment_context=true` 并按配置周期刷新
`header.stamp`，但不得在同一 decision 内修改冻结字段。strict 模式下：

- 上下文缺失、失鲜、inactive、错误 schema/source/profile/command/mode 或 deadline 到达即拒绝；
- r2026 语义目标不准入 tank；full 仍保留 tank 兼容；
- 标准靶按同 frame 地图距离把语义目标绑定到圆环几何实例，二者 ID 不要求相等；
- red_cross 按 `id + first_seen` 精确匹配，防止 stable ID 重用；
- decision/fence、冻结内容或实际几何实例变化时稳定帧从零重新累计。

strict watchdog 独立于目标消息流；即使 coordinator 仍续租，几何观测超龄也会主动撤销 ready，
context 失鲜或 deadline 到达同样 fail-closed。因此 coordinator 必须持续续租，但仅续租不能替代
新的视觉观测，也不能依赖 latched 单发消息。

视觉继续发布旧 `/uav_vision/release_evidence`，同时在
`/uav_vision/release_evidence_context` 内嵌旧证据并回显 coordinator header/source、完整 decision/
语义/几何身份、关联距离及 `context_valid/context_reason`。最终释放许可仍由任务/安全层产生。

## 4. stable ID 和地图记忆

- 地图候选默认保留到 `/uav_vision/reset_memory`；
- 相同物理目标使用地图距离、类别投票和连续帧维持 stable ID；
- 收敛到 0.6 m 内的重复标准目标记录会合并并保留最早 ID；
- 目标成功、失败或不可达后的任务终态由消费者维护，视觉不会代替任务层删除目标；
- 缺 CameraInfo、无效关联、缺 TF 或 TF 过旧时地图候选失败关闭。

## 5. 历史参考导航代码

ZIP 根目录 `reference_integration/` 包含阶段 4 使用的消息草案、候选策略和 coverage manager。
它们依赖主工程中的 `patrol_control/uav_mission/Fast-Planner`，不在视觉工作区参与编译，也不
能从交付 ZIP 直接运行。它们只是旧 coverage manager 的历史回归参考，用于展示以下曾经
验证过的行为：

- 非靶标坐标蛇形覆盖；
- 候选有效性过滤和规则权重排序；
- stable ID 的 delivered/failed 终态去重；
- 规划目标重试、20 秒不可达和恢复索引；
- Mission Manager 独占 planner goal 的集成方式。

导航组可以选择复用、改写或完全不用这些参考文件。视觉正式接口仅为本文件列出的 ROS
输入、输出、消息字段和服务。

特别注意：上述清单不是旧 `manager@5144aa8` 的能力；但截至 2026-08-31，本地 clean 导航
任务核心已在独立来源 `a65a616` 实现持久队列、typed result reducer、有限重试、三槽和
510/600 s 调度，并与 planner telemetry `022b763` 一起导入当前联动历史。该实现仍不是外部
远程 `a68925d` 的内容，也尚未通过 live execution bridge 驱动 planner。不得再把任务核心写成
“未实现”，也不得反向把纯测试写成联合飞行闭环。

## 6. 已验证与限制

阶段 4 无 GUI SITL 完成靶标区域 12/12 覆盖、五类五 ID、权重队列和返航，零碰撞、越界
和 Servo 调用。最终地图误差四类为 2.18-5.72 cm，`pillbox` 为 22.97 cm 离群值。

尚未验收：搜索后的末端对准误差收敛、三次投递、30-seed、OrangePi ROS 相机/TF 和实机。
仿真结果不能描述为 RKNN 板端验收。

## 7. 2026-08-30 profile 与候选准入补充

导航消费者必须同时记录 `class_profile`：

- `full` 保留 `tank/tent/pillbox/bridge/panzer/red_cross` 资产和历史回归；
- `r2026` 只准入 `tent/pillbox/bridge/panzer/red_cross`；
- tank 的资产、权重和 raw 误检诊断不删除，但 `r2026` refiner 不让 tank 消耗蓝环
  一对一关联，其 `association_valid=false`、拒绝原因为
  `class_profile_disallowed`；
- `r2026` operational selected/accepted 中 tank 必须为 0，但 raw 诊断中出现 tank
  不等于资产或分类表应删除。

该 profile 修正不会生成模型未检出的类别。相同 seed、默认 640 的
`pillbox@3.6 m` 复跑仍为 `raw_classifier`、`P_confirm=P_selected=0`，因此不得
将该改动描述成高空 pillbox 召回修复，也不改默认 640 工作点。

## 8. 当前 V-SIM-04 交付边界

13/23 和 20/23 是 2026-08-29/30 的阶段快照。当前补充运行为：

- formal seed=11：23/23 完整，`P_confirm=P_selected=22/23`，终态
  `MEASURED/NOT_GATED`；
- static diagnostic：25/25 完整，`P_confirm=P_selected=24/25`；
- sparse-speed diagnostic：30/30 完整，`P_confirm=P_selected=25/30`。

三组均为 visual-only，`P_interrupt=null`。`P_selected` 不是导航接受，更不是任务中断。
导航只能在同一 stable ID 已 accepted、adapter 实际 `SEARCH→APPROACH`、并发布
`MissionCommand.APPROACH` 时计数。这些单 seed 结果在采用门槛与有效重复数冻结前，
不是比赛漏失概率或联合 Gate PASS。

## 9. 导航任务层需要返回的最小合同

视觉输出是观测和提案，导航/任务层必须对同一 stable ID 返回可重放的生命周期：

1. `candidate_accepted` 和 `approach_started`；
2. 分阶段 `result`，区分抵近不可达、捕获/对准超时、释放拒绝和投递成功；
3. `retry_scheduled/retry_exhausted`，包含 retryable、attempt 和冷却终点；
4. 持久队列摘要，区分 `pending/cooldown/executing/terminal`；
5. 剩余时间、第三投优先、停止覆盖、返航余量和超时终止决策。

每个 accepted/result/retry 事件至少携带 event sequence、stable ID、class、profile、
前后状态、reason、attempt 和 source stamp。导航决策必须显式携带 stable ID，不得在
相邻目标下仅靠空间距离反推。持久队列、重试与剩余时间调度属导航/任务层，
不在视觉 adapter 中复制实现。

本节描述的是 2026-08-30 外部远程快照：导航远程 `main` 为 `a68925d`（仅地图实验），
旧运行 manager 为 `5144aa8`。2026-08-31 后本地 clean 任务核心已实现上述生命周期并通过
纯测试，最新边界见第 11 节；V-CL-06 A/B 仍 FAIL，`a68925d` 不推广，600 s 三投 Gate
尚未启动。

## 10. 2026-08-31 可消费工作域与稳定性更新

视觉侧最新稳定性证据是
`logs/vsim04_soak600b_seed11_20260831_011055/`：vision
`8b3b88cd321469e3b61b6127ec2574d770848109`，wall/source 600.024/564.863 s，输入/完整
mapped 15.019/13.336 FPS，六产物完整、errors=[]。`P_interrupt=null`；该入口没有启动
导航、控制或执行机构。它只覆盖固定五目标、不含 tank 的 Gazebo 合成 D435i + 笔记本
Ultralytics，不是 OrangePi/RKNN、新实物相机、随机场或联合三投证据。run 名含 seed11，
但 soak manifest 没有 seed 字段；tank selected=0 只说明这条路线未选中 tank。

六个边界 trial 在同 revision、同固定 seed=11 下各独立重复三次，聚合为
`logs/vsim04_repeat_aggregate_boundary6-seed11-r3-final-307ac5c4/`。bridge/panzer 低空
2 m/s 均 0/3；pillbox 低空 2 m/s 2/3；pillbox 3.6 m 在 0.5/2.0 m/s 分别 1/3、0/3；
静态 pillbox 3.6 m 0/3。三次源 run 均 exit 0、配置一致、完整 `DIAGNOSTIC_ONLY`；聚合
FAIL 是诊断语义，不是编排失败，也不是多 seed 统计。

导航消费建议据此收紧：默认 imgsz=640 不变，不把 2 m/s 作为所有类别通用搜索速度，不把
3.6 m 作为当前保证识别航高；先冻结更保守航高/航速及 capture 半径，再请求视觉在其邻域
补多 seed 和横偏。视觉算法、模型、阈值本轮未变，formal23/static25/sparse30 不重跑，
引用时保留原 revision。

截至 2026-08-31，外部导航远程仍仅
`main@a68925d15293e5510e2b4351c6b3d9bc5aa136ab`、无 branch/tag，旧运行 manager 来源仍为
`5144aa8f536bdcd214aea2f39ada558383b3bcb0`。本地 clean 任务核心已补齐
accepted/result reducer、持久队列、有限重试和剩余时间调度并通过纯测试；但 live execution
bridge 尚未合入，因此 V-CL-06 A/B 仍 FAIL、导航 600 秒三投未启动。下一联合测试应先接入
typed bridge 与 planner/target-stage 事件，再测真实 `P_interrupt`；`P_selected` 仍不得代替
接受或中断。

## 11. 2026-08-31 V-CL-06 当前合并态

来源必须按三层记录：

- 外部导航远程：`main@a68925d15293e5510e2b4351c6b3d9bc5aa136ab`，仍只是地图实验；
- 本地 clean 导航实现：manager/runtime
  `a65a616f209bfc7dd4d788ebb36609589cea5418`，planner telemetry
  `022b7636b1661304ce2d47e9368c392adc67997d`；
- 本视觉集成仓：导入基线 `7dd2c49acf8eb6f8a58a333195b77096810ad285`，合入一次性
  start gate、冻结对准上下文与 typed evaluator 后为
  `db80dfd6d22f31ab682d02ebb39a52a1e082384e`。

已实现且有纯测试证据：按 stable ID/first_seen 的持久队列、profile 准入、权重排序、有限
冷却重试、三槽、510 秒硬返航、600 秒任务上限、typed `NavigationDecision/NavigationResult`
及 planner `goal_seq` 遥测合同；随机场 READY/truth/anchor/manager-IDLE 一次性 start gate；
`AlignmentTargetContext/ReleaseEvidenceContext` 严格上下文；V-SIM 的
`visual_only|typed_contract|target_stage` 分层评分。新增功能默认关闭或保持 visual-only，旧
`ReleaseEvidence` MD5 为 `07fbec53d6c6a8bdc19fddf37c081d04`，旧默认链不变。

尚未完成的是 live execution 层：当前分支没有合入 bridge，故新 manager decision 尚未实际
产生 `/fastplanner/goal`，也没有 planner/target-stage `STARTED/CAPTURE` 回流。真实
`P_dispatch/P_planner_arrival/P_interrupt`、新合并态 90 秒 smoke 和 600 秒三投/返航/落地
Gate 均保持未验收；不得把 typed-contract 单测或旧 adapter 证据替代它们。

相机侧最新状态为：用户已在仓外完成新相机内参标定，等待 YAML、原始标定图片、标定板规格、
运行分辨率及采集设置后由视觉组复核并接入 CameraInfo。安装外参、CameraInfo ROS 话题/时间戳、
OrangePi 和实拍真值仍未闭合。

## 12. 2026-08-31 PR #6 Ready 与地图阻断解除

本节覆盖上文第 231～261 行的历史状态。导航 PR #6 当前为 Open/Ready，HEAD 为 `d95377c`；
其边界保持为运动执行桥，不包含视觉集成层的 target transaction、LAND、stop ACK 或系统地图
Gate，也尚未合并 `main`。当前视觉集成分支已实际接入单一 planner bridge，正式图只有
`/navigation/planner_bridge` 发布 `/fastplanner/goal`。

仿真地图根因已定位并修复：Gazebo MID360 的 XYZ 点型改走仿真专用解析，传感器碰撞保护环移到
射线原点上方且保留接触包络，FreeDOM 静态地图补齐当前时间戳。`preflight3` 的原始、配准和静态
点云分别约为 20000/7900/5600 点，静态地图为 `camera_init`、非零时间戳、约 10 Hz。最新
`vcl06_map_guard_fix_gate90_v3_20260831_125136` 中地图约 10 Hz、pose 约 30 Hz，合同错误、碰撞、
越界和超高均为 0，运行中不再出现 `map_missing`/`map_stale`。

V-CL-06 整体仍未通过：Gate v3 以 `wall_timeout` FAIL，仅有 3 个 decision/5 个 result，未产生
selected、APPROACH 或同一 stable ID target transaction，投递、返航、LAND 也未成立；
`start_gate_started_once=false` 仍需核对 evaluator 的订阅/锁存时序，`P_interrupt` 保持 `null`。
下一联合 P0 是沿 manager candidate→decision→bridge result 路径定位目标为何未进入事务；90 秒
目标事务 Gate 通过前不启动 600 秒三投。
