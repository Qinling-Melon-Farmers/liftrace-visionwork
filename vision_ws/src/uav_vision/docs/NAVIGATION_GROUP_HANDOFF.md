# uav_vision 导航组接口说明

交付版本：`20260807-beta1`
包版本：`uav_vision 0.2.1`
边界：视觉只输出观测、地图候选、记忆和像素对准证据，不输出飞行或执行机构命令。

> 版本阅读规则：本文保留 `20260807-beta1` 作为历史交付标识；对正式任务链、
> profile、实测结论或导航职责的表述如有冲突，**2026-08-30 补充优先于旧 beta1**。

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

特别注意：上述“权重队列、delivered/failed 去重、规划重试和不可达后恢复”不是
`manager@5144aa8` 的当前能力。正式 manager 目前仍缺 accepted/result/retry 回流、持久候选
队列和剩余时间调度；不得用本节的历史 coverage manager 清单宣称这些任务层缺口
已由导航上游完成。

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

截至 2026-08-30 再次 fetch，导航远程 `main` 仍为 `a68925d`（仅地图实验），正式
manager 仍为 `5144aa8`，上述任务生命周期尚未交付。V-CL-06 A/B 仍 FAIL，
`a68925d` 不推广，600 s 三投 Gate 尚未启动。

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

截至 2026-08-31，导航远程仍仅
`main@a68925d15293e5510e2b4351c6b3d9bc5aa136ab`、无 branch/tag，manager 仍为
`5144aa8f536bdcd214aea2f39ada558383b3bcb0`；联动分支 `8255aa4` 已是当前性能分支
HEAD 的祖先。accepted/result/retry、持久队列和剩余时间调度仍未交付，V-CL-06 A/B 仍 FAIL，
导航 600 秒三投未启动。下一联合测试必须等导航 feature revision 补齐这些生命周期事件后
再测真实 `P_interrupt`；`P_selected` 仍不得代替接受或中断。
