# 2026 视觉组主路线与仿真验收方案

更新时间：2026-08-29
状态：**当前唯一任务优先级来源（Single Source of Truth）**

本文回答四件事：视觉组现在做到哪里、目标架构是什么、下一步具体做什么、如何用仿真证明算法有效。其他文档只负责环境操作、接口细节、部署或历史证据；若“下一步”表述冲突，以本文为准。

## 1. 一页结论

当前不需要更换 LIO 或局部规划算法，也不需要继续无目标地扩数据和训练模型。桌面 PT、板端
离线 RKNN 和实拍视频回放已有证据，但尚未完成 OrangePi 上的 ROS 视觉链、10 分钟稳定性和
机载 CameraInfo/TF 接线验收。视觉组的主线是：

板端模型、图片指标、PT 对照和视频资产的单一汇总见
[BOARD_MODEL_COMPLETE_EVALUATION_20260716.md](/home/xhj/liftrace/docs/BOARD_MODEL_COMPLETE_EVALUATION_20260716.md)。
旧 standard+tank 的全集图片评测已完成；旧双模型视频仅准备回放脚本，本轮因整机负载风险暂缓。

1. V-SIM-00 至 V-SIM-03 最小基建已经完成；30-seed 和延迟继续记录，但不再作为视觉
   任务闭环的前置阻塞；
2. 当前首要任务改为 V-CL 系列：地图坐标契约、稳定目标记忆、新视觉与旧控制固定三投、
   Fast-Planner 目标复用、候选接近与恢复搜索；
3. `uav_mission` 已建立任务/安全释放边界和最终下降投递承诺：视觉高位稳定证据锁定目标、
   槽位、模式、位置和时间，下降后当前观测变陈旧时仍须通过超时、漂移、状态和防重放
   检查生成短时 `release_permission`；`patrol_control` 以参数化最小补丁消费许可，旧模式
   默认行为保持不变，`actuator_pwm` 未改；
4. V-CL-00B/01 的 L0 契约已落地：中心来源、实例关联、拒绝原因和 TF 年龄可审计；
   Phase D 对无效地图点失败关闭；物理地图目标跨标准类别保持 ID，并以连续帧、置信度和
   累计投票抑制 tent/panzer 单帧抖动；
5. V-CL-02 已通过完整无 GUI toudi3/SITL：新视觉动态确认 `tent/pillbox/bridge` 三类地图
   候选，旧固定路线完成三次受许可 guarded mock ACK，零重复、零越权；通过日志为
   `logs/newvision_fixed3_20260806_062124/`；
6. V-CL-03 已通过完整无 GUI toudi3/SITL：默认关闭的外部任务模式保留旧路线，Mission
   Manager 独占 `/fastplanner/goal`，单个新视觉候选完成接近、对准、受控投递和恢复；通过
   日志为 `logs/external_candidate_20260807_041914/`，默认旧模式回归为
   `logs/legacy_mode_regression_20260807_042403/`；
7. V-CL-04 的靶标区域覆盖导航 Gate 已通过：12 个非靶标坐标蛇形端点全部到达，五类标准
   靶恰好形成五个物理 ID，完成安全返航且零碰撞、零越界、零释放；通过日志为
   `logs/target_area_navigation_20260807_190817/`。按当前范围，北区走廊避障不计入本 Gate；
8. 圆环关联、地图新鲜度、H 结构门控和 `release_evidence` 继续按闭环失败证据修正；召回和
   时延优化服从于“能发现、能记住、能接近、能重捕获、不会错误投放”的业务目标；
9. PT/ONNX 与 RKNN 离线部署门禁已有首轮结果：两款 FP32 RKNN 可运行，四款 INT8 RKNN
   在 v5merge 全集上零有效检测；OrangePi ROS 视觉链和稳定性验收仍未完成。
10. 当前并行核查原 2025 完整飞行载体：自动起飞和三次软件 mock 投递已完成，但原始
   0.8 m 门洞穿越、北区巡航、返航和降落尚未完成。该项是控制/规划/仿真联合前置，
   不计入视觉算法完成度；直达航点通过也不能替代 Fast-Planner 避障验收。分层 Gate 和
   world 派生规则见
   [2025原始链完整仿真阻塞与WORLD改造方案_20260802.md](/home/xhj/liftrace/docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md)。
11. V-CL-04 三投闭环已实跑成立：`logs/toudi4_coverage_r6_v2_20260813_221737/`
    完成 12/12 覆盖、五类五 ID、tank/panzer/bridge 按权重 3/3 mock 投递、0 碰撞、
    0 越界、377.6 s 降落。bridge 根因修复 = 兼容桥 circle 地图点跟随（0→2524 次）
    + 投递前高位圆环捕获阶段；Gate 断言已改按全程累计事实。另一次实跑
    （v3）出现 Fast-Planner 可达性退化（返航不可达），0 碰撞，属规划侧已知问题，
    不阻塞视觉闭环验收。2026-08-28 干净复跑（rerun2，main@7a0b612）任务侧全指标达标，
    tank 一次 Fast-Planner 下降段异常经裁定按规划器波动外置口径处理，V-CL-04 关闭
    （详见变更记录 2026-08-28 条目）。
12. 当前 `toudi4_copy.world` 缺少旧 0.8 m 门洞的 `Wall_15` 及北区侧墙 `Wall_16`，
    不能通过空场直飞代替走廊验收；H 模型存在，但 H 视觉许可降落仍未验收。
13. `V-EXP-01` 斜下辅助相机探索已于 2026-08-23 冻结。原型只在保留的 feature 分支，
    从未合入 `main`；主干继续使用单个下视 RGB 相机 + MID360。辅助方向完成 Step 1–2，
    证明粗投影、候选和下视交接机制可行，但未证明第二相机稳定提速，真实辅助 YOLO、单
    runtime 双输入和随机世界 A/B 均不再推进。冻结证据和恢复条件见
    [VEXP01辅助相机探索冻结说明_20260823.md](/home/xhj/liftrace/docs/VEXP01辅助相机探索冻结说明_20260823.md)。
14. 导航组 `liftrace-controlwork@5144aa8` 的 Python 搜索 manager/策略已在保持源码逻辑的
    前提下接入新视觉和旧控制投递链。正式 headless 日志
    `logs/navigation_upstream_visual_delivery_headless_model_20260826_023411/` 在 600 s 内
    到达 9/16 覆盖点，发现 pillbox/tent/bridge，tent/bridge 完成顺序槽位两投，最终因
    第三投未完成而 `mission_timeout`。这证明跨组链路可以闭合，但暴露出 manager 只接受
    当前新鲜 `selected_target`、缺持久全局权重队列/失败重试/剩余时间调度；按 V-CL-06
    继续联合验收，不能把两投结果写成完整 Gate PASS。详细 handoff 见
    [导航组任务链与新视觉联调HANDOFF_20260826.md](/home/xhj/liftrace/docs/导航组任务链与新视觉联调HANDOFF_20260826.md)。
15. 2026-08-29 已把 V-CL-06 正式链切到可复现 `toudi3_random.world`：导航 manager 仍冻结
    为 `5144aa8`，地图实验 `a68925d` 仅作为可切换场外 anchor profile。r2026 保留 tank
    资产和 full 回归但不准入 tank；双 READY + operational SEARCH 门控、实际 Gazebo contact、
    raw/filtered tank 审计、首个任务活动双时钟和 30 秒预检/90 秒固定路线 A/B 已实跑。
    baseline 30 秒预检 PASS；同 seed 的 baseline 90 秒因最大高度 4.798 m FAIL，`a68925d`
    90 秒虽以最大高度 3.254 m PASS 并把高度 RMS 漂移从 1.106 m 降到 0.513 m，但共同进度
    耗时退化 19.43%，超过 10%，比较器 `promote_candidate=false`。默认仍为 baseline，未跑
    600 秒，V-CL-06 继续 FAIL 并将高度/耗时与持久队列/重试/时序缺口回流导航组。

现有 `toudi3.world` 五类标准靶和 H 已直接用于固定真值场景，红十字按评测场景插入；
`uav_vision_eval` 已能自动生成 CSV/JSON/report，shadow 输出也已隔离。当前仍不能宣称完整
视觉 Gate 通过：尚缺 30-seed/10 min 回归，红十字和部分标准类召回低于 0.95，笔记本
正式召回仍未达标，30-seed/10 min 未完成，实拍圆环缺人工真值。

## 2. 当前事实基线

### 2.1 已有能力

- `vision_ws/src/uav_vision` 是唯一新视觉开发主线；
- 生产/比赛准备固定为单个下视 RGB 相机 + MID360；辅助相机原型只保留在实验分支；
- dev/sim 六分类检测器、RKNN 入口、红十字/圆环/H 几何检测器均已存在；
- 已形成以下处理链：

```text
/uav_vision/detections
  -> detections_resolved
  -> detections_refined
  -> detections_mapped
  -> targets / selected_target
  -> drop_offset / drop_ready
```

- `phase_d_map_mock.launch`、`phase_d_mock_patrol_regression.launch` 等软件回归入口已存在；
- `uav_mission` 已新增 `ReleasePermission/ReleaseResult`、任务层许可仲裁、受控旧 Servo 代理
  和纯软件 raw Servo mock；确定性回归已验证旧控制非 Aligning 拒绝、三槽顺序、过期、
  错槽、重放与重复目标拒绝；`visual_delivery_audit.py` 可记录完整事件链并自动判定三投；
- 当前默认仿真已切换到仓库内 `iris_mid360_downward_camera` 集成模型：单个下视 RGB
  相机并保留 MID360；PX4 autostart 仍使用 `iris_mid360`。独立投影视场回归已同步到
  `toudi4_copy.world`，此前 toudi3 结果保留作历史基线；仿真外参取自当前 SDF，旧工程
  写死外参只登记为实机复标候选；
- 完整飞行链补齐 `camera_init -> D435i::camera_color_frame` 后，`detections_mapped`、地图
  `targets` 和 `selected_target` 已可产生；`drop_aligner` 已先按观测新鲜度过滤再排序，
  确定性回归通过；
- 已形成不含控制/规划/执行代码的 `uav_vision 20260717-alpha1` 交付入口和中文接口说明；
  dev/board 模型、相机、CameraInfo、map frame 和地面高度均由 launch 参数传入，控制组可
  直接消费合法 `TargetCandidate.map_point` 开始联调；
- 联合环境入口已解决两个 catkin 工作区互相覆盖，并有 `V-SIM-00 PASS` 自动验证；
- `uav_vision_eval` 已建立固定场景、独立 Gazebo 真值、记录器和报告器；
- headless shadow 入口已将视觉输出隔离到 shadow 命名空间，并保护旧控制/执行接口；
- `toudi3.world` 已包含五类标准靶和 H，红十字模型可独立插入；
- 笔记本 dev/sim 默认六分类模型已选 `merged_standard`；现有 ONNX 可加载，板端已用
  `merged_standard_fp32.rknn`、`region_focus_aug_fp32.rknn` 做离线视频与 v5merge 全集评测；
  板端运行库仍有 Toolkit 2.3.2 与 `librknnrt 1.5.2` 版本警告，不能据此宣称机载部署 Gate 已通过。

### 2.2 不能误判为完成

- 旧控制默认仍是固定航点巡航；参数化外部任务模式和单候选闭环已通过，临时 manager
  曾完成覆盖权重三投；导航组新 manager 已完成覆盖、候选接近和索引恢复接线，但 600 s
  只完成两投，且尚无持久全局候选队列、失败重试和剩余时间调度；
- `selected_target.map_point` 已由独立 Mission Manager 完成单候选接近、横向对准、投递和
  恢复；现有 `/detect/waypoint_mark_point` 像素兼容输出仍不能冒充地图坐标；
- `drop_ready` 仍只是兼容观测；结构化 `release_evidence` 已实现，任务层第一版
  `release_permission` 和旧 Servo 安全代理已落地；最新完整 SITL 中旧控制两次进入
  `Aligning/drop_circle`，新视觉已产生地图候选，对准失败主要前移为
  `offset_exceeds_limit`（偶发观测过期/未确认）。由于旧控制未消费新地图目标且旧路线出现
  `Invalid servo ID: 0`，raw mock 调用仍为 0。尚未完成视觉驱动修正、三投、速度/机构状态
  互锁和 Mission Manager 上下文；
- 地图候选与当前 `last_seen` 已分离并有 mock，仍缺真实同步位姿回放和 30-seed 跨视角验收；
- `TargetDetection/TargetCandidate` 已追加 `center_source`、`association_valid`、
  `reject_reason`、`transform_age_sec` 和连续命中计数；Phase D/板端只接受有效地图候选；
- H 已做外圈 + 内部结构 + landing 阶段门控，仍缺实拍 H/普通黑圈真值集；
- 实拍圆环等比例回放自洽关联率为 58.80%，但缺逐帧实例/中心真值，不能当召回率；
- 当前固定场景的标准类召回约 0.765-0.935、红十字 0.646，未达到正式 0.95 Gate；
- 仿真贴图比真实场景干净，仿真结果不能替代实拍回放和板端性能验证。
- v5merge 全集共 1395 张（train+val），用于板端全量覆盖审计；由于包含训练集，全集 P/R/mAP
  不能当泛化精度，正式模型比较仍以独立 val 232 张和压力集为主。
- 板端原始视频回放默认关闭去畸变；固定内参只用于 `/dev/video0` 实时相机，避免将视频元数据
  旋转/相机标定差异混入 PT↔RKNN 性能比较。

## 3. 视觉组业务边界

### 3.1 视觉组负责

- 相机输入契约、CameraInfo、时间戳和 frame 合法性；
- 标准目标、红十字、蓝色圆环、H 的检测与几何质量；
- 同帧融合、类别—圆环实例关联、中心精修；
- 像素观测投影到统一地图坐标；
- 多帧确认、跨视角去重、稳定 ID、`last_seen` 和拒绝原因；
- 投递/降落对准偏差与可审计的视觉释放证据；
- mock、Gazebo、实拍回放、板端性能的自动评测；
- 旧 `/detect/*` 接口的限期兼容和语义说明。

### 3.2 视觉组不负责

- 全场覆盖路径生成、候选接近、绕障和搜索恢复；
- LIO、PX4 EKF 或局部规划算法替换；
- 最终解锁、起飞、飞控模式、舵机/PWM 动作；
- 仅凭目标类别决定是否投递或降落。

### 3.3 联合接口责任

任务/控制组需要提供明确任务阶段、允许中断标志、统一位姿和搜索重置时机；视觉组返回候选、质量、位置、年龄、对准状态和拒绝原因。任务/安全层组合飞行状态、目标身份、视觉证据、机构状态和规则约束，生成最终 `release_permission`。

## 4. 目标架构

```text
Gazebo / 实机相机 / rosbag
  image + CameraInfo + timestamp + camera_frame
                       |
                       v
              target_detector_rknn          # dev/sim 可替换为 PyTorch
                       |
       +---------------+----------------+
       |               |                |
 cross geometry   circle geometry   H structure
       +---------------+----------------+
                       v
              detection_fusion
                       v
              target_refiner             # 类别—圆环实例关联
                       v
    TF + pose ---- target_map_projector
                       v
              target_memory               # stable_id/last_seen/state
                       |
             candidate proposal
                       v
 Mission/Search Manager（控制/规划组）
 SEARCH -> VERIFY -> APPROACH -> DROP_ALIGN -> RETREAT -> RESUME
                                  |
                                  v
            drop_aligner / landing verifier
                                  |
                       release_evidence
                                  v
             Safety/Action Arbiter（非视觉组）
                       release_permission
                                  |
                                  v
                      guarded /Servo proxy
                                  |
                         /legacy/Servo_raw
                         (sim mock / real adapter)
```

### 4.1 现有接口继续保留

- `/uav_vision/detections`
- `/uav_vision/detections_resolved`
- `/uav_vision/detections_refined`
- `/uav_vision/detections_mapped`
- `/uav_vision/targets`
- `/uav_vision/selected_target`
- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`
- `/uav_vision/release_evidence`
- `/uav_vision/align_mode`

### 4.2 当前接口契约与后续补充

后续消息修订优先补字段，不以新增大量零散 topic 代替清晰契约：

- `header.stamp`：保持产生观测的源图像时间，不使用转发时刻伪装新鲜观测；
- `last_seen` / `observation_age`：地图记忆和当前可控观测分开；
- `stable_id`：跨帧、跨视角一致的候选 ID；
- `map_valid`、`transform_age_sec` 已落地；`pose_valid` 留给任务层组合位姿健康；
- `association_valid` 已落地：类别与圆环是否属于同一实例；
- `center_source` 已落地：框中心、圆环、红十字或 H 几何中心；
- `map_quality` 与结构化 `reject_reason` 已落地；
- `mission_stage` / `align_mode`：观测属于哪个业务阶段；
- `target_verified`、`aligned`、`stable_duration`：作为视觉释放证据，不直接代表执行许可。

聚合消息 `/uav_vision/release_evidence` 已由视觉组发布；`/mission/release_permission` 由
`uav_mission/release_permission_arbiter` 发布。旧控制继续调用原 `/Servo`，但新闭环入口中
该服务必须由 `guarded_servo_proxy` 独占，只有短时许可匹配当前顺序槽位时才转发到
`/legacy/Servo_raw`。旧 `drop_ready` 在迁移期保留，但不得再被写成“允许舵机动作”。

## 5. 用仿真证明什么

视觉验证采用四层金字塔。每一层只回答自己的问题，不能用低层通过代替高层验收。

| 层级 | 输入与运行方式 | 能证明 | 不能证明 |
| --- | --- | --- | --- |
| L0 确定性 mock | 合成消息、固定 CameraInfo/TF | 话题、坐标、状态机和异常分支正确 | 图像算法有效 |
| L1 Gazebo 真值场景 | 静态/预设相机位姿、已知靶标姿态 | 检测、中心、关联和地图投影的可量化误差 | 真实域泛化 |
| L2 shadow 飞行 | 固定航线飞行，视觉只观察不控制 | 运动模糊、连续观测、跨视角 ID、延迟和十分钟稳定性 | 自主搜索或精投闭环 |
| L3 任务闭环仿真 | Search Manager 接近/恢复，执行机构为 mock | 搜索发现、阶段门控、对准和释放证据时序 | 实物落点和板端 NPU 性能 |

原 2025 链的 R0～R5 是提供稳定飞行载体的联合前置，不替代上述视觉 L0～L3；视觉可在
R0～R5 中以 shadow 方式采集长期观测，但不得把 mock 靶标/Servo 结果计入视觉召回或闭环。

实拍 rosbag/视频回放是横跨 L1-L3 的域差门禁；OrangePi RKNN 是部署门禁。二者都不能被 Gazebo 替代。

## 6. 仿真真值与评测设计

### 6.1 独立真值原则

真值不得来自检测器输出。建议新增以下视觉测试资产：

```text
vision_ws/src/uav_vision_eval/
  config/sim_target_catalog.yaml
  config/scenarios/*.yaml
  scripts/sim_ground_truth_projector.py
  scripts/vision_metrics_recorder.py
  scripts/vision_metrics_report.py
  launch/toudi3_full_shadow.launch

vision_ws/test_data/sim_eval_<date>/
  scenario_manifest.yaml
  detections.csv
  associations.csv
  map_errors.csv
  events.csv
  summary.json
  report.md
```

`sim_target_catalog.yaml` 至少记录：模型实例名、稳定 ID、类别、世界位姿、目标平面、外框四角和几何中心。真值投影器从 Gazebo model state、CameraInfo 和 TF 投影出像素中心/边界，并计算地面真值点。

第一版只在单目标、无遮挡、目标完整入画的场景评分；遮挡场景必须有独立可见性真值后才能计入召回率，不能把“投影在画面内”直接等同于“视觉可见”。

### 6.2 场景矩阵

每个场景都保存 seed 和实际参数，失败必须可复现：

| 维度 | 首轮取值 |
| --- | --- |
| 类别 | 五类标准目标、red_cross、landing_h、纯背景负样本 |
| 高度 | 0.8、1.2、2.0、3.0 m |
| 偏航 | 0°、45°、90°、135° |
| 画面位置 | 中心、四象限、边缘、部分出画 |
| 光照 | 基线、偏暗、偏亮；参数和 seed 固定 |
| 运动 | 静止、低速横移、转弯；记录角速度与线速度 |
| 多目标 | 同类两个、不同类两个、目标与 H 同帧 |
| 负样本 | 地面纹理、墙体、树、箱体、普通黑圈/残圈 |

首轮目标是 30 个固定种子完成回归，不追求无限随机。任何算法改动都跑同一组 seed，新增场景不能偷偷替换旧失败样本。

### 6.3 首轮工程门禁

以下阈值是首轮工程门禁，不是比赛规则承诺；得到第一版基线后可以在变更记录中有依据地调整：

| 指标 | Gate |
| --- | --- |
| L0 mock 回归 | 所有 assertion 退出码为 0，连续 3 次无偶发失败 |
| 清晰无遮挡检测召回率 | 每类 `>= 0.95` |
| 负样本误报 | `<= 0.05` 次/帧 |
| 中心误差 | 1280×960 下 P95 `<= 20 px`，同时报告归一化误差 |
| 类别—圆环关联召回率 | `>= 0.90` |
| 错误实例关联率 | `<= 0.01` |
| 地图投影有效率 | CameraInfo/TF 有效样本中 `>= 0.95` |
| 地图点误差 | 中位数 `<= 0.10 m`，P95 `<= 0.25 m` |
| 跨视角重复 ID 率 | `<= 0.05` |
| 当前控制观测年龄 | 触发候选时 `<= 0.5 s`；长期地图记忆不算当前观测 |
| 阶段越权 | 0 次；非 `LAND_CONFIRM` 的 H 不得产生降落成立 |
| 错误释放证据 | 0 次；模式、身份、关联、位姿任一无效时必须为 false |
| dev/sim 端到端延迟 | P95 `<= 200 ms`，队列不得持续增长 |
| L2 稳定性 | 10 min 无节点崩溃、无时间戳倒退、无持续积压 |

压力场景的目标不是首轮就达到清晰场景同等召回率，而是建立固定基线并禁止无解释回退。所有报告同时给出总指标和逐类别、逐条件指标，禁止只报平均值。

## 7. 标准验证流程

### 7.1 Gate A：软件接口回归（已建立）

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/vision_ws/devel/setup.bash
roslaunch uav_vision phase_d_map_mock.launch
```

另行运行现有 patrol mock 回归：

```bash
# 使用联合环境入口
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
roslaunch uav_vision phase_d_mock_patrol_regression.launch
```

`phase_d_map_mock.launch` 可在视觉工作区环境单独运行；patrol mock 使用联合环境。验收以
assertion 退出码为准，不以“看到了 topic”代替。圆环坐标、全局一对一关联、地图新鲜度、
H 阶段门控和释放证据均已有确定性回归。

### 7.2 Gate B：当前 Gazebo 链路烟测

预定入口：

```bash
cd /home/xhj/liftrace
bash ./top_level_scripts/run_toudi3_full_competition_sim_gui_new.sh
```

该 GUI 入口只用于人工连通观察，不作为算法指标或自主搜索验收。定量测试使用
`uav_vision_eval`；只观察联调使用 shadow 入口，二者均不得启动执行机构。

详细启动和停止步骤见 `docs/TOUDI3_FULL_SIM_GUI_GUIDE.md`。

### 7.3 Gate C：Gazebo 真值评测（最小版已实现）

固定场景可一次命令完成场景启动、真值记录、视觉记录、超时退出和报告生成：

```bash
./top_level_scripts/run_toudi3_visual_suite.sh
```

当前 suite 覆盖五类标准靶、红十字、H 和背景，并复用 world 现有资产。固定 30-seed
矩阵和自动聚合器已经实现并完成首轮运行，但正式 Gate 尚未通过；最新失败归因和口径
见笔记本基线文档。10 min shadow 仍未执行完成；V-SIM-04 保持后置量化，在单下视全随机
闭环成立后按失败阶段执行，不再阻塞当前 V-CL 主线。

正式阈值可用第三个参数启用：

```bash
./top_level_scripts/run_toudi3_visual_suite.sh 22 /tmp/uav_vision_eval/formal_suite formal
```

### 7.4 Gate D：shadow 飞行（隔离入口已实现，10 min 尚未验收）

shadow 模式必须满足：

- 视觉订阅真实仿真图像、CameraInfo 和位姿；
- `/uav_vision/selected_target`、`drop_ready` 不得改变旧控制状态；
- 禁止启动 `actuator_pwm`，投递输出接 mock；
- 自动记录 CSV/JSON、参数快照和 seed，bag 按调试需要显式启用；
- 同一固定航线至少运行 10 min 或完整赛程，以先到者为准。

当前 shadow 隔离契约已通过；10 min 固定航线尚未完成。Search/Mission Manager 已在
V-CL-02～05 接入并形成闭环证据，因此 10 min 不再作为接入前置，而是在全随机单下视闭环
成立后验证长期候选记忆、陈旧数据、队列积压和错误释放。

### 7.5 Gate E：视觉任务闭环（当前主线）

按以下顺序推进，不使用靶标真值坐标冒充搜索能力：

1. `V-CL-00B`：接口/L0 已完成，下一步在完整 SITL 核对 `camera_init` 与实际 TF 树；
2. `V-CL-01`：L0 已完成，后续在运动相机/跨视角场景量化重复 ID；
3. `V-CL-02`：已完成；三个固定检测点均完成视觉证据、任务许可、受控 Servo 和 mock ACK；
4. `V-CL-03`：已完成；旧控制增加独立外部任务模式，Mission Manager 独占
   `/fastplanner/goal`，单候选完成接近、对准、投放和恢复；
5. `V-CL-04`：2026-08-13 实跑（`logs/toudi4_coverage_r6_v2_20260813_221737/`）
   已完成 12/12 覆盖、五类五 ID、tank/panzer/bridge 按权重 3/3 投递、0 碰撞、0 越界、
   377.6 s 降落。bridge 第三投的根因修复：兼容桥恢复 circle 地图点跟随（旧实跑 0 次
   waypoint 跟随 → 修复后 2524 次）+ 投递前高位圆环证据捕获阶段。Gate 断言已按全程
   累计事实修复。2026-08-28 干净复跑
   （`logs/toudi4_coverage_r6_vcl04_rerun2_20260828_222644/`，main@7a0b612）任务侧全指标
   达标（12/12 覆盖、五类五 ID、三投槽序 [1,2,3]、0 碰撞、0 越界、405.5 s 落地）；Gate
   仅余 4 项同源断言失败，全部由 tank 一次 Fast-Planner 下降段异常（穿透
   `align_height=1.20 m` 至 0.1–0.3 m 悬停 + `No Effective Points`）连锁造成，同场
   panzer/bridge/pillbox 3/3 投递证明对准链健康；经用户裁定按规划器波动外置口径视为
   通过，Gate 关闭（动态期望断言与中断失败的口径缺口移交 V-CL-05/06 收敛）。
   H 视觉降落 Gate 另立；北区走廊独立成 Gate。
6. `V-CL-05`：高权重中断、red_cross 统一入队和随机红十字摆放入口已实现；当前单下视
   首要完成定义为“4 个标准靶 + 1 个红十字全部随机布设，搜索阶段疑似红十字可触发抵近、
   投递前严格复核，完成目标记忆、权重排队、中断/恢复和三次投递”。需同步把任务层允许
   类目和 Gate 期望集合改为场景/profile 可配置；本届无 tank 的 profile 不允许 tank 假阳性
   抢占候选队列。规划器失败独立记录，不通过恢复辅助相机或更换 LIO/planner 绕开。
7. `V-CL-06`：迁移到导航组搜索 manager。保持上游源码不变，由外围适配器接入起飞门控、
   新视觉 CAPTURE、旧控制 ALIGN 和 guarded release；首轮 600 s 完成 2/3 投递后超时。
   seed=11 baseline 30 秒基础设施预检已 PASS；baseline/a68925d 90 秒固定路线 A/B 已完成，
   候选安全且高度漂移改善，但 baseline 不安全、候选共同进度耗时退化 19.43%，不满足 10%
   上限，故保持 baseline 且不进入 600 秒 Gate。下一完成定义仍是导航组先收敛搜索高度跟踪和
   耗时，并由双方冻结“实时中断候选/持久地图队列/
   投递结果”接口，再由导航组任务层实现全局权重、失败冷却重试和剩余时间调度；同一
   headless Gate 在 600 s 内完成三投、返航和落地。视觉组负责候选质量和新鲜度，不在
   `uav_vision` 内实现任务队列。

L3 初期只把陈旧数据、队列积压和错误释放作为硬失败；搜索阶段统一 P95 `<=200 ms`
暂不作为阻塞。建立投递承诺时的视觉证据必须新鲜（默认最大年龄 `0.5 s`）；最终释放
使用持续重算的短时任务许可，不要求下降后相机仍能拍全靶标。

## 8. 下一步任务表

按顺序执行；上一 Gate 未通过时，不以“并行做更多模型”绕开阻塞。

| 状态 | 顺序 | ID | 任务 | 下一完成定义 |
| --- | ---: | --- | --- | --- |
| 已完成 | 1 | V-SIM-00 | 双 catkin 联合环境 | 保持 overlay 自动验证通过 |
| 已完成 | 2 | V-SIM-01 | 独立真值包与场景契约 | world 资产变化时同步 catalog |
| 已完成 | 3 | V-SIM-02 | headless shadow 隔离 | 保持全局控制/执行话题零写入 |
| 已完成 | 4 | V-SIM-03 | CSV/JSON/report 自动评测 | runner 失败保持非零退出 |
| 部分完成 | 5 | V-ALG-01 | 圆环检出和实例关联 | 补实拍人工真值；关联召回 `>=0.90`、错配 `<=0.01` |
| 部分完成 | 6 | V-ALG-02 | 地图投影与记忆新鲜度 | 完成 30-seed 跨视角 ID/地图误差与同步实拍 pose 回放 |
| 部分完成 | 7 | V-ALG-03 | H 结构与任务阶段门控 | 补真实 H/普通黑圈负样本和完整模式矩阵 |
| 视觉完成 | 8 | V-ALG-04 | `release_evidence` 分层 | 控制/安全层另行实现最终 `release_permission` |
| 已完成 | 9 | V-CL-00A | 任务层投放许可与旧 Servo 安全边界 | mock 回归与完整旧控制三投均 PASS；保持顺序载荷、防重放和 positive ACK 门控 |
| 接口完成/集成待验收 | 10 | V-CL-00B | 统一 mission frame 与地图坐标契约 | L0 已拒绝无效 TF；完整 SITL 核对 `camera_init` TF 与导航点一致性 |
| L0 完成/跨视角待验收 | 11 | V-CL-01 | 物理目标 stable ID 与类别时序投票 | 连续帧/置信度/投票回归通过；待运动相机量化重复 ID |
| 已完成 | 12 | V-CL-02 | 新视觉 + 旧控制固定三投 | `newvision_fixed3_20260806_062124` 三类候选、三次 permission→mock ACK、审计 PASS |
| 已完成 | 13 | V-CL-03 | 外部任务模式复用 Fast-Planner | `external_candidate_20260807_041914` 单候选接近→对准→ACK→恢复，唯一 goal 发布者；默认旧路线回归 PASS |
| 已完成 | 14 | V-CL-04 | 覆盖搜索、候选队列和恢复 | 2026-08-28 干净复跑（`toudi4_coverage_r6_vcl04_rerun2_20260828_222644`，main@7a0b612）任务侧全指标达标：12/12 覆盖、五类五 ID、三投槽序 [1,2,3]、0 碰撞、0 越界、405.5s 三投+返航+落地；Gate 27 项断言仅 4 项同源失败，均由 tank 一次 Fast-Planner 下降段异常（穿透 align_height 1.20m 至 0.1–0.3m 悬停 + No Effective Points）连锁造成，同场 3/3 投递证明对准链健康；经用户裁定按规划器波动外置口径视为通过（动态期望断言与中断失败的口径缺口移交 V-CL-05/06 收敛） |
| 部分完成/待干净复跑 | 15 | V-CL-05 | 搜索-投递策略：高权重中断 + red_cross 统一 | 中断机制已实跑（tank 提前执行并恢复搜索、覆盖 12/12），pillbox 端到端投出，tank/panzer 证据锁定但低空下降受规划器波动影响超时；red_cross 统一入队、随机摆放入口与动态期望 Gate 已落地。剩余完成定义：按规则场地全随机布设（4 个 1 m 标准靶 + 1 个 0.35 m 红十字全部随机摆放、H 固定为起降点，真值仅落盘），类目/profile 驱动允许集合与 Gate（本届 profile 排除 tank），随机十字独立发现/投递 + 沉降门控验证（规划器失败只记录不迭代） |
| 30 s PASS/A-B 已测不推广，Gate FAIL | 16 | V-CL-06 | 导航组 manager + 新视觉正式任务链接入 | 上游 manager `5144aa8` 保持原逻辑，raw/planner goal 所有权与 READY/SEARCH/contact 门控通过；seed=11 baseline 30 s PASS。90 s baseline 最大高度 4.798 m FAIL；`a68925d` 最大高度 3.254 m、漂移改善但共同进度慢 19.43%，比较 FAIL，保持 baseline 且不跑 600 s。导航组下一步收敛高度/耗时并实现持久全局权重、失败重试和剩余时间调度；同 seed 600 s 三投+返航+落地才 PASS |
| 已冻结 | 17 | V-EXP-01 | 斜下辅助相机搜索可行性 | Step 1–2 原型与接口证据保留在 feature 分支；不再实现辅助 YOLO、单 runtime 双输入或双相机随机世界 A/B，恢复须有单下视无法满足比赛时限的量化证据 |
| 最小矩阵已量化/待性能修复 | 18 | V-SIM-04 | L1/L2 阶段性能与后续 30-seed | 当前合并态 `vsim04_seed11_current_20260829_195506` 已 23/23、六产物、无 infrastructure gap，终态 MEASURED；P_confirm=P_selected=13/23=0.5652，P95 processing=123.0 ms、P95 exposure=0.430 s、map-invalid=24.67%、map-unavailable=36.43%、TF failure=0、地图误差 P95=0.0789 m、接收 FPS=15.64。相对旧阈值/旧 partial 语义的 9/23 基线增加 4 项；剩余失败集中在四类标准靶全部 3.6 m、panzer 动态 3.0 m，以及 red_cross 静态 1.2/3.6 m 和动态 1.8 m。先收敛检测连续性/高空投影，再决定 30-seed/10 min，不把 MEASURED 等同算法达标 |
| 待真值 | 19 | V-REAL-01 | 实拍回放域差复核 | 标注圆环实例/中心/H/红十字，采集同步 CameraInfo/pose |
| 离线部分完成/ROS待验收 | 20 | V-DEPLOY-01 | PT/ONNX/RKNN 与 OrangePi 验收 | FP32 RKNN 离线有效；待完成单下视 ROS 相机/TF、5–10 Hz 和 10 min Gate |

### 8.1 已完成的最小交付与当前入口

V-SIM-00 至 V-SIM-03 已交付：

- 1 个标准靶、1 个红十字、1 个纯背景场景；
- 3 个固定相机高度；
- 像素中心误差、地图点误差、观测延迟三个指标；
- headless、无执行机构、视觉不夺取控制；
- 每次运行产生 `manifest.json + frames.csv + summary.json + report.md`；30-seed 另生成
  `matrix_summary.json + matrix_report.md`。

当前量化结果、局限和下一步见
[docs/VISION_LAPTOP_SIM_BASELINE_20260715.md](/home/xhj/liftrace/docs/VISION_LAPTOP_SIM_BASELINE_20260715.md)。

V-SIM-04 最小框架固定使用 `r2026`（tent/pillbox/bridge/panzer/red_cross，不含 tank）和
`seed=11`。一次“完整入画到离开”定义为一个 trial；`P_confirm` 必须满足 target memory
当前完整准入，`P_selected` 必须关联同一 stable ID。visual-only 的 `P_interrupt` 明确为
`null`，不得由 selected 代替；导航实际接受事件在 V-CL-06 联合报告中另行评分。
当前合并态 `vsim04_seed11_current_20260829_195506` 已完成 23/23 并通过终态/产物完整性校验，
证明评测链可用；确认/选择由旧配置 9/23 提升为 13/23，但四类标准靶 3.6 m、高位 panzer
动态和部分 red_cross 尺度仍是明确性能缺口。red_cross 失败项已有有效检测/地图帧却无法形成
三次连续准入，应优先检查双路重叠与完整空帧间断，而不是放宽地图门槛。不能把 `MEASURED`
写成算法 Gate PASS，也不应在这些失败收敛前直接扩 30-seed。

## 9. 暂不推进

- 替换 FAST-LIO、Fast-Planner 或重写主控；
- 继续斜下辅助相机、双摄像头 YOLO 或单 runtime 双输入探索；
- 在视觉包中实现全场航线规划；
- 没有新失败证据驱动的数据扩充或重复训练；
- 在 OrangePi 上用 PyTorch 作为实时主路径；
- 用 Gazebo 干净贴图结果宣称真实比赛识别率；
- 在释放证据和安全仲裁未完成前连接真实舵机。

## 10. 文档职责与清理规则

| 文档 | 职责 | 是否维护当前优先级 |
| --- | --- | --- |
| `AGENTS.md` | 安全、环境、编码和 agent 工作规则 | 只给入口，不复制任务表 |
| `README.md` | 项目状态和文档导航 | 只列最近一个里程碑 |
| `VISION_2026_ROADMAP.md` | 架构、任务顺序、验收 Gate | **是，唯一来源** |
| `SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md` | 仿真分层、环境和入口选择 | 否 |
| `docs/TOUDI3_FULL_SIM_GUI_GUIDE.md` | toudi3 逐步操作与排障 | 否 |
| `docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md` | 原 2025 链阻塞证据、world 派生规则和 R0～R6 Gate | 否 |
| `VISION_WORKSPACE_GUIDE.md` | 工作区和代码归属 | 否 |
| `VISION_MIGRATION_CHECKLIST.md` | 迁移/发布 Gate 清单 | 只记录通过状态 |
| `VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md` | RKNN/板端部署门禁 | 只维护部署任务 |
| `VISION_CHAIN_COMPARISON.md` | 新旧链事实对照 | 否 |
| `docs/*_YYYYMMDD.md` | 某次评测或决策的历史证据 | 否，不滚动改写历史结论 |

原 `VISION_2026_COMPLETE_DEVELOPMENT_PLAN.md` 与执行计划、迁移清单和进度评估重复，且把历史阶段和当前任务混在一起，本轮删除；其仍有价值的架构、门禁和任务内容已收敛到本文。

每次代码、launch、配置或文档修改后，仍须追加 `docs/仿真联调变更记录.md`。指标门禁如需调整，记录原值、样本、原因和新值，禁止只改数字不留证据。
