# 视觉迁移与发布 Gate

更新时间：2026-08-30

本文件只记录 2025 旧视觉链迁移到 `uav_vision` 的通过状态，不维护任务优先级。执行顺序和指标见 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。

状态：`[x]` 已有可重复证据；`[~]` 已实现但未完整验收；`[ ]` 未完成。

## Gate M0：工作区与主入口

- [x] `vision_ws` 独立工作区已建立并可编译；
- [x] `uav_vision` 是新视觉唯一运行主包；
- [x] `migration_refs` 仅作旧代码快照，不参与编译；
- [x] dev/sim、board、map mock、patrol mock launch 已分开；
- [x] `uav_vision_eval` 独立评测包已建立。

## Gate M1：检测节点迁移

- [x] 标准目标 dev/sim 六分类入口；
- [~] RKNN 入口已实现，FP32 六分类已完成 OrangePi 离线实测；缺完整 ROS 相机/TF 验收；
- [x] 红十字几何检测进入统一 detections；
- [x] 蓝色圆环检测进入统一 detections；
- [x] landing/H 外圈检测进入统一 detections；
- [x] H 外圈 + 内部结构验证与 `landing` 阶段门控已有固定 Gazebo 回归；正例 458 TP、
  0 FP/FN，landing-active 纯背景 511 帧 0 FP；
- [~] 固定正例/背景负例已通过；残圈、普通黑圈、实拍负样本和完整视觉降落 Gate 仍待扩充。

## Gate M2：融合与实例语义

- [x] `detection_fusion` 已有同帧裁决；
- [x] `target_refiner` 使用同源时间戳和全局一对一类别—圆环关联；
- [~] 单目标 Gazebo 中 actionable 标准靶可精修；逐类正式召回率/错配率 Gate 未全部达标；
- [ ] 实拍圆环真值已补齐并通过域差回放；
- [x] 关联失败的标准靶不会进入 target memory、旧控制接口或有效释放证据；
- [x] 输出包含 `center_source`、`association_valid`、TF 年龄和结构化拒绝原因。

## Gate M3：地图投影与记忆

- [x] `target_map_projector` 已消费 CameraInfo + TF + 地面平面；
- [x] `target_memory` 已支持像素/地图匹配和 reset 服务；
- [x] Phase D/板端对无效地图投影失败关闭；缺 TF 不得创建或刷新候选；
- [x] 标准靶按同一地图位置跨类别保持物理 ID，并使用连续高置信帧与累计投票切类；
- [x] 地图坐标采用 `map_quality` 加权融合；相距较远的地图目标不得由像素近邻误合并；
- [x] mock 投影、stable ID、reset 与记忆 assertion 已有；
- [~] 单目标固定 Gazebo 场景地图误差小于 0.015 m；30-seed/跨视角 Gate 未完成；
- [~] stable ID、类别抖动和地图距离去重 L0 已通过；跨视角重复 ID 率尚无 30-seed 报告；
- [x] 长期地图记忆与当前 `last_seen` 观测明确分离；
- [x] 过期观测不会因候选重发重新获得控制新鲜度；
- [~] reset 服务有回归；任务开始/结束的控制侧调用仍待联调。

## Gate M4：任务阶段与对准

- [x] `/uav_vision/align_mode` 已存在；
- [x] `drop_aligner` 已输出 `drop_offset/drop_ready`；
- [x] 旧主控已订阅新视觉候选、偏差、对准和短时任务许可；
- [~] `disabled/drop_circle/drop_cross/landing` 模式有回归；任务层完整阶段矩阵待联调；
- [x] H 在非 `landing` 阶段不能进入 operational 候选；
- [x] `red_cross` 与 H 按当前 align mode 过滤；
- [x] `drop_ready` 明确只表示视觉对准，不表示动作许可；
- [x] `/uav_vision/release_evidence` 已实现身份、结构、新鲜度、稳定帧和拒绝原因；
- [x] `/mission/release_permission`、顺序载荷、防重放、最终下降投递承诺和旧 Servo 安全
  代理已通过固定路线完整 SITL；许可绑定 `Aligning=2`，三投 JSON 审计 PASS。速度/机构
  状态互锁与 Mission Manager 上下文属于后续外部任务模式/实机阶段。
- [x] 默认关闭的外部任务模式已接入 `MissionCommand`；单候选完成接近、对准、guarded
  ACK 和恢复，外部模式下 Mission Manager 是 `/fastplanner/goal` 唯一发布者；默认旧路线
  回归保持由 `patrol_control` 发布。
- [~] 导航组 `liftrace-controlwork@5144aa8` 原始 Python manager/策略已通过外围适配器接入；
  manager 独占 `/navigation/goal_raw`，适配器独占 `/fastplanner/goal`。当前 manager 只对
  当前新鲜 `selected_target` 做合法性/新 ID 准入，尚无持久全局权重队列、失败重试和
  剩余时间调度。

## Gate M5：旧接口兼容

- [x] `/yolo_detect` 无标准目标时发布 `Nothing`；
- [x] `red_cross/circle/landing_pad` 不误入 `/yolo_detect`；
- [x] `/detect/cross_status`、`/detect/tank_status` 已兼容；
- [~] `/detect/waypoint_mark_point`、`cross_mark_point`、`land_mark_point` 仅有限兼容；
- [x] 默认不把像素点伪装成旧世界点；
- [ ] 旧接口消费者清单、下线条件和下线日期已确定。

## Gate M6：仿真证据

- [x] 旧链 toudi3 无 GUI 起飞—巡航—降落回归有记录；
- [x] 新视觉 toudi3 GUI launch/脚本和联合环境入口可解析；GUI 仍只作人工烟测；
- [x] 统一 overlay/环境脚本通过 `rospack`、消息 import 和 launch 解析；
- [x] 新视觉 headless shadow 入口存在且全局控制/执行输出受保护；
- [x] `newvision_fixed3` 无 GUI Gate 完成三类地图候选和三次受许可 mock ACK，零重复/越权；
- [x] `external_candidate` 与 `legacy_mode_regression` 无 GUI Gate 均通过，分别验证外部闭环
  和默认旧路线兼容；
- [x] `target_area_navigation` 无 GUI Gate 完成靶标区域 12/12 非真值坐标覆盖、五类五个
  stable ID、安全返航、零碰撞/越界/Servo；北区走廊避障不属于本轮 Gate；
- [x] `coverage_r6` 按权重三次 guarded mock 投递已实跑 3/3；2026-08-13 v2 的
  tank/panzer/bridge、0 碰撞/越界、377.6 s 历史证据保留，2026-08-28
  `toudi4_coverage_r6_vcl04_rerun2_20260828_222644` 已按规划器波动外置口径关闭 V-CL-04；
- [~] V-CL-05 搜索-投递策略：高权重中断投递（red_cross=10/tank=5 发现即中断搜索、
  投完恢复，`logs/toudi4_coverage_r6_vcl05b_20260813_234314/` 中断机制与 pillbox
  端到端投递已验证）、red_cross 统一入队（无独立任务模式）、随机红十字摆放（真值
  仅落盘）与动态期望 Gate 已落地；随机十字独立发现/投递与沉降门控待复跑验收；
- [~] V-CL-06 导航组 manager 联调：
  `logs/navigation_upstream_visual_delivery_headless_model_20260826_023411/` 使用 7.14
  `merged_standard` 模型，600 s 到达 9/16 覆盖点，发现 pillbox/tent/bridge，tent/bridge
  完成槽 1/2 guarded mock 投递，pillbox 因 capture timeout 安全拒绝；第三投、返航和
  降落未完成，Gate 为 `mission_timeout`，不得标记 PASS。当前正式随机场 baseline 30 s
  预检 PASS，baseline/`a68925d` 90 s A/B 比较 FAIL，保持 baseline，尚未进入新的 600 s Gate；
- [x] 仿真真值来自 target catalog、Gazebo/model state、CameraInfo/TF，不依赖检测输出；
- [x] 五类标准靶、红十字、H、背景固定场景与自动 recorder/report 已落地；
- [x] L0 圆环坐标、全局关联、记忆新鲜度、地图/释放证据连续 3 次通过；
- [~] V-SIM-04 L1 operating surface 首轮完成：formal23 为 22/23，static25 为 24/25，
  sparse30 为 25/30；formal/sparse processing P95=195.3/153.9 ms，地图 P95=0.0792/0.1103 m，
  TF failure=0。测量完整但采用门槛未冻结，结论仍为 `NOT_GATED`；
- [x] 全类别 hard audit、动态 pose/速度/yaw/横偏遥测、实际产物核验和 failure capture
  schema-v3 已通过 mock 与真实 diagnostic；r2026 下 tank/disallowed/policy-rejected selected=0；
- [~] pillbox 3.6 m 的 1280 diagnostic 恢复 P_confirm，但共视 bridge 被 selected、
  P_selected=0 且 processing P95=449.3 ms，超过 200 ms；保持 640 默认，不推广 1280；
- [ ] L2 10 min 无崩溃、积压或时间戳异常；
- [ ] 30-seed 报告已归档。

## Gate M7：实拍与板端

- [x] 六分类模型已有实拍回放和独立压力集；
- [~] 圆环实拍等比例回放已建立，但缺人工中心/实例真值；H/地图同步真值仍缺；
- [x] 笔记本 ONNX 与 PyTorch 在同一 `640x640` fixed-letterbox 的 12 图、19 框上数值一致：
  missing/extra=0、最低 IoU=0.9999991；该项不代表 RKNN 或板端 ROS 验收；
- [ ] RKNN 与 ONNX 精度差异有逐样本报告；
- [ ] OrangePi 搜索阶段达到 5-10 Hz；
- [ ] OrangePi 10 min 无队列积压、内存持续增长或节点崩溃；
- [ ] CPU、NPU、内存、P50/P95 时延和温度有记录。
- [x] 已形成不含控制代码的 `uav_vision` Alpha 交付入口、中文接口说明和可复核 ZIP；

## 发布判定

- M0-M3 未通过：只能做算法开发或 mock，不得宣称地图目标闭环；
- M4 未通过：不得连接真实释放动作；
- M6 未通过：不得宣称仿真已证明算法有效；
- M7 未通过：不得宣称 OrangePi 实机可部署；
- 旧接口兼容不等于新接口业务闭环完成。
