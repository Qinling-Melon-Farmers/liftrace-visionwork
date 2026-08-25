# RoboCup 无人机投递工程（2025 基线 → 2026 视觉升级）

更新时间：2026-08-26

## 1. 项目定位

本仓库保留 2025 年机载定位、规划、控制和投递工程，并面向 2026 规则升级视觉搜索、目标记忆、投递精定位和阶段门控。视觉组的日常主工作区是 `vision_ws/src/uav_vision`；主集成工作区用于导航、规划、控制和安全联调。

当前视觉组唯一任务路线见 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。其他文档不再各自维护一份“下一步”。

## 2. 当前结论

### 已具备

- `vision_ws` 和主集成工作区已有可编译基线；
- `uav_vision` 已有六分类 dev/sim 检测、RKNN 入口、红十字/圆环/H 几何检测、融合、精修、地图投影、候选记忆、对准和旧接口兼容；
- 现有主链为：

```text
detections
  -> detections_resolved
  -> detections_refined
  -> detections_mapped
  -> targets / selected_target
  -> drop_offset / drop_ready / release_evidence
```

- 已有确定性 mock、联合 overlay、`uav_vision_eval` 独立真值/自动报告和 headless shadow；
- `AstraDroneOpen + PX4 SITL + Gazebo Classic + iris_mid360` 可作为外部仿真底座；
- 当前统一仿真默认使用根目录 `toudi4_copy.world`，并加载从
  `iris_mid360_downward_camera.zip` 提取的单下视相机 + MID360 机架；
- 斜下辅助相机 `V-EXP-01` 已冻结且从未合入 `main`；生产与比赛准备只维护单个下视 RGB
  相机。实验分支和日志完整保留，冻结结论见
  [VEXP01辅助相机探索冻结说明_20260823.md](/home/xhj/liftrace/docs/VEXP01辅助相机探索冻结说明_20260823.md)；
- `toudi3.world` 及其旧航点/旧机架入口继续保留，用于历史回归；
- 已有旧视觉链完整入口，以及新 `uav_vision` Phase D、固定视觉 suite 和 shadow 入口；
- 主集成工作区 `uav_mission` 已建立任务层释放许可、受控旧 `/Servo` 代理、
  `/legacy/Servo_raw` 仿真 mock、旧控制 `Aligning` 状态互锁、三投 JSON 事件审计和顺序
  三槽确定性回归；旧控制/执行源码保持不变（仅外部任务模式最小补丁，快照见
  `legacy_baseline/`）；
- V-CL-04 覆盖搜索三投闭环已实跑：`logs/toudi4_coverage_r6_v2_20260813_221737/`
  12/12 覆盖、五类五 ID、tank/panzer/bridge 按权重 3/3 投递、0 碰撞、0 越界、
  377.6 s 降落；V-CL-05 高权重中断投递（red_cross=10/tank=5 发现即中断搜索、投完
  恢复）、red_cross 统一入队与随机红十字摆放已落地并实跑验证中断机制；
- 已将导航组 `liftrace-controlwork@5144aa8` 的原始 Python 搜索 manager/策略接入当前
  单下视新视觉链，导航源码保持原逻辑，外围适配器承接旧控制 ALIGN 与 guarded mock
  投递。2026-08-26 headless 在 600 s 内完成 tent/bridge 两投，第三投因任务超时未完成；
  当前只能判定“跨组主链接通”，不能判定完整比赛任务 PASS；
- 笔记本 dev/sim 已使用 `merged_standard` 六分类模型；已有实拍回放、压力集和 ONNX
  候选，当前不缺“再训练一个模型”；
- 笔记本 dev/sim 视觉链已运行；OrangePi 已完成统一六分类 FP32 RKNN 离线图片、视频和
  短时实时相机烟测，但尚未运行带 CameraInfo/TF 的完整 ROS 视觉链。

### 尚未完成

- 导航组 manager 当前只消费实时 `selected_target`，没有持久候选队列、全局权重重排、
  失败冷却重试或剩余时间调度；最新 600 s 联调只到 9/16 覆盖和 2/3 投递；
- V-CL-04/V-CL-05 的 Gate PASS 尚缺一次 Fast-Planner 正常轮次的干净复跑（低空下降
  速率与返航可达性波动属规划侧，外置处理，不阻塞视觉业务闭环）；
- H 视觉降落 Gate（landing 阶段 + H 结构证据 + 落地）未验收；北区走廊 Gate 因
  toudi4 缺门洞实体未立；随机红十字摆放的独立发现/投递待复跑验收；
- 固定 Gazebo 场景部分标准类/红十字召回低于 0.95 Gate（红十字约 0.646）；
- 尚未完成 30-seed 和 10 min shadow 稳定性回归；
- 实拍圆环回放自洽关联率 58.80%，缺人工实例/中心真值和同步 pose；
- H 结构已实现并离线验证（贴图 4 尺度 + 历史渲染图 PASS），仍缺实拍 H/普通黑圈负样本；
- PT/ONNX 八图对照仍有 1 missing/1 extra；FP32 RKNN 已有离线证据，OrangePi ROS
  相机/TF 与 10 min 稳定性未验收，INT8 当前零有效检测；
- `drop_ready`/`release_evidence` 不是最终动作许可；`/mission/release_permission`
  仲裁已落地并在实跑三投中实际放行。

## 3. 当前最重要的里程碑

视觉任务闭环已打通（V-CL-02 固定三投 → V-CL-03 外部单候选 → V-CL-04 覆盖搜索
权重三投 3/3 → V-CL-05 高权重中断 + red_cross 统一），当前里程碑：

1. 在不修改导航组已验证 manager 源码的前提下，先由双方冻结“实时中断候选 + 持久地图
   候选队列 + 投递结果/重试”接口；600 s 内完成三投、返航和落地，再扩到 4 个标准靶 +
   1 个红十字全随机布设；
2. 完成运动相机 stable ID/地图误差 Gate、H 视觉降落和实拍人工真值；北区走廊由联合 Gate
   独立验收；
3. 闭环稳定后完成单下视 30-seed、10 min 和 OrangePi ROS 相机/TF/RKNN
   （V-SIM-04/V-REAL-01/V-DEPLOY-01）。

第一周的具体任务和验收阈值见 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。

## 4. 目录职责

| 目录 | 职责 | 使用原则 |
| --- | --- | --- |
| `vision_ws/src/uav_vision/` | 2026 视觉运行包 | 新视觉业务代码主线 |
| `vision_ws/src/uav_vision_eval/` | 视觉评测包 | 只放真值、记录、评分和场景测试代码 |
| `vision_ws/test_data/` | 数据集、回放和评测证据 | 大文件不纳入版本管理建议 |
| `patrol_uav_ws-patrol_planner/` | 定位、规划、控制和整机集成 | 视觉只做必要接口联调 |
| `patrol_uav_ws-patrol_planner/src/uav_mission/` | 任务调度/释放安全边界 | 不放视觉算法，不直接访问 PWM |
| `Visual/`、`detect_ws/` | 2025 历史视觉资产 | 只对照，不作为新开发入口 |
| `vision_ws/migration_refs/` | 旧视觉代码快照 | 不放回 `src/` 编译 |
| `Desktop_patrol_uav_ws-patrol_planner/` | 历史副本 | 不修改为主线 |
| `top_level_scripts/` | 仿真及历史板端入口 | 启动前确认是否包含硬件动作 |
| `docs/` | 操作手册、评测证据和变更台账 | 日期文档视为历史快照 |

## 5. 仿真入口怎么选

| 目的 | 入口 | 结论边界 |
| --- | --- | --- |
| 消息、投影、状态 assertion | `phase_d_map_mock.launch` 等 | 证明软件契约，不证明图像算法 |
| 检查 toudi3 场景/飞机/相机 | `patrol_world.launch` | 不发布完整任务航路 |
| 旧控制链回归 | `run_toudi3_full_competition_sim_headless_old.sh` | 证明旧任务链，不证明新视觉 |
| 新视觉 GUI 连通烟测 | `run_toudi3_full_competition_sim_gui_new.sh` | 只作人工烟测 |
| 固定 Gazebo 真值回归 | `run_toudi3_visual_suite.sh` | 当前笔记本 L1 基线，不代表板端 |
| headless shadow | `uav_vision_eval/toudi3_full_shadow.launch` | 只观察隔离；10 min Gate 待完成 |
| 释放安全边界回归 | `run_release_guard_regression.sh` | 纯 mock；证明许可、槽位和防重放，不证明飞行闭环 |
| 新视觉+旧控制受控投放 SITL | `uav_mission/toudi3_visual_delivery_guarded.launch` | raw 端为 mock；三投已实跑 3/3（v2 日志），Gate 待干净复跑 |
| 导航组 manager + 新视觉完整任务 | `uav_mission/navigation_search_delivery_toudi4.launch` | 2026-08-26 为 2/3 投递、600 s 超时；用于跨组联调，尚非 PASS |

完整环境和入口判断见 [SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md](/home/xhj/liftrace/SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md)，逐步操作见 [docs/TOUDI3_FULL_SIM_GUI_GUIDE.md](/home/xhj/liftrace/docs/TOUDI3_FULL_SIM_GUI_GUIDE.md)。原 2025 链窄门阻塞、航点语义和派生 world 方案见 [docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md](/home/xhj/liftrace/docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md)。

## 6. 安全边界

- 仿真默认不启动 `actuator_pwm`；只允许 guarded `/Servo` 转发到纯软件 raw mock；
- 未经用户确认，不执行实机解锁、起飞、飞控模式切换、投递、PWM 或遥控接管；
- `drop_ready`、`selected_target` 都不是最终动作许可；
- OrangePi 主推理路径使用 RKNN/NPU，PyTorch 只用于本机训练和 dev/sim；
- 相机话题、CameraInfo、TF、模型路径和输出话题必须参数化。

## 7. 推荐阅读顺序

1. [AGENTS.md](/home/xhj/liftrace/AGENTS.md)：环境、安全和工作规则；
2. [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)：视觉架构、任务和验收；
3. [VISION_WORKSPACE_GUIDE.md](/home/xhj/liftrace/VISION_WORKSPACE_GUIDE.md)：代码应该放在哪里；
4. [vision_ws/src/uav_vision/README.md](/home/xhj/liftrace/vision_ws/src/uav_vision/README.md)：节点、接口和 launch；
5. [SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md](/home/xhj/liftrace/SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md)：仿真层级与入口；
6. [docs/TOUDI3_FULL_SIM_GUI_GUIDE.md](/home/xhj/liftrace/docs/TOUDI3_FULL_SIM_GUI_GUIDE.md)：完整操作；
7. [VISION_MIGRATION_CHECKLIST.md](/home/xhj/liftrace/VISION_MIGRATION_CHECKLIST.md)：迁移 Gate；
8. [VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md](/home/xhj/liftrace/VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md)：板端部署；
9. [docs/仿真联调变更记录.md](/home/xhj/liftrace/docs/仿真联调变更记录.md)：最近变更和验证证据。
10. [docs/导航组任务链与新视觉联调HANDOFF_20260826.md](/home/xhj/liftrace/docs/导航组任务链与新视觉联调HANDOFF_20260826.md)：导航组源码边界、候选语义、headless 结果和接续任务。
11. [docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md](/home/xhj/liftrace/docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md)：旧链完整赛程阻塞与 world 改造 Gate。
12. [docs/VISION_LAPTOP_SIM_BASELINE_20260715.md](/home/xhj/liftrace/docs/VISION_LAPTOP_SIM_BASELINE_20260715.md)：当前笔记本定量基线与局限。

## 8. 历史与专题文档

- 新旧视觉链事实对照：[VISION_CHAIN_COMPARISON.md](/home/xhj/liftrace/VISION_CHAIN_COMPARISON.md)
- 数据与自动标注历史：[VISION_DATASET_AUTO_LABEL_PLAN.md](/home/xhj/liftrace/VISION_DATASET_AUTO_LABEL_PLAN.md)
- 视觉/控制规则适配设计记录：[docs/VISION_CONTROL_RULE_ADAPTATION_20260707.md](/home/xhj/liftrace/docs/VISION_CONTROL_RULE_ADAPTATION_20260707.md)
- 2026-07-14 进度快照：[docs/VISION_PROGRESS_ASSESSMENT_20260713.md](/home/xhj/liftrace/docs/VISION_PROGRESS_ASSESSMENT_20260713.md)
- 实拍完整链评测：[docs/REAL_TARGET_FULL_CHAIN_EVAL_20260706.md](/home/xhj/liftrace/docs/REAL_TARGET_FULL_CHAIN_EVAL_20260706.md)
- 六分类红十字评测：[docs/YOLO_6CLS_REDCROSS_EVAL_20260713.md](/home/xhj/liftrace/docs/YOLO_6CLS_REDCROSS_EVAL_20260713.md)
- toudi3 资产状态：[TOUDI3_SIM_ASSET_CHECKLIST.md](/home/xhj/liftrace/TOUDI3_SIM_ASSET_CHECKLIST.md)
- 当前责任边界：[docs/当前问题与责任边界.md](/home/xhj/liftrace/docs/当前问题与责任边界.md)
