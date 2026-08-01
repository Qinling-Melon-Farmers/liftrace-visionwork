# RoboCup 无人机投递工程（2025 基线 → 2026 视觉升级）

更新时间：2026-07-16

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
- `toudi3.world` 的五类标准靶、H 和独立红十字模型已恢复；
- 已有旧视觉链完整入口，以及新 `uav_vision` Phase D、固定视觉 suite 和 shadow 入口；
- 主集成工作区新增 `uav_mission`：任务层释放许可、受控旧 `/Servo` 代理、
  `/legacy/Servo_raw` 仿真 mock、旧控制 `Aligning` 状态互锁、三投 JSON 事件审计和顺序三槽
  确定性回归；旧控制/执行源码保持不变；
- 笔记本 dev/sim 已使用 `merged_standard` 六分类模型；已有实拍回放、压力集和 ONNX
  候选，当前不缺“再训练一个模型”；
- 笔记本 dev/sim 视觉链已运行；OrangePi 已完成统一六分类 FP32 RKNN 离线图片、视频和
  短时实时相机烟测，但尚未运行带 CameraInfo/TF 的完整 ROS 视觉链。

### 尚未完成

- 没有真正的全场自主搜索；旧主控仍是固定航点加机会式中断；
- 固定 Gazebo 场景已可量化，但部分标准类/红十字召回低于正式 0.95 Gate；0.15 s
  有界融合下代表 tent/tank/red_cross 的 P95 延迟已为 172/176/182 ms；
- 尚未完成 30-seed 和 10 min shadow 稳定性回归；
- 实拍圆环回放已修复纵横比并有 58.80% 自洽关联率，但缺人工实例/中心真值和同步 pose；
- H 结构、地图新鲜度和 release evidence 已实现，仍缺扩展实拍/跨视角正式 Gate；
- 地图契约已记录中心来源、关联、TF 年龄和拒绝原因；Phase D 无效 TF 不建候选；物理
  stable ID 已加入连续帧、置信度、类别投票和质量加权坐标融合 L0 回归；
- PT/ONNX 八图对照仍有 1 missing/1 extra；FP32 RKNN 已有离线性能证据，OrangePi ROS
  相机/TF 和 10 min 稳定性未验收，INT8 当前零有效检测；
- `drop_ready` 与 `release_evidence` 都不是最终动作许可；第一版
  `/mission/release_permission` 已落地。完整旧控制 SITL 已恢复地图 `selected_target`，但
  旧控制没有消费其 `map_point` 完成稳定导航闭环，raw Servo 仍为 0；完整三投尚未验收。

## 3. 当前最重要的里程碑

“可测”最小基建已经完成，当前里程碑改为视觉任务闭环：

1. `camera_init` 地图候选和单下视相机链已在完整 SITL 恢复；
2. 已形成不含控制代码的 `uav_vision` Alpha 交付包，控制组应直接消费合法 `map_point`；
   后续联合完成三个固定检测点 permission→guarded Servo→mock ACK；
3. 复用 Fast-Planner 完成单候选接近、重捕获和恢复搜索；
4. 再扩为覆盖航线、权重队列和三次 mock 投放；30-seed/延迟按闭环失败阶段后置收紧。

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
| 新视觉+旧控制受控投放 SITL | `uav_mission/toudi3_visual_delivery_guarded.launch` | raw 端为 mock；完整三投尚待验收 |

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
10. [docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md](/home/xhj/liftrace/docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md)：旧链完整赛程阻塞与 world 改造 Gate。
11. [docs/VISION_LAPTOP_SIM_BASELINE_20260715.md](/home/xhj/liftrace/docs/VISION_LAPTOP_SIM_BASELINE_20260715.md)：当前笔记本定量基线与局限。

## 8. 历史与专题文档

- 新旧视觉链事实对照：[VISION_CHAIN_COMPARISON.md](/home/xhj/liftrace/VISION_CHAIN_COMPARISON.md)
- 数据与自动标注历史：[VISION_DATASET_AUTO_LABEL_PLAN.md](/home/xhj/liftrace/VISION_DATASET_AUTO_LABEL_PLAN.md)
- 视觉/控制规则适配设计记录：[docs/VISION_CONTROL_RULE_ADAPTATION_20260707.md](/home/xhj/liftrace/docs/VISION_CONTROL_RULE_ADAPTATION_20260707.md)
- 2026-07-14 进度快照：[docs/VISION_PROGRESS_ASSESSMENT_20260713.md](/home/xhj/liftrace/docs/VISION_PROGRESS_ASSESSMENT_20260713.md)
- 实拍完整链评测：[docs/REAL_TARGET_FULL_CHAIN_EVAL_20260706.md](/home/xhj/liftrace/docs/REAL_TARGET_FULL_CHAIN_EVAL_20260706.md)
- 六分类红十字评测：[docs/YOLO_6CLS_REDCROSS_EVAL_20260713.md](/home/xhj/liftrace/docs/YOLO_6CLS_REDCROSS_EVAL_20260713.md)
- toudi3 资产状态：[TOUDI3_SIM_ASSET_CHECKLIST.md](/home/xhj/liftrace/TOUDI3_SIM_ASSET_CHECKLIST.md)
- 当前责任边界：[docs/当前问题与责任边界.md](/home/xhj/liftrace/docs/当前问题与责任边界.md)
