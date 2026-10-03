# uav_vision

更新时间：2026-09-05
定位：RoboCup 2026 在线视觉运行包。评测真值、场景和报告代码位于同工作区的
`uav_vision_eval`，不进入本包实机依赖。OrangePi 已完成新相机、CameraInfo、统一 RKNN
ROS 像素链、完整视频回放和 600 秒稳定性验收；2026 安装平移已写入板端入口，仍待整机
TF、有效地图投影和导航/LIO 并发验收。完整模型结果见
[板端模型完整评测报告](/home/xhj/liftrace/docs/BOARD_MODEL_COMPLETE_EVALUATION_20260716.md)。

面向未改造控制工程的纯视觉交付入口为 `control_handoff_dev.launch` 和
`control_handoff_board.launch`。两者默认不发布旧 Pose 兼容话题，也不启动任何控制、规划、
MAVROS 指令或执行机构节点。接口、单目地图点公式和接入要求见
[控制组联调说明](docs/CONTROL_GROUP_HANDOFF.md)。

面向导航组的候选字段、话题契约和阶段 4 参考实现边界见
[导航组接口说明](docs/NAVIGATION_GROUP_HANDOFF.md)。视觉包不发布 planner goal；ZIP 中
可选的 coverage 参考代码不进入视觉工作区编译，采用方式由导航组决定。

## 1. 节点

| 节点 | 职责 | 当前边界 |
| --- | --- | --- |
| `target_detector.py` | dev/sim 六分类检测 | PyTorch，只用于本机开发/仿真 |
| `target_detector_rknn.py` | 板端六分类检测 | 需 OrangePi/RKNN 实测 |
| `cross_detector_node` | 红十字几何观测 | 负责中心/形状复核，不独占目标决策 |
| `circle_detector_node` | 蓝色圆环几何观测 | 等比例 letterbox；实拍绝对中心真值仍缺 |
| `landing_detector_node` | H 外圈 + 内部 H 结构观测 | 仍需扩真实黑圈/H 负样本 |
| `detection_fusion.py` | 同源时间戳聚合与阶段裁决 | 笔记本仿真等待参数仍需降延迟 |
| `target_refiner.py` | profile 感知的全局一对一类别—圆环关联与中心精修 | 未关联标准靶不进入 operational 链；禁用类不消耗蓝环 |
| `target_map_projector.py` | CameraInfo + TF 地面投影 | 固定 Gazebo 有真值；实拍同步 pose 仍缺 |
| `target_memory.py` | 连续帧确认、物理 stable ID、类别投票、地图融合与新鲜度 | 跨视角正式 Gate 仍待完成 |
| `drop_aligner.py` | 偏差、`drop_ready` 与结构化释放证据 | 最终许可仍属任务/安全层 |
| `detect_compat_bridge.py` | 旧 `/detect/*` 与 `/yolo_detect` 回归兼容 | 只供 legacy launch，正式 external 链不启动 |

## 2. 处理链与话题

```text
detectors
  -> /uav_vision/detections
  -> detection_fusion
  -> /uav_vision/detections_resolved
  -> target_refiner
  -> /uav_vision/detections_refined
  -> target_map_projector
  -> /uav_vision/detections_mapped
  -> target_memory
  -> /uav_vision/targets
  -> /uav_vision/selected_target

/uav_vision/align_mode + selected_target
  + /uav_vision/alignment_target_context (VCL06 strict mode, optional)
  -> drop_aligner
  -> /uav_vision/drop_offset
  -> /uav_vision/drop_ready
  -> /uav_vision/release_evidence
  -> /uav_vision/release_evidence_context
```

`detect_compat_bridge.py` 作为完整旧接口适配器保留给 legacy 回归；它不再订阅没有旧接口映射的
`/uav_vision/drop_ready`。`phase_d.launch` 和 `phase_d_board.launch` 默认关闭该桥，只有
`phase_d_patrol_internal.launch` 等旧链包装器显式启用。正式 external 链直接消费 typed 视觉输出，
不在视觉侧增加 H-only 模式或第二套消息。

输入默认值来自 `config/default.yaml`：

```text
image_topic: /camera/image_raw
camera_info_topic: /camera/camera_info
odom_topic: /mavros/local_position/odom
```

toudi3 wrapper 会在 launch 层改为 `/camera/color/image_raw` 和 `/camera/color/camera_info`。不要为某个仿真环境修改源码硬编码。

## 3. 消息语义

### `TargetDetectionArray`

各阶段的检测数组使用同一消息类型。消费者必须看 header、frame、`map_valid`、中心来源和置信度，不能只看类别字符串。

`header.stamp` 始终保留源图像时间；`center_source`、`association_valid`、
`reject_reason` 和 `transform_age_sec` 说明中心、关联及地图投影是否可用于闭环。

### `TargetCandidateArray` / `selected_target`

`targets` 是候选记忆，`selected_target` 是当前排序建议。地图候选可以长期存在，但
`selected_target` 和释放证据使用源观测 `last_seen` 执行独立新鲜度门禁；消费者仍不得仅凭
topic 刚收到就认定目标刚被看到。

当前 `config/target_memory.yaml` 中：

- 像素候选 TTL：3 s；
- 地图候选 TTL：0（直到 reset）；
- 地图匹配距离：0.5 m；
- 候选确认：连续 3 帧；漏检会清零连续计数，但不删除 TTL 内已确认地图记忆；
- `CONFIRMED` 保持为长期记忆状态；`selected_target` 仍要求当前连续 3 帧、地图/关联有效、
  无拒绝原因且观测年龄满足 `0 <= age <= 0.5 s`；未来时间戳、非有限置信度/地图质量/坐标
  和空 map frame 均 fail-closed，不进入 selected；
- stable ID、融合地图点和地图质量继续作为历史诊断记忆保留，但候选消息的 `map_valid`
  只表示当前帧投影有效；漏检或当前投影失败时不得借历史地图继续 selected；
- 类别切换：连续 2 帧且置信度不低于 0.70，同时累计置信度投票胜出；
- Phase D/板端 `require_map_for_candidates=true`，无效或陈旧 TF 不得刷新候选；
- 类别优先级用于候选排序，权重取自赛委会确认的得分权重：tent=1、pillbox=1.5、bridge=2、panzer=2.5、tank=5、red_cross=10。
- `class_profile=full` 保留六分类兼容；`class_profile=r2026` 保留 tank 诊断记忆但禁止其发布为
  `selected_target`。同一 profile 也下传 `target_refiner`：tank 原始框继续保留诊断，但不能抢占
  tent/pillbox/bridge/panzer 的蓝环关联。未知 profile 会令节点非零退出。

### `align_mode`

现有模式：`disabled`、`drop_circle`、`drop_cross`、`landing`。融合和记忆均按当前模式过滤：
H 只在 `landing`，红十字只在 `drop_cross`，标准靶/圆环只在 `drop_circle` 进入 operational 链。

### `drop_ready`

当前由像素中心偏差、最小置信度和稳定帧数生成，只表示“视觉观测满足当前对准阈值”。它不包含完整目标身份、地图/位姿质量、飞行速度、机构状态或规则互锁，因此不是 `release_permission`。

### `release_evidence`

聚合目标身份、确认状态、几何/中心验证、观测年龄、对准、稳定帧和拒绝原因。它仍是视觉
证据，不读取飞行速度、机构或规则互锁，也不等于任务/安全层最终 `release_permission`。

### `AlignmentTargetContext` / `ReleaseEvidenceContext`

VCL06 coordinator 可发布不依赖 `uav_mission` 的冻结上下文。其 command 常量与
`NavigationDecision v1` 同值，完整围栏为
`mission_id + decision_seq + semantic_target_id + semantic_target_first_seen + attempt + payload_slot`；
`has_target` 显式区分无目标与合法 `target_id=0`。默认 `require_alignment_context=false`，旧
`/uav_vision/release_evidence` 的类型、topic 和行为不变；正式接线显式开启 strict 后，缺失、
失鲜、错误 profile/command、deadline 到达或冻结字段变化都会清零稳定帧并 fail-closed。
strict watchdog 不依赖目标数组继续到达；目标流中断后，即使 coordinator 仍续租 context，当前
几何观测超龄也会主动发布无效证据并撤销 `drop_ready`，context 失鲜/deadline 到达同样处理。

标准投放区的语义候选 ID 与蓝环几何 ID 不要求相同，必须使用冻结语义 `target_pose` 与当前
圆环地图点做同 frame 距离关联；红十字按 `id + first_seen` 精确实例关联。
`ReleaseEvidenceContext` 内嵌旧证据并回显 context source/header、decision/语义身份、实际几何
身份、地图位姿、关联距离与 `context_valid/context_reason`，供导航/安全层做可审计关联。

## 4. launch

| launch | 用途 |
| --- | --- |
| `phase_b.launch` | 早期检测/兼容组合 |
| `phase_c.launch` | 融合与候选组合 |
| `phase_d.launch` | dev/sim PyTorch 完整在线链 |
| `phase_d_board.launch` | RKNN 板端链 |
| `phase_d_board_perf_mock.launch` | 板端接口/性能 mock |
| `control_handoff_dev.launch` | 控制组笔记本联调：纯视觉 PT 完整链 |
| `control_handoff_board.launch` | 控制组 OrangePi 联调：纯视觉 RKNN 完整链 |
| `board_camera_vision.launch` | OrangePi 新相机 + CameraInfo + RKNN + 可关闭的 2026 相机静态外参 |
| `phase_d_map_mock.launch` | 地图投影、记忆和对准 assertion |
| `target_memory_physical_mock.launch` | 类别抖动、连续帧、地图融合和物理 ID assertion |
| `map_rejection_mock.launch` | 缺 TF 时失败关闭 assertion |
| `phase_d_mock_patrol.launch` | 新视觉与旧主控 mock 接线 |
| `phase_d_mock_patrol_regression.launch` | 上述接线的自动 assertion |
| `phase_d_mode_mock.launch` | align mode 行为测试 |
| `circle_geometry_mock.launch` | 圆环坐标恢复测试 |
| `alignment_context_mock.launch` | VCL06 冻结上下文、围栏、租约和几何身份 assertion |
| `video_replay_annotation.launch` | MP4 逐帧可靠回放真实 ROS 像素链并生成单一标注视频；不启动地图/选靶/控制 |

真实视频人工审片入口、颜色图例和边界见
[当前 ROS 像素链 MP4 回放与标注](docs/VIDEO_REPLAY_ANNOTATION.md)。该入口复用正式
detector/fusion/refiner，不产生地图点、stable ID 或 selected target。

评测场景、真值、自动报告和 shadow 入口位于 `uav_vision_eval`。一键运行当前八个固定视觉
场景：

```bash
cd /home/xhj/liftrace
./top_level_scripts/run_toudi3_visual_suite.sh
```

该脚本只启动 Gazebo 相机和视觉链，不启动 PX4、MAVROS、旧控制、解锁或执行机构。

dev/sim：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/vision_ws/devel/setup.bash
roslaunch uav_vision phase_d.launch
```

板端整机视觉入口：

```bash
roslaunch uav_vision board_camera_vision.launch \
  video_devices:=/dev/v4l/by-id/<camera-id> \
  model_path:=/absolute/path/merged_standard_fp32.rknn
```

该入口默认发布 `body → downward_camera_optical_frame`：平移 `[0,0,-0.16] m` 来自
2026-09-05 机械安装汇报，旋转沿用单下视光学帧约定。雷达 IMU 到相机的主机械测量为
正下方 `0.21 m`，但当前 FAST-LIO 的 `body` 来自飞控 IMU，二者不得混填。若整机顶层已
发布同一 TF，使用 `publish_camera_extrinsic:=false`。详细基准和待验收边界见
[2026 实机安装外参基线](/home/xhj/liftrace/docs/2026实机相机与投递机构安装外参基线_20260905.md)。

板端离线评测约定：历史原始 MP4 回放不凭空附加新相机内参；实时相机入口默认使用当前
1280×720 `plumb_bob` 标定 profile。当前建议使用 `merged_standard_fp32.rknn`；INT8 产物
虽能加载但在 v5merge 全集无有效检测，不能作为主模型。

## 5. 最小回归

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/vision_ws/devel/setup.bash

roslaunch uav_vision phase_d_map_mock.launch
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
roslaunch uav_vision phase_d_mock_patrol_regression.launch

# 推荐：统一检查 PASS marker，避免 roslaunch 掩盖 required assertion 非零退出
/home/xhj/liftrace/top_level_scripts/run_visual_mock_regressions.sh
```

以 required assertion 节点退出码为准。当前 map mock 可在视觉工作区独立运行，patrol
regression 使用联合环境。任何消息、投影、模式或兼容语义改动都至少重跑相关 mock。

预定的完整 toudi3 新视觉入口：

```bash
cd /home/xhj/liftrace
bash ./top_level_scripts/run_toudi3_full_competition_sim_gui_new.sh
```

GUI 入口仍只算人工连通烟测；定量结论使用 `uav_vision_eval`，只观察联调使用 shadow 入口。

## 6. 已知高优先级缺口

1. 30-seed、10 min shadow 和正式阈值报告尚未完成；
2. 固定 Gazebo 中部分标准类与红十字召回低于 0.95；代表场景 P95 延迟已降至 200 ms 内；
3. 实拍圆环回放缺实例/中心人工真值，普通 MP4 也缺同步 CameraInfo/pose；
4. H/普通黑圈/残圈实拍负样本仍不足；
5. 笔记本完整 SITL 已用 MAVROS 位姿核对 `camera_init` TF；实机安装平移已配置，真实
   LIO 下的完整 TF、下视旋转和有效地图投影仍待验收；
6. 视觉兼容桥已收口为 legacy-only 并移除无效 `drop_ready` 订阅；正式 VCL06 关闭兼容桥、直接
   消费 typed H 的导航侧改动仍须提交并通过最终联合 Gate；
7. 笔记本 PT/ONNX 已在相同 fixed-letterbox 输入的 12 图/19 框 Gate 通过；RKNN 仍需用
   同一批输入做逐框对照，尚不能据此冻结最终板端部署模型；
8. 六分类 FP32 RKNN 的 OrangePi ROS 像素链与 600 秒稳定性已通过；安装 TF 的机械平移
   已接线，仍缺与真实 LIO/导航并发的地图投影验收；四款 INT8 当前全量 P/R/mAP 为 0。
9. 当前地图投影是单目射线与固定 `ground_z` 平面求交；不需要深度相机，但尚未接入
   MID360 局部地面拟合，非平地和错误地面高度必须判为额外风险。

## 7. 下一接口版本方向

现有消息已覆盖源 `header.stamp`、`last_seen`、stable ID、关联、中心来源、TF 年龄和拒绝
原因；后续只在真实消费者需要时补 `pose_valid/observation_age/stable_duration`，避免重复字段。

视觉已发布 `/uav_vision/release_evidence`；最终 `/mission/release_permission` 由任务/安全层
拥有。迁移期继续发布 `drop_ready`，但不得直接连接真实舵机。

## 8. 开发约定

- 图像订阅 queue size 为 1；
- debug image 默认关闭；
- 所有话题、frame、内参、模型和阈值参数化；
- OrangePi 不使用 PyTorch 主路径；
- 修改后运行对应 mock、固定仿真 seed 和实拍回放；
- 评测数据和报告放 `vision_ws/test_data`，不让评测包成为运行依赖；
- 每次修改追加根目录 `docs/仿真联调变更记录.md`。

## 9. 设计与验收入口

- 主路线：[../../../VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)
- 工作区职责：[../../../VISION_WORKSPACE_GUIDE.md](/home/xhj/liftrace/VISION_WORKSPACE_GUIDE.md)
- 仿真分层：[../../../SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md](/home/xhj/liftrace/SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md)
- 迁移 Gate：[../../../VISION_MIGRATION_CHECKLIST.md](/home/xhj/liftrace/VISION_MIGRATION_CHECKLIST.md)
