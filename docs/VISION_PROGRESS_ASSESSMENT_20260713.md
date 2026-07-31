# 视觉组规划进度评估（截至 2026-07-14）

> 历史快照：本文冻结 2026-07-14 的评估证据，不再滚动维护“下一步”。当前状态、任务顺序和仿真验收统一见 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。

## 1. 总体结论

当前已完成“六分类检测模型训练 + 飞行对地增强 + 离线回放 + 特征图产出”这一算法基线阶段，但尚未完成规则要求的“自主搜索—地图记忆—实例确认—精确投递—自主降落”整机闭环。

当前默认 dev/sim 配置仍指向历史候选：

```text
vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/weights/best.pt
```

2026-07-14 新训练的两个待选模型（尚未切换默认）：

```text
vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt
vision_ws/runs/liftrace_6cls_v5_region_focus_aug_20260714/weights/best.pt
```

两者均使用 v5 合并六分类数据集，训练参数均为 epochs=100、patience=20、batch=16、imgsz=640、device=0；区域聚焦增强模型已完成训练。

当前最重要的事实不是继续训练一个模型，也不是等待完整仿真，而是先把视觉业务闭环做成可回放、可 mock 验收的工程链。bridge 与 red_cross 在 v3 中的重叠来自红十字复用桥梁靶标底座的历史数据构造，正式比赛不复用该底座，因此不再把 bridge 抑制开关列为当前工作项。

本轮已恢复运行时 `priority_bridge=2.0`。最终模型仍为六分类；目前只有五类正式得分权重，`tank=1.0` 仅为临时工程排序，不代表正式得分。

2026-07-13 的无 GUI 旧链 toudi3 长回归已经完成旧控制链验证：PX4/MAVROS、起飞、航点巡航、两个无目标检测点超时切换和最终降落/解锁状态均已观测。该证据只覆盖旧视觉节点和旧主控的仿真时序，不代表标准靶标识别、圆环投递或新视觉链已经在 toudi3 世界中完成验收；显式 `AUTO.LAND` 成功日志也尚未捕获。

## 2. 按完整规划阶段评估

| 阶段 | 状态 | 已完成 | 主要缺口 |
| --- | --- | --- | --- |
| 阶段 0：基线与回归 | 已完成视觉离线基线 | v5 合并数据、两款 20260714 候选、真实视频双模型回放、独立压力集、坐标/resize 契约、纯 ROS mock | 外部输入和 RKNN 可行性门禁；压力集仍是合成验证 |
| 阶段 1：外部仿真接线 | 部分完成 | AstraDroneOpen + PX4 + `iris_mid360`、旧控制链、旧视觉链、FAST-LIO/FreeDOM 已可由统一入口启动并完成旧航点回归 | 新视觉 Phase-D 与 `patrol_control` 的真实图像/内参/TF/odom 端到端接线和 10 分钟稳定烟测 |
| 阶段 2：红十字六分类 | 已完成离线训练与压力评估（RKNN 除外） | v5 六类数据、标准模型、区域聚焦模型、真实视频和 8 条件压力评估 | 六分类 ONNX/RKNN 导出和板端验证；模型尚未按压力集结果冻结 |
| 阶段 3：YOLO—几何融合 | 已完成软件链 | YOLO、cross/circle/landing、fusion、精修字段和 refiner | 真实回放中的关联阈值和 H 语义 |
| 阶段 4：目标地图化 | 已完成软件 mock，真实数据待补输入 | `CameraInfo + TF + 地面平面` 投影、地图字段、稳定 ID、reset 服务；已加入真实视频评估入口 | 当前 MP4 无同步 CameraInfo/TF/位姿，不能给出地图绝对误差 |
| 阶段 5：类别—圆环关联 | 真实回放已开始，当前指标不达标 | 多圆环输出、原图中心恢复、类别—圆环关联、关联失败不作为精修中心；已产出真实标注视频/CSV | 严格圆环规则只检出少量局部蓝色区域，需针对外圈和视角重新标定 |
| 阶段 6：H 与安全释放 | 部分完成 | 外圈候选、`drop_ready`、landing 模式存在 | H 语义验证、规则安全余量、`ReleasePermission` 接口 |
| 阶段 7：搜索状态机与接口收口 | 部分完成 | 主控已订阅 selected/offset/ready，发布 align mode | target_id 锁定、搜索恢复、正式模式切换、旧触发源退场 |
| 阶段 8：OrangePi 部署 | 未开始 | RKNN 入口和 perf 消息字段存在 | 六类模型、真实 NPU、性能、热稳定性和输出一致性 |
| 阶段 9：任务验收 | 部分完成 | 离线模型、视频回放证据、旧链无 GUI 起飞—巡航—降落证据 | 有目标场景、真实视觉投递、静态机架、低风险飞行、随机场地闭环 |

## 2.1 最终增强模型完整链回放

使用：

```text
vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/weights/best.pt
```

回放配置为 `unified5`、当前 `cross_detector`、当前 `landing_detector`，明确关闭 bridge 抑制，仅做离线等价链路评测：

| 视频 | 处理帧 | 任一路检测 | red_cross 几何帧 | landing_pad 帧 | 原始 bridge+red_cross 同帧 | 抑制帧 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `real_target.mp4` | 1156 | 996 | 515 | 359 | 160 | 0 |
| `redcross.mp4` | 1443 | 1415 | 1415 | 1016 | 0 | 0 |

关键判断：`real_target` 的样本帧可同时出现 `red_cross` 与 `landing_pad(H 外圈)`，这不是视觉输出错误，而是多路观测并存。控制侧必须依据任务状态决定当前只允许 `drop_circle`、`drop_cross` 或 `landing` 中的一种模式拥有对准/释放权；不能用视觉层整帧互斥来替代状态机。当前结果也不支持继续推进 bridge 抑制策略。

## 2.2 `vision_ws` 拓扑图资产

当前 SVG 位于 `vision_ws/` 根目录：

- `Nodes_Topics(active)2.svg`
- `Nodes_Topics(all).svg`
- `Nodes Only.svg`
- `rosgraph.svg`
- `toudi3_full_sim_nodes.svg`、`toudi3_old_chain_nodes.svg`、`toudi3_new_visual_chain_nodes.svg`

前四个是历史 Qt/rqt 拓扑导出，后三个是当前 toudi3 旧链/新链操作视图；XML 均已确认有效。具体接口判断仍以 `uav_vision` 当前 launch、消息定义和源码为准。

## 3. 当前接口成熟度

### 3.1 视觉组自身的执行顺序

完整项目仍需外部仿真作为联调依赖，但视觉组可以独立按以下顺序交付，不必等待完整比赛世界：

1. 冻结图像坐标、时间戳、frame、来源和 `center_px/radius_px` 契约（已完成软件基线）；
2. 完成 H 结构验证与质量输出，明确“检测到 H”和“允许降落确认”不是同一语义；
3. 实现 `CameraInfo + TF + 地面平面` 参数化投影、地图记忆和稳定目标 ID（已完成软件 mock）；
4. 输出多圆环实例，完成标准类别—圆环关联和投递靶心质量判定（已完成软件 mock）；
5. 将 `drop_ready` 与规则级 `release_permission` 分离；
6. 最后做旧接口回归、RKNN 导出和 OrangePi 性能验证。

外部 Astra/PX4 和 `patrol_control` 状态门控属于联调依赖：它们要消费这些视觉接口，但不应成为视觉组继续推进业务闭环的前置阻塞。

### 已可作为联调输入

- `/uav_vision/detections`
- `/uav_vision/detections_resolved`
- `/uav_vision/targets`
- `/uav_vision/selected_target`
- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`
- `/uav_vision/align_mode`

### 真实视频圆环/地图评估（2026-07-13）

已新增 `vision_ws/scripts/eval_ring_map_video.py`，复用现有 v5 完整链检测记录，
不重新运行 YOLO，在真实 `real_target.mp4` 上执行当前 C++ 圆环规则和类别关联。
本次使用：

```text
video: vision_ws/test_data/real_target.mp4
detections: vision_ws/test_data/real_target_full_chain_v5_flight_aug_20260713_stride4/detections_long.csv
output: vision_ws/test_data/real_target_ring_map_eval_20260713_stride4
frames: 4623 total, 1156 processed at stride 4
```

结果：

- 当前 MP4 解码为 `2560×1080`，历史 v5 检测 CSV 为 `1080×2560` 坐标；评估器已显式应用并记录 `portrait_to_landscape_ccw`，避免把方向错误算成检测失败；
- 严格当前圆环规则产生 18 个候选，标准目标 623 个，成功关联 3 个，关联率约 `0.48%`；
- 可视化显示少量候选存在“局部蓝色区域被拟合为圆”的情况，且 YOLO 框主要覆盖内层图案，外部蓝色圆环常在框外；因此当前问题首先是外圈检出/几何质量，不是简单增大 `target_refiner` 距离阈值；
- 已生成 `circle_detections.csv`、`association_long.csv`、`frame_summary.csv`、标注视频和 623 行 `ring_ground_truth_template.csv`，后者可由人工补填真实圆环中心后计算像素误差；
- 本次未提供同步 CameraInfo、camera→map 位姿和目标地图坐标，地图投影绝对误差为“不可测”，没有伪造结果。现有 `camera_param.yaml` 为 `1920×1080`，与该视频 `2560×1080` 也不匹配，不能直接用于误差验收。

### 旧完整视觉链真实视频推演（2026-07-13）

使用旧入口和旧资产在 `rl_drone` 环境中重新处理同一 `real_target.mp4`：

```text
入口：vision_ws/scripts/eval_legacy_full_vision_chain_video.py
模型：Visual/src/yolov5_detect/best.pt + Visual/src/yolov5_detect/tank.pt
圆环参数：vision_ws/migration_refs/patrol_control_visual/config/circle_detection_params.yaml
处理：4623 总帧，1156 帧，stride=4；rl_drone/OpenCV 解码为 1080×2560
```

旧链结果：

- 旧四类模型检测 760 个目标，旧 tank 独立模型触发 147 次 `image2center` 中心服务调用；
- 旧圆环节点产生 114 帧最大椭圆中心，中心输出仍遵循 640×512 resize 后的 `center×2` 投影输入；
- 旧 cross 几何检测 206 帧，旧 landing 几何检测 133 帧；
- 旧三帧类别累计输出 `Nothing=138`、`pillbox=107`、`bridge=32`、`panzer=97`、`tent=11`。这是旧 `/yolo_detect` 时序输出，不等同于逐帧框检测数量；
- 旧中心候选没有类别—圆环关联，抽查显示会把反光或局部蓝色区域作为最大椭圆，不能直接作为投递靶心；
- `legacy_projection_inputs.csv` 已记录旧硬编码内参得到的反投影射线，但没有同步 map TF/odom，因此地图点仍不可得。

旧链回放说明：旧工程并非“没有中心计算”，而是中心计算存在且能输出候选；问题在于它把全图最大蓝色椭圆直接当作中心，未关联标准类别、未验证外圈完整性，也无法在脱离 TF 时完成地图闭环。

### 不应误认为已完成的接口

- `/uav_vision/semantic_targets`：尚未实现；
- `/uav_vision/standard_target_instances`：尚未实现；
- `/uav_vision/release_permission`：尚未实现；
- 正式新接口模式：`patrol_control` 仍保留旧 `/yolo_detect` 和 `/detect/*` 订阅，compat bridge 仍存在。

## 4. 下一步工作排序

### P0-1：视觉回放和接口回归闭环（当前优先）

先用现有图像/rosbag 和纯 ROS mock 固化视觉链，完成：

1. 多圆环原图坐标与中心质量回归；
2. 类别框—圆环实例关联回归；
3. CameraInfo/TF 地面投影和 TF 缺失降级回归；
4. 地图 ID 去重、候选恢复和 reset 回归；
5. `drop_ready` 主点对准回归。

该阶段不启动 PX4/Gazebo、不要求真实释放，也不启动执行机构。

### P0-2：建立 H 与投递/巡检的阶段所有权

以现有 `desiredAlignMode()` 为起点，正式定义并测试：

- `SEARCH/APPROACH/VERIFY` 阶段只记录候选，不允许 H 触发降落；
- `DROP_ALIGN` 只允许 `drop_circle` 或 `drop_cross`，屏蔽 `landing_pad` 对准输出；
- `LAND_APPROACH/LAND_CONFIRM` 才允许 `landing`，并要求外圈、H 结构和连续帧稳定同时满足；
- H 丢失、验证失败或超时只能回退/悬停，不能把“超时”解释为 H 已确认。

### P0-3：真实回放阈值标定与旧链兼容（已启动，当前被数据契约和外圈检出卡住）

软件链已复用旧链中正确的 `CameraInfo + image_geometry + TF + 地面平面` 计算思路，并去掉硬编码内参、固定 `camera_link`、错误的缩放补偿和固定主点。下一步交付：

- 先修正/固定视频旋转方向和原图坐标契约；
- 从 `ring_ground_truth_template.csv` 选取起降、斜视、模糊、遮挡和边缘截断代表帧，人工补填圆环中心，计算原始框中心与圆环中心误差；
- 针对真实外圈重新评估蓝色掩膜连通性、椭圆/圆拟合、外圈完整度和框外扩展 ROI，再调整 `circle_detector`/`target_refiner`，不直接放宽关联距离；
- 在获得同帧 CameraInfo、camera→map 位姿和目标地图坐标后，再标定地图投影误差；
- 回归旧 `/detect/*` 输出不受新字段影响；
- 明确关联失败或地图投影不可信时禁止进入可靠投递中心；
- 再将结果交给 `patrol_control` 状态机联调。

### P1：正式模式、旧链复用收口、RKNN 和完整闭环

- 给 `patrol_control` 增加显式新/旧接口模式；
- 实现目标锁定、丢失、恢复和已投递回写；
- 将旧 `circle_detector`、`image2center` 的可复用几何/投影逻辑迁移为参数化新组件，保留旧 `/detect/*` 仅作回归；
- 导出六类 RKNN 并在 OrangePi 记录 perf；
- 完成 H 语义和规则释放许可；
- 最后进入静态机架和低风险飞行验收。

## 5. 暂不建议做的事

- 继续无指标扩大 v5 数据规模；
- 继续训练多个只在原始 val 上刷高分的模型；
- 在没有实例关联和地图投影可信度前调整释放阈值；
- 在没有外部仿真输入契约前编写地图投影；
- 在没有 RKNN 输入/输出一致性证据前修改板端主链；
- 恢复完整 `toudi3.world` 或处理 `fast_planner_node` 历史根因作为视觉组主线；
- 为正式比赛不存在的“红十字复用桥梁底座”继续增加 bridge 抑制策略。

## 2026-07-14 独立压力验证集与双模型评估

压力集由 v5 合并标准集的原始 val 232 张图生成，输出目录：

```text
vision_ws/test_data/yolo_stress_val_v5_20260714
```

每张源图生成 8 种条件，共 1856 张图；images/train 为空，不能用于训练。条件为运动模糊、旋转、局部裁切、目标变小、强光、暗光、遮挡、多目标同帧。该集合是可复现的合成压力验证集，不替代真实独立采集集。

| 条件 | 标准 mAP50-95 | 区域增强 mAP50-95 |
| --- | ---: | ---: |
| motion_blur | 0.977 | 0.971 |
| rotation | 0.242 | 0.234 |
| local_crop | 0.953 | 0.959 |
| small_target | 0.958 | 0.940 |
| strong_light | 0.979 | 0.976 |
| low_light | 0.981 | 0.973 |
| occlusion | 0.980 | 0.976 |
| multi_target | 0.950 | 0.938 |

完整结果位于：

```text
vision_ws/test_data/yolo_stress_eval_v5_merged_standard_20260714/summary.csv
vision_ws/test_data/yolo_stress_eval_v5_region_focus_aug_20260714/summary.csv
```

结论：区域聚焦训练没有在全部飞行退化条件上形成普遍收益；两者共同的明显薄弱项是旋转后的高 IoU 定位质量。当前不能仅凭压力集选择区域增强模型，也不能继续无指标扩大增强强度。

## 2026-07-14 旧控制流审计与视觉主线调整

旧机载工程应准确描述为“固定航点 + 机会式目标中断”：标准靶依赖预设 Detect_point，红十字/tank 可在巡航期间中断并执行近距离处理；它不是全场自主搜索。全场覆盖率、主动观察、候选接近、恢复搜索、搜完判据和剩余时间排序均未实现。

因此当前视觉组不把更换 LIO 或 Planner 作为首要任务。视觉组的下一工作顺序为：

1. 真实 CameraInfo/TF/odom 回放中的地图投影误差、稳定 ID 和跨视角去重；
2. 标准类别—蓝色圆环实例关联与可靠投递中心；
3. H 结构语义确认、拒绝原因和阶段门控；
4. `drop_ready` 与 `release_permission` 的接口分离及旧接口回归；
5. 两款 ONNX 的后处理一致性，随后再做 RKNN 转换和 OrangePi 性能验证。

主控/规划并行任务是新增 Search/Mission Manager、固定航点恢复、LIO→PX4 唯一位姿、局部规划接管和投递/降落超时安全策略。视觉记住地图点，不等于任务系统已经搜索或访问目标。
