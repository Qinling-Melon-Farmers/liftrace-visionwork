# 新旧视觉链工作逻辑对比（截至 2026-07-14）

> 文档定位（2026-07-15）：本文是新旧链事实对照和迁移依据，不维护当前任务优先级。当前架构、仿真 Gate 与下一步统一见 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。

## 1. 结论

旧 2025 链和当前 2026 链的本质差异，不是“用了不同模型”这么简单，而是整条视觉到主控的接线方式已经变了。

旧 2025 链是：

- 多条专用检测链并行
- 大量结果直接走旧 `/detect/*` 和 `/yolo_detect`
- 主控自己拼装类别、状态和对准逻辑
- 更适合“飞到预设检测点后再识别/再精修”

当前 2026 链是：

- 原始检测统一收敛到 `/uav_vision/detections`
- `detection_fusion.py` 先做同帧冲突裁决
- `target_memory.py` 负责候选确认、去重和 `selected_target`
- `drop_aligner.py` 负责像素偏差和 `drop_ready`
- `patrol_control` 已开始直接订阅新接口

截至本次更新，新链 dev/sim 的标准 detector 已切换为 v5 六分类飞行增强模型；该模型已完成离线回放和六类特征图，但不代表外部仿真、RKNN 或正式任务状态机已经完成。

一句话总结：

- 旧链偏“多节点直连主控”
- 新链偏“统一视觉总线 + 主控显式消费”

## 2. 旧 2025 实际运行链

本节只认当前仓库里真正能对应到旧运行入口的代码和 launch，不拿 `migration_refs` 代替真实历史入口。

### 2.1 启动入口

旧链主入口来自：

- `top_level_scripts/vision.sh`
- `Visual/src/yolov5_detect/launch/detect.launch`
- `patrol_uav_ws-patrol_planner/src/patrol_control/launch/circle_detection.launch`
- `patrol_uav_ws-patrol_planner/src/patrol_control/launch/cross_detection.launch`
- `patrol_uav_ws-patrol_planner/src/patrol_control/launch/landing_detector_node.launch`

`vision.sh` 当前能明确看到旧链会依次拉起：

1. `camera_sdk camera_video0.launch`
2. `patrol_control circle_detection.launch`
3. `patrol_control cross_detection.launch`
4. `Visual yolov5_detect detect.launch`
5. `patrol_control landing_detector_node.launch`

这说明旧链本来就是“多条检测链并行运行”，不是一个统一视觉总线。

### 2.2 旧标准目标分类链

代码入口：

- `Visual/src/yolov5_detect/scripts/yolo_detect.py`

工作方式：

1. 订阅 `/detect/class_control`
2. 订阅 `/camera/color/image_raw`
3. 用第一套标准目标模型做检测
4. 对 `bridge/panzer/pillbox/tent` 做 3 帧投票
5. 最终发布 `/yolo_detect`

关键语义：

- `/yolo_detect` 只承担标准目标字符串语义
- 当 3 帧内四类都没有命中时，发布 `Nothing`
- 在 3 帧投票没结束前，旧脚本会发空串
- 这条 4 类标准目标链本身不负责输出投递中心，真正用于标准靶末端对准的中心来自后面的圆环几何链

需要特别说明两点：

1. 从代码语义看，它是“4 类标准目标模型 + 单独 tank 模型”的 split detector。
2. 当前复制回来的旧 `detect.launch` 参数名虽然指向 `best_rknn_model` / `tank_rknn_model`，但本地离线“旧链风格证据”使用的是 `best.pt + tank.pt`，目的是复现这种 split detector 语义，而不是复刻 launch 文件里某一个具体权重路径。

### 2.3 旧 `tank` 专项链

同样在：

- `Visual/src/yolov5_detect/scripts/yolo_detect.py`

但它是独立支路：

1. 订阅 `/detect/tank_control`
2. 用第二套 `tank` 模型检测
3. 检测到 `tank` 后，把像素中心发给 `/visual/service`
4. `/visual/service` 由 `Visual/src/yolov5_detect/src/image_process.cpp` 提供
5. `image_process.cpp` 再把像素点通过 `camera_link -> map` 投影成 `/detect/tank_status`

旧链里 `tank` 被单列，主要是历史原因，不是今年任务逻辑上必须单列。

### 2.4 旧 `red_cross` 实际运行链

真实入口不是旧 `cross_detector_node`，而是：

- `patrol_control/launch/cross_detection.launch`
- 其中实际启动的是 `simple_cross_detect`

`simple_cross_detect.cpp` 当前可确认：

- 订阅 `/cross/control`
- 发布 `/detect/cross_status`
- 发布 `/detect/cross_mark_point`
- 用轮廓矩 `moments` 求红十字中心
- 直接使用硬编码相机模型与 `camera_link`
- 查 `map <- camera_link` TF
- 输出的是 `frame_id=map` 的世界系 `PoseStamped`

这就是为什么“旧 cross 实现”和“旧链实际运行行为”不能混为一谈。旧仓库里虽然还有 `cross_detector_node.cpp`，但真正启动的是 `simple_cross_detect`。

### 2.5 旧投递圆环链

入口：

- `patrol_control/launch/circle_detection.launch`
- `patrol_control/src/circle_detector_node.cpp`

当前代码可确认：

- 发布 `/detect/waypoint_mark_point`
- launch 同时拉起：
  - `base_link2map_tf`
  - `camera2base_link` 静态 TF
- 用 `fitEllipse` 的椭圆中心作为投递圆环中心
- 节点内部使用 `camera_link -> map` 投影
- 输出为世界系 `PoseStamped`
- 相机模型和相机 frame 假设高度耦合

旧圆环链不是单纯“图像域检测”，而是“图像检测 + TF 投影 + 直接给主控世界点”。

### 2.6 旧降落标识链

入口：

- `patrol_control/launch/landing_detector_node.launch`
- `patrol_control/src/landing_detector_node.cpp`

当前代码可确认：

- 订阅 `/detect/landing_control`
- 发布 `/detect/land_mark_point`
- 同样以拟合椭圆中心作为降落标识中心
- 使用 `camera_link -> map` 投影
- 输出为世界系 `PoseStamped`

它检测的是降落标识外圈与 `H` 相关区域，不是旧 YOLO 的一个类别。

### 2.7 旧 `patrol_control` 如何驱动这些检测链

`patrol_control.cpp` 当前能直接对应到旧链控制/消费接口：

订阅：

- `/detect/waypoint_mark_point`
- `/detect/cross_mark_point`
- `/detect/land_mark_point`
- `/detect/tank_status`
- `/detect/cross_status`
- `/yolo_detect`

发布：

- `/detect/class_control`
- `/detect/tank_control`
- `/cross/control`
- `/detect/landing_control`

旧逻辑的关键点：

- `ClassCallback` 用 `/yolo_detect` 与 `goal[]` 比较，控制 `align_ok`
- 圆环、十字、降落、tank 各自直接写回主控使用的目标点或状态
- 主控自己负责把“类别识别、几何点、任务阶段”拼到一起

### 2.8 旧链中“发现类别”和“计算中心”的职责划分

| 链路 | 是否计算中心 | 中心语义 | 默认输出 |
| --- | --- | --- | --- |
| 标准目标 4 类 `bridge/panzer/pillbox/tent` | 否 | 只做类别投票，不给投递中心 | `/yolo_detect` |
| `tank` | 是 | YOLO 框中心，再经服务投影到世界系 | `/detect/tank_status` |
| `red_cross` | 是 | 红十字轮廓质心，再投影到世界系 | `/detect/cross_mark_point` |
| 标准投递圆环 | 是 | 椭圆中心，再投影到世界系 | `/detect/waypoint_mark_point` |
| `landing_pad(H 外圈)` | 是 | 外圈椭圆中心，再投影到世界系 | `/detect/land_mark_point` |

这说明旧 2025 链里“识别出标准图案”和“算出投递/降落中心”本来就是分层的，不是同一个检测器一次完成。

## 3. 当前 2026 新链

### 3.1 启动入口

当前主入口来自：

- `vision_ws/src/uav_vision/launch/phase_d.launch`

当前这条链会启动：

1. `target_detector.py`
2. `cross_detector_node`
3. `circle_detector_node`
4. `landing_detector_node`
5. `detection_fusion.py`
6. `target_memory.py`
7. `drop_aligner.py`
8. `detect_compat_bridge.py`

板端入口则是：

- `vision_ws/src/uav_vision/launch/phase_d_board.launch`

其区别只是把标准目标 detector 换成 `target_detector_rknn.py`。

### 3.2 原始检测层

当前原始检测节点包括：

- `target_detector.py`
  - dev/sim 路径
  - 当前 dev/sim 默认使用统一六分类：`bridge/panzer/pillbox/tent/tank/red_cross`
- `target_detector_rknn.py`
  - board 路径
  - 当前已具备 unified / split assets 推理代码路径
- `cross_detector_node`
  - 输出 `class_name=red_cross`
- `circle_detector_node`
  - 输出 `class_name=circle`
- `landing_detector_node`
  - 输出 `class_name=landing_pad`

这些节点统一输出到：

- `/uav_vision/detections`

需要额外说明：

- 当前标准目标 detector 也会携带 `center_px`
- 但这个 `center_px` 本质上只是检测框中心，不是投递精对准中心
- 它默认也不是世界系目标点
- 标准靶真正用于末端投递的中心，仍主要来自后续 `circle_detector + drop_aligner`

### 3.3 融合与冲突裁决层

入口：

- `vision_ws/src/uav_vision/scripts/detection_fusion.py`

工作方式：

1. 按 `header.stamp` 聚合同一图像帧的多路 `TargetDetectionArray`
2. 对同帧结果做冲突裁决
3. 当前能力：
   - 支持按几何结果对 `bridge ↔ red_cross/landing_pad` 做同帧冲突裁决
4. 当前默认：
   - `suppress_bridge_on_red_cross=false`
   - `suppress_bridge_on_landing_pad=false`
   - 也就是默认不再主动 suppress `bridge`
5. 输出到：
   - `/uav_vision/detections_resolved`

需要注意：v3 的 `bridge/red_cross` 重叠来自红十字复用桥梁靶标底座，正式比赛不会复用该底座。当前回放保留原始观测、关闭 bridge 抑制；`target_memory.yaml` 与 `detect_compat_bridge.py` 中的相关开关只作为历史兼容配置，不再作为当前工作项或正式验收口径。

### 3.4 候选记忆层

入口：

- `vision_ws/src/uav_vision/scripts/target_memory.py`

当前功能：

1. 当前 Phase D 订阅 `/uav_vision/detections_mapped`（节点默认值可被 launch 参数覆盖）
2. 跨帧匹配同类目标
3. 管理状态：
   - `DETECTED`
   - `OBSERVING`
   - `CONFIRMED`
   - `REJECTED`
   - `EXPIRED`
4. 发布：
   - `/uav_vision/targets`
   - `/uav_vision/selected_target`

当前优先级口径：

- 正优先级：
  - `red_cross=10`
  - `panzer=2.5`
  - `bridge=2.0`
  - `pillbox=1.5`
  - `tent=1.0`
  - `tank=5.0`
- 当前为 `0`：
  - `landing_pad`
  - `circle`

这意味着当前真实行为是：

- `bridge` 能出现在 `detections` / `detections_resolved` / `/yolo_detect`，也能进入 `selected_target`
- `selected_target` 只是候选建议；旧主控当前不会因为标准目标的地图点自动生成接近/恢复搜索任务
- `circle` 和 `landing_pad` 也不会进入 `selected_target`
- 这两类由 `drop_aligner` 在特定 `align_mode` 下单独消费

### 3.5 模式化对准层

入口：

- `vision_ws/src/uav_vision/scripts/drop_aligner.py`

输入：

- `/uav_vision/targets`
- `/uav_vision/selected_target`
- `/uav_vision/align_mode`

输出：

- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`

当前模式映射：

- `drop_circle -> circle`
- `drop_cross -> red_cross`
- `landing -> landing_pad`
- `disabled -> 不输出有效对准许可`

### 3.6 旧接口兼容层

入口：

- `vision_ws/src/uav_vision/scripts/detect_compat_bridge.py`

当前作用：

- 把新接口兼容映射回旧接口
- 但不再默认伪造世界系 Pose

当前兼容语义：

- `/yolo_detect` 只允许：
  - `bridge/panzer/pillbox/tent/tank`
  - 无有效标准目标时发布 `Nothing`
- `red_cross/circle/landing_pad` 不进入 `/yolo_detect`
- `/detect/*_mark_point` 和 `/detect/tank_status` 的 Pose 兼容输出默认关闭

### 3.7 新链中“发现类别”和“计算中心”的职责划分

| 链路 | 是否计算中心 | 中心语义 | 默认输出 |
| --- | --- | --- | --- |
| dev/sim 默认统一六分类 `bridge/panzer/pillbox/tent/tank/red_cross` | 部分 | 检测框中心，仅作图像域候选信息 | `/uav_vision/detections` |
| `red_cross` | 是 | 红十字轮廓中心，图像域 | `/uav_vision/detections` |
| 标准投递圆环 `circle` | 是 | 椭圆中心，图像域 | `/uav_vision/detections` |
| `landing_pad(H 外圈)` | 是 | 外圈椭圆中心，图像域 | `/uav_vision/detections` |

和旧链相比，当前新链没有取消“中心计算”这件事，而是把职责重新拆开了：

- detector 负责产出图像域检测和中心
- `drop_aligner` 负责按 `align_mode` 生成 `drop_offset/drop_ready`
- `patrol_control` 再把偏差转成小步世界系对准目标

因此，新链当前并不是“发现目标后没有中心”，而是“不再默认由 detector 直接发布世界系目标点”。
同时需要补充一点：

- 当前 dev/sim 默认 detector 已经把 `red_cross` 纳入 unified detector
- 但高价值随机靶在当前完整链里仍然保留 `cross_detector` 作为几何验证与补充来源

## 4. 当前 `patrol_control` 如何消费新链

`patrol_control.cpp` 当前已直接订阅：

- `/uav_vision/selected_target`
- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`

并主动发布：

- `/uav_vision/align_mode`

当前已完成的行为：

1. `selected_target` 可更新标准目标 `goal`
2. 新鲜 `selected_target=red_cross` 可触发十字任务中断
3. 新鲜 `selected_target=tank` 可按开关触发坦克中断
4. `drop_offset` 已在主控内转换成小步世界系对准目标
5. `drop_ready` 已参与圆环/十字/降落末端条件判断
6. `desiredAlignMode()` 已把主控状态映射到：
   - `drop_circle`
   - `drop_cross`
   - `landing`
   - `disabled`

当前仍未完成的不是“主控完全没接新链”，而是：

- 更深层的搜索状态机仍主要沿用旧 waypoint 驱动
- 外部 `PX4 + Gazebo` 上的端到端接线还没有收口成稳定基线
- H 与投递/巡检的任务阶段所有权尚未完全收口；同帧出现多个几何候选时，必须由 `align_mode` 和控制状态共同决定谁能影响对准
- 地图坐标、长期 ID、类别—圆环实例关联和统一投递靶心仍未完成

## 5. 新旧接口对照

### 旧链主接口

- `/yolo_detect`
- `/detect/waypoint_mark_point`
- `/detect/cross_mark_point`
- `/detect/land_mark_point`
- `/detect/tank_status`
- `/detect/cross_status`
- `/detect/class_control`
- `/detect/tank_control`
- `/cross/control`
- `/detect/landing_control`

### 新链主接口

- `/uav_vision/detections`
- `/uav_vision/detections_resolved`
- `/uav_vision/targets`
- `/uav_vision/selected_target`
- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`
- `/uav_vision/align_mode`

最核心的变化是：

- 旧链大量依赖“旧世界点 Pose 直喂主控”
- 新链先保留图像域语义，再由主控按模式消费

## 6. 完成度矩阵

| 项目 | 当前状态 | 说明 |
| --- | --- | --- |
| 标准目标 `selected_target` 更新 | 已完成 | `patrol_control` 已可从新链更新 `goal` |
| `red_cross` interrupt | 已完成 | 新鲜 `selected_target=red_cross` 可触发十字任务 |
| `tank` interrupt | 已完成 | 已有单独中断入口 |
| `drop_offset` 小步世界系投影 | 部分完成 | `patrol_control` 有代码路径，但外部仿真和真实释放闭环未验证 |
| `drop_ready` 驱动圆环/降落末端条件 | 部分完成 | 已参与末端判定，但还不是规则级释放许可 |
| `/yolo_detect=Nothing` 清空旧类别状态 | 已完成 | 兼容桥已显式发布 |
| 新旧证据视频导出 | 已完成 | `unified5` vs `legacy_split` |
| 新视觉结果参与主控 | 部分完成 | 已有直连，但搜索仍主要 waypoint 驱动 |
| `bridge` 进入 `selected_target` | 已实现 | 当前配置 `priority_bridge=2.0`；不等于旧主控会自动接近 |
| 外部 `PX4+Gazebo` 新视觉连通 | 部分完成 | toudi3 GUI 新视觉入口已存在；缺独立真值、自动评分和 shadow 隔离 |
| OrangePi 5 Plus RKNN 真机验证 | 未完成 | WSL 仅验证了安全退化和 perf 接口 |
| H/投递/巡检阶段门控 | 部分完成 | `desiredAlignMode()` 已有基础门控，但降落任务类型、旧 flag 和超时回退仍未收口 |
| 地图记忆与投递靶心 | 部分完成 | 参数化投影、圆环关联和地图记忆已实现；误差、stable ID、新鲜度和实拍关联未验收 |
| `fast_planner_node` 稳定性恢复 | 非视觉范围 | 不作为视觉 L0/L1 评测基建的前置条件 |

## 7. 当前最重要的边界判断

1. 不应再把旧 `cross_detector_node.cpp` 当作旧链真实运行入口，真实入口是 `simple_cross_detect`。
2. 不应再把旧链描述成“统一检测器 + 若干辅助节点”，真实情况是 split detector + 多个旧 `/detect/*` 直连。
3. 不应再把新链描述成早期未落地状态，当前已经进入“主控更深层接线 + 外部仿真接入 + 板端验证”的阶段。
4. 不应再把 v3 的 bridge 重叠当作正式比赛冲突；当前工作重点是 H 与投递/巡检阶段门控、地图记忆和投递靶心计算。
