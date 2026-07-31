# `real_target.mp4` 完整视觉链离线评测（2026-07-06）

## 1. 目的

这轮评测不是再单独看一遍五分类 YOLO 带框视频，而是回答一个更实际的问题：

- 当真实视频里同时出现标准靶标、`H` 相关降落标识、`red_cross` 时，
- 现有“完整视觉链”能否把这些目标区分开，
- 以及五分类 YOLO 当前会在哪些场景把 `H/red_cross` 误判成 `bridge`。

## 2. 输入与模型

- 输入视频：
  - `vision_ws/test_data/real_target.mp4`
- 五分类模型：
  - `vision_ws/runs/liftrace_5cls_v3_bridge_manual_20260703/weights/best.pt`
- 传统视觉参数：
  - `vision_ws/src/uav_vision/config/cross_detector.yaml`
  - `vision_ws/src/uav_vision/config/landing_detector.yaml`

## 3. 评测链路

本次使用离线等效完整视觉链脚本：

- `vision_ws/scripts/eval_full_vision_chain_video.py`

链路包含：

1. 标准目标检测器
   - 当前链：五分类统一 YOLO
   - 旧链风格：旧 `4 类 best.pt + 单独 tank.pt`
2. `cross_detector` 等效逻辑
   - 输出：`red_cross`
3. `landing_detector` 等效逻辑
   - 输出：`landing_pad`
   - 注意：这里的 `landing_pad` 表示 `H` 相关降落标识外圈检测链，不是独立 YOLO 类别

## 4. 输出位置

- 历史全量输出目录：
  - `vision_ws/test_data/real_target_full_chain_v3_20260706`
- 本轮“15 FPS 等效对照”输出目录：
  - `/tmp/real_target_full_chain_current_stride4_20260706`
  - `/tmp/real_target_full_chain_legacy_stride4_20260706`
- 持久化双链标记视频证据：
  - `vision_ws/test_data/real_target_chain_evidence_20260706/current_chain_stride4/annotated_full_chain.mp4`
  - `vision_ws/test_data/real_target_chain_evidence_20260706/legacy_chain_stride4/annotated_full_chain.mp4`
- 证据目录说明：
  - `vision_ws/test_data/real_target_chain_evidence_20260706/README.md`
- 关键文件：
  - `annotated_full_chain.mp4`
  - `detections_long.csv`
  - `frame_summary.csv`
  - `summary.json`
  - `samples/`

## 5. 结果摘要

### 5.1 历史全量评测基线（frame_stride=1）

- `total_frames=4623`
- `processed_frames=4622`
- `frames_with_any_detection=3750`
- 分辨率：`1080x2560`
- 帧率：`59.985 FPS`
- 五分类 YOLO 检测计数：
  - `bridge=1021`
  - `panzer=1177`
  - `pillbox=537`
  - `tank=1251`
- 传统视觉链触发计数：
  - `red_cross_frames=1694`
  - `landing_pad_frames=1442`
- 冲突统计：
  - `bridge_with_red_cross_frames=756`
  - `bridge_with_landing_pad_frames=380`
  - `aux_only_frames=315`

### 5.2 本轮对照回归（统一约 15 FPS，frame_stride=4）

统一设置：

- `processed_frames=1156`
- `frame_stride=4`
- `output_fps=15`

#### 当前链（`detector=unified5`，`cross=current`，`landing=current`，带冲突抑制）

- 标准目标模型：`vision_ws/runs/liftrace_5cls_v3_bridge_manual_20260703/weights/best.pt`
- `YOLO raw bridge detections=255`

- `red_cross_frames=515`
- `landing_pad_frames=359`
- `raw_bridge_with_red_cross_frames=202`
- `raw_bridge_with_landing_pad_frames=87`
- `bridge_with_red_cross_frames=101`
- `bridge_with_landing_pad_frames=0`
- `suppressed_bridge_frames=112`
- `aux_only_frames=210`

对照实现前的同采样旧基线：

- `red_cross_frames: 419 -> 515`
- `landing_pad_frames: 359 -> 359`
- `bridge_with_red_cross_frames: 186 -> 101`
- `bridge_with_landing_pad_frames: 87 -> 0`

#### 旧链风格对照（`detector=legacy_split`，`cross=legacy_simple`，`landing=legacy_old`）

- 标准目标模型：`vision_ws/src/yolov5_detect/best.pt`
- 单独坦克模型：`vision_ws/src/yolov5_detect/tank.pt`
- `YOLO raw bridge detections=71`

- `red_cross_frames=206`
- `landing_pad_frames=133`
- `raw_bridge_with_red_cross_frames=49`
- `raw_bridge_with_landing_pad_frames=9`
- `bridge_with_red_cross_frames=16`
- `bridge_with_landing_pad_frames=0`
- `suppressed_bridge_frames=36`
- `aux_only_frames=146`

这个对照说明：旧 `simple_cross_detect` 的宽松判分思路值得吸收，但连同旧 `4+1` 检测器整体回退后，标准目标触发和 `red_cross/H` 覆盖率都会明显下降。

### 5.3 实时链 ROS 烟测

为确认“冲突抑制”不是只存在于离线脚本，本轮增加了一个最小 ROS 烟测：

- 输入：同一 `header.stamp` 下分别发布 `bridge` 和 `red_cross`
- 节点：`vision_ws/src/uav_vision/scripts/detection_fusion.py`
- 输出：`/uav_vision/detections_resolved`
- 结果：`resolved_classes=red_cross`

说明融合节点已经能在实时链里对同帧 `bridge` 做显式抑制。

### 5.4 人工复核视频证据

为方便直接人工审片，本轮额外导出了两份持久化标记视频：

- 当前链：
  - `vision_ws/test_data/real_target_chain_evidence_20260706/current_chain_stride4/annotated_full_chain.mp4`
- 旧链风格：
  - `vision_ws/test_data/real_target_chain_evidence_20260706/legacy_chain_stride4/annotated_full_chain.mp4`

二者都采用：

- `frame_stride=4`
- `output_fps=15`
- 相同输入视频

但标准目标检测器不同：

- 当前链：新五分类统一模型
- 旧链风格：旧 `4 类 best.pt + tank.pt`

画面说明：

- 绿色框：最终保留下来的标准目标 YOLO 检测
- 橙色框：被冲突裁决抑制掉的 `bridge`
- `SUP`：`suppressed`，表示该 `bridge` 候选被当前冲突裁决压掉
- 红色标记：`red_cross`
- 黄色标记：`landing_pad`
- 顶部第一行：模式信息
- 顶部第二行：当前帧检测摘要

当前 `SUP bridge` 仍然是：

- 同帧 frame-level 全局 suppress
- 不是 ROI / overlap 感知 suppress

## 6. 结论

结论已经比第一次全量评测更明确：

1. `red_cross` 和 `landing_pad(H 外圈)` 不是“检测不到”，而是此前缺少更合理的冲突处理。
2. 当前真正的问题是新五分类 YOLO 会把部分 `red_cross/H` 场景误报为 `bridge`，因此必须在视觉链内部做显式裁决。
3. 当前更合理的主链不是整体回退到旧实现，而是：
   - 保留新链图像域输出和质量分数
   - 吸收旧 `simple_cross_detect` 的宽松特征
   - 在融合层先抑制 `bridge ↔ red_cross/H` 冲突
4. `/yolo_detect` 不应继续承担 `red_cross/landing_pad` 语义；它只保留标准目标分类接口。
5. 当前导出的“旧链风格”视频已经不是“新五分类模型 + 旧 cross/landing”，而是更接近历史方案的“旧 4+1 检测器 + 旧 cross/landing”。

## 6.1 接口口径补充

当前评测对应的新链接口语义应统一理解为：

- `/uav_vision/detections`
  - detector 原始输出
- `/uav_vision/detections_resolved`
  - `detection_fusion.py` 裁决后的输出
- `/yolo_detect`
  - 仅保留标准目标兼容字符串接口
  - 不再承担 `red_cross/landing_pad` 语义

## 7. 当前建议

近期优先级建议调整为：

1. 当前主链继续沿用“新链 + 宽松特征 + 冲突裁决”，不要整体回退到旧链。
2. 下一步重点盯剩余 `bridge_with_red_cross_frames=101` 的残留误报样本，而不是盲目继续堆桥梁正样本。
3. `landing_pad` 当前冲突已压到 `0`，优先做人工抽检“小圈/残圈/不完整椭圆”样本，而不是重写 `landing_detector`。
4. 后续做板端或联调回放时，应以这份 15 FPS 对照结果和 `resolved_classes=red_cross` 的 ROS 烟测作为回归基线。
