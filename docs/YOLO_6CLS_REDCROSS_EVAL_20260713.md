# `red_cross` 并入统一六分类后的离线回归（2026-07-13）

> 历史证据说明（2026-07-15）：本文保留 v3/v4 当日对照，不维护当前默认模型或下一步。
> 笔记本 dev/sim 当前默认已改为 `merged_standard`；红十字 operational 链要求 YOLO 与
> 几何双确认。最新固定仿真、实拍圆环和 PT/ONNX 结论见
> [VISION_LAPTOP_SIM_BASELINE_20260715.md](/home/xhj/liftrace/docs/VISION_LAPTOP_SIM_BASELINE_20260715.md)。
> OrangePi 尚未运行该统一六分类链。

## 1. 目的

本轮回归用于回答两个直接问题：

1. 将手工标注的 `red_cross` 数据并入统一 detector 后，单模型层面是否真的改善了红十字识别。
2. 在完整视觉链里，新的六分类权重是否能减少旧五分类把 `red_cross` 误报成 `bridge` 的问题。

本轮比较坚持两条原则：

- 旧五分类和新六分类使用同一套离线脚本回放。
- `detection_fusion` 按当前默认口径评测：
  - `suppress_bridge_on_red_cross=false`
  - `suppress_bridge_on_landing_pad=false`

## 2. 模型与输入

旧五分类基线：

- `vision_ws/runs/liftrace_5cls_v3_bridge_manual_20260703/weights/best.pt`

新六分类模型：

- `vision_ws/runs/liftrace_6cls_v4_redcross_manual_20260712/weights/best.pt`

输入视频：

- `vision_ws/test_data/real_target.mp4`
- `vision_ws/test_data/redcross.mp4`

## 3. 单模型视频推理结果

### 3.1 `real_target.mp4`

旧五分类：

- 输出目录：`vision_ws/test_data/real_target_infer_v3_recheck_20260713`
- `frames_with_detection=3435`
- `class_counts`
  - `panzer=1177`
  - `bridge=1021`
  - `tank=1251`
  - `pillbox=537`

新六分类：

- 输出目录：`vision_ws/test_data/real_target_infer_v4_6cls_20260713`
- `frames_with_detection=3726`
- `class_counts`
  - `red_cross=1781`
  - `bridge=695`
  - `tank=712`
  - `panzer=543`
  - `pillbox=662`
  - `tent=1`

直接结论：

- 新六分类已经能在真实目标视频里直接输出大量 `red_cross`。
- 与此同时，原先被旧五分类分摊到 `bridge/tank/panzer` 的一部分误检被重新吸收到 `red_cross` 类别。
- 单看 YOLO 层，`bridge` 检测数从 `1021` 降到 `695`，说明“红十字被当成桥梁”的问题确实被明显压下。

### 3.2 `redcross.mp4`

旧五分类：

- 输出目录：`vision_ws/test_data/redcross_infer_v3_5cls_20260713`
- `frames_with_detection=1175`
- `class_counts`
  - `bridge=716`
  - `tank=700`
  - `pillbox=61`
  - `panzer=40`

新六分类：

- 输出目录：`vision_ws/test_data/redcross_infer_v4_6cls_20260713`
- `frames_with_detection=1415`
- `class_counts`
  - `red_cross=1460`

直接结论：

- 旧五分类在纯红十字视频上几乎完全靠误报工作，主要误报成 `bridge` 和 `tank`。
- 新六分类在同一视频上基本收敛为单一 `red_cross` 类别，单模型收益非常明显。

## 4. 完整视觉链离线评测结果

说明：

- 统一使用当前 `cross_detector` / `landing_detector`。
- `real_target.mp4` 使用 `frame_stride=4`，便于和既有 15 FPS 证据链口径接近。
- `redcross.mp4` 使用 `frame_stride=1`。

### 4.1 `real_target.mp4` full-chain

旧五分类：

- 输出目录：`vision_ws/test_data/real_target_full_chain_v3_recheck_20260713_stride4`
- `processed_frames=1156`
- `raw_yolo_class_counts`
  - `bridge=255`
  - `tank=320`
  - `panzer=295`
  - `pillbox=134`
- `red_cross_frames=515`
- `landing_pad_frames=359`
- `raw_bridge_with_red_cross_frames=202`
- `raw_bridge_with_landing_pad_frames=87`

新六分类：

- 输出目录：`vision_ws/test_data/real_target_full_chain_v4_6cls_20260713_stride4`
- `processed_frames=1156`
- `raw_yolo_class_counts`
  - `red_cross=445`
  - `bridge=167`
  - `tank=176`
  - `panzer=139`
  - `pillbox=166`
- `red_cross_frames=515`
- `landing_pad_frames=359`
- `raw_bridge_with_red_cross_frames=161`
- `raw_bridge_with_landing_pad_frames=58`

直接结论：

- 传统几何链触发数量没有变：
  - `red_cross_frames` 仍是 `515`
  - `landing_pad_frames` 仍是 `359`
- 但 YOLO 层的桥梁误报冲突明显下降：
  - `raw_bridge_with_red_cross_frames: 202 -> 161`
  - `raw_bridge_with_landing_pad_frames: 87 -> 58`
- 这说明把 `red_cross` 纳入 unified detector 后，确实减轻了完整链中“标准 detector 把随机靶和相关场景压成 bridge”的问题。

### 4.2 `redcross.mp4` full-chain

旧五分类：

- 输出目录：`vision_ws/test_data/redcross_full_chain_v3_5cls_20260713`
- `processed_frames=1443`
- `raw_yolo_class_counts`
  - `bridge=716`
  - `tank=700`
  - `pillbox=61`
  - `panzer=40`
- `red_cross_frames=1415`
- `landing_pad_frames=1016`
- `raw_bridge_with_red_cross_frames=662`
- `raw_bridge_with_landing_pad_frames=530`

新六分类：

- 输出目录：`vision_ws/test_data/redcross_full_chain_v4_6cls_20260713`
- `processed_frames=1443`
- `raw_yolo_class_counts`
  - `red_cross=1460`
- `red_cross_frames=1415`
- `landing_pad_frames=1016`
- `raw_bridge_with_red_cross_frames=0`
- `raw_bridge_with_landing_pad_frames=0`

直接结论：

- 在纯红十字视频上，新的六分类模型已经把旧五分类的大部分 `bridge/tank` 误报清空。
- 完整链里的 `cross_detector` 触发数量没有下降，说明“加一个 YOLO `red_cross` 类别”没有压坏几何链。
- 这个结果足以证明：对随机靶而言，新六分类不是边际优化，而是链路层面的明显改善。

## 5. 是否切换 dev/sim 默认权重

本轮结论是：

- **可以切换 dev/sim 默认 `target_detector` 到新六分类权重。**

原因：

1. `redcross.mp4` 上收益非常明确，旧五分类的主要误报已经基本被消除。
2. `real_target.mp4` 上，完整视觉链中的 `bridge ↔ red_cross/H` 冲突显著下降。
3. 当前切换只影响 dev/sim 的 PyTorch 路径，不会误动板端 RKNN 主路径。

但仍需明确：

- 这不等于板端已经完成升级。
- `target_detector_rknn.py` 仍未切到六分类 RKNN。
- `H(landing_pad)` 的误判问题也没有因为六分类模型自动消失。

## 6. 本轮输出目录

单模型推理：

- `vision_ws/test_data/real_target_infer_v3_recheck_20260713`
- `vision_ws/test_data/real_target_infer_v4_6cls_20260713`
- `vision_ws/test_data/redcross_infer_v3_5cls_20260713`
- `vision_ws/test_data/redcross_infer_v4_6cls_20260713`

完整视觉链：

- `vision_ws/test_data/real_target_full_chain_v3_recheck_20260713_stride4`
- `vision_ws/test_data/real_target_full_chain_v4_6cls_20260713_stride4`
- `vision_ws/test_data/redcross_full_chain_v3_5cls_20260713`
- `vision_ws/test_data/redcross_full_chain_v4_6cls_20260713`

## 7. 下一步

1. 将 dev/sim 默认 `target_detector` 权重切到六分类模型。
2. 继续用这套 6 类权重回放 `real_target.mp4` 和其他真实视频，重点观察：
   - `H` 相关误判
   - `bridge` 剩余误报
   - `red_cross` 与几何链是否出现双重粘连
3. 后续再单独推进六分类 RKNN 导出与板端验证。
