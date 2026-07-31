# v5 六分类训练与特征图评测

> 日期：2026-07-14；2026-07-15 增补当前模型选择与仿真结果  
> 数据集：`vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713`  
> 模型：YOLO11n，640 输入，RTX 4060，Ultralytics 8.4.33

## 1. 训练产物

| 版本 | 训练数据 | 结果目录 | 训练记录 |
| --- | --- | --- | --- |
| baseline | v5 原始 train，923 张 | `vision_ws/runs/liftrace_6cls_v5_baseline_20260713` | 59 个 epoch 结果行 |
| flight_aug | v5 原始 train + 923 张增强 train；原始 val 232 张不变 | `vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713` | 100 个 epoch，best 为 epoch 80 |

增强样本按固定 seed 生成，分布为：geometry 198、motion blur 170、crop 200、occlusion 181、photometric 174。增强只作用于 train，避免验证集泄漏。

## 2. 原始 v5 val 结果

| 类别 | baseline P | baseline R | baseline mAP50 | baseline mAP50-95 | flight_aug P | flight_aug R | flight_aug mAP50 | flight_aug mAP50-95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| bridge | 1.000 | 0.988 | 0.995 | 0.990 | 0.998 | 1.000 | 0.995 | 0.991 |
| panzer | 0.996 | 1.000 | 0.995 | 0.995 | 0.996 | 1.000 | 0.995 | 0.995 |
| pillbox | 0.996 | 1.000 | 0.995 | 0.995 | 0.996 | 1.000 | 0.995 | 0.995 |
| tent | 0.996 | 1.000 | 0.995 | 0.992 | 1.000 | 1.000 | 0.995 | 0.995 |
| tank | 0.987 | 1.000 | 0.995 | 0.995 | 0.993 | 1.000 | 0.995 | 0.995 |
| red_cross | 0.975 | 1.000 | 0.995 | 0.896 | 1.000 | 1.000 | 0.995 | 0.909 |
| all | 0.992 | 0.998 | 0.995 | 0.977 | 0.997 | 1.000 | 0.995 | 0.980 |

当日选择 `flight_aug` 作为 dev/sim 候选：总体 mAP50-95 提升 0.003，`red_cross` 提升
0.013，其余类别没有下降或仅保持原水平。该结论是 2026-07-13/14 阶段记录；当前默认
模型已在后续 toudi3 逐类场景对照后切换为 `merged_standard`。

## 3. 真实视频回放

最终增强模型在 `real_target.mp4` 上生成：

```text
vision_ws/test_data/real_target_infer_v5_flight_aug_20260713/summary.json
frames_with_detection: 3747 / 4623
red_cross: 1721
bridge: 642
tank: 970
pillbox: 524
panzer: 311
tent: 6
```

在 `redcross.mp4` 上：

```text
vision_ws/test_data/redcross_infer_v5_flight_aug_20260713/summary.json
frames_with_detection: 1415 / 1443
red_cross: 1415
```

这证明增强版仍保持红十字视频上的纯 `red_cross` 输出，但真实比赛场景中的类别选择和几何链融合仍需继续回放验证。

## 3.1 最终增强模型完整视觉链回放

使用 `best.pt` 接入 `unified5`、当前红十字几何检测和当前降落外圈检测，离线运行时明确关闭 bridge 抑制：

| 视频 | 处理帧 | 任一路检测 | red_cross 几何帧 | landing_pad 帧 | 原始 bridge+red_cross 同帧 | 抑制帧 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `real_target.mp4` | 1156 | 996 | 515 | 359 | 160 | 0 |
| `redcross.mp4` | 1443 | 1415 | 1415 | 1016 | 0 | 0 |

回放输出：

```text
vision_ws/test_data/real_target_full_chain_v5_flight_aug_20260713_stride4/
vision_ws/test_data/redcross_full_chain_v5_flight_aug_20260713/
```

`real_target` 中存在同帧 `red_cross + landing_pad`，说明视觉输出应允许多路观测共存；H 是否能影响控制必须由 `align_mode=landing` 和降落阶段共同决定。v3 的 bridge 重叠属于数据构造历史，不再列为当前融合策略工作项。

## 4. 六类特征图

特征图脚本：`vision_ws/scripts/visualize_yolo_feature_maps.py`

输出目录：

```text
vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/feature_maps/
├── bridge/
├── panzer/
├── pillbox/
├── tent/
├── tank/
├── red_cross/
└── summary.json
```

每类目录包含：

- `*_layer22.jpg`：验证样本 backbone/FPN 激活叠加图和标注框；
- `mean_activation_layer22.jpg`：该类样本平均激活热力图；
- `channels_layer22.jpg`：代表样本的高响应通道网格。

这些是模型中间激活的诊断可视化，不等同于严格的 Grad-CAM 归因图；用于检查模型是否在目标附近形成稳定响应，以及比较增强前后的特征关注区域。

## 5. 当前默认模型

2026-07-14 当时 dev/sim 使用：

```text
vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/weights/best.pt
```

2026-07-15 笔记本 dev/sim 默认已切换到：

```text
vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt
```

这次切换只影响笔记本 PyTorch 仿真/回放。板端 `target_detector_rknn.py` 尚未同步导出或
验证统一六分类 RKNN，不能将该 PyTorch 权重视为 OrangePi 交付模型。

## 6. 2026-07-14 新模型与独立压力验证

在不改默认 detector 配置的前提下，完成两款新模型训练：

| 模型 | 权重 | 训练事实 | 干净 v5 val mAP50-95 |
| --- | --- | --- | ---: |
| merged_standard | vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt | epochs=100、patience=20；best epoch 78，实际在 98 epoch 停止 | 0.980 |
| region_focus_aug | vision_ws/runs/liftrace_6cls_v5_region_focus_aug_20260714/weights/best.pt | epochs=100、patience=20；best epoch 13，实际在 33 epoch 停止 | 0.979 |

压力集由原始 v5 val 的 232 张图生成 8 种变体，共 1856 张；训练目录为空。逐条件结果位于 yolo_stress_eval_*_20260714/summary.csv。

两款模型的压力集 mAP50-95（标准 / 区域增强）为：motion_blur 0.977/0.971，rotation 0.242/0.234，local_crop 0.953/0.959，small_target 0.958/0.940，strong_light 0.979/0.976，low_light 0.981/0.973，occlusion 0.980/0.976，multi_target 0.950/0.938。

压力集只用于发现薄弱条件，不作为训练集，也不单独决定默认模型。后续结合 toudi3
五类标准靶、红十字、H 和背景固定场景后，笔记本 dev/sim 已选择 merged_standard；
region_focus_aug 保留为对照候选。

## 7. 2026-07-14 ONNX 板端准备

两款候选均已导出固定输入 ONNX，暂不代表 RKNN 已验收：

| 模型 | ONNX | 输入/输出 | 状态 |
| --- | --- | --- | --- |
| merged_standard | `vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.onnx` | `1×3×640×640 → 1×10×8400` | ONNX checker + ONNX Runtime 通过 |
| region_focus_aug | `vision_ws/runs/liftrace_6cls_v5_region_focus_aug_20260714/weights/best.onnx` | `1×3×640×640 → 1×10×8400` | ONNX checker + ONNX Runtime 通过 |

导出参数为 opset 12、batch=1、dynamic=false、simplify=true、nms=false。输出保留六分类原始预测，后处理仍需在 RKNN 侧复现并与 PyTorch/ONNX 回放对齐。模型选择暂不作为主线堵点，两个 ONNX 都保留为板端转换候选。

## 8. 2026-07-15 当前验证补充

- merged_standard 在 toudi3 五类标准靶固定场景均能形成蓝圈关联后的 actionable 检测；
  逐类召回为 0.765-0.935，尚未全部达到正式 0.95 Gate；
- red_cross 使用 YOLO + 几何双确认后 precision=1.0、recall=0.646；H 结构阶段门控场景
  precision/recall=1.0；纯背景 0 FP；
- PT 与现有 ONNX 的 8 图逐框对照仍有 1 missing、1 extra，最大置信度差 0.536，未通过；
- 本节均为笔记本 WSL2/RTX 结果，未执行 RKNN 转换或 OrangePi 运行。

完整边界与下一步见
[VISION_LAPTOP_SIM_BASELINE_20260715.md](/home/xhj/liftrace/docs/VISION_LAPTOP_SIM_BASELINE_20260715.md)。
