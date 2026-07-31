# OrangePi / 本机模型完整评测汇总（2026-07-16）

## 1. 结论摘要

本报告汇总当前 v5merge 六分类数据集上的本机 PT、板端六分类 RKNN，以及旧
standard+tank 双模型 RKNN 的图片评测和真实视频带框结果。

当前结论如下：

1. `merged FP32` 是目前唯一同时满足六分类语义、板端有效检出和稳定视频输出的主候选。
   `region-focus FP32` 结果接近，可保留作 A/B 备选。
2. 四个 INT8 产物的 NPU 时延较低，但在本次图片全集和视频回放中均没有达到
   `conf=0.25` 的有效检测，不能部署；这属于模型转换/量化输出契约仍未打通，不能用
   本次结果简单证明“INT8 模型本身不可用”。
3. 旧 standard+tank 双 RKNN 在 v5merge 全集的正式阈值评测为全零，且旧 standard
   不包含 `red_cross`，不适合作为 2026 主模型。
4. 香橙派此前出现的是整机高负载/死机风险。本报告不把死机归因于某一款模型；后续
   应先做资源、温度、进程并发和 USB 相机稳定性隔离，再进行长时板端压力测试。
5. 旧双模型视频回放器已准备，但按用户要求本轮暂缓执行，因此报告不伪造旧 RKNN
   视频文件或视频时延结果。旧双模型已有完整图片评测结果。

## 2. 评测范围和统一口径

数据集为：
`vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713`。

- 六类顺序固定为 `bridge, panzer, pillbox, tent, tank, red_cross`。
- 共 1395 张图片，训练/验证划分为 1163/232，标注目标 1394 个。
- 板端图片评测使用完整 train+val 集合，`conf=0.25`、IoU=0.5。
- 这套全集包含训练图，只能作为覆盖/适配审计，不能替代独立验证集泛化结论。
- PT 的独立验证结果使用 232 张 `val`；PT 全集结果只作为同口径覆盖审计。
- 原始 MP4 回放不去畸变；仅板端实时相机窗口使用固定相机标定去畸变。
- 板端视频为每 12 帧抽 1 帧的带框回放，输出约 5 FPS、1280×540，不能当成
  60 FPS 实时处理结果。JSON 中保留实际 NPU 推理和总处理时延。

指标含义：P=Precision，R=Recall，F1 为 P/R 调和平均，M50=`mAP@0.5`，
M50-95=`mAP@[0.5:0.95]`。

## 3. 模型与资产清单

| 组合 | 类别 | 输入/运行位置 | 状态 |
|---|---|---|---|
| merged PT | 六分类 | 本机 RTX / `.pt` | 已评测 |
| region-focus PT | 六分类 | 本机 RTX / `.pt` | 已评测 |
| merged FP32 RKNN | 六分类 | OrangePi NPU | 已评测，有效 |
| region-focus FP32 RKNN | 六分类 | OrangePi NPU | 已评测，有效 |
| merged INT8 8img | 六分类 | OrangePi NPU | 已转换，输出全零 |
| merged INT8 50img | 六分类 | OrangePi NPU | 已转换，输出全零 |
| region INT8 8img | 六分类 | OrangePi NPU | 已转换，输出全零 |
| region INT8 50img | 六分类 | OrangePi NPU | 已转换，输出全零 |
| 旧 standard RKNN | bridge/panzer/pillbox/tent | OrangePi NPU | 已评测，不含红十字 |
| 旧 tank RKNN | tank | OrangePi NPU | 已评测，单类补充 |
| 旧 standard+tank | 上述两者合并为六分类坐标 | OrangePi NPU | 已完成图片合并评测；视频暂缓 |

模型文件和原始转换日志保留在板端评测目录；本机报告只引用评测产物，不把板端
临时模型目录纳入工程源代码。

## 4. v5merge 全集图片结果

### 4.1 本机 PT 与板端 RKNN 总表

| 模型 | P | R | F1 | M50 | M50-95 | 处理 P50/P95 |
|---|---:|---:|---:|---:|---:|---:|
| merged PT（全集审计） | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 0.98547 | 13.19/25.08 ms |
| region PT（全集审计） | 0.9979 | 1.0000 | 0.9989 | 1.0000 | 0.97584 | 13.17/23.55 ms |
| merged FP32 RKNN | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 0.98587 | 57.92/106.86 ms |
| region FP32 RKNN | 0.9979 | 0.9993 | 0.9986 | 0.99835 | 0.97438 | 63.14/99.60 ms |
| merged INT8 8img | 0 | 0 | 0 | 0 | 0 | 49.57/73.92 ms |
| merged INT8 50img | 0 | 0 | 0 | 0 | 0 | 45.80/56.39 ms |
| region INT8 8img | 0 | 0 | 0 | 0 | 0 | 45.85/53.11 ms |
| region INT8 50img | 0 | 0 | 0 | 0 | 0 | 47.22/54.61 ms |
| 旧 standard+tank RKNN | 0 | 0 | 0 | 0 | 0 | 123.18/163.40 ms（合并两次推理） |

全集审计的旧双模型统计为：TP=0、FP=0、FN=1394；旧 standard 和旧 tank 分开
运行也均未形成有效框。其 raw 输出最高分约为 `10^-3` 量级，低于正式 `conf=0.25`
门限；不能把降低阈值后的噪声框当成识别成功。

### 4.2 PT 独立 val 结果

| 模型 | P | R | M50 | M50-95 |
|---|---:|---:|---:|---:|
| merged PT，232 张 val | 0.99754 | 1.0000 | 0.9950 | 0.98037 |
| region PT，232 张 val | 0.98969 | 0.99865 | 0.9950 | 0.97904 |

这张表比全集审计更适合作为当前桌面泛化参考。两款 PT 的 red_cross 类
`mAP50-95` 分别为 0.91148 和 0.91088，仍应结合实拍中的小目标、运动模糊和
遮挡继续观察。

### 4.3 merged FP32 逐类结果

| 类别 | GT | TP | FP | FN | M50-95 |
|---|---:|---:|---:|---:|---:|
| bridge | 132 | 132 | 0 | 0 | 0.99192 |
| panzer | 364 | 364 | 0 | 0 | 0.99792 |
| pillbox | 313 | 313 | 0 | 0 | 0.99896 |
| tent | 343 | 343 | 0 | 0 | 0.99649 |
| tank | 147 | 147 | 0 | 0 | 0.99703 |
| red_cross | 95 | 95 | 0 | 0 | 0.93293 |

完整逐类字段、预测框和评测参数见：

- [merged FP32 全集报告](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/v5merge_reports/merged_fp32.json)
- [region FP32 全集报告](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/v5merge_reports/region_fp32.json)
- [旧 standard+tank 全集合并报告](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/v5merge_reports/legacy_standard_tank_combined.json)
- [PT 独立 val 报告](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/local_pt_v5merge_val_metrics.json)
- [PT 全集审计报告](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/local_pt_v5merge_full_metrics.json)

## 5. 真实视频带框资产与性能

### 5.1 板端六分类 RKNN 视频

输入是同一个 `real_target.mp4`，板端禁用 MP4 旋转元数据自动旋转，保持原始
2560×1080 像素坐标；输出是采样带框视频。

| 模型 | 总 P50/P95 | NPU infer P50/P95 | 由 P50 推算 FPS | 中位框数 | 结果 |
|---|---:|---:|---:|---:|---|
| merged FP32 | 63.91/69.24 ms | 56.04/60.68 ms | 15.65 | 1 | 有效框 |
| region FP32 | 61.95/68.46 ms | 53.98/60.29 ms | 16.14 | 1 | 有效框 |
| merged INT8 8img | 48.10/52.66 ms | 40.73/44.57 ms | 20.79 | 0 | 无有效框 |
| merged INT8 50img | 49.43/53.61 ms | 41.95/45.81 ms | 20.23 | 0 | 无有效框 |
| region INT8 8img | 48.97/54.03 ms | 41.48/46.16 ms | 20.42 | 0 | 无有效框 |
| region INT8 50img | 50.48/56.66 ms | 43.12/48.39 ms | 19.81 | 0 | 无有效框 |

视频链接：

- [merged FP32](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/videos_raw_merged_fp32.mp4)
- [region FP32](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/videos_raw_region_fp32.mp4)
- [merged INT8 8img](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/videos_raw_merged_int8_8img.mp4)
- [merged INT8 50img](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/videos_raw_merged_int8_50img.mp4)
- [region INT8 8img](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/region_int8_8img.mp4)
- [region INT8 50img](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/region_int8_50img.mp4)

对应的原始性能 JSON 为：

[board_eval_20260716 视频 JSON 目录](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/)

### 5.2 本机 PT 视频

PT 视频在同一段原始视频上由本机 RTX 运行，检测阈值为 `conf=0.5`。它们用于
观察框的时间稳定性和模型行为，不与板端 P50 直接横比。

| 模型 | 总帧 | 有框帧 | 有框比例 | 类别框计数 |
|---|---:|---:|---:|---|
| merged PT | 4623 | 3353 | 72.53% | bridge 767，panzer 677，pillbox 440，tank 489，red_cross 1113 |
| region PT | 4623 | 3512 | 75.97% | bridge 640，panzer 477，pillbox 606，tank 692，red_cross 1149 |

视频：

- [merged PT 带框视频](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/pt_videos/merged_pt/annotated.mp4)
- [region PT 带框视频](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/pt_videos/region_pt/annotated.mp4)
- [merged PT 视频摘要](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/pt_videos/merged_pt/summary.json)
- [region PT 视频摘要](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/pt_videos/region_pt/summary.json)

### 5.3 旧 RKNN 视频状态

旧双模型图片评测已经完成，但旧双模型视频本轮暂缓，原因是用户已确认板端
高负载死机属于整机负载问题，当前不宜继续在板端追加长时双模型回放。

为下次恢复保留了工具：

- [旧双模型回放器](/home/xhj/liftrace/top_level_scripts/board_legacy_rknn_dual_viewer.py)
- [旧双模型映射/三帧投票测试](/home/xhj/liftrace/vision_ws/scripts/test_legacy_rknn_dual_mapping.py)

该工具采用 standard 4 类 + tank 1 类独立推理并将 tank 偏移到全局 class id=4，
视频默认显示阈值为 0.001 以保留旧 RKNN 的诊断候选；正式性能仍以 0.25 阈值为准。
旧原始源码的 tank 输出还会通过 `/visual/service` 发出框中心，standard 则三帧
投票发布类别；工具已把这些语义写入 JSON 结构，但尚未产生实际旧视频 JSON/MP4。

## 6. 旧链路与新模型的解释

旧 `Visual/src/yolov5_detect/scripts/yolo_detect.py` 的实际业务是：

```text
standard 模型（4 类）→ 每帧计数 → 连续三帧后发布类别
tank 模型（1 类）→ 仅 tank_flag 开启时运行 → tank 框中心调用 /visual/service
```

它不是一个统一六分类模型，也没有 red_cross 语义。当前旧 RKNN 只有二进制，不能
确认其原始量化输入、后处理和 NMS 实现；本次图片评测使用统一的 640 letterbox、
RGB、float32/255 和可审计 NMS，并把两份输出合并。因此旧模型全零首先说明“当前
板端二进制+评测契约”不可用，不能单凭此结果断言当年 ROS PT 链在原环境中也全零。

新六分类 FP32 直接输出 6 类，避免旧双模型的两次推理、类别拼接和 tank 开关；这
也是它适合作为 2026 主部署入口的工程原因。

## 7. 板端负载、转换和安全边界

- 板端为 RK3588、8 核、约 15.6 GiB 内存；RKNN Toolkit2/compiler 与 rknnlite
  均为 2.3.2，但运行时仍有 compiler/runtime 警告，需后续统一版本。
- 本轮高负载死机按“整机资源/温度/并发/驱动稳定性问题”记录，不归因于某个模型。
- 全集评测和视频回放均为离线文件输入，没有启动 ROS、MAVROS、PX4、Gazebo、
  actuator_pwm，也没有执行解锁、起飞或投递。
- 原始视频不去畸变；相机实时窗口才使用 `camera_param.yaml` 的固定 K/D。
- INT8 当前不能部署，不是因为板端死机，而是因为其有效输出在当前统一评测中为零。
- 后续重新上板前，应先单模型、低并发、短窗口监测 CPU/NPU/内存/温度，再逐步增加
  视频时长；不应并行运行多模型、相机窗口和大数据集评测。

## 8. 最终建议和未完成项

当前部署排序：

1. `merged_standard_fp32.rknn`：六分类、全集与视频均有效，作为主候选。
2. `region_focus_fp32.rknn`：作为 A/B 备选，实拍视频框稳定性需人工复核。
3. 四个 INT8：暂不部署，先检查量化校准集、输入 dtype/布局、输出 scale/zero-point
   和后处理契约。
4. 旧 standard+tank：仅保留历史兼容和对照，不作为 2026 主链。

下一步回到视觉主线，而不是继续扩大板端压力：

- 使用真实视频补人工框/圆环中心/H/红十字负样本真值；
- 以有效 FP32 模型继续做圆环实例关联、靶心精修、CameraInfo/TF 地图投影误差；
- 让 target memory 的 stable ID、地图点新鲜度和拒绝原因可审计；
- 继续保持 `drop_ready` 与 `release_permission` 分离，接入旧控制前先完成视觉证据闭环；
- 旧 RKNN 视频只在板端负载基线稳定后再补做，不阻塞视觉算法开发。

## 9. 原始结果索引

- [板端全集评测目录](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/v5merge_reports/)
- [旧 standard 预测框](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/v5merge_reports/legacy_standard_predictions.json)
- [旧 tank 预测框](/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/v5merge_reports/legacy_tank_predictions.json)
- [板端评测脚本](/home/xhj/liftrace/top_level_scripts/board_rknn_dataset_eval.py)
- [板端单模型视频脚本](/home/xhj/liftrace/top_level_scripts/board_realtime_rknn_viewer.py)
- [板端模型与部署计划](/home/xhj/liftrace/VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md)
