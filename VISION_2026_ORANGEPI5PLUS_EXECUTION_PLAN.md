# OrangePi 5 Plus 视觉部署计划

更新时间：2026-07-16  
定位：仅维护 RKNN/NPU 部署 Gate；视觉业务优先级见 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。

## 1. 当前结论

- OrangePi 5 Plus 主推理路径固定为 RKNN/NPU；
- 已在 OrangePi 5 Plus 上完成两款六分类 FP32 RKNN 的真实视频和 v5merge 全集离线评测；
  另四款 INT8 已加载并完成同口径评测，但在 v5merge 全集上零有效检测；
- 板端离线脚本、带框视频、性能 JSON 和完整数据集评测报告已落盘，汇总见
  [板端模型完整评测报告](/home/xhj/liftrace/docs/BOARD_MODEL_COMPLETE_EVALUATION_20260716.md)；尚未接入 ROS 视觉链、
  `CameraInfo/TF` 和 10 分钟稳定性测试；
- 当前推荐 `merged_standard_fp32.rknn` 作为板端 A 候选，`region_focus_aug_fp32.rknn`
  作为 B 候选；不推荐旧 standard+tank 或任何当前 INT8 产物；
- 旧 standard+tank 已完成 v5merge 全集图片合并评测，P/R/F1/mAP 均为 0；旧双模型真实
  视频回放器已准备但因整机负载风险暂缓，不能把其视频评测写成已完成；
- 板端模型由 Toolkit 2.3.2 转换，运行库 `librknnrt 1.5.2` 仍产生版本警告，正式冻结前需
  统一运行时或完成风险验收；
- 香橙派死机按整机高负载、温度、进程并发或驱动稳定性问题处理，不归因于单个模型；
  重新上板前先做单模型短时资源监控。
- 板端不运行 Ultralytics/PyTorch 作为比赛实时主路径。

## 2. 冻结输入

当前保留两款 ONNX 候选用于数值对照，其中 `merged_standard` 是笔记本 dev/sim 默认：

```text
vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.onnx
vision_ws/runs/liftrace_6cls_v5_region_focus_aug_20260714/weights/best.onnx
```

区域增强模型不是所有压力条件下都更好。两款 FP32 RKNN 已完成板端推理，但不能把视频框
稳定性等同于规则准确率；INT8 需要重新检查输入契约/量化校准后再转换。

## 3. 运行架构

```text
camera (queue_size=1)
  -> RKNN 六分类粗检测，SEARCH 5-10 Hz
  -> CPU ROI 几何精修，APPROACH/DROP_ALIGN 15-20 Hz（按模式启用）
  -> fusion/refiner/map/memory
  -> release_evidence
```

部署原则：

- 搜索阶段不同时常开多套全图模型；
- debug image、bag 和详细日志默认关闭；
- 使用消息源时间戳计算端到端延迟；
- 队列只保留最新帧，禁止为追吞吐处理过期图像；
- 相机话题、内参、frame、模型、输入尺寸、量化参数和输出布局均配置化；
- RKNN 后处理的 resize/letterbox、颜色顺序、归一化、输出尺度、NMS 和类别顺序必须与 ONNX 对齐。

## 4. 部署前置 Gate

只有同时满足以下条件才开始板端主验收：

- [~] `VISION_MIGRATION_CHECKLIST` 的 M0-M4 已有最小实现，正式指标/接口联调未全通过；
- [ ] L1 Gazebo 真值场景达到首轮指标；
- [ ] L2 shadow 运行 10 min 无积压和崩溃；
- [ ] 实拍圆环/H/红十字回放有可审计真值；
- [ ] 候选 ONNX 与 PyTorch 输出在同一预处理下完成逐样本对照并通过（当前 8 图失败）；
- [ ] 板端相机 CameraInfo 和 TF 契约明确。

## 5. 板端任务

| ID | 任务 | 交付物 | 完成定义 |
| --- | --- | --- | --- |
| OPI-01 | ONNX 输出审计 | 输入/输出节点、shape、dtype、类别顺序说明 | 已完成首轮，六分类 `1×3×640×640 -> 1×10×8400` |
| OPI-02 | RKNN 转换 | 转换脚本、量化集 manifest、RKNN 文件、日志 | FP32/INT8 产物已生成，版本警告待收口 |
| OPI-03 | 数值一致性 | PyTorch/ONNX/RKNN 逐样本 CSV | FP32 已完成全集 P/R/mAP；逐框数值 Gate 未冻结 |
| OPI-04 | ROS 接入 | `phase_d_board.launch` 参数和 topic 记录 | 未完成，尚无板端 `/uav_vision/detections` 证据 |
| OPI-05 | 性能测试 | perf JSON/CSV、系统资源记录 | 离线视频已完成，ROS 端到端与资源曲线未完成 |
| OPI-06 | 10 min 稳定性 | rosbag 摘要、内存/温度/频率曲线 | 未完成 |
| OPI-07 | 实拍回归 | 与 dev/sim 同一评测报告格式 | v5merge 全集与 real-target 视频已完成首轮，压力真值仍缺 |

当前 OPI-01/02 已有首轮证据，OPI-03/05/07 为部分完成；OPI-04/06 仍未完成。仓库中的
RKNN 文件和窗口显示不能替代 ROS 接入、资源稳定性和逐条件真值验收。

## 6. 2026-07-16 首轮板端实测摘要

评测目录：`vision_ws/test_data/board_eval_20260716/`；v5merge 全集为 1395 张 train+val
图像，原始视频回放不去畸变，统一 `conf=0.25`、IoU=0.5。全集包含训练图，P/R/mAP 仅作
覆盖审计；泛化判断使用 val 232 张和压力集。

| 模型 | P | R | F1 | mAP50 | mAP50-95 | 单图总耗时 P50/P95 |
|---|---:|---:|---:|---:|---:|---:|
| merged FP32 RKNN | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 0.9859 | 57.9/106.9 ms |
| region-focus FP32 RKNN | 0.9979 | 0.9993 | 0.9986 | 0.9984 | 0.9744 | 63.1/99.6 ms |
| merged INT8（8图） | 0 | 0 | 0 | 0 | 0 | 49.6/73.9 ms |
| merged INT8（50图） | 0 | 0 | 0 | 0 | 0 | 45.8/56.4 ms |
| region INT8（8图） | 0 | 0 | 0 | 0 | 0 | 45.9/53.1 ms |
| region INT8（50图） | 0 | 0 | 0 | 0 | 0 | 47.2/54.6 ms |
| 旧 standard+tank | 0 | 0 | 0 | 0 | 0 | 123.2/163.4 ms（合并 P50/P95） |

结论：当前部署首选 `merged_standard_fp32.rknn`；INT8 只说明“能加载且更快”，没有通过
精度 Gate，禁止直接上机载主链。

## 7. 必报指标

- 模型和 RKNN toolkit/runtime 版本；
- 输入分辨率、量化方式、量化集 hash/manifest；
- 逐类 precision/recall 或与真值匹配的等价指标；
- PyTorch → ONNX → RKNN 的框、类别、置信度差异；
- detector 和完整视觉链 P50/P95 时延；
- 实际处理频率、图像年龄和丢帧率；
- CPU、NPU、RSS 内存、温度和频率；
- 10 min 内内存斜率、最大队列年龄和节点重启次数。

初始性能 Gate：搜索阶段 5-10 Hz、完整链 P95 `<= 200 ms`、图像队列不持续增长、10 min 节点崩溃 0 次。若板端能力不足，先降低输入分辨率/频率和按模式启停几何链，再评估模型调整，不回退到 PyTorch 主路径。

## 8. 明确不做

- 在系统 Python 中 `pip install` ML 包；
- 把 `.pt` 直接作为板端比赛主模型；
- 只报模型单帧耗时，不报采集到输出的端到端延迟；
- 用纯随机图片做量化集；
- 转换后不做逐样本数值对照；
- 为性能测试连接真实舵机或执行起飞；
- 因板端慢而删除观测新鲜度、安全门控或拒绝原因。

## 9. 验收结论模板

```text
模型/数据版本：
RKNN toolkit/runtime：
相机与输入尺寸：
搜索频率、P50/P95 时延：
CPU/NPU/RSS/温度：
10 min 稳定性：
与 ONNX 数值差异：
实拍逐条件回归：
通过/不通过的 Gate：
遗留问题与回滚模型：
```
