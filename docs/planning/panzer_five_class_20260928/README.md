# 五分类实拍强化模型与回放验收

2026-09-28；研究候选，替换目标检测模型，导航阈值和投递许可保持原值。模型与数据大文件不进Git。

## 来源与训练

基线是2026-07-14训练的 `liftrace_6cls_v5_merged_standard_20260714/weights/best.pt`。这里的v5是数据集版本，网络实际为 **YOLO11n**。原数据为20260713六类合并标准集。本次保留 bridge、panzer、pillbox、tent、red_cross，移除147张含tank标注的原训练/验证图，避免把仍可见的坦克当无标注普通背景。红十字由旧ID5变为新ID4。

- 数据：`/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v6_5cls_flight_h_20260928`。
- 训练：`/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_flight_h_20260928`。
- 训练图1774张（含原图和派生增强）；原集验证203张。
- 实拍训练来源：9月27日20:23包66帧、19:41难例29帧。SIFT/RANSAC传播模板框并人工查看拼图，排除28张截断/不确定帧；不是所有帧都经过逐像素人工精标。
- 20:33包61帧整轮留作未参与梯度训练的评估，其中panzer18、bridge4、red_cross20个标注目标。按航次分割，未随机拆相邻帧；但仍是同场地同靶标，不等于跨环境盲测。
- H仅作为投递检测器负样本，独立H几何降落链未删除。训练包含标准H纹理和历史仿真视角；另6张H/背景验证图中只有2张有实质H内容，余下多为边缘/背景，样本不足以证明绝无误检。
- 增强：亮度、gamma、对比度、局部阴影，以及训练期旋转、翻转、缩放/平移和少量mosaic。未把YOLO预测直接作为真值，也未把H设为第六类。
- 迁移时复制五个保留类的输出层权重，完整微调30轮；AdamW初始学习率0.0005，640输入、batch16，固定seed928。所有配置与结果在训练目录。

训练框架的增强参数以[Ultralytics官方文档](https://docs.ultralytics.com/guides/yolo-data-augmentation/)与本机已安装8.4.33的实现为准，没有升级框架。

## 同图对照的初步结论

统一置信度0.50、IoU≥0.50，20:33抽帧集：panzer旧7/18→新18/18；bridge旧3/4→新4/4；red_cross20/20→20/20。对应误检框：bridge1→0、red_cross3→0。原集五类验证在这一阈值均保持全部命中。详见评估JSON与最终报告；截断目标未全量标注，完整视频中的类别混淆仍应保留检查。

这些是检测器的框级结果，**不等于确认数、投递次数或全任务成功率**。旧模型保留独立路径，没有覆盖其权重。阈值0.60高位粗线索、低位确认与释放条件没有因分数上涨而继续放宽。

## 接入与产物

`export/`包含PT、ONNX、RK3588 FP16 RKNN及 `flight_5cls_20260928_metadata.yaml`。ONNX为[1,9,8400]，无objectness，xywh框；输入RGB float32/255、640×640，板端NHWC。RKNN Toolkit2 2.3.2采用非INT8构建。为兼容Toolkit，ONNX1.16.2和NumPy1.26.4仅放到任务的export_deps目录；没有替换rl_drone的训练依赖。

- PyTorch/ONNX数值与板端解码已对照；新解码拒绝“六类输出+五类元数据”，避免把bridge当作objectness导致整体类别错位。
- RKNN工具链CPU模拟器已检查一个panzer样例；**尚无香橙派NPU实时吞吐、温度或实飞验收**。
- 板端四组旧测试及四组扩展测试共享新模型入口、显式`--metadata`，保留静态TF、关闭虚拟顶棚、0.25水平膨胀、现场FAST-LIO/FreeDOM配置。
- 还原旧模型必须同时指定旧 `.rknn` 和 `merged_standard_6cls_metadata.yaml`，不能只换权重。
- 整机与纯视觉打包器现在要求显式匹配的metadata，不再偷偷附旧六类表。

## 复现

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
# 当前工作树tools/model_finetune内各脚本均支持--help；输入路径显式给出。
python tools/model_finetune/prepare.py --legacy <旧数据目录> --annotations <annotations.json> --output <新目录>
python tools/model_finetune/train.py --baseline <旧best.pt> --data <新data.yaml> --output <训练目录> --epochs 30 --batch 16
python tools/model_finetune/evaluate.py --dataset <新数据目录> --baseline <旧best.pt> --candidate <新best.pt> --output <评估目录>
python tools/model_finetune/export.py --weights <新best.pt> --output <导出目录>
python tools/model_finetune/replay_compare.py --replay <现有bag_replay的replay目录> --baseline <旧best.pt> --candidate <新best.pt> --output <视频目录>
```

`replay_compare.py`复用原toolkit的bag图像解包及图像时间戳，按相同帧、相同阈值输出左右新旧模型1×视频。只复跑检测器，不伪造整机因新模型会产生的航线/投递结果。任务层仍需实际闭环验证。

当前去重问题及反例见[DEDUP.md](DEDUP.md)。六组专项排除两组走廊，录像和报告另由板端试飞工作树保存；全部结束后统一汇总。

## 9/28补充：旧集退化与统一高位策略

[逐类退化、数据增强覆盖和特判取消说明](VALIDATION_AND_AUGMENTATION.md)：0.50/0.60旧集全检出不代表没有退化，0.70桥梁15→14，五类AP50–95下降3.18个百分点。五类都有增强，但新增实拍及局部阴影不均衡。panzer高位精修特判已取消；此前模型六组飞行记录保持原版本说明。
