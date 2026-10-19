# 五分类模型更新与10月11日实飞回放

本轮候选：flight_5cls_new_video_v8_20261011。未替换板端默认模型，未做NPU实跑。

## 数据与训练

共2583图：train2313、val203、test67。在v7基础上追加新视频76张人工复核帧、152张尺度/光度派生图，以及195张类别平衡增强；不是把历次数据集重复拼接。训练实例：bridge306、panzer612、pillbox519、tent421、red_cross306。H训练负样本80张来自5个源图及75张增强；测试H6张，均非现场真实H专项独立采集。

在线增强：HSV、180度旋转、平移0.08、缩放0.25、水平/垂直翻转、mosaic0.3（最后5轮关闭）；不启用MixUp/CutMix。离线补充白平衡、亮度、对比度和缩小靶标增强。20轮训练约951秒，按验证集选择best；新403帧bag没有参与训练或选轮。

## 结果和边界

统一batch1、640方形输入重算，历史验证集mAP50-95约93.64%→95.16%。历史测试集仅含bridge/panzer/red_cross，不能代替五类独立实拍验收。历史来源跨split相关，指标不等于新场地泛化准确率。

旧805帧回放的11个人工复核ROI，旧模型正确2处、新模型10处；仍有裁切tent误报pillbox（0.801），0.75阈值也不能滤除。6张H测试图未出现误报，样本有限。

新flight_debug_0-1011-1444.bag完整201.675秒，915张记录图像中提取403帧。用户确认五类齐全、地堡仍为偏色打印件，本轮只飞投递。五个代表ROI新模型均正确：tent0.899、panzer0.937、pillbox0.931、bridge0.941、red_cross0.920；其中地堡旧模型报panzer0.861。仅为代表ROI检查，不是全片准确率。

三次对准到成功软件回执：bridge10.380秒、panzer10.852秒、red_cross11.544秒；对准到冻结分别3.197、3.031、2.455秒。软件回执不证明实物脱离或落点。冻结后约0.05秒下发降低Z指令，不能据此宣称新串行补偿已实飞通过。

## 本地交付

目录：/home/xhj/liftrace-deliverables/new_video_20261011/

- flight_5cls_new_video_v8_20261011_candidate.zip：最终PT/ONNX/FP16 RKNN、metadata、训练及评测摘要、中文替换回滚说明。
- final/TRAINING_REPORT.txt：完整训练报告。
- new_bag_1444/index.html：新bag回放入口。
- new_bag_1444/TIMING.md：投递时序。
- new_bag_1444_comparison/NEW_BAG_REPORT.txt：同帧模型对照。

RKNN为非INT8的FP16内部计算，输入float32不代表内部FP32。本地导出、预处理和解码一致性检查通过，工具15项检查通过；尚未验证RK3588 NPU吞吐、ROS连续运行或候选模型实飞。不自动推广。进入landing对准阶段YOLO暂停推理，起飞/搜索仍需真实H负样本覆盖。