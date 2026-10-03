# 正式两轮中心、视频与长尾诊断

仅使用正式 seed31 `011753`、seed38 `013059`，source `4cb75a3b`。`002834` 准备轮、`003709` 诊断轮均排除。历史版本、模型、膨胀与 H 几何不同，不是严格 A/B。本页是离线诊断，没有修改飞行代码、运行参数或主 REPORT/index。

## 最终视频验证

| seed | 最终 overlay | 帧数 / fps | 时长 | ffprobe / 全片 decode |
|---|---|---:|---:|---|
| 31 | [视频](31_mapped_overlay.mp4) | 1964 / 10 | 196.4s | [PASS，exit0](31_mapped_overlay.validation.json) |
| 38 | [视频](38_mapped_overlay.mp4) | 2054 / 10 | 205.4s | [PASS，exit0](38_mapped_overlay.validation.json) |

两轮均单线程、nice 19。每个采用的原图都匹配到同图像时间戳 mapped 消息，最大差 **0ms**；空数组与无匹配严格区分，没有重跑模型。已目视核对类别混淆、同画面双靶、H 中心及视野边缘帧。

已修正输出 10Hz 网格未覆盖最后一张原图的边界问题，最终分别覆盖全部 **1622 / 1678** 张原图，末帧 ROS196.780 / 205.960，均晚于任务结束 ROS196.071 / 204.408。旧派生视频留在各自 `inspection/prior_endpoint_overlay`；原片未改动。

原始采样不是严格每 100ms：末 10s 最大间隙分别 **177 / 192ms**，超过 150ms 的间隙分别 12 / 9 个，没有超过 300ms 的末段间隙。视频显式标注重复持有原图；不能将完整解码 PASS 表述为相机从未漏采。最终 overlay 无额外末帧遗漏。

## 中心指标与阶段口径

| 指标 / m | seed31 | seed38 |
|---|---:|---:|
| red_cross 低空重获事件 | 0.11841 | 0.19311 |
| bridge 低空重获事件 | 0.13830 | 0.06409 |
| panzer 低空重获事件 | 0.06893 | 0.18181 |
| H mapped 中位数，source 时刻为 landing | 0.03829 | 0.03825 |
| H mapped P95，同上 | 0.04392 | 0.05525 |
| H mapped 数量，同上 | 133 | 95 |
| 最终 Gate H mark | 0.03185 | 0.03593 |

source 时间与接收时间门控不同：seed38 接收时刻 landing 的统计有 97 条，source 时刻 landing 有 95 条，差两条边界样本。主表使用 source 口径；原接收口径保留在 `center_summary.json`，不混用。

初始高空采用首次发布的 SURVEY 支持；中断采用首次出现的 `SURVEY_INTERRUPTED_TOP3.support` 冻结快照；低空采用 reacquisitions 事件。**不使用 final top_hints 充当高空值。** [31 分阶段](31_centers/stages/PHASE_REPORT.md)、[38 分阶段](38_centers/stages/PHASE_REPORT.md) 保留全部假设、source 时间、AGL、最近同类与最近任意实例距离。

## seed31：类别混淆独立列出

28 条 fresh mapped panzer 实际更接近 pillbox，全部位于 HIGH_SURVEY，source ROS21.349–26.196；距 pillbox 中位数 **0.13192m**，距真 panzer **4.64938m**。另有 1 条 bridge 靠近 pillbox。不能把这些同类距离称为纯定位误差。

冻结高空中断时也同时保留真假两个 panzer 假设：真 panzer 对应假设误差 0.34611m，低空重获后 0.06893m；错误假设距 pillbox 0.17653m。任务最终重访、释放对应真 panzer。fresh selected 中未发现该近邻类别混淆。

## seed38：PASS 不能掩盖定位长尾

### bridge

仅取“最近任意实例类别一致”子集，N=193，P95 **0.63032m**、最大 **0.73238m**。保留全部 fresh 有效 bridge 时，N=199，P95 **0.69415m**、最大 **0.82964m**：子集已经排除了 6 个更偏离的点，不能只报子集。

子集最高 5% 的 10 条全部在 DELIVERY。主要异常簇出现在第三次 panzer 投递期间，而 bridge ACK 已于 ROS95.749 完成。

[ROS106.480 同帧](38_centers/inspection/bridge_DELIVERY_106.480.png) 提供明确几何关联异常证据：bridge 图案在右上角且被裁切，记录的类别 ROI 为 `(1178,6,101,169)`，精修中心却在 `(996.218,336.243)`，位于 ROI 外灰背景处；`center_source=circle_geometry`、`association_valid=true`、`map_valid=true`，quality 0.74157，地图误差 0.73238m。同帧中央 panzer 中心误差约 0.092m，当时 selected 为 panzer id4。**这支持视野边缘残圈／圆关联异常，不是单纯类别错误或正常定位噪声。**没有重跑检测器，尚未定位到具体源码分支。

另有一条必须单独说明：ROS95.038、bridge ACK 前，低高度画面中 ROI 高度已占满 720px，记录中心 `(336.886,2.494)` 接近顶边，mapped 误差 **0.37703m**。它进入 targets 融合后误差 **0.06813m**，没有对应的新 selected 发布。因此不能说全部异常都发生在释放后，或完全没有进入记忆链。

### panzer

最近类别一致子集 N=255，P95 **0.32137m**、最大 **0.39966m**。最高 5% 共 13 条：HIGH_SURVEY 3 条、DESCEND 7 条、DELIVERY 3 条。不是均匀分布在全程。

- [ROS46.242](38_centers/inspection/panzer_HIGH_SURVEY_46.242.png)：panzer 正从画面下边缘进入，ROI 触底，中心 y=695.55px，误差 0.33592m。
- [ROS46.636](38_centers/inspection/panzer_DESCEND_46.636.png)：目标已完整入画、像素中心视觉上接近图案中心，但地图误差仍 **0.35653m**，FC AGL 约 2.586m。因此高空／下降初期偏差不能全部归因于裁切。投影、姿态和图像／位姿时序各自贡献尚未分离；现有证据不足以确定唯一根因。
- ROS109.555、110.052、110.154 三条 DELIVERY 异常：FC AGL 约 **0.483 / 0.491 / 0.530m**，类别 ROI 几乎占满图像；被接受的几何中心分别靠近右上角 `(1131.76,60.04)`、`(1177.63,58.82)`、`(1084.80,8.20)`，地图误差 **0.39966 / 0.38009 / 0.39424m**，仍标记关联有效。支持极近距离裁切／局部圆结构被当作靶心的解释。

这三条低高度异常均没有 <=30ms 的已录下视帧，故没有强行给它们画“同帧”叠加。[邻近原图 ROS109.591](38_centers/inspection/panzer_low_AGL_near_109.555_raw.png) 与首条相差 **36ms**，仅用于显示靶标已大幅占满画面，不作为精确像素对应。原视频 overlay 始终遵守 <=30ms。

三条异常进入 targets 后误差约 **0.12220 / 0.12337 / 0.12455m**；该区间没有新的 selected 发布。ACK 前最后一次有效 selected 发布于 ROS108.806，source108.599，误差0.12154m。融合削弱了异常，但不能据此宣称完全拒绝了异常几何，或证明对准输入毫无影响。

## LAND 是否仍有 YOLO 输出

按 source 时刻的 align_mode，两轮 landing 区间的 mapped 类别均只有 landing_pad，没有非 H 类别框。每轮恰有一次 `target_detector` 完成标记，均在录像末帧、detections 为空。完成标记不能证明该帧执行了模型推理，也不等于输出了 YOLO 框。轻量 bag 没有原始 detector 话题，因此结论限于：**记录的 mapped 链在 LAND 没有非 H 输出**；不能宣称 YOLO 进程或推理必然停止。

## 误分类是否进入释放

两轮各 3 个 `payload_committed` ACK，通过 decision_seq 对应到正确 target_id／class 的决策，依次 red_cross、bridge、panzer；决策位置的最近实例类别也一致。fresh selected 均未发现定义阈值下的近邻类别混淆。**未发现错误类别／错误实例的释放决策**。

该结论不等于所有中心几何均正确，也不是包裹落点精度证明。上述 bridge/panzer 异常已经进入 targets 融合；目标类别正确与地图中心长尾必须分别汇报。seed38 红十字重获中心与投递航点也不同，航点受边界处理，不能把 goal XY 当作原始中心。

原始核对数字：[31 inspection](31_centers/inspection/inspection.json)、[38 inspection](38_centers/inspection/inspection.json)。完整结果与供主 index 使用的相对链接：[CENTER_RESULTS.md](CENTER_RESULTS.md)、[center_results.json](center_results.json)。所有编码／解码已结束；未执行清理、提交或运行代码修改。
