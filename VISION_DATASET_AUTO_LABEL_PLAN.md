# 视觉数据自动标注与模型路线方案（历史基线与当前 v5 入口）

> 文档定位（2026-07-15）：本文保留数据/模型演进和标注方法，不维护当前业务优先级。新数据或训练必须由 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md) 中固定仿真或实拍失败证据驱动。

## 1. 当前结论

旧五分类自动标注路线已经收口，当前六分类标准数据路线已建立：

- 旧主模型为统一 5 类：
  - `bridge`
  - `panzer`
  - `pillbox`
  - `tent`
  - `tank`
- `red_cross` 已并入 v5 六分类标准数据集和当前 dev/sim v4 权重
- `landing_pad(H 外圈)` 暂不并入当前 YOLO 主模型
- 旧五分类数据与模型基线冻结在 `v3`
- 当前训练入口为 `yolo_dataset_v5_6cls_redcross_standard_20260713`
- 当前不做无指标扩充；正在规划面向飞行对地场景的受控增强消融

这份文档保留 v1-v3 自动标注基线、数据来源和人工复核原则；v5 的目录契约与飞行增强以 [v5 增强训练计划](docs/VISION_DATASET_V5_AUGMENTATION_PLAN_20260713.md) 为当前执行口径。

## 2. 什么叫“教师模型标注”

这里的“教师模型标注”指：

- 先用已有检测器给原图自动生成伪标签
- 再对伪标签做人工抽检或修正
- 最后把修正后的标签用于训练当前主模型

当前实际采用的教师不是多模态大模型，而是旧检测资产：

- 标准目标教师：旧 `best.pt` 4 类
- `tank` 教师：旧 `tank.pt`

因此当前自动标注基线本质上是：

- 旧 split detector 作为弱教师
- 统一映射到 5 类主数据集

如果后续要引入外部视觉大模型或多模态模型做辅助，它应被视为：

- 新的候选标注器或审核器
- 不是当前基线的一部分

## 3. 当前数据状态

### 3.1 原始静态图像

- `vision_ws/test_data/image/`
  - 当前已有 841 张原始图像

### 3.2 旧旋转坦克图

- `rotated`
  - 已按统一 5 类目标并入同一数据路线
  - 不再单独维护一条独立 `tank` 训练线

### 3.3 视频增量

已补入：

- `vision_ws/test_data/bridge.mp4`
- `vision_ws/test_data/tank.mp4`

并已抽帧进入：

- `vision_ws/test_data/video_sources/bridge_20260624`
- `vision_ws/test_data/video_sources/tank_20260624`

### 3.4 人工桥梁真值

- `vision_ws/test_data/bridge-manual`

这部分已经作为当前 `bridge` 类最重要的真值来源并入 `v3`。

## 4. 当前数据集基线

### 4.1 `v1`

- 数据集：`vision_ws/test_data/yolo_dataset`
- 用途：首版统一 5 类自动标注集

### 4.2 `v2`

- 数据集：`vision_ws/test_data/yolo_dataset_v2_video_20260624`
- 作用：纳入 `bridge.mp4` 与 `tank.mp4` 抽帧增量

### 4.3 `v3`

当前正式冻结基线：

- 数据集：`vision_ws/test_data/yolo_dataset_v3_bridge_manual_20260703`
- 训练结果：`vision_ws/runs/liftrace_5cls_v3_bridge_manual_20260703`
- 可分发压缩包：
  - `vision_ws/test_data/dist/liftrace_5cls_dataset_v3_bridge_manual_20260703_release.zip`

当前 `v3` 真值框统计：

- `bridge=72`
- `panzer=304`
- `pillbox=253`
- `tent=283`
- `tank=147`

### 4.4 `v4/v5` 六分类路线

- v4：`vision_ws/test_data/yolo_dataset_v4_6cls_redcross_manual_20260712`，保留原始合并过程和当前 v4 训练追溯。
- v5：`vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713`，后续训练唯一入口，去除临时导入目录、cache 和无效 test 声明。
- 当前 baseline：`vision_ws/runs/liftrace_6cls_v5_baseline_20260713`；最终 dev/sim 候选：`vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713`。
- v5 已完成 baseline 和飞行增强训练；后续实验仍使用 `liftrace_6cls_v5_*` 独立命名。

## 5. 当前自动标注路线

### 5.1 主路线

当前 v5 主路线固定为：

1. 直接对原图做自动标注
2. 使用旧 split detector 生成伪标签
3. 将旧 v3 五分类资产与人工复核的红十字样本合并为 v5 六分类数据集
4. 对关键问题样本做人审修正
5. 先复现 v5 baseline，再逐组加入飞行对地增强并保留独立实验产物

### 5.2 为什么现在不做 ROI 预裁剪

当前不做 ROI 预裁剪，原因已经明确：

- 真实识别场景里外环、本体、背景关系都存在
- 如果先裁掉外环，模型会丢失完整目标上下文
- 当前实拍问题边界也不是“标准目标必须先裁 ROI 才能识别”，而是：
  - 旧五分类 YOLO 曾在 v3 历史构造样本中把部分 `H/red_cross` 场景误报成 `bridge`；该现象与红十字复用桥梁靶标底座有关

因此 ROI 当前只保留为：

- 难例复核工具
- 几何精修工具
- 冲突抑制可能用到的局部辅助信息

而不是主数据流程。

## 6. 当前模型路线

### 6.1 标准目标

当前统一 YOLO 模型负责：

- `bridge`
- `panzer`
- `pillbox`
- `tent`
- `tank`
- `red_cross`

### 6.2 `red_cross`

当前同时走统一 YOLO 粗发现和传统视觉复核：

- 颜色阈值
- 轮廓/形状验证
- 几何精修
- 黑色有效区验证仍由传统几何链负责

### 6.3 `landing_pad(H 外圈)`

当前也走传统视觉路线：

- 灰度/阈值/轮廓/椭圆拟合

这三者当前不应混成“一个更大的训练数据集再重训一次”的问题。

## 7. 与实拍评测的关系

`real_target.mp4` 当前已有新旧六/五分类对照输出：

- 五分类复核：
  - `vision_ws/test_data/real_target_infer_v3_recheck_20260713`
- 六分类 YOLO 单独推理：
  - `vision_ws/test_data/real_target_infer_v4_6cls_20260713`
- 六分类完整视觉链评测：
  - `vision_ws/test_data/real_target_full_chain_v4_6cls_20260713_stride4`

这轮评测说明：

- `red_cross` 与 `landing_pad` 并不是“完全识别不到”
- 六分类已明显降低历史样本中的红十字被识别为 `bridge` 的问题；当前重点转为 `H` 语义、黑色有效区、阶段门控和投递靶心，bridge 三层开关不再作为当前工作项。

## 8. 当前数据工作的边界

- 不进行没有指标和场景依据的无界扩充；
- 不把增强样本混入原始验证集；
- 不把黑色有效区 ROI 误当成 `red_cross` YOLO 标注框；
- 不直接覆盖 v3、v4 和当前 v4 权重；
- v5 增强只按旋转、透视、模糊、局部裁切/遮挡、光照等组别做消融。

## 9. 当前执行门禁

1. v5 原始 baseline 可复现；
2. train/val 按视频或场景隔离，无相邻帧泄漏；
3. 红十字框语义、类别 ID 和图像/标签配对通过抽检；
4. 各增强组在原始 val、飞行退化 challenge 集和真实视频回放上分别评测；
5. 只有同时改善真实退化集和端到端回放，增强方案才进入默认训练配置。
