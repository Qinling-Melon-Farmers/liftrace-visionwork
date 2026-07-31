# `vision_ws/test_data` 冗余清理计划

> 盘点日期：2026-07-13  
> 本文只形成清理边界；v5 flight_aug 和最新回放已生成，本轮仍未删除或移动任何既有目录。

> 2026-07-14 复核：本轮继续不删除 `test_data`。当前优先保留可追溯数据、回放证据和被脚本引用的历史目录；待引用迁移、摘要归档和 v5/RKNN 回归完成后，再按本文删除前检查逐组处理。

## 1. 当前目录分层

### 必须保留

- `yolo_dataset_v3_bridge_manual_20260703`：五分类手工 bridge 基线，仍被主文档、合并脚本和评测引用。
- `yolo_dataset_v5_6cls_redcross_standard_20260713`：新的六分类标准训练入口。
- `yolo_dataset_v5_flight_aug_20260713`：当前增强训练入口，保留至模型和板端回归完成。
- `yolo_dataset_v4_6cls_redcross_manual_20260712`：v5 的来源和 v4 模型训练可追溯记录；至少在 v5 复现和增强训练验收前保留。
- `video_sources/`、`redcross_frames_20260711/`、根目录视频和 `redcross.yolov11.zip`：原始数据资产，后续增强和重标可能需要。
- `real_target_chain_evidence_20260706/`、`real_target_full_chain_v3_recheck_20260713_stride4/`、`real_target_full_chain_v4_6cls_20260713_stride4/`：链路证据和新旧模型对照。
- `real_target_infer_v3_recheck_20260713/`、`real_target_infer_v4_6cls_20260713/`、`redcross_infer_v3_5cls_20260713/`、`redcross_infer_v4_6cls_20260713/`：六分类切换前后对照报告所引用的回放输出。
- `real_target_infer_v5_flight_aug_20260713/`、`redcross_infer_v5_flight_aug_20260713/`：当前最终增强模型回放输出。
- `real_target_full_chain_v5_flight_aug_20260713_stride4/`、`redcross_full_chain_v5_flight_aug_20260713/`：最终增强模型完整视觉链证据，包含 `summary.json`、帧摘要和 H/红十字/bridge 共存审计结果。
- `bridge-manual/`、`image/`、`rotated/`、`redcross/`：仍被自动标注或调试脚本直接引用，不能仅按目录体积删除。

### 可归档，但当前不直接删除

- `yolo_dataset/`：v1 训练来源，仍被 `train_5cls.py`、`auto_label.py` 和 v1 训练结果追溯引用。
- `yolo_dataset_v2_video_20260624/`：v2 视频自动标注数据，仍被 v2 训练脚本和自动标注脚本引用。
- `real_target_infer_v3_20260703/`、`real_target_full_chain_v3_20260706/`：已有 2026-07-13 recheck 版本，当前只被历史报告或自身 summary 追溯引用。
- `real_target_full_chain_v3_20260706/` 与旧 `real_target_infer_v3_20260703/`：待把文档引用迁移到 recheck 版本并确认不再用于复现实验后，移动到 `test_data/archive/20260713/` 或打包保存。
- v4 中的 `_import_redcross_tmp/`：v5 已验证根目录数据完整后可清理，但目前仍作为 v4 导入过程证据保留。

## 2. 当前重复关系

| 目录 | 关系 | 处理建议 |
| --- | --- | --- |
| v4 六分类数据集 / v5 标准数据集 | 内容来源相同，v5 去掉临时导入目录、cache、无效 test 声明并增加 manifest | v5 作为训练入口；v4 暂留追溯，不做双入口训练 |
| v3 旧回放 / v3 recheck 回放 | 同一五分类模型的不同回放批次，recheck 是较新证据 | 迁移旧文档引用后再归档旧批次 |
| v4 六分类回放 / v3 回放 | 不重复，分别是模型对照证据 | 两者均保留至模型评测报告冻结 |
| `yolo_dataset` / v2 / v3 | 不是简单重复，分别对应不同自动标注/手工修订阶段 | 保留历史入口，禁止按体积判断删除 |

## 3. 建议清理顺序

1. [x] 使用 v5 `data.yaml` 完成无增强 baseline 训练并保存结果。
2. [x] 完成 v5 图像/标签配对、类别 ID、增强分布和红十字框语义检查。
3. [x] 完成 v5 增强训练与六分类回放评测，追溯信息已写入训练评测报告。
4. [ ] 将历史回放报告引用统一标注为历史，再归档旧 v3 回放目录。
5. [ ] 最后处理 v4 的 `_import_redcross_tmp/`、`merge_summary.json` 以及确定不再使用的旧临时文件；删除前先列出绝对路径和目录大小。

## 4. 删除前硬性检查

- `rg` 全仓库搜索目标目录名，确认没有活动脚本、README、报告或训练参数引用；
- 对待删除目录保留 `summary.json`、`frame_summary.csv` 或压缩归档；
- 确认 v5 `images` 与 `labels` 数量一致，且每个标签对应图像；
- 确认当前默认 detector 和回放报告使用的模型路径不依赖待删除目录；
- 一次只处理一组目录，删除后立即执行相关脚本的路径检查；
- 不删除原始视频、原始压缩包和历史五分类基线。
