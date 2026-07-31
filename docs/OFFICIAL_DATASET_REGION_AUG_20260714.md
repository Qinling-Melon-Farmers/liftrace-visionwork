# 官方旋转标注集与区域聚焦增强记录

## 结论

`vision_ws/test_data/officIal-JPG.yolov11.zip` 的 240 张原图已经直接合并进 v5 标准
训练集，而不是作为独立官方数据集：

```text
vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713/
```

合并记录在该目录的 `official_merge_manifest.yaml`。错误的独立目录
`yolo_dataset_v5_official_rotated_region_aug_20260714` 已清理。

## 输入审计

- 压缩包内有 `train/images`、`train/labels` 和 `data.yaml`，共 240 张图像和 240 个标签。
- 类别顺序为 `bridge=0`、`panzer=1`、`pillbox=2`、`tent=3`。
- 四类各 60 张；没有 `tank`、`red_cross` 样本，因此输出保持六分类 YAML，但不补造这两类标签。
- 所有旋转近重复图保留在训练侧；验证使用既有 v5 验证集，避免同一原图的不同角度泄漏到验证集。

## 区域聚焦增强

脚本：

```text
vision_ws/scripts/augment_yolo_region_focus_dataset.py
```

实际生成结果：

| 类型 | 数量 | 处理范围 |
| --- | ---: | --- |
| 合并 v5 原始训练图 | 1163 | 保留原框；其中 1 张为空标签 hard negative |
| `region_blur` | 1162 | 每个有标注样本的标注框内及很小羽化边缘做高斯模糊 |
| `region_erase` | 1162 | 每个有标注样本的标注框内做部分遮挡，保留目标语义 |
| `region_crop` | 1162 | 围绕所有标注框取局部并缩放回原分辨率，同时变换标签 |

运行命令：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
cd /home/xhj/liftrace
python vision_ws/scripts/augment_yolo_region_focus_dataset.py \
  --input-dataset vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713 \
  --output-dataset vision_ws/test_data/yolo_dataset_v5_region_focus_aug_20260714 \
  --variants-per-image 3 \
  --overwrite
```

输出 `data.yaml` 使用六分类顺序：

```text
bridge, panzer, pillbox, tent, tank, red_cross
```

`dataset_manifest.yaml` 记录了源 v5、生成数量、空标签数量和增强策略。训练前建议
重点检查 `region_erase` 是否符合手工标注人员希望模拟的遮挡程度，并用真实视频验证
模糊/裁切增强是否改善起降阶段的目标召回。
