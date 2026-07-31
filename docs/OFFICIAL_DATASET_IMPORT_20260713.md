# 比赛方官方靶标图片导入记录

日期：2026-07-13

## 1. 输入与类别语义

输入压缩包：`实物组数据集.zip`，共 60 张图片，每类 10 张：

```text
bridge / car / H / pillbox / tank / tent
```

当前六分类模型类别顺序保持不变：

```text
0 bridge
1 panzer
2 pillbox
3 tent
4 tank
5 red_cross
```

映射规则：

| 官方文件前缀 | 工程语义 | 处理方式 |
| --- | --- | --- |
| `bridge` | `bridge` | 模型框过滤为 `bridge` |
| `car` | `panzer` | `car → panzer`，但模型必须实际输出 `panzer` 才纳入 |
| `pillbox` | `pillbox` | 模型框过滤为 `pillbox` |
| `tank` | `tank` | 模型框过滤为 `tank` |
| `tent` | `tent` | 模型框过滤为 `tent` |
| `H` | 降落标志 | 不改成 `red_cross`，作为六分类 hard negative |

`H1.jpg` 的内容确认为黑色 H 降落标志，不是红十字目标。它可以作为后续
`landing_detector` 的独立数据来源，但不能污染六分类 `red_cross` 类。

## 2. 自动标注入口

脚本：

```text
vision_ws/scripts/import_official_6cls_dataset.py
```

默认使用：

```text
vision_ws/runs/liftrace_6cls_v5_flight_aug_20260713/weights/best.pt
```

脚本先复制现有 v5 标准数据集，再按官方图片的类别前缀生成唯一文件名，使用
六分类模型推理，并将模型框映射回原图 YOLO 坐标。模型没有输出期望类别的正样本
只进入 audit，不被伪造为空标签。

## 3. 当前产物

主扩展数据集：

```text
vision_ws/test_data/yolo_dataset_v5_6cls_redcross_official_20260713/
```

审计目录：

```text
vision_ws/test_data/official_dataset_audit_20260713/
```

结果：

| 项目 | 数量 |
| --- | ---: |
| 压缩包图片 | 60 |
| 纳入扩展集 | 38 |
| 自动标注框 | 28 |
| H hard negative | 10 |
| 漏检正样本 | 22 |

新增框分布：

```text
bridge=1
panzer=0
pillbox=10
tent=7
tank=10
red_cross=0
```

扩展集由原 v5 的 923 train / 232 val 变为 955 train / 238 val。`data.yaml`
和 `dataset_manifest.yaml` 已写入扩展集根目录，原始 v5 标准入口没有覆盖。

## 4. 质量结论

当前最终模型对官方整块圆环靶板图片存在明显域差：

- `pillbox/tank` 可较稳定检出；
- `bridge` 仅检出 1/10；
- `car → panzer` 检出 0/10；
- 模型会把部分 `car/bridge/tent` 误判为 `tank` 或 `pillbox`。

将阈值从 `0.25` 降到 `0.05` 没有恢复期望类别；对中心内圈做 `0.55` 比例裁剪
反而降低召回。因此当前 38 张是“自动标注候选扩展集”，不能把 60 张全部直接
用于训练。

下一步应人工复核 audit 中的 22 张漏检正样本，至少补齐 `bridge` 和 `panzer`；
同时可把 10 张 H 图片另行整理为 `landing_detector` 数据集。完成人工复核后，
再将确认框合并到正式训练版本。
