# 投递候选两轮评测（2026-10-07）

两轮 source manifest 均为 **66c36a9e**，控制修复 **1c700dd3**。当前结论：**不推广，候选维持默认关闭**。新版同时改变下降捕获控制和仿真相机标定，不能纯算法归因。

| seed | 正确类别 ACK / 恢复 | 源时刻插值平均误差 | 原始 Gate | 最终状态 |
|---|---|---|---|---|
| 31 | 3 / 3 | 6.775cm（历史6.592cm） | PASS | H接地、COMPLETE、已解除武装；0碰撞 |
| 38 | 3 / 3 | 4.095cm（历史7.920cm） | FAIL / actual_collision | H接地后7.739s软件异常，未证明解除武装；11次包络接触 |

## 报告与数据

- [两轮汇总](REPORT.md)
- [seed31报告及轻量误差分解](seed31/REPORT.md)
- [seed38报告及完整接触统计](seed38/REPORT.md)
- [两轮精简指标](two_seed_metrics.json)
- [seed31逐槽比较](seed31/comparison.csv) / [seed38逐槽比较](seed38/comparison.csv)

主指标为 ACK 消息源 stamp 插值的 Gazebo 身体/model-origin XY 到正确类别真值靶心距离，不是实物包裹落点。历史 receipt 最近样本口径另列；未投槽位不补零。物理接地、飞控落地状态、解除武装、软件结果与原始 Gate 分开记录。

## 视频

- [seed31三视图及指标对照](seed31/review_seed31_comparison.mp4)：1080p、10fps、2285帧、228.5s；[验证摘要](seed31/video_validation.json)。
- [seed38三视图及指标对照](seed38/review_seed38_comparison.mp4)：1080p、10fps、3808帧、380.8s；[验证摘要](seed38/video_validation.json)。

两部视频均完成全片解码、原视频/CSV帧数一致性核对和抽帧检查，历史视频复用。seed38第三槽及后半程主视角被墙面遮挡，接触/H触地结论依据接触、轨迹和事件日志；视频末帧早于ABORT日志75ms，对照页明确标注。视频及逐帧素材为本地附件，未随精简报告归档时，仓库中的视频相对链接不可直接播放。

## 数据来源

- 新seed31：`logs/drop_precision_new_seed31_20261007_135022`
- 新seed38：`logs/drop_precision_new_seed38_20261007_140702`
- batch索引：`logs/drop_precision_20261007_entryfix/matrix.json`
- 历史原snake3：`logs/snake3_camera2m_snake3_seed31_20261005_095902`、`logs/snake3_camera2m_snake3_seed38_20261005_101903`
- 旧分析口径：`docs/verification/drop_precision_20261006/precision_eval.py`

134730启动失败轮为INFRA，排除于飞行统计。后续评测工具修复不改变上述飞行manifest版本。完整指标、碰撞和相机证据边界见汇总及逐轮报告。

## 离线复现

以下命令在 H 工作树根目录的 WSL bash 中执行，仅处理闭包文件，不启动 ROS 节点。使用现有环境；输出选择新路径，脚本拒绝覆盖已有结果。

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python docs/verification/drop_precision_20261007_results/analyze_closed.py --seed 31 --run logs/drop_precision_new_seed31_20261007_135022 --closed --output-dir /tmp/drop_precision_recheck31
python docs/verification/drop_precision_20261007_results/analyze_closed.py --seed 38 --run logs/drop_precision_new_seed38_20261007_140702 --closed --output-dir /tmp/drop_precision_recheck38
```

视频输入为对应闭包的 follow/overview/downward MP4、同名 CSV、pose/事件文件、CameraInfo，以及逐轮 `precision.json`。offset 可在独立 WSL shell 中用系统 ROS Python 离线导出：

```bash
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/verification/drop_precision_20261006/projection_evidence.py --bag logs/drop_precision_new_seed38_20261007_140702/vision_metrics.bag --export > /tmp/drop_precision_recheck38/exact_offsets.jsonl
```

回到 rl_drone 环境，复现 seed38 三视图主体；seed31 替换 run、输入及输出路径即可。归档成片另附 comparison.csv 数值生成的静态对照页，主体和对照页帧数在验证 JSON 中分别记录。

```bash
python docs/verification/drop_precision_20261007_results/compose_review.py logs/drop_precision_new_seed38_20261007_140702 --output /tmp/drop_precision_recheck38/review.mp4 --precision-json /tmp/drop_precision_recheck38/precision.json --exact-offset-data /tmp/drop_precision_recheck38/exact_offsets.jsonl --flight-limit 4 --corridor-limit 1.2 --wall-height 4 --corridor-bounds 8 9.5 -5 5 --case-label seed38_66c36a9e_centerK_D0
```

seed31误差分解使用 `precision.json` 与三份本地 bag 导出：`exact_offsets.jsonl`、`fc_pose_source.jsonl`、`alignment_context.jsonl`。将其放在新数据目录后执行：

```bash
python docs/verification/drop_precision_20261007_results/decompose_seed31.py --data-dir /tmp/drop_precision_recheck31
```

其中 exact_offsets 按上面的旧导出器生成；另两份的离线导出格式如下。此段用已加载 Noetic 的系统 Python 执行，时间保留 bag receipt 与消息源 stamp，不运行节点：

```python
import json, rosbag, yaml
from pathlib import Path
out = Path('/tmp/drop_precision_recheck31')
bag_path = 'logs/drop_precision_new_seed31_20261007_135022/vision_metrics.bag'
with rosbag.Bag(bag_path, 'r') as bag, (out/'fc_pose_source.jsonl').open('x') as fc, (out/'alignment_context.jsonl').open('x') as ctx:
    for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/pose', '/uav_vision/alignment_target_context']):
        if topic == '/mavros/local_position/pose':
            p = msg.pose.position
            fc.write(json.dumps(dict(t=t.to_sec(), source=msg.header.stamp.to_sec(), xyz=[p.x,p.y,p.z]))+'\n')
        else:
            ctx.write(json.dumps(dict(t=t.to_sec(), m=yaml.safe_load(str(msg))))+'\n')
```

逐帧 JSONL 和视频为可再生本地输入，保持排除；逐轮 `precision.json` 是视频/分解直接使用的小型结果输入。工具只复用现有参数和数据，不拟合相机参数或改写原 Gate。
