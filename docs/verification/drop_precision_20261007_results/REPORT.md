# 两轮投递候选：独立评测汇总

**当前不推广，候选维持默认关闭。** 两轮均三槽正确类别 ACK、三次恢复并实际触及 H；seed31 原 Gate PASS、已解除武装，seed38 原 Gate FAIL（11 次包络碰撞），H 接地后软件异常且未证明解除武装。精度表现分化，不能用局部误差降低抵消碰撞或尾程失败。

两轮飞行 manifest 均为 **66c36a9e**，控制修复 **1c700dd3**；启动入口修复之外控制/camera 与 074f8381 一致。之后的评测工具提交不计入飞行版本。134730 启动失败轮只计 INFRA，不进入以下统计。

## 实测精度（cm）

ACK 消息源 stamp 插值身体/model-origin XY；均为对应正确类别。历史来自原 snake3 seed31/38，不使用后来的 resume_on_fixed 替代。括号内为本轮−历史，负数表示误差较小。

| seed | red_cross 历史→本轮 | bridge 历史→本轮 | panzer 历史→本轮 | 三槽均值 历史→本轮 |
|---|---:|---:|---:|---:|
| 31 | 3.474→3.953 (+0.479) | 11.431→11.072 (−0.359) | 4.871→5.299 (+0.428) | 6.592→6.775 |
| 38 | 1.861→6.286 (+4.425) | 7.927→3.597 (−4.330) | 13.972→2.402 (−11.571) | 7.920→4.095 |

六个配对中三个变小、三个变大；seed31 未证实整体改善，seed38 平均较小主要受 panzer 变化影响。每 seed 仅一轮，且同时改变控制和仿真相机，不能分离纯算法收益或给出统计提升结论。指标不是实物包裹落点；历史 receipt 最近样本口径、时间跨度及年龄均在逐轮报告另列，未投不补零。

seed31 bridge 的轻量实跑分解：相对捕获投影 **4.786cm**、跟踪 **5.344cm**、定位变化 **1.442cm**，向量同向叠加成 **11.072cm**；不能全归为几何。捕获源帧由最终命令与 bag offset 唯一匹配，详见 [seed31 报告](seed31/REPORT.md)，其中保留两处下降命令重建的毫米级残差和采样时间不确定性。

## 成功、接触和软件结果分列

| 项目 | seed31 | seed38 |
|---|---|---|
| 正确类别 ACK / 恢复 | 3 / 3 | 3 / 3 |
| 走廊有序穿过 | 是 | 是，record-only 继续 |
| 障碍 actual collision episodes | 0 | **11** |
| 碰撞峰值深度 / 最长事件 / 最大采样力 | 无 | **1.024mm / 2.238s / 5.849N**，峰值不必同事件 |
| H 首次支撑接触 ROS 时刻 | 214.034s | 362.082s |
| 任务起点到 H 接触 | 202.432s | 350.258s，含补搜 |
| 软件最终状态 | COMPLETE 218.602s | ABORTED 369.821s |
| 接地后异常 | 未见同类异常 | 接地后 **7.739s**，probe_pose_exception:ValueError |
| 解除武装 | 已记录 armed=false 219.741s | 未证明；最后 armed=true / AUTO.LAND |
| 原始 Gate | **PASS / all_checks_passed** | **FAIL / actual_collision** |
| wrapper / 最终清理 | PASS / 零残留 | PASS / 零残留 |

seed38 全量接触分为树边2次、Wall_22_north 2次、Wall_9 7次，不只引用最初树边0.39mm；完整数字见 [seed38 报告](seed38/REPORT.md)。H support 接触与障碍 actual collision 分开，不将小深度接触删除或改判 Gate。

历史两 seed 的原 Gate 都为 FAIL / manager_failed：seed31 有 H 触地但未记录解除武装，seed38 已记录解除武装却软件尾程超时。故本报告分别比较 ACK、恢复、过门、物理触地、飞控落地状态、解除武装及原 Gate，不把模型接地等同完整软件成功。

## 路径与归因限制

seed38 经 TOP3 中断、低空 pillbox 纠正原 panzer 线索并 disproved，再投红十字/bridge、LOW_COVERAGE 补搜；decision 25 才接受真实 panzer。冻结 resume=false，未开启续扫。第三槽前的补搜耗时不能归因于投影变慢或续扫回归。

新版仿真相机是居中 K/D0；历史为偏心 K/非零畸变。两轮真实 CameraInfo/runtime 配置证据齐备，但配置检查与数学离线投影不等于实跑像素/投递精度。seed31 原 runtime FAIL 与序列化精度补充 3/3 PASS 均保留，seed38 runtime 检查为 PASS；详细来源在逐轮报告。不修改真实相机或生产阈值。

## 报告与附件

- [seed31 精简报告](seed31/REPORT.md)；[seed38 精简报告](seed38/REPORT.md)。
- `two_seed_metrics.json` 保留原 Gate、最终状态、完整接触摘要和比较来源；逐轮 comparison.csv/json 可直接复核数值。
- [seed31 对照视频](seed31/review_seed31_comparison.mp4)（2285帧 / 228.5s）；[seed38 对照视频](seed38/review_seed38_comparison.mp4)（3808帧 / 380.8s）。均为1080p、10fps，完整解码与原视频/CSV帧数一致性检查通过，抽帧和对照页已检查；历史视频不重做。
- seed38 原录像第三槽及后半程主视角被墙面遮挡，可见性受限；碰撞/H触地结论依据接触/轨迹/事件数据，不能写成视频目视证实。公共末帧早于 ABORT 75ms，后续日志异常单列。
- 大视频、逐条 JSONL、抽帧 PNG 已在本目录 `.gitignore` 排除，保留为本地附件。未附带视频文件的仓库副本无法直接播放相对视频链接。
