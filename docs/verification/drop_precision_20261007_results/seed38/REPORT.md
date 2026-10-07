# seed38：闭包独立复核

**原 Gate 保留 FAIL / actual_collision，不推广。** 三槽 mock ACK、恢复各 3/3，走廊按序穿过、H 已有物理支撑接触；接地后 7.739s 出现软件 ABORTED，最后记录仍 armed=true / AUTO.LAND，未证明解除武装。碰撞统计采用闭包完整 **11 episodes**，不是最初树边两次。

运行 `logs/drop_precision_new_seed38_20261007_140702`，manifest source `66c36a9e6fab51c9e3cf942a9620fb11dfd70ccc`。后续评测工具提交不属于本轮生产版本。

## 三槽精度

单位 cm，ACK 源 stamp 插值 Gazebo 身体/model-origin XY；三槽最近真值类别均与请求相符。指标不是实物包裹落点。

| 槽 / 类别 | ACK 源时刻/s | 原历史 snake3 | 前候选 7d312fa2 | 本轮 | 本轮−历史 |
|---|---:|---:|---:|---:|---:|
| 1 / red_cross | 81.744 | 1.861 | 4.704 | 6.286 | +4.425 |
| 2 / bridge | 100.904 | 7.927 | 9.771 | 3.597 | −4.330 |
| 3 / panzer | 275.296 | 13.972 | 未投，空值 | 2.402 | −11.571 |
| 三槽均值 | — | 7.920 | 不作三槽均值 | 4.095 | −3.825 |

本轮最大误差 6.286cm；红十字比历史大，bridge/panzer 较小，不是所有类别改善。前候选只有两槽，第三槽不补零，也不用两槽平均替代三槽平均。

| 类别 | 历史 receipt 最近样本/cm | 本轮 receipt 最近样本/cm | 本轮 GT 插值跨度/ms | ACK 源时刻最近 GT 年龄/ms |
|---|---:|---:|---:|---:|
| red_cross | 1.941 | 6.269 | 100 | 48 |
| bridge | 7.725 | 3.646 | 101 | 46 |
| panzer | 13.982 | 2.411 | 101 | 7 |

三次 ACK 的 receipt−source 均 0ms；这不意味着所有 GT/图像同步。GT 仍为 recorder 收到 ModelStates 的采样时间，0.25s 插值跨度门限不变。恢复交接源时刻 84.233 / 102.430 / 277.629s，逐项核对与 ACK 的 mission、decision、attempt、slot、类别一致；无重复 ACK，无开始后未完成 RPC。

## 补搜路径与耗时

本轮不是三目标连续下降投递：高位 TOP3 中断后，低空初始 panzer 线索被 pillbox 纠正并 disproved；先投 red_cross、bridge，再 LOW_COVERAGE 补搜，第三槽后才进入尾程。

operator 实时记录的内部时刻为 52.318s TOP3、60.668s 类别纠正、60.718s disproved、102.465s LOW_COVERAGE；闭包 high_view 订阅记录相应阶段接收时刻为 52.444s DESCEND、60.756s REVISIT、102.517s LOW_COVERAGE。两种时钟来源分列，不把接收时刻差当处理耗时。

闭包 decision 25 于 263.752s 以 `high_weight_search_interrupt` 接受 panzer，275.296s ACK；LOW_COVERAGE 到该 decision 接收间隔约 161.235s。冻结参数 `resume_survey_enabled=false`，记录中 resume_attempted/resume_completed 均 false。此段包含补搜与任务路径，不能称为投影变慢或续扫回归，也不修改规划补偿。

## 完整接触统计

均为 `competition_guard_collision` 与以下模型/墙碰撞体的接触。监视器正式计入 actual collision，保留该口径；小深度不等于无碰撞或 Gate 通过。

| 对象 | episodes | 采样 duration 合计/s | 最大深度/mm | 最大采样力/N |
|---|---:|---:|---:|---:|
| juniper_Tree_1 | 2 | 0.026 | 0.387796 | 0.437889 |
| Wall_22_north | 2 | 2.615 | 1.024094 | 3.747128 |
| Wall_9 | 7 | 4.034 | 0.233933 | 5.849468 |
| 合计/各自峰值 | **11** | **6.675** | **1.024094** | **5.849468** |

树边事件在 285.473/285.550s；Wall_22_north 在 329.588/335.446s；Wall_9 在 337.389–342.944s。最长单 episode **2.238s**（Wall_9，340.546s 开始），另有 Wall_22_north 的 2.017s 事件。各维度峰值来自不同事件，不把最大深度和最大力说成同一瞬间。duration 为监视器首末采样定义，单样本 episode 记 0s，不说明真实接触持续时间严格为零。逐 episode 数字见 `contact_summary.json`。

接触与碰撞保持区分：起飞前 H 和终点 H 共两段 support episode，另行记录；终点 H 最大支撑采样力不混入上述障碍碰撞峰值。碰撞后 record-only 继续飞行，未因首个 Gate FAIL 自动中止。

## 物理路径与软件尾程

- 完整 Gate 数据记录 corridor_entry → Wall_20 → Wall_22 的有序穿越；投后四段返回交接完成，不能拿首个 FAIL 快照中未发生的门/H检查判定最终漏完成。
- H 实际支撑接触 **362.082s**，持续至最后样本 **370.065s**；相对任务源起点 11.824s，为 **350.258s** 后触地。
- 软件 **369.821s ABORTED / probe_pose_exception:ValueError**，在首次 H 接触后 **7.739s**，属于接地后的独立尾程异常。
- 最后 FC 状态记录 **361.933s armed=true、AUTO.LAND**；最后 extended_state **361.543s landed_state=4**。之后未记录到落地状态=1或解除武装，不能用模型触地代替这两项证据。
- 原 Gate 最终 reason=`actual_collision`；errors 同时包含 actual_collision、manager_failed、manager_aborted。不制造 COMPLETE、PASS 或分项修订 Gate。
- 原 wrapper 清理 PASS，最终零残留 PASS；不采用 cleanup 文件通用 note 中与实际字段相矛盾的“Original wrapper FAIL”字样。

原历史 seed38 同样三 ACK / 三恢复 / 0 collision，H 首次接触 374.982s（任务后 363.540s）；它后来记录 landed_state=1、armed=false，但原 Gate 仍为 **FAIL / manager_failed**，软件最终 `safety_motion_timed_out`。两轮都须分开报告物理接地与软件结果；不能仅比较 Gate 或总耗时。

## 标定、视频及数据边界

实际相机 K 为 fx=fy=725.3510059644452、cx=640、cy=360，1280×720、D0。`camera_runtime_review_20261007/result.json` 为 PASS，包含 123 CameraInfo、124 image、118 精确 stamp 配对；其明确 scope 是 runtime 配置观测，不是像素标定验收。

控制修复与仿真标定同时变化，不能纯算法归因或宣称板端收益；不以几张理想像素投影代替实跑精度。

`precision.json`、`comparison.csv/json`、`contact_summary.json` 为精简指标；原逐条 offset JSONL 和视频保持本地，历史视频不重做。

[三视图及历史对照视频](review_seed38_comparison.mp4)：1920×1080、10fps、3808帧、380.8s。全片解码 PASS；follow/overview/downward 原视频分别 2542/2542/3076 帧，与各自 CSV 完全一致。已抽查三次 ACK 后、墙接触、H 支撑及末帧，并检查对照页；校验摘要见 `video_validation.json`。

**原录像可见性限制**：第三槽与后半程主视角大面积被墙面遮挡，不能靠主画面目视证明墙碰撞或 H 触地；对应结论依据接触、轨迹和事件日志，叠加字幕不当作额外影像证据。技术校验通过不表示每个事件均可见。视频公共末帧为 369.746s，早于 ABORT 日志 75ms；对照页明确标注后续异常来自日志，不补造影像或解除武装状态。
