# seed31：闭包独立复核

本轮原 Gate **PASS / all_checks_passed**；三次 mock ACK、三次恢复交接、走廊三处检查点通过，H 接地后已记录落地与解除武装，实际碰撞 **0**。精度总体与历史接近，不能据此宣称全面改善或推广。[两轮汇总](../REPORT.md)另列seed38结果。

运行：`logs/drop_precision_new_seed31_20261007_135022`；manifest 的主仓、控制、视觉 HEAD 均为 `66c36a9e6fab51c9e3cf942a9620fb11dfd70ccc`。有效运行入口修复之外，控制/camera 内容与 `074f8381` 一致（operator 说明）。启动失败的 134730 轮为 INFRA，不计入飞行统计。

## ACK 身体中心误差

单位 cm。主指标用 ACK 消息源 stamp 在线性插值的 Gazebo 身体/model-origin XY 上计算；真值靶表与历史完全一致。三槽请求类别与最近真值类别均相符，机体 XY 均在对应名义板范围内。这不是包裹落点。

| 槽 / 类别 | ACK 源时刻/s | 原历史 snake3 | 前候选 7d312fa2 | 本轮 | 本轮−历史 |
|---|---:|---:|---:|---:|---:|
| 1 / red_cross | 84.840 | 3.474 | 6.646 | 3.953 | +0.479 |
| 2 / bridge | 119.216 | 11.431 | 15.378 | 11.072 | −0.359 |
| 3 / panzer | 140.275 | 4.871 | 3.346 | 5.299 | +0.428 |
| 三槽均值 | — | 6.592 | 8.457 | 6.775 | +0.183 |

本轮最大误差 11.072 cm（历史 11.431 cm）。相对前候选，红十字/bridge 缩小、panzer 增大；相对原历史，小幅变化且方向不一致。只有一轮同 seed 观测，不足以建立统计收益。

历史 receipt 时刻最近样本口径另列，避免与源时刻插值混用：

| 类别 | 原历史/cm | 本轮/cm | 插值跨度/ms | 本轮 ACK 源时刻最近 truth 样本年龄/ms | receipt−source/ms |
|---|---:|---:|---:|---:|---:|
| red_cross | 3.317 | 3.947 | 101 | 30 | 3 |
| bridge | 11.420 | 11.120 | 102 | 22 | 6 |
| panzer | 4.815 | 5.157 | 101 | 36 | 0 |

均满足原 0.25s 插值间隔门限。truth Pose 的时间来自 recorder 收到 ModelStates，仍有接收延迟不确定性；没有把这些数据称作曝光同步精确真值。CSV/JSON 保留原始数值、transaction identity、类别、插值跨度和年龄。未投槽位的工具口径为空值，不补零。

## 恢复、碰撞和完成分开核对

| 事项 | 原历史 seed31 | 本轮 |
|---|---|---|
| ACK / 恢复 | 3 / 3 | 3 / 3 |
| 走廊检查点 | corridor_entry、Wall_20、Wall_22 均记录 | 同三处均记录 |
| actual collision | 0 | 0 |
| 原始 Gate | **FAIL / manager_failed** | **PASS / all_checks_passed** |
| 物理接地 | H 支撑接触 ROS 216.369，任务后 204.517s | H 支撑接触 ROS 214.034，任务后 202.432s |
| 落地 / 解除武装记录 | 日志最后仍 landed_state=4、armed=true；不能从接触补判解除武装 | landed_state=1 源时刻 217.536；armed=false 源时刻 219.741 |
| 软件尾程 | 224.110 ABORTED，probe_pose_exception:ValueError | 218.602 COMPLETE；Gate 任务时间 206.994s |

本轮恢复交接源时刻为 87.198 / 121.771 / 141.562s，与各 ACK 的 mission、decision_seq、attempt、slot、target 一致。RPC ACK、恢复交接、接地、软件 COMPLETE、解除武装是不同事件，不互相代替。

接触监视器记录三个 support episode：起飞前原 H、终点 H 首次接触与再次稳定支撑；`events` 无 actual collision。终点稳定支撑开始于 214.751s。不能把“0 collision”写成全程完全无接触。

原 wrapper 清理 PASS、后续零残留检查 PASS，依据 `cleanup_verification.json` 的字段及原 wrapper 文本；该文件附带的通用 note 提及“Original wrapper FAIL”，与本轮字段不符，未采用它覆盖实际 PASS 证据。

## 轻量误差分解（实际闭包数据）

从实际最终 setpoint 找到与 float32 命令坐标唯一相等的捕获 offset：红十字源时刻 79.121s、bridge 113.710s、panzer 137.142s。对应几何 ID 为 8/7/5，语义 ID 为 8/6/4；用 bag 的 alignment context 核对类别、mission、decision、attempt、slot，未要求两类 ID 数字相等。

冻结绝对 XY 分别为 `(7.416583858,-0.993716621)`、`(5.052172103,-3.661540625)`、`(4.015879693,-0.535386628)`。各槽最后下降命令直接相等的样本数为 1/1/5。复现既有 3D 限幅，下降期间 43/44、39/40、24/24 条 setpoint 可重建至 1μm 内；红十字 83.672s 和 bridge 115.267s 两条在固定 FC 接收时间窗口 `[-120,+2]ms` 内最小残差为 3.09/6.17mm，保留未解释，不扩大时间窗使其强行通过。捕获源帧依据命令值唯一匹配，不是 ACK 附近最近观测。

设冻结目标为 G、真值靶心为 P、源帧/ACK 的 FC 与 GT 为 F/Gt。按 XY 向量计算：

`跟踪 T = F_ack − G`

`相对投影 E = G − P − (F_src − Gt_src)`

`定位变化 L = (F_src − Gt_src) − (F_ack − Gt_ack)`

三项之和等于 `Gt_ack − P`，数值闭合；模长不能相加。

| 类别 | 原始地图目标−真值/cm | 相对投影 E/cm | 跟踪 T/cm | 定位变化 L/cm | 最终/cm |
|---|---:|---:|---:|---:|---:|
| red_cross | 3.391 | 3.680 | 4.711 | 3.628 | 3.953 |
| bridge | 5.106 | 4.786 | 5.344 | 1.442 | 11.072 |
| panzer | 1.657 | 2.053 | 5.826 | 1.811 | 5.299 |

bridge 的向量（cm，X/Y）为：T=`(-5.291,-0.746)`，E=`(-3.443,-3.324)`，L=`(-1.359,-0.481)`，合计 `(-10.093,-4.551)`。**投影与跟踪项量级接近且同向，定位变化较小但继续叠加**；不能把 11cm 全部归为视觉几何，也不能把地图点−世界靶心直接称为纯投影误差。源帧 FC−GT 为 `(-0.870,+0.590)cm`，ACK 时为 `(+0.489,+1.071)cm`。ACK 插值跟踪差 5.344cm 小于捕获容差 5.774cm；这是事后数值核对，不取代控制回调的实时许可判定。

红十字多项抵消，panzer 的跟踪项最大；不能仅凭最终较小误差判定投影更准。相对投影 E 仍混合中心提取、姿态/TF、平面和时间误差，此分解没有进一步区分这些成因，也没有用理想像素重投影代替实跑精度。

FC 源时间插值跨度 31–35ms；GT 跨度 101–108ms。bridge 源帧 GT 最近年龄 10ms、ACK 22ms；对应 GT 两端 XY 位移 0.427/0.267cm，仅反映时间采样敏感性，不是严格误差界。完整向量、插值年龄、容差与匹配残差见 `ack_decomposition.json`。原始 FC/offset/context 逐条导出仅本地保留。

## 相机和归因边界

实际 CameraInfo：1280×720，fx=fy=725.3510059644452，cx=640、cy=360，D0；精确投影与控制候选启用，quality-ordered NMS=false，三槽补偿为零。

相对历史，同时变化了控制下降捕获行为和仿真标定配置，不能纯算法归因，更不能据此修改真实相机标定。runtime 原始 FAIL 仍保留；补充审查将 world_sdf Euler 序列化角比较容差限定为 2e-5rad，三份 runtime/CameraInfo 均 PASS。依据原 run 的 `camera_runtime_review_20261007/pose_diagnosis/REPORT.md`；这是配置/安装核对，不等同图像像素或投递精度验收。

## 产物与复现

- `precision.json`：复用旧 precision_eval 的本轮逐 ACK 结果。
- `historical.json`：原 snake3 历史同口径结果。
- `comparison.csv` / `comparison.json`：原历史、前候选、本轮逐槽比较。
- `completion_evidence.json`：原 Gate、接触及恢复事件；原文件不改写。
- `exact_offsets.jsonl`：闭包 bag 离线导出，277 条 drop_offset；其余为 align_mode 与 metadata。观测不是冻结控制目标。
- [三视图及历史指标对照视频](review_seed31_comparison.mp4)：1920×1080、10fps，228.5s；包括 220.5s 源时间对齐视频（含末帧停留 2s）及 8s 对照页，历史视频不重做。
- `video_validation.json`：完整解码、原始三视频帧数/CSV 一致性、ACK 后三处及最终 COMPLETE 画面检查。源图像公共终点 219.718s 比解除武装源时刻早 23ms；解除武装依据事件日志，未补造视频帧。

视频、逐条 JSONL、PNG 与草稿由本目录 `.gitignore` 排除，保留为本地素材；精简归档包括报告、逐槽比较、误差分解和视频验证摘要。

数据源：历史 `logs/snake3_camera2m_snake3_seed31_20261005_095902`，前候选 `logs/drop_precision_seed31_20261007_020102`，本轮路径见首段。
