# 视觉笔记本仿真与回放基线（2026-07-15）

## 1. 结论边界

本报告的推理、Gazebo 识别、视频回放和 PT/ONNX 对照全部运行在笔记本 WSL2 环境；
PyTorch 路径使用笔记本 GPU/`rl_drone` conda 环境。当前尚未在 OrangePi 5 Plus 上运行
六分类 RKNN，也没有板端 NPU 性能、温度或 10 min 稳定性证据。

本轮复用 `toudi3.world` 已有五类标准靶和 H；红十字由固定评测场景单独插入，纯背景
使用同一 world 的无靶视野。`uav_vision_eval` 的作用不是再造一套靶标，而是为 world 中
既有实例补充独立类别/世界位姿真值、固定相机位姿、自动记录和可比较报告。

V-SIM-00 至 V-SIM-03 的最小基建已经落地。当前算法结果是首个可重复基线，不等于
`VISION_2026_ROADMAP.md` 的完整首轮工程 Gate 已通过。

## 2. 已完成的基础能力

- 双 catkin 联合环境入口可同时发现 `uav_vision`、`patrol_control`，并可导入
  `uav_vision.msg`、解析关键 launch；
- `uav_vision_eval` 已包含 target catalog、固定场景、独立像素/地图真值、CSV/JSON/report；
- headless shadow 将所有视觉输出隔离到 `/uav_vision_shadow/*`，保护全局旧控制话题，
  不启动 `actuator_pwm`、解锁、起飞或真实投递；
- 标准靶只有“类别与蓝色圆环完成同帧一对一关联、几何中心已精修”后，才可进入
  `target_memory`、旧兼容接口和投递证据；
- `target_memory` 已区分长期地图记忆和 `last_seen` 当前观测，并对控制候选执行 0.5 s
  新鲜度门禁；
- H 使用外圈与内部 H 结构联合验证，并受 `align_mode=landing` 门控；
- `/uav_vision/release_evidence` 已表达身份、结构、中心、新鲜度、稳定帧和拒绝原因；
  最终 `/mission/release_permission` 仍由任务/安全层拥有；
- 圆环运行节点和离线回放均使用等比例 letterbox，避免竖屏视频被强制拉伸。
- 统一 L0 runner 已连续 3 次通过圆环坐标、全局一对一关联、记忆新鲜度和地图/释放证据；
  runner 显式检查 PASS marker，避免 ROS1 `roslaunch` 掩盖 assertion 非零退出。

## 3. 固定 Gazebo 场景基线

模型：`liftrace_6cls_v5_merged_standard_20260714/weights/best.pt`。以下表格是修正融合前的
首版 0.8 s 笔记本基线，并且只统计经过业务门控的 actionable detections，保留用于前后对照。

| 场景 | Recall | Precision | FP | 平均像素误差 | 平均地图误差 | P95 延迟 | 最小报告判定 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| pillbox | 0.808 | 1.000 | 0 | 2.76 px | 0.0089 m | 0.839 s | 通过 |
| bridge | 0.809 | 1.000 | 0 | 4.39 px | 0.0143 m | 0.836 s | 通过 |
| tank | 0.765 | 1.000 | 0 | 4.39 px | 0.0143 m | 0.835 s | 通过 |
| tent | 0.935 | 1.000 | 0 | 4.39 px | 0.0143 m | 0.831 s | 通过 |
| panzer | 0.874 | 1.000 | 0 | 4.38 px | 0.0143 m | 0.840 s | 通过 |
| red_cross | 0.646 | 1.000 | 0 | 1.00 px | 0.0043 m | 0.825 s | 通过 |
| landing_h | 1.000 | 1.000 | 0 | 0.67 px | 0.0026 m | 0.827 s | 通过 |
| background | 负样本 | 1.000 | 0 | - | - | 0.832 s | 通过 |

“最小报告判定”只说明场景 runner、真值、记录器和宽松冒烟阈值工作正常。按主路线首轮
工程 Gate 复核时，red_cross、pillbox、bridge、tank、tent、panzer 的召回率未全部达到
0.95，旧 P95 延迟也明显高于 200 ms，因此 **V-SIM-04 尚未通过**。

评测入口已支持 `smoke` 与 `formal` 两套 profile，formal 还会实际检查 P95 像素误差
`<=20 px`、P95 地图误差 `<=0.25 m` 和 P95 延迟 `<=200 ms`。一次 18 s tent 正式运行
得到 precision=1、recall=0.730、P95 像素误差=4.36 px、P95 地图误差=0.0142 m、
P95 延迟=829 ms，报告按预期 FAIL（召回和延迟未过），证明正式 Gate 不再被冒烟阈值掩盖。

```bash
./top_level_scripts/run_toudi3_visual_eval.sh \
  standard_tent 18 /tmp/uav_vision_eval/formal_tent formal
```

随后融合改为“完整正样本提前发布 + 0.15 s 不完整帧兜底”，笔记本 detector 显式使用 GPU；
评测延迟按消息真实到达 recorder 的时间计算，不包含 recorder 自身的 0.8 s 聚合等待。
代表场景结果：tent recall=0.709/P95=172 ms（formal 延迟通过、召回失败），tank
recall=0.675/P95=176 ms，red_cross recall=0.716/P95=182 ms；后两项通过 smoke。
该结果后来证明混合了两个评测口径问题：独立 `queue_size=1` 检测器并不保证处理完全
相同的相机帧；Gazebo `/clock` 也不能替代墙钟算法延迟。因此这些数值只保留为历史
对照，不能作为 V-SIM-04 的最终延迟结论。

### 3.1 V-SIM-04 当前进展（尚未通过）

- 已建立固定 30-seed 矩阵：五类标准靶各 4 个位姿、红十字 4 个、H 3 个、背景 3 个；
  每个 seed 使用独立 ROS/Gazebo 端口，并校验 seed、相机位姿和基础设施日志；
- 首轮完整 30/30 运行得到 aggregate precision=0.9312、recall=0.8595，仅 6 个 seed
  通过；但其中 1 个 seed 相机 spawn 超时，且延迟仍按 Gazebo 仿真时钟计算，因此该轮
  只用于发现问题，不能作为最终性能基线；
- `TargetDetectionArray` 已增加 `source/completed_sources`，区分“分支已处理但未检出”与
  “该分支未处理该帧”；识别 recall 只统计实际推理帧，输入覆盖率单独设 formal Gate
  `>=0.15`；
- 融合器支持 50 ms 可配置近似时间同步，以适配各 `queue_size=1` 分支在负载下抽取相邻
  帧；正常分支齐全时立即输出，0.8 s 仅是缺分支兜底；
- 固定静态评测入口允许 0.10 s 年龄上限的 latest-TF 回退，动态/实机路径仍要求图像时刻
  TF；H 评测直接设置默认 `landing` 阶段，避免启动时模式话题竞争；
- 纯本机 runner 已强制 `ROS_IP=127.0.0.1`，完整 L0 回归再次通过；延迟改为
  `perf_counter` 测量 recorder 收到图像至收到 mapped 输出的墙钟时间，不再使用 Gazebo
  real-time factor 波动的 `/clock`；
- 最新 7-seed 诊断中，pillbox/bridge/tent 正视、red_cross 和 H 的已评分帧多为
  precision/recall=1，但 6/7 seed 的 P95 墙钟延迟仍超过 200 ms；135° tent 仍出现
  panzer 错分（precision=0.6275、recall=0.5、FP=19），H seed 的地图误差曾因 TF/连接
  缺失为 N/A。该子集本身因缺少其余 23 个 seed，矩阵报告按设计为 FAIL。

当前结论：V-SIM-04 仍为 **FAIL/进行中**。需先稳定分支同步与 TF、复跑干净 30-seed，
再执行 10 min shadow；不得把子集识别率或笔记本墙钟延迟外推为 OrangePi 性能。

## 4. 实拍视频圆环回放

输入：`real_target.mp4`，按 4 帧步长处理 1156 帧。修复纵横比后：

- 标准类检测 631 个，圆环候选 546 个，完成类别—圆环关联 371 个；
- 自洽关联率从旧强制拉伸版本的 1.74% 提升到 58.80%；
- 各类关联/总检测：bridge 149/209、panzer 74/180、pillbox 88/117、tank 60/125；
- 一部分未关联的 `panzer` 实际是 YOLO 将 H 误判为标准类，而画面中不存在蓝色圆环，
  几何链拒绝关联是正确行为。

该视频没有逐帧圆环中心真值，也没有同步 CameraInfo、TF/pose 和目标地图真值。因此
58.80% 只能称为链路自洽率，不能称为圆环关联召回率；绝对中心误差和实拍地图误差
目前不可计算。输出位于：

```text
vision_ws/test_data/real_target_ring_map_eval_merged_letterbox_20260715_stride4/
```

## 5. PT/ONNX 笔记本一致性

历史 8 图检查曾记录 8 个框匹配、PT 缺失 1 个、ONNX 新增 1 个，最小 IoU 0.954、最大
置信度差 0.536。2026-08-30 复核确认该结论混入了不同预处理：PT 后端使用动态矩形输入，
固定形状 ONNX 使用方形 letterbox，比较的并非同一输入张量几何。该历史结果保留用于说明
`backend-native` 的风险，不再作为导出数值不一致证据。

`compare_yolo_backends.py` 现默认先生成同一 `640x640` fixed-letterbox 图像，再将完全相同
的几何输入交给两种后端；也支持重复 `--source` 形成聚合 Gate，并落盘未匹配框和双方置信度。
覆盖标准靶诊断采集 9 图、red_cross、landing H 和纯背景共 12 图的受控复测结果为：19 个框
全部匹配，缺失/新增均为 0，最低/平均 IoU 为 0.9999991/0.9999997，最大置信度差
`5.19e-6`，最大框坐标差 `6.11e-5 px`，判定通过。相同手工 letterbox 的原始 PT/ONNX
输出张量也数值接近（最大绝对差约 `9.46e-4`）。该结论只关闭笔记本 PT→ONNX 导出一致性，
不是 RKNN、NPU 性能或 OrangePi ROS 链验收。

## 6. 下一阶段可执行任务

1. 用 loopback、墙钟延迟、来源/覆盖率和 50 ms 近似同步口径复跑完整 30-seed；先清零
   spawn、TF、ROS 连接类基础设施失败，再按 yaw/高度聚合算法失败；
2. 对 P95 `>200 ms` 的 seed 分解 target detector、几何、融合和投影阶段墙钟延迟，完成
   10 min shadow 稳定性；不得通过取消几何验证或降低正式阈值绕过；
3. 从实拍视频抽取并人工标注代表帧圆环中心、目标实例和负样本，计算真正的关联召回、
   错配率与精修前后中心误差；普通 MP4 无法补出地图误差，后续采集需同步 CameraInfo
   与 pose/TF；
4. 用独立 held-out 实拍/仿真样本扩充 fixed-letterbox PT/ONNX 回归，禁止退回不同后端各自
   预处理后再比较；
5. 获得 OrangePi 和匹配相机后，执行同样本 PT→ONNX→RKNN 逐框对照、
   5-10 Hz 性能和 10 min 稳定性验收。上板前不宣称机载可运行。
