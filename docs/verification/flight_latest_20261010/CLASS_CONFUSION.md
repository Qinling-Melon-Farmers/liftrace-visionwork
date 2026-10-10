# 2026-10-10 类别混淆与高位提前中断

**核实结果：真实 pillbox 被输出为 panzer/3，同时短暂输出 bridge。另一块真实装甲车为 panzer/4。高位提前中断使用的是 bridge、panzer、red_cross 三个类别线索，并非将两个 panzer 直接计成两类。旧 bridge 线索来自 pillbox 所在位置；panzer/4 在另一位置成为较强精修线索后，原位置的 bridge/panzer 冲突解除，旧 bridge 继续参与中断。**

对应导航记忆补丁为 `85d10fc4`：跨类别冲突保留非首选有效物理位置，两个根因用例旧版失败、修复后通过，相关回归 97/97 PASS。尚未新增动态飞行验证。

## 类别、实例与时间

人工类别依据现场用户确认及原图核对。ID 是视觉记忆实例 ID，不是模型类别编号。下表时间使用 bag 接收秒；图像源时间另列，不混用。

| 实物/位置 | 视觉记忆输出 | 首次出现 / 确认 | 记忆坐标约值（camera_init） |
| --- | --- | --- | --- |
| 帐篷 | tent/1 | 146.200 / 146.712 秒 | (1.682, 0.938) |
| 用户确认的地堡 pillbox | **panzer/3** | 147.276 / 147.538 秒 | (-0.085, 1.109) |
| 另一块真实装甲车 | panzer/4 | 148.152 / 148.243 秒 | (1.138, -1.220)，高位线索点约 (1.139,-1.213) |
| 红十字 | red_cross/6 | 162.977 / 163.088 秒 | (3.749,-0.935) |

整个 bag 没有 pillbox 检测输出。panzer/3 与 panzer/4 对应两个不同位置的靶，不能据此认定一次实例重复建 ID。circle/2 和 circle/5 是相应位置的圆环几何辅助 ID，不能当作额外可投类别。

原图源时间 **147.287311 秒**：右侧地堡被同时框为 panzer=0.7993 和 bridge=0.5083；左侧帐篷为 tent=0.7383。147.491 秒相机原图仍可见地堡下缘裁切；拼图回贴的是最近的 147.467579 秒检测，时间差 23.8ms，不伪称完全同帧。158.040 秒原图显示另一块完整装甲车，panzer=0.8975。

[原图/标签核对拼图](class_confusion_review.jpg)。仅有这张派生拼图留在 verification；全部九张原图已移至被忽略的 [analysis 原图目录](../../../试飞产物/analysis_flight_latest_20261010/class_confusion_frames/)。例如 [147.287 原图](../../../试飞产物/analysis_flight_latest_20261010/class_confusion_frames/source_00147.287.jpg) 与 [真实装甲车原图](../../../试飞产物/analysis_flight_latest_20261010/class_confusion_frames/source_00158.040.jpg)。

## 中断实际使用的集合

bag 内 probe 状态明确写出：

- required_classes = bridge、panzer、red_cross；
- completion_policy = COMPLETE_ROUTE_OR_SUPPORTED_TOP3；
- coarse_enabled=true，interrupt_refined_classes=[]；
- SURVEY_INTERRUPTED_TOP3 事件源时间为 **162.976636 秒**；
- 首次包含该事件的 probe 状态接收时间为 **163.527331 秒**；
- LOCAL_DESCENT_TRANSIT 的 decision 6 接收时间为 **163.773 秒**。

事件源时间是运行逻辑记录的时间，状态/指令的录包接收有延后；不使用这些差值推算模型端到端延迟。

中断时 top_hints：

| 类别 | 完整 key 中的 source / id | 位置 | 证据 |
| --- | --- | --- | --- |
| bridge | bbox / 0 | **(-0.066536,1.047701)** | 两张独立源图像支持，bbox 粗线索；最后源时间 147.525123 秒，已有约 15.45 秒历史 |
| panzer | vision / 4 | **(1.139380,-1.212664)** | 精修线索，evidence_count=8 |
| red_cross | bbox / 3 | (3.759325,-0.935434) | 两张独立源图像支持 |

bbox key 的数字不是 TargetCandidate.id：bridge bbox/0 不等于 circle/0，red_cross bbox/3 也不等于 panzer/3。完整身份还包含 source 和 first_seen_ns。

中断支持记录仍保留两个 panzer 空间假设，其中 pillbox 位置是 bbox 弱线索、装甲车位置是 vision 较强线索；实际 top_hints 只取后者。帐篷线索也存在，但不是本轮 required_classes 的成员。不能把这里的“三类”理解成任意三块靶或五类全齐。

## 原框和投影核对：2.56m 是跨靶比较

逐条读取原 bag 的 /uav_vision/navigation_hints，并与 detections/mapped 的**同一源图像时间**及 ROI 配对。旧 bridge 粗点的直接来源是：

```text
source image = 147.525123 秒
raw bridge score = 0.6787109375
ROI = x868 y621 width156 height98
bbox center = (946,670)
projector = coarse_navigation_projector
map_frame = camera_init
coarse point = (-0.066535925,1.047700816,-0.277739008)
receipt = 147.661691 秒
```

对应同一源图像、同一 bridge 框的几何精修点为：

```text
center_source = circle_geometry
map point = (-0.075384717,1.088570012,-0.277739008)
receipt = 147.661822 秒
```

两者 XY 差 **4.18cm**。再与误识 panzer/3 的记忆点比较，差 **6.42cm**，均在本次关联半径 0.6m 内。与 panzer/4 的精点相比则为 **2.5619m**，因为那是另一块装甲车。

另两个配对检查：

| 源图像秒 | 粗/精配对 | XY 差 |
| --- | --- | --- |
| 147.114962 | 同框地堡 bridge 粗点 (-0.1241,1.2177)，panzer 圆环精点 (-0.1218,1.2224) | 约 5.31mm |
| 147.467579 | panzer 粗点 (-0.05624,1.04305)，同源 panzer 精点 (-0.05709,1.04750) | 约 4.53mm |

第二组精修消息接收时间为 148.314147 秒，源时间仍为 147.467579 秒；按源时间关联才能避免错误地将它配给随后 ID 4 的图像。

147.287 同框的 panzer 属于 ID 3 所在地堡位置，不是后来 ID 4 的装甲车。现有配对证据**不支持这处存在 2.56m 的另一投影定义错位**；也没有外部位置真值可据此保证绝对地图精度。

## 提前中断的因果链

1. 地堡短暂同时被输出为 bridge 与 panzer。共有 7 条 bridge 原始检测，源时间 147.115–147.525 秒，分数约 0.508–0.750；其中 6 条原始消息同时含 panzer，二者 ROI IoU 为 0.940–0.973，确实是同一个可见物体的两类框。
2. 粗投影把两个标签放在同一位置。probe 记录约 147.210 秒源时间的 navigation_evidence_conflict，bridge 和 panzer 进入冲突。
3. 另一位置 panzer/4 出现较强 vision 证据后，约 148.503 秒源时间记录 navigation_conflict_resolved。旧 bridge 与旧 panzer 粗假设仍在 support 中，并未由真实地堡分类纠正。
4. 当前 NavigationMemory._refresh 按每个类别取最高等级的 finalists：panzer/4 的 vision 等级高于地堡位置的 panzer bbox，所以后者不再参与后续同等级的跨类别冲突检查。bridge 的同位置竞争标签因此从该检查集合消失；这与 bag 内事件和支持状态相符。这里是代码机制解释，板端无 Git，不能仅凭本地源码认定完整板端 revision。
5. 红十字形成两张独立图像支持后，bridge+panzer+red_cross 满足集合条件，高位搜索被替换为就近下降/复访。

**需关注的是局部分类冲突如何跨较强的异地同类别假设保留/解除；缩短 TTL 不能解释或修复这个机制。** 本轮修复保留非首选物理位置参与冲突判断，没有缩短 TTL 或提高视觉阈值。

随后首次 APPROACH 选中的是真实装甲车 panzer/4，并没有在本包里实际复访、投递错误的地堡 panzer/3。类别混淆影响了提前中断的依据；首投几何收敛失败另见主报告，不把两者混为同一个根因。

## 模型映射与执行优化边界

bag 元数据和本地五类 metadata 一致：**bridge=0、panzer=1、pillbox=2、tent=3、red_cross=4**。消息构造按 handle.names[class_id] 转换类别名，没有发现误套旧六类 red_cross=5 映射的证据。该检查验证映射配置与转换方式一致，**未读取模型原始张量，也不证明模型训练标签/二进制内部语义已独立校验**。

[现场 YOLO 回收报告](/home/xhj/liftrace-deliverables/board_update_20261010/vision_return/REVIEW.md)说明 FP16/native 输入与 latest worker 为现场执行层改动，B 未合入。本地仅有 RKNN、无对应 PT；本轮未连板、未运行推理或训练。现有 bag 的输出分数不能判断 FP16 转換是否改变了类别结果，**不能认定性能优化导致误识**。历史少量等价测试的原始对照数据已不可复算，也不能替代本次困难帧的等价性检验。

## 有价值的后续模型验证/改善

**先建立这条错误轨迹的困难样本，再做执行路径对照；本轮仅列建议。**

- 按真实地堡标注，重点收集 pillbox 与 panzer/bridge 的混淆图：下缘进入/离开视野、斜视、裁切、暗光、轻微模糊、原靶图不同朝向。147.287 帧地堡 ROI 约 154×142 原图像素，缩到 640 输入后约 77×71；样本应覆盖该实际尺度。保留完整原图、原 header 和可见部分框，不能用历史预测 panzer 标签自动回灌训练。
- 同时保留正确 panzer、真正 bridge、tent/red_cross 对照，检查原训练标注是否把这些图案混标。共同蓝白圆环不是类别真值；过度依赖圆环/背景可能掩盖中心图案差别，这是待验证的模型假设。严重裁切且无法人工辨认的样本标为不可判定，不强行监督为高置信类别。
- 后续具备离线 NPU 条件时，用**相同已解码源图像、相同权重和 SDK**对照原 FP32 提交路径与 native FP16 路径；保持 letterbox/RGB/归一化、后处理完全相同。分别保存输入、原始输出、每类分数/类别排序、NMS 框和 ROI，比较数值差异与 pillbox/panzer/bridge 决策是否改变。先串行离线，不引入 latest worker 跳帧；worker 的调度/帧龄影响另做时间序列对照。没有本地 PT 时不宣称完成导出前模型对照。
- 训练/验证/测试**按整次飞行或连续采集段分组**，同一飞行相邻帧不拆到不同集合；进一步保留不同打印靶、光照/高度的独立测试组，避免同一靶图跨集合形成捷径。九张同 bag 图片只能定位错误，不能作为独立泛化测试或计算可信召回率。
- 评估优先看 pillbox→panzer、pillbox→bridge 的实例级混淆、同物体多类别框，以及错误高位中断是否减少；按物理靶统计，避免连续高频帧或两个 ID 抬高样本数。阈值保持原值，模型前后使用同一批冻结测试帧和相同后处理条件。

## 交付与验证

新增范围仅本报告、[精简摘要](class_confusion_summary.json)、review 拼图和 analysis 中的原图/原始记录。精简摘要约 28KB，原始 probe JSON 5,784,552 字节已移到 `试飞产物/analysis_flight_latest_20261010/class_confusion_probe.json`，全量帧索引和完整中间摘要也留在 analysis。verification 无原图目录；九张原图均在 ignored analysis。

核查来自原 bag、已导出的 data.json 和用户确认；未改原 bag、共享文档、模型、运行代码或视觉门槛，未 commit，未启动 ROS/实机/仿真。遗留是模型原始数值输出对照、更多独立飞行真值与导航记忆冲突机制的维护者复核。
