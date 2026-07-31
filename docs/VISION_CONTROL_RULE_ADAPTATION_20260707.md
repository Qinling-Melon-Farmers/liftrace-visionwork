# 现有视觉算法、控制配合与 2026 规则适配说明

> 设计记录：本文保留规则映射和早期接口推演；当前接口所有权、释放证据分层和任务 Gate 以 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md) 为准。

> 2026-07-13 更新：本文保留机制说明；v5 六分类训练和最终增强模型完整链回放已完成。v3 的 bridge/red_cross 重叠来自红十字复用桥梁靶标底座，正式比赛不复用该底座；
> 当前问题优先级、责任归属和历史兼容开关状态，
> 以 [当前问题与责任边界](当前问题与责任边界.md) 为准。

## 1. 目的

本文档用于把当前仓库中“视觉算法本体、与 `patrol_control` 的配合方式、对 2026 规则书的覆盖程度、当前问题与后续验证方向”统一说明清楚。

文档边界固定为：

- 重点说明视觉侧机制与视觉-控制接口
- 说明现机制如何完成规则任务，以及哪些地方仍不满足规则本意
- 搜索部分只写到“视觉侧搜索感知接口责任”，不把完整搜索轨迹和规划策略写成视觉组独立实现

对应规则书见：

- [1779762009913486(1).pdf](/home/xhj/liftrace/1779762009913486(1).pdf)

相关现有文档：

- [VISION_CHAIN_COMPARISON.md](/home/xhj/liftrace/VISION_CHAIN_COMPARISON.md)
- [docs/REAL_TARGET_FULL_CHAIN_EVAL_20260706.md](/home/xhj/liftrace/docs/REAL_TARGET_FULL_CHAIN_EVAL_20260706.md)

## 2. 2026 规则对视觉与控制的直接要求

### 2.1 与 2025 相比的核心变化

规则书当前最关键的两条变化是：

1. 不再给出靶标的大致位置坐标。
2. 避障分只有在全程不发生碰撞时才能得到。

这两条变化意味着：

- 视觉不能只做“到达预设检测点后的末端识别”
- 控制不能只依赖固定航点把目标区域扫一遍后等待视觉确认
- 视觉需要提供候选发现、确认、缓存和触发中断的能力
- 控制/规划需要消化这些视觉候选，形成真正的自主搜索流程

### 2.2 规则任务分解

按规则书，整场任务可分成：

1. 携带三个快递盒起飞
2. 在非障碍区自主飞行、避障
3. 穿越障碍区门洞
4. 自主搜索并找到合适靶标
5. 完成快递投送
6. 完成自主降落

其中视觉直接参与的重点是：

- 标准投放区目标识别
- 随机靶 `red_cross` 识别
- 投递末端圆环对准
- 自主搜索中的候选发现、确认与缓存
- 降落区 `H` 相关标识外圈检测

### 2.3 当前和视觉强相关的规则细节

- 四个标准投放区为 `1m x 1m`，外圈为蓝白圆环，中心图案为帐篷、地堡、桥梁、装甲车、坦克。
- 一个随机靶标为红色十字，尺寸 `0.35m x 0.35m`。
- 投递计分按落点与靶标环位置计分，随机靶按黑色外圈范围计分。
- 规则书明确的权重是：
  - `tent=1`
  - `pillbox=1.5`
  - `bridge=2`
  - `panzer=2.5`
  - `red_cross=10`
- `tank` 在当前规则书权重表中仍未明确。

这意味着视觉系统不能只输出类别，还必须至少支持：

- 圆环/靶标几何中心
- 可用于末端投递的偏差
- 随机靶有效区域相关判定依据

## 3. 当前视觉算法机制

## 3.1 总体结构

当前新链的主路径是：

```text
detector / geometry detector
  -> /uav_vision/detections
  -> detection_fusion.py
  -> /uav_vision/detections_resolved
  -> target_memory.py
  -> /uav_vision/targets
  -> /uav_vision/selected_target
  -> drop_aligner.py
  -> /uav_vision/drop_offset + /uav_vision/drop_ready
  -> patrol_control
```

同时保留旧接口兼容：

```text
/yolo_detect
/detect/cross_status
/detect/tank_status
/detect/waypoint_mark_point
/detect/cross_mark_point
/detect/land_mark_point
```

这说明当前系统不是“旧链已经完全退场”，而是新旧混合态。

## 3.2 标准目标检测

当前 dev/sim 默认粗检测已使用统一六分类模型：

- `bridge`
- `panzer`
- `pillbox`
- `tent`
- `tank`
- `red_cross`

入口：

- `vision_ws/src/uav_vision/scripts/target_detector.py`
- `vision_ws/src/uav_vision/scripts/target_detector_rknn.py`

作用：

- 提供标准目标粗类别和 ROI
- 作为标准投放区候选的主来源
- 可附带检测框中心，但那只是候选框中心，不是最终投递中心

局限：

- v3 历史构造样本曾出现 `H/red_cross` 与 `bridge` 重叠；该现象来自红十字复用桥梁靶标底座，不作为当前正式比赛冲突模型
- 标准目标 detector 本身不负责完成最终投递中心精定位
- 无论旧链还是新链，H 的语义确认和任务阶段门控都不能由 detector 单独完成

## 3.3 `red_cross` 检测

入口：

- `vision_ws/src/uav_vision/src/cross_detector_node.cpp`

核心机制：

- HSV 双红区间分割
- 形态学去噪与连通
- 轮廓面积、点数、长宽比、solidity 验证
- 已加入宽松评分模式，吸收旧 `simple_cross_detect` 的有效特征

当前补充能力：

- 可输出图像域中心、ROI、几何置信度
- 可选黑色外圈检查，但当前默认关闭

当前定位：

- 这是随机靶和高权重目标的关键链路
- 当前不是“完全没有检测能力”
- 但仍未达到“稳定比赛可用”的程度
- 即使 dev/sim 默认 unified detector 已纳入 `red_cross`，完整链仍保留几何红十字检测作为高价值目标的验证来源

## 3.4 投递圆环检测

入口：

- `vision_ws/src/uav_vision/src/circle_detector_node.cpp`

核心机制：

- 蓝色 HSV 分割
- 形态学处理
- 椭圆拟合
- 按半径、长宽比和轮廓面积筛选

输出语义：

- `class_name=circle`
- 图像域中心点与半径

当前定位：

- 用于标准投放区投递末端对准
- 不参与自动搜索目标选择
- 它才是当前标准靶末端投递中心的主要几何来源

## 3.5 `landing_pad(H 外圈)` 检测

入口：

- `vision_ws/src/uav_vision/src/landing_detector_node.cpp`

核心机制：

- 灰度化
- 高斯模糊
- 自适应阈值
- 形态学
- 轮廓筛选与椭圆拟合

输出语义：

- `class_name=landing_pad`
- 本质上是“降落标识外圈几何结果”

当前必须明确的一点是：

- 它更接近“外圈触发器”
- 不是“强 H 结构语义检测器”

这也是为什么当前 `H` 的效果不理想，会表现为：

- 小圈
- 残圈
- 不完整椭圆
- 外圈存在但真实 `H` 语义不够稳

更准确地说，当前 `H` 的主问题不是“漏检主导”，而是“误判主导”：

- 场地内普通靶标本身带黑色外圈
- 当前 `landing_detector` 主要抓的是外圈几何
- 所以普通靶标外圈很容易被误触发成 `landing_pad`

## 3.6 融合与冲突裁决

入口：

- `vision_ws/src/uav_vision/scripts/detection_fusion.py`

当前能力：

- 支持在同帧里保留 YOLO、红十字几何和降落外圈多路观测
- 历史 bridge 裁决开关仍存在，但正式回放和当前规划不再依赖它

输出：

- `/uav_vision/detections_resolved`

历史 `SUP bridge` 的含义是同帧 frame-level 全局 suppress，不是 ROI/overlap 感知 suppress。由于正式比赛不复用该数据底座，它只保留作历史回放和兼容说明，不列为当前优化方向。多路观测是否能影响控制，统一由任务阶段和 `align_mode` 决定。

## 3.7 候选记忆与选择

入口：

- `vision_ws/src/uav_vision/scripts/target_memory.py`

当前机制：

- 3 帧确认
- TTL 超时
- 候选去重
- 优先级排序

输出：

- `/uav_vision/targets`
- `/uav_vision/selected_target`

当前优先级为：

- `red_cross=10`
- `panzer=2.5`
- `bridge=2.0`
- `pillbox=1.5`
- `tent=1.0`
- `tank=1.0`
- `landing_pad=0`
- `circle=0`

这意味着当前真实行为是：

- `red_cross`、`panzer`、`bridge`、`pillbox`、`tent`、`tank` 可以进入 `selected_target`
- 标准目标进入 `selected_target` 只代表候选排序，不代表旧主控会按其地图点自动接近
- `circle` 和 `landing_pad` 也不会进入 `selected_target`

## 3.8 模式化对准

入口：

- `vision_ws/src/uav_vision/scripts/drop_aligner.py`

当前模式：

- `drop_circle -> circle`
- `drop_cross -> red_cross`
- `landing -> landing_pad`
- `disabled`

输出：

- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`

其作用是：

- 把“是否进入末端对准”这件事从检测器里拿出来，交给主控显式决定

## 3.9 当前链中的“发现目标”和“计算中心”是分层的

这一点需要单独写清楚，否则很容易误以为“YOLO 检到目标就已经有了最终投递/降落中心”。

当前真实分工是：

- 标准目标 unified5 detector 负责提供类别候选和 ROI，不负责给出最终投递中心
- `circle_detector` 负责给标准投放区的圆环几何中心
- `cross_detector` 负责给 `red_cross` 的图像域几何中心
- `landing_detector` 负责给 `H` 外圈的图像域几何中心
- `drop_aligner` 根据 `align_mode` 消费这些几何结果，输出 `drop_offset/drop_ready`
- `patrol_control` 再把 `drop_offset` 转成小步世界系对准

因此，当前链不是“发现后不算中心”，而是“中心存在，但默认先停留在图像域，由对准层和主控层继续消化”。

## 4. 当前与控制的配合方式

## 4.1 当前主控仍是航点驱动

`patrol_control` 当前主骨架仍来自：

- `patrol_real.yaml`
- `patrol_sim.yaml`

核心模式仍是：

- `Takeoff_point`
- `Detect_point`
- `Nothing_point`
- `Land_point`

也就是说：

- 当前并不是自由搜索
- 而是依赖预设 `Detect_point` 去接近候选区域

## 4.2 新链接入主控的方式

当前 `patrol_control` 已直接消费：

- `/uav_vision/selected_target`
- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`

并主动发布：

- `/uav_vision/align_mode`

当前已实现的能力：

- `selected_target` 可更新标准目标 `goal`
- `selected_target=red_cross` 可触发十字任务中断
- `drop_offset` 可在主控中转成小步世界系对准点
- `drop_ready` 可作为圆环、十字、降落的末端稳定条件

## 4.3 标准目标链为什么仍是双回路

当前标准目标闭环不是完全由新链直接驱动，而是双回路：

1. `selected_target`
   - 更新 `goal`
2. `/yolo_detect`
   - 仍通过旧 `ClassCallback` 控制 `align_ok`

因此当前标准目标投递的真实链路更接近：

```text
selected_target -> goal 更新
/yolo_detect -> align_ok
align_ok + circle -> Aligning -> drop_circle -> drop_ready -> 投递
```

这说明：

- 新链已经参与“目标选择”
- 但旧字符串接口仍参与“允许对准”

这是一种可联调的过渡态，但不是最终干净结构。

## 4.4 红十字与坦克中断仍是双触发源

当前红十字和坦克任务中断存在两套来源：

1. 新链
   - `selected_target=red_cross/tank`
2. 旧兼容链
   - `/detect/cross_status`
   - `/detect/tank_status`

这说明当前主控不是单纯依赖新视觉总线，而是保留了旧中断语义作为并行来源。

优点：

- 降低迁移期联调失败风险

隐患：

- 可能出现重复触发
- 也可能出现状态耦合与语义分叉

## 4.5 降落链的当前闭环

当前降落闭环是：

```text
Land_point -> 启用 landing detection
landing_pad -> /uav_vision/detections
-> detections_resolved
-> drop_aligner(landing)
-> drop_ready
-> LandDetectDone()
-> 降落
```

但这里仍要强调：

- `drop_ready=true` 代表“当前外圈几何结果稳定”
- 不等于“已经对真实 H 语义做了充分鲁棒验证”

## 5. 当前机制如何完成规则任务

## 5.1 标准靶标投递

当前思路是：

1. 无人机按预设航点到达 `Detect_point`
2. 标准目标 detector 给出候选类别
3. `selected_target` 更新 `goal`
4. `/yolo_detect` 匹配 `goal` 后拉起 `align_ok`
5. 主控进入 `Aligning`
6. `circle_detector + drop_aligner(drop_circle)` 提供偏差与稳定门控
7. 主控下降并投递

这条链当前能工作，但它依赖：

- 已知或预先设计好的检测航点
- 近距圆环精修

因此它更像“2025 路线的升级版”，还不是 2026 意义上的自主搜索投递。

## 5.2 随机靶 `red_cross`

当前思路是：

1. 主任务运行期间持续允许 `red_cross` 候选出现
2. 新链 `selected_target=red_cross` 或旧 `/detect/cross_status` 可触发中断
3. 主控切到 `CROSS_MISSION`
4. `align_mode=drop_cross`
5. `cross_detector + drop_aligner(drop_cross)` 提供偏差与稳定门控
6. 主控完成下降与投递

这是当前最接近 2026 高价值目标中断逻辑的一条链。

## 5.3 自主降落

当前思路是：

1. 到达 `Land_point`
2. 启用 `landing_detector`
3. `drop_aligner(landing)` 输出偏差和 `drop_ready`
4. `LandDetectDone()` 根据稳定结果继续降落

这条链当前能提供“外圈对准式降落”，但对真实 `H` 标识的语义鲁棒性还不够强。

## 5.4 自主搜索

当前只能说“部分具备视觉侧搜索感知能力”，不能说已经完成自主搜索任务。

已有能力：

- 候选发现
- 候选确认
- 候选缓存
- 视觉中断接口

未完成能力：

- 覆盖搜索路径本身
- 基于候选质量的全局搜索策略
- 完整的搜索终止/恢复策略

因此当前应把“搜索”分成两层：

- 视觉组负责：搜索感知接口
- 控制/规划联合负责：搜索执行策略

## 6. 当前设计的合理性

## 6.1 合理之处

1. 算力路线合理
   - 粗检测走统一 detector / 未来 RKNN
   - 精修走传统几何
   - 符合 OrangePi 5 Plus 的资源约束

2. 新接口方向正确
   - `detection_fusion`
   - `target_memory`
   - `drop_aligner`
   把旧散装视觉链收成了统一语义总线

3. 兼容策略务实
   - 保留 `/yolo_detect` 和旧 `/detect/*`
   - 能在重构过程中维持主控联调

4. `red_cross` 已具备 unified detector 粗发现 + 几何链复核的双来源
   - 这比“只靠旧五分类”更符合当前数据资产和 2026 搜索需求

## 7. 当前主要隐患

## 7.1 结构性隐患

1. 搜索仍依赖固定 `Detect_point`
   - 与 2026 “自主搜索”要求存在根本差距

2. 标准目标链仍是双回路
   - `selected_target` 与 `/yolo_detect/align_ok` 并存
   - 存在语义分叉风险

3. 红十字/坦克中断仍是双来源
   - 新链触发和旧链触发并行
   - 容易产生重复触发和状态耦合

4. 标准目标候选与旧任务控制仍未闭环
   - `bridge` 当前配置优先级已为 2.0，可进入 `selected_target`
   - 旧主控仍不会依据标准目标地图点自动接近并恢复搜索

## 7.2 算法性隐患

1. `cross_detector` 的黑色外圈检查默认关闭
   - 对随机靶有效区域的判定支撑不足

2. `landing_detector` 更偏外圈几何检测
   - 不是强 `H` 结构检测
   - 这正是当前 `H` 效果不理想的主要原因

3. 旧检测 flag、旧 `/detect/*` 回调与新 `align_mode` 并行
   - 同一帧多路观测可能被不同阶段误消费
   - `LandDetectDone()` 超时直接成功，不能作为 H 确认

4. `drop_offset` 的像素到米制投影仍是近似法
   - 当前更偏工程上可用
   - 不是严格几何投影闭环

## 8. 当前最关键的问题：`H` 与 `red_cross`

## 8.1 `red_cross`

根据当前实录视频与离线评测结论：

- `red_cross` 不是完全检测不到
- 当前主问题是：
  - 稳定性不足
  - 残留误判与黑色有效区验证不足
  - 与降落/投递任务阶段的控制语义需要解耦

## 8.2 `H(landing_pad)`

当前更弱的是 `H` 相关链路。

原因不是单一阈值没调好，而是机制层面就偏弱：

- 当前链主要盯的是外圈
- 不是内外联合语义验证
- 真实画面中容易出现：
  - 小圈
  - 半圈
  - 残圈
  - 亮度导致的边界断裂

因此当前更准确的结论是：

- `landing_pad` 当前具备外圈触发能力
- 但还不足以称为“真实比赛条件下稳定的 H 检测能力”

## 8.3 为什么不能只靠 unified detector 解释问题

当前 unified detector 的历史问题是：

- 旧五分类和 v3 构造样本中，部分 `H/red_cross` 场景与 `bridge` 发生混淆
- 新六分类后，`red_cross` 误报已明显下降；当前不再把 bridge 抑制作为主线

但这不是全部问题。

即使历史 bridge 混淆已经不再是正式场景，`H` 本身仍有独立问题：

- 外圈拟合质量不稳定
- 真实 `H` 语义没有被强验证

所以当前对 `H` 的优化不能建立在 bridge 抑制之上，而应落实为 H 结构验证、阶段门控和安全回退。

## 9. 下一步开发与验证方向

## 9.1 近期重点不是继续堆实现，而是先把证据和边界写清

当前更合理的第一优先级是：

1. 技术文档收口
2. 问题证据分类
3. 验证路径固定

而不是立刻进入新一轮工程实现。

## 9.2 视觉组近期应负责的内容

1. 把视觉侧搜索感知接口写清楚
   - 候选发现
   - 候选确认
   - 候选缓存
   - 候选失效
   - 中断触发条件

2. 把 `H/red_cross` 问题样本归类
   - `H` 漏检
   - `H` 小圈/残圈/不完整圈
   - `red_cross` 漏检
   - 同帧多路观测的阶段误消费
   - H 语义误触发降落

3. 固定一套回归证据集
   - 当前链 `real_target` 证据
   - 旧链风格对照证据

## 9.3 后续工程方向优先级

在文档和问题分类完成后，更合理的工程优先级应为：

1. 以显式状态机隔离 `landing` 与 `drop_circle/drop_cross`
2. 复用旧链投影与几何中心思路，完成地图记忆和投递靶心
3. 完成标准目标—圆环实例关联与关联失败拒绝
4. 再做外部仿真与板端 RKNN 验证闭环

## 9.4 当前验证顺序建议

建议验证顺序固定为：

1. 最终增强模型的 `real_target`/`redcross` full-chain 结果已归档，后续修改继续以其为回归基线
2. 完成 H 外圈、H 结构、黑色有效区和同帧多路观测的样本分类表
3. 用外部 `PX4 + Gazebo + iris_mid360` 做接口烟测
4. 再进入地图投影、投递靶心和状态门控实现

## 10. 结论

当前系统已经不再是“只有旧 2025 视觉链”，而是一个“2025 航点驱动主控 + 2026 风格新视觉总线”的混合系统。

它已经能提供：

- 标准目标粗分类
- 红十字中断
- 圆环末端对准
- 降落外圈对准
- 候选缓存与统一视觉接口

但它距离完整满足 2026 规则，还差三类关键能力：

1. 真正的自主搜索闭环
2. `H` 链的稳定比赛可用性
3. 新旧双回路语义的进一步收敛

因此当前最务实的结论不是“立刻重构全部工程”，而是：

- 先把现机制、边界和证据写清楚
- 先把 `H/red_cross` 问题分类做扎实
- 再决定后续算法和工程优先级
