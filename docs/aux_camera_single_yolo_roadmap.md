# 辅助摄像头主动搜索架构升级路线图

## 1. 文档目标

本文用于指导视觉组辅助摄像头主动搜索架构的后续开发。

当前系统已经具备：

* 斜下辅助相机仿真模型；
* 辅助候选检测；
* 候选坐标地图投影；
* 候选记忆；
* 动态目标访问；
* 下视视觉复核；
* 覆盖搜索 fallback。

后续目标不是继续增加独立视觉链，而是逐步收敛到：

> 单 YOLO 推理实例 + 双摄像头输入 + 辅助粗发现 + 下视精确认。

最终形成：

```
辅助相机
    |
    |-- OpenCV/YOLO 粗发现
    |
    ↓
Candidate Memory
    |
Mission Manager
    |
动态目标访问
    |
下视相机
    |
同一 YOLO Runtime
    |
严格视觉确认
    |
任务完成
```

---

# 2. 当前架构状态

## 2.1 已完成能力

当前辅助搜索分支已经实现：

### 辅助候选生成

辅助视觉产生：

* 粗目标位置；
* 地图坐标；
* 置信度；
* 时间戳。

候选进入辅助目标记忆模块。

---

### 候选记忆

目标不会因为单帧检测立即触发导航。

流程：

```
Detection
    |
地图投影
    |
候选聚类
    |
连续观测
    |
Aux Candidate
```

作用：

* 消除单帧误检；
* 平滑坐标；
* 合并同一目标多次观测。

---

### 动态目标访问

当前采用：

```
Coverage Search

        ↓

发现辅助候选

        ↓

生成临时目标点

        ↓

Fast-Planner执行

        ↓

下视确认

        ↓

继续搜索或恢复Coverage
```

即：

辅助目标属于动态任务目标，而不是直接修改原始覆盖航点。

---

# 3. 第一阶段：稳定当前辅助主动搜索闭环

目标：

先不改变视觉结构，确认主动搜索思想成立。

## 任务

保持：

* 当前辅助检测；
* 当前Candidate Memory；
* 当前动态目标访问；
* 当前fallback逻辑。

完成：

```
辅助发现
 ↓
记录坐标
 ↓
导航访问
 ↓
下视确认
```

闭环。

---

## 开发内容

### 1. 固化接口

统一辅助候选消息：

```
TargetCandidate

字段：

id
position
source
confidence
timestamp
status
```

建议：

```
source:

AUX_CV
AUX_YOLO
DOWNWARD_YOLO
```

提前为后续多来源融合准备。

---

### 2. 完善候选状态

增加：

```
DETECTED

APPROACHING

VERIFYING

CONFIRMED

REJECTED
```

状态转换：

```
Detected
    |
导航访问
    |
Approaching
    |
下视确认
    |
Confirmed
```

---

## 测试

完成：

* 固定世界运行；
* 多次重复搜索；
* 记录：

```
搜索时间
路径长度
发现数量
fallback次数
```

目标：

证明：

辅助候选访问不会破坏原任务闭环。

---

# 4. 第二阶段：辅助视觉接口抽象

目标：

让辅助来源可以自由替换。

当前：

```
Aux Blue Detector
        |
        ↓
Candidate
```

升级为：

```
Aux Proposal Provider

        |
        |
-----------------
|               |
OpenCV       YOLO
Blue Ring    Semantic
-----------------

        |
        ↓

Candidate Memory
```

---

## 新增模块

创建：

```
aux_proposal_provider
```

统一输出：

```
Candidate:

position
confidence
source
class_hint
uncertainty
```

上层导航不关心：

* OpenCV产生；
* YOLO产生；
* 其他算法产生。

---

# 5. 第三阶段：辅助相机加入YOLO粗检测

目标：

解决OpenCV只能发现蓝环的问题。

引入：

辅助相机 YOLO 推理。

用途：

辅助相机负责：

* red_cross粗发现；
* 标准目标类别提示；
* 蓝环检测辅助。

注意：

此阶段仍然允许：

```
辅助YOLO
+
辅助OpenCV
```

同时存在。

---

## 推理结构

```
Aux Camera

    |
    +------ OpenCV
    |
    +------ YOLO

            |
            ↓

     Candidate Fusion

            |
            ↓

      Mission Manager
```

---

## 融合规则

辅助阶段：

采用高召回：

```
OpenCV发现
OR
YOLO发现

=
候选目标
```

不要要求两个同时成立。

---

下视确认阶段：

保持严格：

```
YOLO分类

+

几何验证

+

连续帧确认
```

---

# 6. 第四阶段：单 YOLO 双输入架构

目标：

减少板端资源占用。

最终结构：

```
             AUX Camera
                 |
                 |
             Frame Scheduler
                 |
                 |
DOWN Camera ---->|

                 |
                 ↓

          Single YOLO Runtime

                 |
        ----------------
        |
     Detection Result

```

---

## 核心原则

不是：

```
Camera A
    |
YOLO A


Camera B
    |
YOLO B
```

而是：

```
Camera A
Camera B

共享：

一个模型

一个推理上下文

一个NPU资源
```

---

# 7. 单 YOLO 调度策略

## SEARCH阶段

目标：

快速发现。

策略：

```
辅助相机:

高频输入


下视:

低频或关闭YOLO输入
```

例如：

```
AUX:
5-10Hz

DOWN:
0-2Hz
```

---

## APPROACH阶段

目标：

完成视觉交接。

策略：

```
AUX:

继续提供方向


DOWN:

逐渐增加检测频率
```

---

## VERIFY阶段

目标：

精确认。

策略：

```
DOWN:

最高优先级

AUX:

停止
```

---

# 8. Camera Handoff设计

新增状态：

```
CAMERA_HANDOFF
```

流程：

```
AUX发现目标

        ↓

接近目标

        ↓

AUX + DOWN交替检测

        ↓

DOWN稳定检测

        ↓

关闭AUX
```

避免：

* 相机切换盲区；
* 目标丢失；
* 时间同步问题。

---

# 9. 红十字处理路线

红十字不能依赖蓝环辅助。

建议：

## 第一层

辅助YOLO：

```
red_cross suspect
```

产生粗候选。

---

## 第二层

下视确认：

```
red_cross detection

+

geometry

+

连续帧
```

形成正式目标。

---

## 第三层

如果辅助YOLO效果不足：

增加轻量提示：

```
红色区域

+

黑色外环

```

只作为proposal。

不作为最终确认。

---

# 10. 全随机世界测试路线

固定世界：

用于：

* 调试；
* 回归测试。

随机世界：

用于：

* 验证泛化。

---

## 随机因素

包括：

* 标准投放区随机位置；
* 红十字随机位置；
* 目标旋转；
* 边界距离；
* 遮挡情况。

---

## 对比组

保持同一随机seed：

```
Baseline:

完整Coverage


Aux:

主动搜索
```

比较：

* 搜索时间；
* 飞行距离；
* 完成率；
* fallback次数。

---

# 11. Corner Case测试

重点测试：

## Case 1

辅助看到全部目标

验证：

最大收益。

---

## Case 2

辅助看到部分目标

验证：

是否仍优于Coverage。

---

## Case 3

辅助完全没有发现

验证：

是否退化为Baseline。

---

## Case 4

红十字不在辅助视场

验证：

不会因为辅助策略漏掉高优先级目标。

---

# 12. 推荐最终生产架构

最终收敛：

```
                AUX Camera
                    |
          ---------------------
          |                   |
       OpenCV              YOLO
          |                   |
          ---------------------
                    |
             Candidate Memory
                    |
             Mission Manager
                    |
          Dynamic Goal Switch
                    |
              Fast-Planner
                    |
              DOWN Camera
                    |
              Same YOLO
                    |
          Final Verification
                    |
              Mission Finish
```

---

# 13. 实施顺序

推荐开发顺序：

## Step 1

完成当前辅助主动搜索闭环。

目标：

跑通。

---

## Step 2

抽象 Aux Proposal Provider。

目标：

允许CV/YOLO替换。

---

## Step 3

加入辅助YOLO。

目标：

解决红十字粗发现。

---

## Step 4

实现单YOLO双输入调度。

目标：

降低资源占用。

---

## Step 5

随机世界验证。

目标：

证明泛化能力。

---

# 14. 2026-08-20 落地状态与 Gate 划分

本路线图的阶段顺序继续有效，但当前工程状态需要进一步细分，避免把视觉交接通过与完整
导航任务通过混为一谈。

## Step 1 当前完成度

已经在 `feat/oblique-active-search-stability` 分支补齐：

* 辅助候选稳定 ID；
* `AUX_CV` 来源记录，后续可扩展为 `AUX_YOLO`；
* `DETECTED → APPROACHING → VERIFYING → CONFIRMED/REJECTED` 生命周期；
* 当前辅助候选与下视候选的地图距离关联；
* 只接受复核阶段新产生、未过期的下视观测；
* 抵近无进展和复核超时后的拒绝/fallback；
* 搜索、交接、返航、释放安全四个子 Gate 分离记录。

当前只在 `uav_vision_eval` 隔离外挂内验证这些字段，没有擅自修改公共
`TargetCandidate` 消息，也没有修改正式 Mission Manager、Fast-Planner 或旧控制链。

## 固定世界实跑结论

1. 默认 5 候选触发轮：
   `logs/oblique_active_search_step1_smoke_v2_20260820_160802/` 整体 PASS；稀疏扫描直接由
   下视链发现五类，搜索 152.701 s、路径 22.769 m，辅助只形成 2 个 `DETECTED` 候选，
   没有触发交接。两次所谓不可达已改名为事实准确的 `goal_progress_timeout_25s`。
2. 2 候选强制交接轮：
   `logs/oblique_active_handoff_step1_smoke_20260820_162021/` 中两个被访问候选均由下视确认，
   分别关联 `pillbox`（0.110 m）与 `tent`（0.313 m）；随后 fallback 找齐五类，搜索完成
   时间 316.499 s，零 Servo/释放输出。完整任务因返航阶段 Fast-Planner
   `run out of memory / generate new traj fail` 撞到 1000 s 墙钟而 FAIL。

因此可得出的结论是：候选生命周期和辅助→下视交接机制已经成立；“过早用 2 个候选中断”
在本固定世界没有节时收益；完整主动任务稳定性尚未通过。返航规划失败单列为规划侧结果，
不篡改为视觉失败，也不在视觉分支调整规划器。

## 下一完成定义

Step 1 还需在固定世界完成至少 3 轮无 GUI 重复，比较默认/候选触发策略的搜索完成率、
净搜索时间、净路径、交接确认/拒绝和 fallback。只有确认触发策略不会降低完成率，才进入
Step 2 的 `Aux Proposal Provider`；随后再依次推进辅助 YOLO、单 YOLO 双输入和全随机世界。

---

# 总结

最终目标不是增加更多视觉模块，而是形成：

```
轻量粗发现

        ↓

动态目标访问

        ↓

高精度下视确认
```

的主动搜索体系。

辅助摄像头负责扩大搜索范围和减少无效飞行；

下视摄像头负责最终决策；

单YOLO双输入负责控制板端资源。

这是当前无人机任务中更适合工程落地的视觉架构。
