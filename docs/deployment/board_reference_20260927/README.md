# 板端参考分支：八组专项交接

> 2026-09-29：八组共用入口已改为定位一致后才启动空白FreeDOM，完整视觉链持续输出后才READY；保留静态TF及25/20/10cm膨胀。[启动修复、高位门槛与昨晚记忆轮结论](../mapping_startup_20260929/REPORT.md)。仅离线验证，尚未上板实测。

2026-09-27。本轮按用户要求对齐现场负载/建图配置，八组水平膨胀统一为 **0.25m**。参考入口包含匹配版本的导航、视觉、控制、专项脚本和场景，不要求试飞组另从研究分支拼接代码。

## 仓库与维护方式

- 导航组参考分支：[sakelier/liftrace-controlwork → 板端参考分支](https://github.com/sakelier/liftrace-controlwork/tree/板端参考分支)。以视觉仓八组测试整套源码创建，供试飞组拉取、比较和后续采纳。
- 视觉仓维护分支：[feat/board-deployment-flight-20260920](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/tree/feat/board-deployment-flight-20260920)。首次发布两个远端指向同一提交，避免消息/控制/策略版本拼接错误。
- 现场调参来源：导航组 `板载代码@48541a7a8d91a2993711a10b556f7028504c3e1c`。此次不覆盖该分支，不修改任一main，也没有上传或运行实机。
- 视觉实现仍以视觉仓为来源；参考分支不是将视觉源码改为两处独立维护。后续现场调整记录来源提交，按文件选择回流。

本参考分支是完整专项集成版本，**不是以现场“板载代码”为基础只贴几个YAML**。该分支上的普通旧入口、历史仿真报告不能替代下方八组入口。导航组原有“板载代码”里的 `minimal_delivery_test.launch/high_view_priority_search.launch` 不属于本交付的启动接口；不要跨分支复制它们与本地新消息/控制器混用。

## 此次对齐的项目

公共配置：[board_load.yaml](../../../deployment/board_trials_4x4/common/uav_board_trials/config/board_load.yaml)。它在机型标定及FreeDOM共用默认值之后加载，不修改MID360、IMU标定或相机外参。

| 配置项 | 原八组 | 当前八组 |
| --- | --- | --- |
| FAST-LIO特征提取 | false | true，与板载测试相同 |
| FAST-LIO局部地图边长 | 1000m | 20m，与板载测试相同 |
| 点过滤 / 最大迭代 | 3 / 3 | 保留3 / 3 |
| surf/map滤波大小 | 0.15 / 0.15m | 保留 |
| FAST-LIO det_range | 100m | 6m，配套20m地图，见下节 |
| FreeDOM sub_voxel_size / voxel_depth / block_depth | 0.1m / 0 / 默认 | 0.1m / 2 / 5，与板载测试相同 |
| FreeDOM清空/恢复计数 | 6 / 20 | 保留 |
| FreeDOM范围 | 15m、Z −3至5m | 按低位/高位/走廊三档，见下表 |
| 水平/上/下膨胀 | 0.275 / 0.20 / 0.10m | **0.25 / 0.20 / 0.10m** |
| 重访排序代价图水平膨胀 | 0.275m | 0.25m，与专项名义值一起更新 |

FreeDOM档案在启动定位时选择，sensor与raycast范围成对设置，Z裁剪相对传感器；不会在飞行中修改只在构造时读取的FreeDOM参数。

| 档案 | 对应专项 | max_range | min_z / max_z | 来源 |
| --- | --- | ---: | --- | --- |
| low | 01视觉中断、03 H、05低位多投 | 6m | −1 / 2m | 现场最简投递 |
| high | 02完整圈、06提前中断、07只记忆、08完整任务 | 6m | −1 / 3.2m | 现场高位入口 |
| corridor | 04走廊/H | 5m | −1 / 1.5m | 现场走廊入口 |

08全程保留high档，以覆盖高位搜索及后续环节，不在三投后重启地图。03的H爬升仍为AGL1.8m；表中Z是传感器相对裁剪范围，不是飞机任务高度上限。板端和同链仿真均选择同名档案；仿真仍使用PointCloud2 lidar_type=4，板端仍用driver2 CustomMsg lidar_type=1。

### 为什么det_range没有照搬100m

在 `FAST_LIO/src/laserMapping.cpp::lasermap_fov_segment()` 中，距边界≤`1.5*DET_RANGE`就触发局部地图搬移。20m立方体中心到边界仅10m，若保留100m，静止中心也满足10≤150，并计算出50m搬移步长，明显不适合作为20m配置的配套值。

因此保留现场20m地图，但将这项局部地图阈值配套设为6m：中心距离10m大于9m，运动触边时搬移步长为3m。没有修改FAST-LIO算法、没有更改雷达硬件量程。这是依据源码的参数一致性修正，**不等于已证实该问题造成现场历史停滞**。

### 0.25m的准确含义

当前SDF分辨率仍为0.10m，代码按 `ceil(inflation/resolution)` 膨胀，0.25与此前0.275均为3格，约0.30m轴向扩展。因此本次名义值已按要求调整，不能把它宣传为窄门实际空隙立即增加5cm。没有更改55cm工程碰撞包络、体素分辨率或三维膨胀算法。

SDF继续使用适合4×4专项的10×6×3.8m地图、2Hz显示和4.5/3/3m局部更新半范围；不照搬远端7.2m长搜索路线的18×6×4m地图。新的0.25m最终覆盖在共享研究默认参数之后，planner、重访排序以及preview/flight/仿真入口均检查一致。

FreeDOM `voxel_depth=2` 会恢复现场较粗的空闲网格行为；它与树冠中部柱是不同处理层。地面杂点、门边清空效果、实际CPU/内存负载仍需下一轮现场数据，不能用此前voxel_depth=0的通过录像代替验收。

## 仍然保留的设计

- 现场单位静态 `map→camera_init` 和双向数值适配；已知机架外参、相机q=[0,1,0,0]、像素矩阵[-1,0,0,1]、三个不同槽位偏移。
- 全部虚拟顶棚关闭，包括高位切低位及最后投递切走廊；任务与指令限高继续保留。
- 高位02/06/07/08使用修正后的中部障碍柱，保留局部连通分量限制和有限填充；不继承远端旧全高柱或2.88m顶棚。
- 巡航上限0.5m/s、加速度0.35m/s²，低位AGL1.4m、高位2.6m、投递默认AGL0.60m，自动采集静置地面参考。
- 最新轨迹进度/FSM恢复、粗线索与低位确认、近墙末端边界约束；不退回旧轨迹消息。
- 默认模拟投递，01/02/05/06/08有显式实投入口；原有raw舵机服务 `/legacy/Servo_raw` 由现场启动，专项不启动PWM驱动。
- 相机原片/视觉叠加/任务与轨迹记录继续开启；新实测FOV尚未替换CameraInfo，本轮没有修改相机内参。

此处只调整八组专项的公共覆盖层；研究分支、原4×4历史镜像和原始仿真结果保留各自版本。远端最简/高位默认capture_height与auto_land_height的冲突不复制进八组，也没有在本轮修改试飞组原分支。

## 八组入口

| 序号与目录 | 流程 | 结束方式 | 档案 |
| --- | --- | --- | --- |
| [01_visual_interrupt](../../../deployment/board_trials_4x4/01_visual_interrupt/README.md) | 低位直飞、视觉中断、对齐、一投 | 恢复后原地自动落 | low |
| [02_high_view_revisit](../../../deployment/board_trials_4x4/02_high_view_revisit/README.md) | 低位入场、高位完整一圈、冻结1–3目标、逐个重访投递 | 最后目标附近自动落 | high |
| [03_h_landing](../../../deployment/board_trials_4x4/03_h_landing/README.md) | 前方约2m的H、低位接近、定点升高识别对齐 | 自动降H | low |
| [04_corridor_landing](../../../deployment/board_trials_4x4/04_corridor_landing/README.md) | 实测引导点＋在线避障＋末端H | 自动降H | corridor |
| [05_low_multi](../../../deployment/board_trials_4x4/05_low_multi/README.md) | 低位中断、投递、恢复，默认两投，可设1–3投 | 完成计数后原地落 | low |
| [06_high_priority](../../../deployment/board_trials_4x4/06_high_priority/README.md) | 高位TOP3支持成立提前退出，低空重访 | 已冻结目标完成后末目标附近落 | high |
| [07_memory_only](../../../deployment/board_trials_4x4/07_memory_only/README.md) | 完整高位一圈、形成记忆、下降；零APPROACH/投递 | 原地落 | high |
| [08_full_mission](../../../deployment/board_trials_4x4/08_full_mission/README.md) | 高位先搜、重访/必要补搜、三投、走廊、H | 自动降H | high |

04/08的实测航点和H位置故意留空，未填时拒绝启动。八组名义工作区4×4m，起飞边缘中点，+X初始机头向前、+Y向左。02与06不是同一种高位停止条件，07也不会调用投递机构。

## 拉取与构建

在香橙派新目录中拉取参考分支，避免覆盖现有现场工程：

```bash
git clone --single-branch --branch '板端参考分支'   https://github.com/sakelier/liftrace-controlwork.git liftrace-board-reference
cd liftrace-board-reference
OPENCV_CMAKE_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4   BUILD_JOBS=2 bash top_level_scripts/build_competition.sh
export UAV_VISION_RKNN_MODEL_PATH=/现场已有模型目录/merged_standard_fp32.rknn
```

该分支已带匹配的两个workspace及相对符号链接，**无需再拉视觉研究分支拼包**。依赖仍使用板端已安装的ROS Noetic、OpenCV4.2、Livox SDK2/driver2、RKNN Lite2；模型权重、SDK安装、设备串口配置、真实舵机驱动、bag/录像及笔记本build/devel不随Git交付。FAST-LIO/FreeDOM和视觉均从本checkout构建，不与旧工程overlay混用。

板端先启动现有MAVROS、driver2、相机/CameraInfo。首次预览示例：

```bash
bash deployment/board_trials_4x4/01_visual_interrupt/start.sh preview
```

退出预览后，现场按既有试飞流程选择 `start.sh flight`；仍默认模拟投递。实投仅使用有该入口的模块 `start_real.sh`，复用现场接线并经过释放许可，不需新增舵机实现。flight READY仅表示链路就绪，不自动解锁或调用任务启动。完整步骤见[共同操作说明](../../../deployment/board_trials_4x4/README.md)。

录像写入该工程 `logs/board_<专项>_<时间>/`，收尾有 `result.json`、`index.html`、raw/annotated视频、事件/轨迹及实际高度参考；默认5fps、640宽，不录全量bag。`ground_reference.json`新增所选mapping_profile，便于回查。源码中的旧仿真视频链接需要额外取得对应logs，clone不会下载视频。

## 本轮验证与限制

- 24项模块回归通过。
- 8组×preview/flight＝16个板端应用入口，以及配套16次定位展开通过；检查实际最终参数、profile、driver2机型标定、静态TF、顶棚、膨胀、mock隔离和任务runtime构造。
- 8组同链仿真入口离线展开通过；检查相同负载/建图档案与0.25m膨胀，同时保留模拟雷达类型及模拟投递。
- 两workspace构建：通过（笔记本WSL，视觉及导航工作区，BUILD_JOBS=2）。
- 本轮没有启动Gazebo、ROS仿真或实机。此前6PASS/2INCOMPLETE属于本次改参前的版本，见[历史八组录像/报告](../../verification/board_modules_20260927/REPORT.md)，不能写成新配置已经动态通过。

下一步按01/03→05→07/02/06→08做现场逐项验收，优先看点云规模/更新延迟、定位稳定性、候选与释放事务。当前提交的构建和离线验证足以证明入口参数连通，不证明实际板端负载、窄门轨迹或实物投递精度。

## 2026-09-28 后续：取消panzer特判

八组当前生成配置统一`interrupt_refined_classes: []`。06/08可凭两帧一致的panzer粗线索参与TOP3提前结束；02/07仍完成各自完整环线。低空复核与释放许可不变。本次仅离线验证，之前六组结果不代表取消后的动态验收。完整[策略、逐类退化和增强明细](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/high-view-search-research/docs/planning/panzer_five_class_20260928/VALIDATION_AND_AUGMENTATION.md)。

本轮视觉策略来源：`liftrace-visionwork / feat/high-view-search-research@efbc8fec1f42ff2e050811449ea9aaee8b493650`。
