# 设计依据与计算方法

本轮设计日期2026-09-13。下列“已实现/已测量”均限定于对应来源，不能跨分支、场景或设备推广。

## 用户提供的信息

本轮粘贴的师生对话14:43–14:56：老师建议初始升高观测，若找到目标即可减少搜索；提到3 m候选高度、4 m场高，树随机布设，并明确不是必须策略。对话发生日期未额外推定。

此前用户确认纸箱只在树下垫高。高位策略的具体规则适用仍需结合最新书面规则，不把老师建议等同赛委会对“高位环扫+降高”的正式计分裁定。

## 本轮直接阅读的仓库材料

| 资料 | 本轮使用方式 |
|---|---|
| [本分支相机说明](../../CAMERA_AND_FLIGHT.md) | 16 cm光心偏置、坐标方向、历史高度换算与基线参数 |
| [相机K/D标定](../../../vision_ws/src/camera_sdk/param/calibration_1280x720.yaml) | 1280×720、fx/fy/cx/cy，平面投影量级 |
| [RKNN元数据](../../../vision_ws/src/uav_vision/config/merged_standard_6cls_metadata.yaml)及[预处理](../../../vision_ws/src/uav_vision/scripts/target_detector_rknn.py) | 640×640、letterbox缩放0.5，不把填充像素当有效目标尺寸 |
| [r2026类别profile](../../../patrol_uav_ws-patrol_planner/src/uav_mission/config/competition_profiles.yaml) | 当前执行权重与三槽，不变更类别集合 |
| [TargetCandidate](../../../vision_ws/src/uav_vision/msg/TargetCandidate.msg)、[目标记忆](../../../vision_ws/src/uav_vision/scripts/target_memory.py) | 稳定ID、历史地图保留与当下观测字段的区分 |
| [任务核心](../../../patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_core.py) | 0.5 s候选新鲜度、任务阶段、槽位和SEARCH/RESUME语义 |
| [完整低走廊runtime](../../../patrol_uav_ws-patrol_planner/src/uav_mission/config/vcl06_full_low_corridor_runtime.yaml) | 24点、全搜索ROI、600 s、early_return关闭和阶段高度切换 |
| [整机接口](../../INTERFACES.md)、[当前验收](../../VALIDATION.md) | 唯一任务/规划权威、SITL与实机边界 |

本分支飞行源码与上述文件均基于`86e382d`。设计文档提出的是后续改动，没有把拟定子阶段或参数写入这些源码。

## 平行研究和历史资料

这些文档在本机`r2026-coverage-efficiency-research`工作树读取，以已提交`4da9614`固定引用，未将其飞行原型带入本分支。

- [新版规则及技术会议复盘](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/4da9614/docs/competition/RULES_20260906.md)：2026-09-11 PDF19页的既有复盘。本轮复用该复盘，没有重新宣称逐页核验PDF或获取新的官方解释。
- [对应最新PDF](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/4da9614/1789106295928436.pdf)：规则来源，不是本轮新增规则文件。
- [搜索效率计划与28轮历史汇集](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/4da9614/docs/planning/search_efficiency_20260911/PLAN.md)：复访现象、航带与实录口径。
- [地图记忆第一阶段](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/4da9614/docs/planning/search_efficiency_20260911/STAGE1.md)：观测状态不等于自由空间和检出完成。
- [资源报告](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/4da9614/docs/verification/coverage_resources_20260911/REPORT.md)：核心耗时与整节点资源差异。
- [固定树32/34对照](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/4da9614/docs/verification/fixed_tree_efficiency_20260912/REPORT.md)：四轮完整记录、时间退化和0.53–0.59核观察器采样。
- 本机`r2026-board-frame-fix`的`d55da83`及本会话记录：坐标适配/小场配置已开发，动态验证未完成。本分支不继承这些未验收硬件改动。

GitHub地址用于指向已读取的仓库版本，不代表本轮进行了网络资料调研或重新确认官方规则。

## 几何计算复核方法

`geometry.json`保存标定、每个高度的视场/像素量级、全部示例航点及计算结果。数字和图均由已有conda环境NumPy/Matplotlib进行文档计算，不调用ROS/Gazebo、检测器、路径规划器或飞行控制。

- 目标ROI X[-4.3,4.3]、Y[0,7.1]；0.05 m方格中心采样，面积61.06 m²。
- FC AGL H下光心高h=H−0.16。固定yaw=0、零滚俯时，投影到地面：`x=−(v−cy)h/fy`，`y=−(u−cx)h/fx`。实际主点偏心保留。
- 每个直线骨架段按不超过0.025 m的位置间隔取样，将其投影矩形覆盖的格子取并集。
- 原图边界u=[0,1280]、v=[0,720]；裁边敏感性为u=[128,1152]、v=[72,648]。裁边不是经验检测门槛。
- yaw=90°候选将地面偏移旋转为(x,y)→(−y,x)。所有结果为固定航向骨架，不能代替圆角转弯时的姿态重建。
- 0.35 m/1 m像素量级取`0.5×min(fx,fy)×d/h`，不考虑内图案尺度、畸变、靶板厚度、旋转和遮挡。
- 路长仅相邻示例航点欧氏长度和，不包括起飞点接入、爬升下降和绕障。闭环、补中线、往返航带三种形状不保证有相同覆盖，不用单一路长排列判断优劣。

`height_tradeoff.png`和`route_geometry.png`为静态设计图；任何“覆盖率”前都应保留“理想点覆盖”限定，不能用作实测召回或飞行验收。
