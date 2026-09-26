# 高位粗线索与低空确认间隔（2026-09-26）

本轮按用户要求放宽搜索准入，范围为高位研究分支及其导航 feature；不更改试飞组“板载代码”分支，不运行仿真或实机。

## 接口约定（视觉→导航）

- 新增可配置 /uav_vision/navigation_hints，类型沿用 TargetDetectionArray；只由现有 target_map_projector 的可选粗投影回调发布，不新增节点或状态机。
- 输入为 target_detector 的原始类别框；使用源图像时间戳、CameraInfo、TF 和地面平面计算框中心的粗坐标。无需同帧圆环或红十字几何。
- 数组 source=coarse_navigation_projector；center_source=bbox_navigation_only；center_refined、geometry_verified、association_valid 均为 false。map_valid 仅表示粗射线投影有效。该话题不能接入 target_memory、drop_aligner 或 release_evidence。
- 只用于高位 SURVEY 的 REVISIT_HINT_ONLY：置信度≥0.60、源图像年龄≤0.5秒、TF年龄≤0.1秒、当前位姿有效且已达高位观测高度，单次检测即可记忆。粗坐标规划不确定度暂按0.45米，属于待仿真/实拍验证的配置假设，并非测得误差上界。
- 复用现有有限 NavigationMemory（至多五类，冲突每类至多两处）；类别/位置冲突仍转低空复核。精修线索优先于粗线索。粗、精内部键区分来源，不冒充投递候选ID。
- 三个高权重类别均有无冲突线索时，沿用原有高位中断→受避障约束下降→逐点重访。低空仍须取得新鲜正式候选，才能进入APPROACH/ALIGN和投递事务。

## 低空确认

- 保留三次实际合格检测；仅搜索模式 disabled 下，将相邻合格源图像时间间隔放宽至1.0秒，中间空帧不立即清零。
- 每个有效命中仍需原有类别、几何、关联、地图和年龄检查；未命中的当前帧 map_valid=false，不用历史坐标伪装新观测。
- 超过1.0秒再命中重新计数；同一源图像重复/倒序不增加计数。进入对准/降落模式时清空短期计数，重新取得连续精修证据。长期物理ID和位置保留。
- 发布消息 consecutive_observe_count 在启用该搜索选项时表示“允许指定漏检间隔的命中串”，不表示逐帧连续；默认关闭时保持原语义。导航只在搜索/重访准入使用，释放链仍使用独立的新鲜对准证据。

## 启用边界

底层新增选项默认关闭，仅 full_strategy.launch 显式开启；对准阈值、稳定帧数、槽位补偿、释放许可和舵机反馈不变。需要在下一次获授权的仿真中观察粗线索误报/错关联、重访成功率及完赛时间；离线测试不能替代实跑验收。


## 本轮离线验证与复跑结论

已覆盖：单帧粗线索只进入导航记忆、三线索触发下降但无投递候选、错误帧/时间/高度拒绝、同类两位置冲突、异类同位置冲突、精修替代粗线索、重复帧去重、漏帧累计、模式切换后重新确认。
19:12实飞包原始检测与822幅同步图像重新推理对照均显示：去程只有一次panzer检出，允许1秒间隔仍无法CONFIRMED；返程PT重推理由52.495秒提前到52.397秒，但任务已RETURN_HOME，因此不触发APPROACH。没有用这份低空包冒充高位策略验收。

导航改动位于 liftrace-controlwork 的 feat/high-view-liveness-20260919；配套 uav_high_view/uav_vision 和回放工具位于 liftrace-visionwork 的 feat/high-view-search-research。导航仓旧视觉副本未整目录覆盖，联合运行必须使用匹配的视觉研究 overlay。

导航来源：e92f1f3ff08fd611f24023354fe9619a05937b46（liftrace-controlwork / feat/high-view-liveness-20260919），仅逐文件集成导航入口、策略及新增测试。

## 完整工作流与阈值

修改前后全链、launch覆盖值及检测/融合/确认/高位/释放/H的完整阈值见[WORKFLOW_AND_THRESHOLDS.md](WORKFLOW_AND_THRESHOLDS.md)。本次已离线展开完整入口，特别区分对准5次与基础3次、搜索间隔1秒与证据年龄0.5秒，以及既有释放承诺的下降期约束。
