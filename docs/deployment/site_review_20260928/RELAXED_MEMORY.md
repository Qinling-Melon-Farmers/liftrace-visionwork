# 最后一次memory：高度窗口离线对照

[新视频](../../../logs/site_download_20260928/board_memory_only_20260928_224603_replay/dashboard_relaxed_height.mp4) · [原始回放及新视频](../../../logs/site_download_20260928/board_memory_only_20260928_224603_replay/index.html)

原高度窗口为[2.0,2.0]m；对照改为[1.8,2.2]m，实际飞行上限仍为2m。仅复用记录的抽样粗检测与最近先前位姿，经生产ingest_coarse方法计算，测试夹具提供初始状态；TF年龄设为0，因为该抽样日志没有保留此字段。不重跑YOLO，不重新投影，不模拟飞行响应或释放；因此是隔离高度门槛的诊断对照，不是完整机载闭环重放。

截至实际ABORT前，145条观测在旧窗口均被coarse_not_at_high_view拒绝；放宽后42条返回coarse_accepted、103条返回coarse_conflict。这些是观测条数，不是目标数量。

保存了panzer、bridge、red_cross三类假设，同时三类都有冲突位置。saved中出现三类不等于三类均可提前中断或允许投递。视频新增面板显示存储假设及冲突，不改变上方面板的原始任务记录。实际飞机仍因航点规划无进展ABORT，本次重算不证明修复该规划故障。

当前片段结束后的显示为最后快照；没有继续制造新观测。完整视频219.1秒、1倍速，前置准备和接管段均保留。
