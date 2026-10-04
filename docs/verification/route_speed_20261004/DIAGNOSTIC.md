# 正式矩阵前的同步缺口（2026-10-04）

5e72b732的旧矩形31在ROS23.404s、首个高位粗线索进入时ABORT：`navigation_hints_exception:AttributeError`。高位研究已集成的HighViewFull.ingest_coarse引用policy.high_min_agl，但SurveyPolicy未同步462d33e0添加的字段及catalog传参。此为分支配套遗漏，非路线避障或直线权重导致（该轮优化关闭）。

已同步该已有修复；默认仍2.0m观测下限，无额外阈值放宽。先前31诊断完整保留，紧随的rectangle31启动在起飞前被停止；两者不计正式八轮、不用于节时。补齐后所有八轮从统一新版本重新开始。批次异常检查增加发现callback异常后停批，避免重复触发已知接口故障。

诊断目录：logs/route_speed_20261004_diagnostic_batch。原始run仍保留各时间戳。

修复后完整任务392项检查通过（4个整机前端专测跳过），高位视觉99项通过；两入口脚本执行权限恢复，研究默认膨胀与现用0.25m统一。
