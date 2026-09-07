# 2026 竞赛集成路线图

本文件是当前任务优先级来源。R56 最新完整验证为 PASS（37/37），任务 182.924 ROS s，三投/恢复、11 航段、三门、H 对准与落地/disarm完成，零碰撞和零残留。[完整记录](docs/verification/r56_final/REPORT.md)。

0. **当前用户任务：R57错列墙。** 0.85m整场已PASS，现执行严格0.80m验收。每轮独立记录，原R56三框门PASS只代表旧场景。

1. **V-CL-06 已完成当前 SITL 功能闭环。** 门前占据的接触代理自遮挡、FreeDOM 清空尺度/视场/积压问题已修复；原 goal 19 首条可执行轨迹等待 .072 ROS s，本次顺利过门。H 1.60 m 实际取景与降落已验证。历史失败与 R55 INCOMPLETE 保留原结论。
2. **V-SIM 交付。** WSL 原生 logs；只发布完整成功全量记录。精简整机与原始资产集成分支保留，main 按 --no-ff 接收已验证内容并打 annotated tag。没有追加仿真。
3. **后续五项验收。** 固定版本多 seed/启动与负载波动；OrangePi RKNN ROS 10 min 实时链；真实标定/安装/测高；照明/风/载荷/掉帧与遥控急停；机械三槽 ACK、真实落点与落地轮廓余量。

本轮是一次完整配置成功，不据此保证比赛成功率。实时倍率约 .240，板端实时预算仍须实测；真实舵机/PWM 不包含在精简分支，槽位补偿仍暂不启用。

[参数与阈值](docs/verification/r56_final/PARAMETERS.md) · [验证汇总](docs/VALIDATION.md) · [交付说明](docs/verification/r56_final/DELIVERY.md)
