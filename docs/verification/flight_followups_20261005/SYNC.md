# 本轮同步对象与验证（2026-10-05）

公共源提交：视觉高位 c832f113（释放/控制/接触），98038274（一次续扫/下高恢复）。原profile不自动启用续扫。分支发布记录由高位研究分支 docs/verification/flight_followups_20261005/PUSH_STATUS.json 汇总。

| 分支 | 内容 |
| --- | --- |
| 视觉 feat/high-view-route-speed-20261004 | 本轮源实现、回归与报告 |
| 视觉 feat/high-view-search-research | 高位研究同步 |
| 视觉 feat/board-deployment-flight-20260920 | 九组测试适配，续扫仅第五组/整场显式开关 |
| 视觉 feat/r2026-competition-integrated | 完整控制与硬件入口保留，续扫默认关闭 |
| 导航 feat/high-view-liveness-20260919 | 新控制包/服务/任务链；联调仍依赖对应视觉完整包 |
| 导航 板端参考分支 | 板端专项及公共链同步 |
| 视觉 feat/ev-continuity-20261004 | 独立候选 b2632da2，不默认接入以上入口 |

main、旧VCL06、旧main-integration和历史运动约束工作树未改。没有新分支、没有上板、没有启动SITL。

验证：
- 高位任务层496项（492通过4不适用）；控制62项、旧C++11项通过；释放177项定向通过。
- 整机控制/uav_mission构建通过。构建使用现有系统EmPy，避开本机旧用户目录缓存，未安装新依赖。
- 整机任务层41个模块隔离运行，512项通过。初次整进程运行出现不稳定Python/YAML异常；改为模块隔离、补齐ROS库路径，纯Python契约使用既有conda环境、ROS绑定使用系统Python后完成，不据此宣布初次整进程检查通过。
- 板端专项44项（42通过2不适用），整机专项37项（33通过4不适用）。
- 独立EV32项与构建通过。无本版动态SITL或硬件执行验证。

新增ServoAction服务和释放消息必须成套重新构建；不得只覆盖脚本而沿用旧消息/旧控制二进制。旧Servo.srv及raw硬件接口保留。来源旧控制快照在视觉板端分支f1bd9d3e的legacy_baseline/20261005_async_servo。
