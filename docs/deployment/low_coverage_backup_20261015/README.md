# 2026-10-15 低位整场备份交付

导航0928已推送`2f0c932a`，需要先更新这一提交再使用工作台新卡片。当前未上板、未新增仿真/实飞；默认模拟投递。

- 导航仓：`sakelier/liftrace-controlwork`，分支`liftrace_board_trial_20260928`。
- 新入口：`deployment/board_trials_4x4/10_low_coverage_full/start.sh`。
- 默认配置：同目录`settings.yaml`；测量地图参考：`site_waypoint_map_20261015.yaml`。必须确认实际场地，导入/保存地图后再生成任务命令。
- 工作台卡片：**备用10 · 低位遍历整场**，选择**牛耕/回字形**、**FC1.8/1.9m**。默认limited速度0.5m/s、0.35m/s²；运动优化和移动恢复继承08。
- 投递：08与10目标0.25m，08原窗口0.10～0.43m。没有把所有独立正赛/其他专项默认参数一律覆盖。

## 工作台下载

[Windows/WSL同包：保留前线新版UI并加入备用10](flight_workbench_20261015_low_coverage_shapes.zip)

解压后Windows运行包内`start_windows.cmd`，WSL按包内README启动。不要从压缩包预览/临时目录直接运行；旧用户连接信息可以重新填写，分发包不含凭据、会话和录制日志。源码工作台位于本分支`tools/flight_workbench`；此ZIP是在用户前线新版UI上同步同一新卡片和启动修复的完整交付。

[路线/FOV图与详细参数](../../verification/low_coverage_backup_20261015/README.md) · [新增LIO积压分析](../../verification/flight_1015_1656_lio_20261015/REPORT.md) · [当前投递/H模式梳理](../../verification/flight_1015_1501_20261015/CURRENT_DROP_LANDING_MODES.md)

导航相关89+4项测试通过；两份工作台各8项新增回归通过，前线总回归16通过/3依赖跳过；两图案两高度四组生产CLI展开和浏览器9场景通过。下载包不是新的飞行验收结论。

2026-10-15现场默认更新：本页同路径ZIP已替换为完整前线最新版，源功能提交34e589f1。08默认08_full_mission/site_waypoint_map_20261015.yaml（高位2.6m/rectangle），备用10默认10_low_coverage_full同名地图，不勾手动即可选择牛耕/回字及1.8/1.9m；手动override优先。08/10默认competition 1.2m/s、1.0m/s²，H捕获/对准1.1m继承板端。SSH只读probe真实连接通过；现有8771的设备会话保留，常驻后端需协调收尾后重载才能使用最终磁盘版本，不应在运行设备时直接替换进程。板端check-config及部署由主代理负责。