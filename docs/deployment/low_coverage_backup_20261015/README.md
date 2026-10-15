# 2026-10-15 现场工作台完整交付

[Windows/WSL完整工作台ZIP](flight_workbench_20261015_low_coverage_shapes.zip)

本包保留完整前线UI、默认地图preview兼容修复及低位搜索地图显示。解压到独立目录后Windows运行start_windows.cmd，WSL使用包内启动脚本。包内不含本机profile、凭据、会话或录制日志，不会自动连接板端或部署设备。

- 08默认`deployment/board_trials_4x4/08_full_mission/site_waypoint_map_20261015.yaml`：高位FC2.6m、max3.2m、rectangle。
- 备用10默认其目录下同名YAML，可选牛耕/回字与FC1.8/1.9m。
- 无需导入或手填地图，未勾手动时直接用板端已确认YAML；显式manual override优先。
- 三套路线默认competition：规划1.2m/s、1.0m/s²，H捕获/对准FC1.1m；08显式有限空间档仅作对照。
- 投递目标FC0.25m，08/10释放窗口当前0.10～0.45m；其他投递/H策略继承板端。

工作台代码在视觉试飞分支tools/flight_workbench；完整前线副本额外保留既有UI功能。板端导航更新由主代理部署，23100bcb五组合无site配置生成CONFIG_VALID由主代理报告。

## 当前在线使用

刷新`http://127.0.0.1:8771/?ui=20261015-live2`加载版本化脚本。真实SSH/probe已恢复live/fresh；08及备用10四组合页面预览命令与当前后端纯API完全一致。旧后端若不支持显式速度参数，前端省略覆盖并继承板端已确认competition；无需中断活跃设备会话。操作者再决定是否执行配置检查或实际入口，未由本agent代点启动。

两份工作台定向单测各9项通过、完整前线浏览器9场景通过，低位地图测试随包；这些软件验证不代表新路线实飞验收。pure trial/command只生成命令，卡片preview可能启动应用链。

[路线与FOV参考](../../verification/low_coverage_backup_20261015/README.md) · [LIO积压分析](../../verification/flight_1015_1656_lio_20261015/REPORT.md)

最终包：126项，493242字节；包含地图补丁1280f48b及可读提示db301780。Noether两版地图测试与主代理截图目视通过。运行记录仅代表验证时刻；最新会话可能已退出，请以实时状态和日志为准，不以旧READY推断当前状态。
