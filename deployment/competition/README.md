# 独立正赛入口

完整操作、版本来源和待验收范围见 [整机说明](../../docs/deployment/competition_integration_20261003/README.md)。本目录不依赖九组专项。field.example.yaml需按现场测量填写；默认缺走廊/H时不能飞行。

当前有效参数、规则/技术会差距、实测FOV覆盖及同步状态见 [10月4日正赛复核](../../docs/verification/competition_config_20261004/REPORT.md)。其中生成的runtime/control仅供离线参数分析，不可直接用于飞行。

2026-10-04新增[矩形/蛇形与速度衔接候选](../../docs/planning/motion_optimization_20261004/README.md)。候选位于 `candidates/`，默认模板不变，尚未SITL/实飞验收；按相机2.6m设计，对应当前rig的FC2.76m。
