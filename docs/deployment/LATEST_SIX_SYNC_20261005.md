# 最新公共补丁与六轮动态验证同步（2026-10-05）

公共记忆改判、四项运动约束及None/pending/ACK修复已同步高位研究、整机候选、板端试飞、导航liveness与板端参考分支；导航liveness包含新版patrol_control。核心十个文件已逐项与本批87258798比较一致。配置按入口职责分别保留，不通过复制实验室参数统一整机。

最新六轮全部固定87258798，不修改中途源码，不择优重跑；完整报告和视频索引在[高位研究报告](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/high-view-search-research/docs/verification/latest_six_20261005/REPORT.md)。原始bag/视频保留本机logs及deliverables分享包，不入Git。九组现场命令见[操作手册](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/board-deployment-flight-20260920/docs/deployment/board_redeploy_20261001/NINE_TRIALS_20261005.md)。

| 配置 | 原始Gate | COMPLETE ROS秒 | 原因 |
|---|---|---:|---|
| 31 rectangle | PASS | 197.14 | all_checks_passed |
| 31 snake2 | PASS | 218.98 | all_checks_passed |
| 31 snake3 | PASS | 232.53 | all_checks_passed |
| 38 rectangle | PASS | 233.53 | all_checks_passed |
| 38 snake2 | FAIL | — | actual_collision |
| 38 snake3 | FAIL | — | manager_failed |

矩形两轮完成正确三投、走廊和H降落；三线seed38也落地上锁，但Bridge因control_state_not_landing未完成软件终态，按报告分开记录；旧panzer/pillbox误投的实际触发路径本次被低空改判阻断。seed38双线LOW_COVERAGE发生真实树体接触，不能算作落地收尾误报。碰撞前LIO估计稳定，实际机体对样条存在约18cm空间偏离，仍需核查弯道跟踪与地图/机体净空；不以缩小膨胀或碰撞包络处理。

最新矩形相对旧未优化矩形的整场物理触地时间并未缩短（seed31慢13.98s，seed38慢9.61s）；不宣称全部速度优化带来整场节时。走廊共同1.0m约束下部分通过轮真实高度仍有5–7cm超调，旧Gate1.2m不能代表严格物理限高通过。

下一轮先低速06→强化03 H→04走廊接H，再08同次全任务。快速补搜须先修复并回归树体接触；09高速采集提供实拍识别/曝光/EV实时性数据。实验三路线保持研究用途，未改整机默认路线、main、EV研究分支或试飞组板载代码；本次没有部署机载电脑。
