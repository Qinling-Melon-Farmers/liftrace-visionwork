# 2026竞赛工程结项交接

日期：2026-10-19。用户确认本次竞赛工程结束，授权整理合入main、清理冗余副本并保留可恢复资料。本文是最终交接索引；历史报告继续保留其原始日期、参数和失败结果。

## 工程与资料位置

- 完整集成源码：`/home/xhj/liftrace`，视觉远端 `Qinling-Melon-Farmers/liftrace-visionwork` 的main。
- 导航权威源码：`sakelier/liftrace-controlwork` 的 `liftrace_board_trial_20260928`；结项取自 `ed1289be`。导航main未变更，避免把现场专用配置覆盖其他成员主线。
- 试飞/工作台来源：视觉试飞分支 `a880b518`。main保留2025原始资产、历史仿真与原作者提交；新运行文件按来源定向集成。
- 本地交付总目录：`/home/xhj/liftrace-deliverables/competition_closeout_20261019/`。其中Git bundle、模型数据集压缩包属于外置资产，不入源码Git。
- 历史录像/分析：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/`。原始bag已按用户授权清理121份，旧文档中的bag链接可能失效；已生成视频、参数和报告保留。

## 阅读顺序

0. [下一届上手指南](NEXT_TEAM_README.md)：从收包、搭环境、构建到现场检查。
1. 本页及 [来源清单](SOURCES.json)：确定归档范围与代码版本。
2. [技术交接明细](TECHNICAL_HANDOVER.md)：模型、部署、工作台、历史验证和剩余问题。
4. [清理与恢复](CLEANUP_AND_RESTORE.md)：已删除副本、归档分支与恢复方法。
5. [验证记录](VALIDATION.md)：本次离线回归、构建和发行检查。

## 启动入口与工具

- 独立正式工程：`deployment/competition/start.sh`，先读 `deployment/competition/README.md`。
- 03 H专项：`deployment/board_trials_4x4/03_h_landing/start.sh`。
- 04走廊接H：`deployment/board_trials_4x4/04_corridor_landing/start.sh`。
- 08整场：`deployment/board_trials_4x4/08_full_mission/start.sh`，登记场地 `site_waypoint_map_20261016.yaml`。
- 10牛耕/回字形：`deployment/board_trials_4x4/10_low_coverage_full/start.sh`；默认与后移1m四份现场YAML分别保存，后移版不是默认。
- 11临时无靶单投：`deployment/board_trials_4x4/11_low_release_test/start.sh`，仅实验入口。
- 工作台：`tools/flight_workbench/start_workbench.sh`（WSL/Linux），`start_windows.cmd`（原生Windows）；默认8771，配置 `workbench.yaml`，不包含本机账户口令。
- 离线回放：`tools/bag_replay/run.sh`；分包合并 `tools/bag_replay/merge_bags.py`，不覆盖输入；Windows回放位于工作台 `offline_replay/`。
- ULog：`tools/flight_logs/README.md`，本地解析无需网站上传ID；下载需设备可连接。
- 模型导出与包验证：`tools/model_release/README.md`。
- 阈值回放、障碍柱可视化、舵机诊断：分别为 `tools/threshold_replay`、`tools/obstacle_columns`、`tools/servo_diagnostics`；舵机诊断会涉及真实动作，不属于离线回放。

全部工具源码及其说明随本次main发布。大模型/视频/训练图像不放Git。工具索引详见 [TOOLS.md](TOOLS.md)。

## 参数和状态的使用规则

归档保留正式模板与现场YAML的差别，不把旧有限场地参数替换为想象中的比赛默认。恢复实机前以选定入口生成的 `run_metadata.json`、`runtime.yaml`、`control.yaml` 为实际参数依据。模型文件名与metadata必须匹配，不能仅依赖旧部署清单。

最新已归档10月16日两轮实际使用v8五分类FP16 RKNN；其原始数据和局限见交接明细。CAPTURE类别变化退出补丁10月16日已部署到192.168.43.99的0928目录，板端提交 `6f7cc4ea`、离线192项通过；未取得修复后的实飞验证记录。

EV预测保持冻结；本次没有把FAST_LIO预测研究改动混入main。导航恢复代码仍按来源保留其候选/关闭状态。解锁后建图仅做过启动时序核查，未实现，不能列为已有功能。无新飞行、舵机动作或SITL实跑；竞赛结项不等于所有研究候选都验收成功。
