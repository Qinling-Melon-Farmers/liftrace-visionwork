# H专项POSCTL末段交接部署检查（2026-10-06）

板端部署与检查已通过，本轮保持应用停止、未解锁，没有启动新一轮飞行。POSCTL末段由飞手用油门完成下降；这份检查不能代替无旋转实飞验收，也不表示AUTO.LAND根因已修复。

## 部署版本与回滚

- 板端：`orangepi@192.168.43.59`，根目录 `/home/orangepi/liftrace_board_trials_20260928`。
- 导航来源：liftrace-controlwork `板端参考分支`，`d35d0304032d4bb0522f4b779c4171a4c69ded12`；集成试飞分支 `feat/board-deployment-flight-20260920`，`fd1019ec82f34298ff4637574e403ff7157190e7`，均已push。
- 修改旧控制链前已保存原包，先行导航快照commit `7fe30950`；集成仓 `legacy_baseline/20261006/posctl_handoff/` 保留来源与清单，不参与编译。
- 板端运行源码及旧二进制备份：`/home/orangepi/liftrace_board_trials_20260928/logs/h_posctl_patch_20261006_164410`。`before/` 按原路径保存，`patrol_control_before` 保存旧可执行文件；`arm_build.log`、`deployment.json` 和 `validation_result.json` 记录此次构建/检查。回滚需同时恢复源码、03配置和旧二进制，并保持应用停止。
- 先部署源码，在板端增量构建实际 `patrol_control`；ARM编译链接通过后才切换03设置为POSCTL。没有重启工作台或设备会话。

## 检查结果

| 检查 | 结果 |
|---|---|
| 本地真实catkin控制器编译链接 | PASS |
| CPP生产方法22、新桥接生产方法13、原桥接契约18 | PASS |
| 旧释放/桥接后续61、配置8、部署后续8、其他专项回归28 | PASS；部署后续含2项既有skip |
| 本地板端18次/仿真8次launch静态展开 | PASS；没有运行仿真 |
| 板端实际ARM增量编译与链接 | PASS |
| 板端CPP生产方法22、桥接13、配置8 | 43项PASS |
| 板端H入口配置检查 | CONFIG_VALID；未启动ROS节点 |
| 板端03/04/08的board/sim展开 | 6项PASS |
| 板端13个补丁文件逐内容比较 | 与本次集成提交一致 |
| 可执行文件包含新交接接口且晚于本次CPP源码 | PASS |

板端检查工具输出在本地 ignored `logs/h_posctl_deployment_20261006/`。新String状态也加入下一轮bag，默认话题 `/patrol_control/external_landing_handoff`，支持现场配置自定义；不增加相机/JPEG编码或原始点云记录。

## 本轮03参数

| 项目 | 参数 |
|---|---|
| 名义H中心 | 起飞点正前方2.0m，横向0m |
| 起飞/低空接近 | FC中心离地1.0m |
| 定点识别 | FC中心离地1.2m |
| H结构/笔画兜底 | 开启；LAND后新观测 |
| 对准门槛 | XY ≤8cm；10个新帧；观测年龄≤0.5s |
| 下降目标/交接高度 | ground+0.40m / ground+0.55m |
| 末段模式 | POSCTL |
| 速度/加速度 | 0.5m/s / 0.35m/s² |
| 交接乱序等待/实际模式到达 | ≤0.5s / ≤2.5s |
| 飞控/ON_GROUND状态年龄 | ≤2.5s，适配约1Hz；定位年龄仍≤0.5s |
| 稳定落地/专项收尾 | 既有默认1.0s稳定；未解锁且ON_GROUND持续3s后收尾 |

03入口：`bash deployment/board_trials_4x4/03_h_landing/start.sh flight --site-config deployment/site_20260928/h_landing_test_area.yaml --motion-optimized`。本轮没有执行此命令。04/08保持默认AUTO.LAND及1.8m识别，各自场地坐标仍须独立实测。

后续获得飞行指令再重新初始化，看到READY后由飞手解锁/拨OFFBOARD。接近、升高和H视觉对准/下降自动进行；看到实际POSCTL后，偏航杆中立、油门接近悬停中位，再由飞手缓慢降低油门落地。触地后解除武装并保持POSCTL，等工作台收尾再切其他模式。提前手动接管、交接后回OFFBOARD或超时会取消旧LAND，不自动恢复。

## 最后一轮日志与回放

`160434`轮原bag已取回，大小85,348,842字节，原始时长101.411585s；`tools/bag_replay/run.sh`离线生成原始相机、标注相机、轨迹和组合四段101.5s视频，完整解码和时长检查通过。入口 `logs/h_flight_review_20261006/replay_160434/index.html`。用户取消追加全高度点云回放，原bag与现有回放保留。

AUTO.LAND回传目标航向连续增至约62.26°，目标X从1.98降至1.14m；切POSCTL后航向目标停止变化。历史EKF重置重放是源码支持的优先候选，今日ULog FTP读取返回0字节，不能当作已定根因。详情见[YAW_DIAGNOSIS.md](YAW_DIAGNOSIS.md)；正常交接身份契约见[INTERFACE_PROPOSAL.md](INTERFACE_PROPOSAL.md)。
