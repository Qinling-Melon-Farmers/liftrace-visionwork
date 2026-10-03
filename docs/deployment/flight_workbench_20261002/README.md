# 试飞验证看板（flight_workbench）

2026-10-02后续：看板启动链review、具体点击/切组步骤、H/走廊/整场自动化适配见
[REVIEW_AND_OPERATIONS.md](REVIEW_AND_OPERATIONS.md)。本轮仅本地提交，未上板或实飞。

2026-10-02：把现场手册 [OPERATIONS.md](../flight_handover_20261001/OPERATIONS.md) 的
"6~7 个终端 + 等 READY + 看日志"做成浏览器里的点击工作台。工具本体在
[tools/flight_workbench](../../../tools/flight_workbench/README.md)。

## 它做什么

| 需求 | 工作台实现 |
|---|---|
| SSH 连接 | 顶栏「板端地址」下拉（默认外场当前 `orangepi@192.168.43.99`，另含 3.15 / 43.59 / 156.193 / 10.231.47.193 / 3.126 五个历史地址并标注出处）+「连接」做板端登录自检（工程根 / 现场环境脚本 / 模型 / 录像空间 / ROS Python）；选中地址写回本机 profile，不改仓库配置 |
| 各终端启动 | 六个常驻终端 + 监测终端各一个 tab（真实 `ssh -tt` 会话），支持「一键启动设备」按 roscore→MAVROS→雷达→相机 顺序启动并逐项等待就绪 |
| 任务组选择与启动 | 左栏现场组号 1–6 与模块目录 01–09；启动前显示**完整命令**，flight 需勾选现场条件，实投需输入确认词「实投」 |
| 初始化监控 | 实时解析 `INITIALIZING` 里的 `pose_samples`/`camera_info`/`image_seen`/`compressed_fresh`/定位一致性原因 |
| READY 监控与回报 | 阶段灯 `启动中→初始化中→MAPPING_READY→READY→飞行中→已上锁→已退出`，READY 弹提示/提示音，一键生成 Markdown 回报并落盘 |
| 飞行日志显示 | 专项入口实时输出 + 板端 `logs/board_*` 产物浏览（tail / 下载小文件） + 操作时间线 |

## 边界（没有变的部分）

- 不自动解锁、不自动请求 OFFBOARD、不自动调用 `/navigation/start_mission`（H 专项的手动任务
  开始按钮也只在 READY 之后可用）；READY 只是应用就绪。
- 只启动现场既有入口命令（`deployment/site_20260928/start_test.sh`、`board_trials_4x4/*/start.sh`
  与设备节点），不新增飞行流程、不改板端代码。
- 口令不入仓库：只在内存或 `~/.config/liftrace-flight-workbench/profile.json`（0600）。
- 不搬大 bag：数百 MB 的 `flight_debug_*.bag` 仍按原流程用 scp/rsync 回传，工作台只做 tail 与小文件下载。

## 验证（本机，无板端）

```bash
cd tools/flight_workbench
python3 tests/test_status.py     # 29 项（含外场地址清单）
python3 tests/test_probe.py      # 10 项（假 rospy，验证板端探针取值与"只订阅不下发"）
python3 tests/selfcheck.py       # 23 项（纸板工程端到端，含地址切换）
python3 tests/smoke_http.py      # 15 项（真起服务：接口/静态文件/SSE/安全拒绝/地址清单）
```

纸板工程按 `run_trial.py` 的真实输出格式回放 `INITIALIZING → MAPPING_READY → READY →
FLIGHT_STATUS → STOPPED`，因此阶段解析、告警节流、回报文本都被真实文本驱动，而不是自造格式。

**尚未验证**：真实板端 SSH 交互（换板/换网/口令提示）、浏览器端人工点检、真实专项入口的
READY 时序；板端 NPU/实飞验收仍需现场按各自 Gate 执行。
