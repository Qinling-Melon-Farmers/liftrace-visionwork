# ULog 电机工具与低空观测扩展

来源是用户提供的 `试飞产物/ulg_motor_viewer.zip`（12,472 字节）。`original/ulg_motor_viewer/` 保留原 `app.py`、README 和 requirements，原说明及归属不作替换；原包没有明确作者/许可证字段，因此不推定其作者或新增授权。顶层 `app.py`、`analysis.py`、`plotting.py`、安全导入工具和测试是此次离线扩展。原 README 的安装步骤只作来源留档；本项目使用已有 `rl_drone`，不要按该步骤另行安装。

ZIP 导入前逐成员检查：拒绝绝对路径、`..`、Windows 盘符/反斜杠、符号链接/特殊文件、大小写重复成员、加密成员，以及超过 8 MiB 的解压总量。先检查全部成员，再逐个以独占创建方式写入；不覆盖已有文件。此次 4 个成员均通过，未执行 ZIP 内源码来完成检查。

## 使用

从工作树根目录在 WSL 内运行，现有 Python 3.9 环境可用，不依赖 pandas：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python tools/flight_logs/ulg_motor_viewer/app.py \
  试飞产物/飞行0.4m*.ulg \
  --output tools/flight_logs/ulg_motor_viewer/outputs/low_hover_001
```

Windows 宿主调用使用 `wsl -e bash -c '...'`；含中文路径不要内联到 Windows→WSL 命令中，可在 WSL 脚本/UTF-8 文件内传入，或使用 `*/*0.4m*.ulg` 匹配当前三份文件。

当前最新三份日志的 Windows→WSL headless 批处理（在本工作树根目录）可直接用：

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-board-vision-tests && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && python tools/flight_logs/ulg_motor_viewer/app.py */*0.4m*.ulg --output tools/flight_logs/ulg_motor_viewer/outputs/latest_three_001'
```

WSL GUI 启动：`python tools/flight_logs/ulg_motor_viewer/app.py */*0.4m*.ulg`。没有 DISPLAY/WAYLAND_DISPLAY、Tk 不可用或显示连接失败时，有输入文件便自动切到 headless 并在工具 `outputs/batch_<时间>/` 保存结果。也可显式用 `--headless`，或用 `--output` 完全跳过 Tk 导入。`--gui` 强制交互，失败时给出可操作的提示。无显示且未提供输入时只给使用提示，不猜日志目录。

多个文件或目录均可输入；目录默认仅一层，`--recursive` 扫描子目录。`--no-plots` 只导出表格与摘要。输出目录必须不存在，防止覆盖旧分析；同名输入用序号分开。一份输入失败不阻止处理其他输入，错误写入 `batch_errors.csv`，整批有失败时退出码为 1。只读取已关闭、完整下载的日志；无效文件头明确失败，不连接飞控，不改参数。

无 `--output` 时保留 Tk 桌面界面：批量导入、后台读取、Ctrl/Shift 多选、M1–M4 开关、模式分区、刻度、缩放/平移、原 CSV 与 PNG 导出。新增开关默认只显示/统计允许比较的空中命令窗口，取消后显示全时段；原模式分区仍可查看。新增 **Export full timeline / CSV / summary** 导出完整低空观测图与表，包含被比较窗口排除的原始数据。没有有效空中窗口的日志不会伪造统计，完整导出中标为未知。交互仍显示前四路；批处理可用 `--motor-count 1..12` 指定通道数，默认四路，不自动猜物理电机位置。

## 输出与语义

每个输入输出 `overview.png`（10 个时间对齐面板）、`summary.txt/json`、`events.csv`、`windows.csv`、`health_timeline.csv`、`motor_statistics.csv`，以及每个主题/实例的原始 CSV。完整 CSV 不做显示抽样，NaN 输出空单元格。JSON 不输出非标准 NaN。横轴为相对 ULog 头部开始的秒数；事件另有 PX4 boot 秒，不能当 ROS epoch 时间直接对齐。记录字段、样本区间、缺失主题和时间倒退在 inventory 中可查；时间倒退按稳定排序处理，同 timestamp 保留最后一条。

- **电机**：`actuator_motors.control[]` 是归一化分配命令，不是电流、RPM、实际推力或最终硬件输出。旧 `actuator_outputs.output[]` 按实例原单位导出，不转换成归一化命令。`actuator_controls_*` 不充当逐电机数据。索引与物理位置/输出口的关系必须查机架和输出映射。
- **姿态/角速率**：四元数先归一化，非法四元数保持未知；显示估计 roll/pitch/yaw 与可用设定值。yaw 单独使用右轴，避免掩盖低空小倾角。角速率从 rad/s 转 deg/s；不从姿态数值差分冒称测得角速率。
- **RC/模式/解锁/接地**：手动输入、原始 RC 通道及遥控开关各按原字段导出。原始 RC 通道不能凭编号猜 throttle/yaw；`RC_MAP_*` 初始参数保留供核对。模式取 `vehicle_status.nav_state`，与遥控意图分开；图中模式/开关保留原 enum ID，原交互仍有名称标注。解锁来自 `actuator_armed.armed`，接地来自飞控 `landed/ground_contact`，均不根据文件名或高度猜测。
- **电池**：图只显示 connected 且正的电压字段（如果记录）；原字段仍完整导出。电池电流是整包电流；没有 ESC 反馈就没有逐电机实测电流/RPM。
- **高度**：`-vehicle_local_position.z` 是本地坐标系估计 up，原点可变，不是实际 AGL。`dist_bottom` 也是飞控估计量，只在 valid 时画；setpoint 明确标作设定值。文件名“0.4m/0.6m”不进入数值和判定。不推断真实飞行高度。
- **重置与 EV**：保留 z/heading/attitude/EV reset 计数变化和最新 delta，uint8 回绕用 mod256 计算；初始非零计数不是本段重置证据。首次记录到计数变化的时间可能晚于实际重置，不能直接归因。所有 EKF 实例原始数据分别保存；活动标志/比率按 `estimator_selector_status.primary_instance` 对齐，无选择器时不假设 instance 0 活动。EV 原始输入缺失时，新鲜度/延迟未知；EV 标志、事件和旧 innovation/test ratio 按可用字段降级，不将主题未录到解释为未融合。heading ratio 并非 EV 专属。

`health_timeline.csv` 使用选定主题时间戳的并集，做因果前值对齐，不回填未来样本；该表不是一个新的实际采样流。状态最大保留年龄默认 2 秒（`--state-age-sec`），超龄为空/未知。EV age 列是相对于最后**记录到**的输入，不把 logger 间隙当成真实输入断流；logger dropout 同时保留。

## 比较窗口

原始曲线淡色、允许比较的电机曲线实色，下方色带分阶段。`motor_statistics.csv` 按以下窗口分别统计；摘要只列 `motor_airborne`，绝不把全时段均值当悬停比较：

| 窗口 | 含义 |
| --- | --- |
| `ground` | 飞控报告 landed 或 ground_contact，保留地面运行数据 |
| `motor_airborne` | 新鲜 armed=true、landed=false、ground_contact=false，lockdown/manual_lockdown/force_failsafe 均已录且为 false，未报告 maybe_landed，未被 kill 或保护窗口排除 |
| `stopped_or_blocked` | 解锁 false、锁定/强制 failsafe，或 kill_switch=2；命令非零仍不能证明电机转动 |
| `transition_guard` | 解锁/锁定、接地/可能接地、kill 变化前后保护带 |
| `suspected_impact` | 本地加速度范数或角速率范数超过可配置阈值的保护带；不是碰撞诊断 |
| `logging_gap` | 电机记录间隙或 logger dropout，优先排除 |
| `unknown` | 缺必要状态、前段未有状态或状态超龄 |

默认变化保护带 **前后 3 秒**（`--guard-sec`），保守排除接地判定延迟附近的冲击/停机；可按明确的手工窗口复核调整。尖峰阈值为 15 m/s²、300 deg/s（`--impact-accel-mps2`、`--impact-rate-dps`），作用于日志所记录的本地加速度和角速率范数，不是独立撞击传感器。默认记录间隙阈值 0.3 秒（`--gap-sec`）。窗口是状态/命令筛选，不保证实际电机运行或稳定悬停；机动、地效和未录到的冲击仍需结合其他数据判断。

均值/分位数是有限原始样本统计；≥0.95/0.999 仅表示命令幅值。累计秒数只在同窗口、相邻有限值且间隔不超阈值的区间上前值保持估算，不跨日志间隙累计。多电机差异 `simultaneous_max_minus_min` 只在同一时间戳全部选定通道有限、且属于比较窗口时计算，避免 NaN 群体不一致。它不能单独诊断故障、推力差或电流差。

## 验证与后续录制

```bash
python -m unittest discover -s tools/flight_logs/ulg_motor_viewer/tests -v
python tools/flight_logs/ulg_motor_viewer/tests/gui_smoke.py */*0.4m*.ulg
```

2026-10-06：使用现有 `rl_drone` 对三个新 `飞行0.4m*` 完成批处理，20 项定向单测通过，覆盖安全 ZIP、缺字段/超龄、接地/kill 过滤、尖峰、间隙/dropout、NaN 同时比较、reset 回绕、活动 EKF 切换、缺数据图表、单文件失败隔离和无显示自动降级。GUI 在现有 WSL 图形环境中以隐藏窗口验证导入、多选、空中/全时段切换、原 CSV/PNG 及完整导出；表格峰值随空中/全时段开关同步更新。显式去掉 DISPLAY/WAYLAND_DISPLAY 对最新三份完成 headless PNG/CSV/摘要导出，也验证自动选择 headless。最终批处理产物为本目录 `.validation_final/`，其他验证产物留 `.validation*`/`outputs/`，由本目录 `.gitignore` 排除；不是日志诊断结论或实飞验收。临时字段清单脚本已依主代理要求移到工作树 `logs/ulg_motor_viewer_inspect_fields.py`。

这三份新日志有命令、姿态/角速率、RC/开关、飞控状态、电池字段、local-position/reset、双 EKF 状态与选择器；未录到 EV 原始输入流或 EV aid-source 主题，因此无法检验 EV 输入发布率、样本年龄或逐融合拒绝时间。

建议主代理的录制入口保留：`actuator_motors`、实际输出实例/输出映射、`actuator_armed`、`vehicle_status`、`vehicle_land_detected`、`vehicle_attitude`/setpoint、`vehicle_angular_velocity`/rates_setpoint、`input_rc`、`manual_control_setpoint/switches`、`battery_status`、`vehicle_local_position`/setpoint、`estimator_selector_status`、全部相关 EKF instance 的 status_flags/event_flags/innovations/test_ratios，以及 **vehicle_visual_odometry（含 timestamp_sample/质量/协方差）和可用 estimator_aid_src_ev_pos/vel/hgt/yaw**。如果硬件支持，再录 `esc_status` 反馈；真实离地高度需同步独立测距/现场量测与视频。这里只提出字段，不改录制配置、板端入口或其他代理的诊断报告。
