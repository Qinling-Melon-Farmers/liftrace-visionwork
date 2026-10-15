# 2026-10-15 实飞投递目标 0.15 m 只读溯源

结论：远端源码中有明确的 08 投递目标 0.15 m 改动，不是只能推测未归档运行 overlay。导航仓 `origin/liftrace_board_trial_20260928`（本次已 fetch 的 HEAD `af599680ff164bab513affb050b053e9c06bcffa`）中，该值由提交 `2df2311e1bc89976bbb62739a96d1d9ae13daca3` 引入，Git 作者和提交者均为 `lihaiyby`，作者/提交时间均为北京时间 **2026-10-14 02:36:52 +08:00**。这是提交身份和提交时间，不能认定此账号对应的个人亲手改动、发出指令或在今天工作台操作，也不能把提交时间当成精确编辑/部署时间。

## 1. 直接证据：08 两入口同步改为十五厘米

提交：`2df2311e1bc89976bbb62739a96d1d9ae13daca3`

标题：`feat(板端): 汇总十五厘米投递与调用后恢复及现场配置`

提交正文：`提交香橙派当前全部改动，包含投递窗口0.10/0.15/0.43m及下限、投递调用事件与恢复接续、配置和回归记录。`

| 路径 | 前值 | 后值 |
| --- | --- | --- |
| `deployment/board_trials_4x4/08_full_mission/settings.yaml` | `drop_agl: 0.20` | `drop_agl: 0.15` |
| 同文件 | `release_min_agl: 0.15` | `release_min_agl: 0.10` |
| 同文件 | `release_max_agl: 0.40` | `release_max_agl: 0.43` |
| `deployment/board_trials_4x4/08_full_mission/competition_speed.yaml` | 目标/下限/上限 `0.20 / 0.15 / 0.40` | `0.15 / 0.10 / 0.43` |

`git log -S 'drop_agl: 0.15' af599680 -- deployment/board_trials_4x4/08_full_mission` 唯一命中该提交。`git blame af599680` 显示 settings.yaml 第12–14行、competition_speed.yaml 第4–6行均归属该提交。已用 `git log -G` 和完整 diff 交叉核对。

同提交在 `docs/仿真联调变更记录.md` 增加“2026-10-14 投递高度更新为最低0.10/目标0.15/最高0.43m”，记录板端备份目录 `/home/orangepi/.codex_backups/drop_015_20261014_013208`，并称已通过两档 preview/check-config、板端相关检查和 patrol_control 构建；记录明确本轮未启动控制器、飞行或舵机。备份目录名提示操作归档线索为10月14日01:32:08，但本次未连接板端，不能独立核实该目录存在、实际内容及精确操作时间，也不能把目录名当作准确修改时间。

## 2. 控制/共用配置同时放行，区别于参数缺省

同提交同步修改：

- `patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp`：开启标定投递时，`drop_release_setpoint_height_ - drop_ground_z_` 的允许下限从 `0.20` 改为 `0.15`。在 af599680 中见第2513–2520行附近。`git log -S 'drop_release_setpoint_height_ - drop_ground_z_ < 0.15'` 命中同一提交。
- `deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_config.py`：生成器目标允许下限 `.20` 改为 `.15`；af599680 第255行。
- `patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/competition_config.py`：正式配置校验的 `drop_agl` 允许下限 `.20` 改为 `.15`；af599680 第88行。

这些是**允许下限**，不是自动设置目标的逻辑。实际目标来自上节两份 08 YAML。

控制器 `nh_.param("drop_system/release_setpoint_height", 0.10)` 的缺省仍为0.10（af599680 第2452–2453行），并非新增0.15缺省。基础控制 YAML 的 `release_setpoint_height` 也仍为0.10。08 生产生成器覆盖基础值：trial_config.py 第244/248行按静置参考求 ground，再计算 `drop = ground + settings['drop_agl']`；第319–320行把 target/min/max 写入 `release_setpoint_height`、`release_min_height`、`height_threshold`。

`trial_speed_profiles.py` 第14–25行从 competition_speed.yaml 读取并覆盖08设置，因此普通和 competition 两档都会继承十五厘米目标。`run_trial.py` 的加载顺序为：模块 settings → site profile → speed profile → 可选 flight-config。后续覆盖确实可能再改窗口；本次只说明已有源码流向，不虚构实际被选中的覆盖文件。

## 3. 时间链与前后值

以下均为 Git 作者时间（+08:00）；这些提交作者和提交者同名。普通08和competition档在10月11日存在历史差异，不把两者强行合成一条一致参数链。

| 提交 | 作者 | 时间 | 08 普通 settings 变化（目标 / 下限 / 上限，m） |
| --- | --- | --- | --- |
| `38647c923923f228e8cf8fb92cda3e8ca4ae7358` | Qinling-Melon-Farmers | 2026-10-10 23:00:34；提交时间23:00:45 | `0.35 / 未显式 / 未显式` → `0.40 / 0.35 / 0.45`，competition也同步 |
| `45cae3c0045080e35dc84ef4dd715472c753a300` | lihaiyby | 2026-10-11 22:11:10 | `0.40 / 0.35 / 0.45` → `0.32 / 0.28 / 0.36` |
| `9d2262de0e0e74ac02ba5cc17331f5dbcd2a3d74` | lihaiyby | 2026-10-11 22:43:59 | `0.32 / 0.28 / 0.36` → `0.30 / 0.26 / 0.35` |
| `06365ca058ad66924346221ecfc447d1522d62de` | lihaiyby | 2026-10-11 22:49:54 | 下限 `0.26` → `0.28`，目标0.30 |
| `9c0690251d413c6613f4cffddfe5b58f8457cf92` | lihaiyby | 2026-10-12 20:17:04 | `0.30 / 0.28 / 0.35` → `0.35 / 0.30 / 0.38`，competition从0.40/0.35/0.45同步成此组值 |
| `6b1135f81ad36edd21fefb0c6671ff99dbe1cbd2` | lihaiyby | 2026-10-12 23:37:06 | 上限 `0.38` → `0.40` |
| `22220dcbea05959f280cc0ec47e49cb2809c52b9` | lihaiyby | 2026-10-14 00:54:43 | `0.35 / 0.30 / 0.40` → `0.20 / 0.15 / 0.40`，两档同步 |
| `2df2311e1bc89976bbb62739a96d1d9ae13daca3` | lihaiyby | 2026-10-14 02:36:52 | `0.20 / 0.15 / 0.40` → **`0.15 / 0.10 / 0.43`**，两档同步，并放宽控制/生成校验 |

`22220dcb` 中追加的台账把0.20目标修改操作记在 **2026-10-13**，备份 `/home/orangepi/.codex_backups/drop_height_20261013_192149`，随后代码放行备份 `drop_height_code_20261013_192847`。因此“何时”应分开表述：台账操作日期与 Git 汇总提交日期不同。

## 4. 用户此前0.40m要求的本地实现对照

视觉仓工作树 `/home/xhj/liftrace-worktrees/r2026-board-vision-tests` 中可核实：

- `7629509d207c2eac47f7bed309ba95aec883df18`，作者/提交者 `Qinling-Melon-Farmers`，作者/提交时间 **2026-10-10 22:40:04 +08:00**，标题“fix: 投递目标回到四十厘米并独立保留释放窗口”。
- 两份08 YAML 均将目标 `0.35` 改 `0.40`，新增显式下限 `0.35`、上限 `0.45`。
- `docs/verification/drop_window_20261010/REPORT.md` 首段和 YAML 明确记录 FC目标0.40m、最终释放0.35～0.45m；同次台账有相同约定。
- 导航现场分支随后由上述 `38647c92` 同步这组数值。

这证实此前0.40m要求已有实现和同步历史。本次没有读取原始聊天逐字记录，因此不将项目记录冒充用户原话；也没有从后续降高提交推断用户曾授权降至0.15m。

## 5. 05/UI的0.15下限不能解释08目标

桌面工作台解包目录 `/home/xhj/liftrace-deliverables/workbench_review_20261015/incoming/飞行工作台` 只读核对结果：

- `web/flight-settings.js:68`：目标输入空值，placeholder“例如0.40”；仅05允许最小0.15，其他组最小0.35。这里的0.15是**输入范围下限**，不是08默认目标。
- `wb_flight_settings.py:9,37`：通用目标范围0.35～1.0，仅05改为0.15～1.0。
- `board_parameter_patch/.../flight_settings.py:9,25` 也是相同语义。
- 导航远端同样的05特例由 `28f47d49608cb7a2edfecd59dfab6b3f12ed3446`（lihe-code，2026-10-14 04:04:42 +08:00）引入，不能据此解释已在02:36提交的08目标0.15。
- af599680 的05实际 settings 目标为0.30、窗口0.28～0.35，并非0.15。
- 工作台当前参数显示从远端读取值；输入框和后端允许范围本身不是一次已保存的参数覆盖记录。

## 6. 今天实飞能确认什么、仍不能确认什么

实飞 metadata：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物/flight_20261015_1501_analysis/metadata.json`。

三个记录项内实际 settings 均为 `mode=high_view_full`、`drop_agl=0.15`、`release_min_agl=0.10`、`release_max_agl=0.45`、`drop_descent_mode=2`、`drop_descent_speed_mps=1.5`。静置FC离地标定0.22m；ground_reference `ground_z=-0.21564697546884418`，`drop_z=-0.06564697546884418`，两者相差0.15m，确认该值是FC目标AGL，而非local Z或释放下限。它不意味着已经在0.15m才实际调用投递，调用高度另由日志/轨迹判定。

远端 af599680 两份08默认释放上限仍为0.43m，下降速度默认1.0m/s；今天实际0.45/1.5说明还有运行前或运行配置差异。本次没有可归因的现场覆盖文件归档，**不能确认是谁把上限改0.45、速度改1.5，也不能确认实际采用的是哪份overlay**。但十五厘米目标本身已经在远端明确提交。

远端 `deployment/site_20260928` 的受版本控制场地配置未找到 drop_agl/release_min_agl/release_max_agl=0.15 的额外来源；CHECKS.json的drop_z、旧README的local-Z说明是历史信息，不能替代今天运行配置。不能排除板端未归档覆盖。

metadata 的 deployment.source_head 仍为 `51f48fc5970b10b5a8507f59874bd04ee8da7640`，recorded_at为10月8日，明显是旧部署元数据。主代理提供“多处controller日志行匹配b42b8f21控制文件”作为线索，本次确认 `b42b8f213523f85539fd8baa21788737fe18a062` 的提交身份是lihaiyby、时间2026-10-14 07:08:13 +08:00，但未把行号匹配当作现场完整HEAD证明。

最终可对用户表述：**08目标0.15m最迟在10月14日02:36:52已以lihaiyby身份提交进导航现场分支；该提交同时改普通/正赛档并放宽控制校验，台账另留01:32备份线索。今天运行确实用了0.15m。谁实际操作/下达修改、准确现场HEAD及其余覆盖来源，仅凭现有提交和metadata无法认定。**

本次仅本地只读Git/源码/metadata/工作台检查，输出写/tmp；未checkout、未修改共享文档、未提交、未连接板端或启动任何飞行/仿真。