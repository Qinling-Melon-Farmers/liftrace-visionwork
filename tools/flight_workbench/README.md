# 试飞验证看板（flight_workbench）

把现场手册 [docs/deployment/flight_handover_20261001/OPERATIONS.md](../../docs/deployment/flight_handover_20261001/OPERATIONS.md)
里的"6~7 个终端 + 等 READY + 看日志"变成浏览器里的点击操作：SSH 连接、各终端启动、
任务组选择与启动、初始化/READY 监视与回报、飞行日志与板端产物浏览。

> 定位：**操作与观测工具**。它只启动现场既有入口命令，不下发解锁、不代替飞手接管、不改板端代码、不把口令写进仓库。
> 当前入口要求飞手**人工解锁并拨入 OFFBOARD**；低空稳定后按该组配置自动启动任务，切换模式会取消自动时序。工作台不请求 OFFBOARD。
> 工作台里的 READY 只是"应用链就绪"，不是起飞许可，也不是飞行验收结论。

2026-10-02 review 已修复实投确认词、第6组速度、设备失败继续启动、旧遥测判就绪及收尾顺序。
H/走廊/整场已适配人工解锁后的自动时序，10-02 已同步至旧板 43.59，尚未实飞验收；
同步范围见 [旧板部署记录](../../docs/deployment/board_redeploy_20261001/DEPLOY_4359_20261002.md)。
任务卡片已补齐「选择此组」按钮、标题点击和刷新后的选择记忆。
已手动启动节点或正在飞行时，主页连接后点「实时观察」或「电机诊断」，分别打开
只读大页 `/observe`、`/motor`，观看位姿/姿态、输出、LIO、电池及低空观察三组曲线。
不新增相机/JPEG/视频路径；页内记录仅在浏览器完成，详见下方操作说明。
当前按钮顺序、切组与更新范围见 [2026-10-06操作说明](../../docs/deployment/flight_workbench_20261006/README.md)。
电机观察已登记用户实机接线：M1右前AUX1、M2左后AUX4、M3左前AUX2、M4右后AUX3。
刷新 `/motor` 或 `/observe` 自动载入 [motor_wiring.json](web/motor_wiring.json)；
当前MAVLink2/输出instance1映射为raw17/20/18/19。曲线、片段和导出保留机臂与接线标签，
其他连接目标默认不沿用；修改接线、输出实例或协议时需更新配置。只更新页面即可生效，不必重启设备会话。
10-02 部署记录属于历史状态，不能据此认为板端已安装此轮更新；本轮只更新源码与工作台。

---

## 1. 快速开始

在 WSL（开发机）里运行服务端，浏览器打开提示的地址：

```bash
cd /home/xhj/liftrace-worktrees/r2026-board-vision-tests
bash tools/flight_workbench/start_workbench.sh              # 默认 127.0.0.1:8791
# 或指定端口/自动开浏览器
bash tools/flight_workbench/start_workbench.sh --port 8792 --open
```

依赖只有系统 Python3 + `pexpect` + `pyyaml`（本机 ROS 环境已自带），不需要 ROS、不需要
在板端安装任何东西。

连接板端：

1. 在页面顶栏的**板端地址**下拉里选现场地址（默认 `orangepi@192.168.3.126`），再点「连接」；
   或先用 `--password-file`／环境变量给一次口令：
   ```bash
   ORANGEPI_SSH_PASSWORD=... bash tools/flight_workbench/start_workbench.sh
   bash tools/flight_workbench/start_workbench.sh --password-file ~/.orangepi.pass   # 文件须在仓库外
   ```
2. 「连接」窗口直接提供历史地址、自定义地址、用户名、端口和密码。自定义地址优先；支持 IP、主机名或 `user@host`。密码默认只留在服务进程内存，不写浏览器存储或文件。只有「连接设置」中明确勾选“记住口令”才保存到本机仓库外的 `~/.config/liftrace-flight-workbench/profile.json`（0600）。
3. 地址默认取 `workbench.yaml` 的 `connection.host`（当前 `orangepi@192.168.3.126`）。
   下拉里的历史地址来自现场部署记录与项目 memoir，选中即写回本机 profile（不改仓库文件）：

   | 地址 | 出处 |
   |---|---|
    | `orangepi@192.168.43.59` | 10-02 换回 28~30 日旧机时的历史地址 |
   | `orangepi@192.168.43.99` | 2026-10-01 第五组实投 |
   | `orangepi@192.168.3.15` | 2026-10-01 现场操作手册 / 九组部署 |
   | `orangepi@192.168.156.193` | 2026-09-28 现场部署（当时的新 IP） |
   | `orangepi@10.231.47.193` | 2026-09-20 现场（onboard_obstacle_reference） |
   | `orangepi@192.168.3.126` | 当前Orange Pi 5（2026-10-03） |

   清单外的地址：下拉最后一项“自定义地址…”或直接点“连接”填写。工程目录、模型等参数用顶栏“连接设置”，不必再双击标题。连接状态刷新不会清空历史地址列表。

不带板端也能先看界面（本机预览模式：**只渲染界面，默认拒绝执行任何设备/入口命令**）：

```bash
bash tools/flight_workbench/start_workbench.sh --transport local --port 8793
# 确实要在本机跑那些命令（自检用）才加： --allow-local-commands
```

---

## 2. 现场怎么用（与手册逐条对应）

| 手册里的终端 | 工作台位置 | 命令（界面会原样显示） |
|---|---|---|
| 1 roscore | 终端 tab「1 · roscore」 | `roscore` |
| 2 MAVROS | 终端 tab「2 · MAVROS」 | `roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600` |
| 3 MID360 驱动 | 终端 tab「3 · MID360 驱动」 | `roslaunch uav_mission mid360_driver2.launch user_config_path:="$PWD/deployment/site_20260928/MID360_config.json"` |
| 4 相机 | 终端 tab「4 · 下视相机」 | `bash deployment/board_trials_4x4/start_camera.sh /dev/video0` |
| 5 舵机（可选） | 「5a · PWM 初始化」「5b · 舵机服务」 | `sudo bash .../init_pwm.sh`、`.../pwm_node1 /Servo:=/legacy/Servo_raw` |
| 6 专项 flight 入口 | 任务组卡片「飞行」按钮 → 终端 tab「6 · 专项 flight 入口」 | 模块 `start.sh flight`（模拟/无投递）或 `start_real.sh`（实投），显式带现场 YAML |
| 7 状态监测 | 右栏状态面板 + 自动探针；终端 tab「7 · 状态监测」可手输命令 | 只读遥测 + 交互 shell |

所有终端都会先 `cd <工程根>` 并 `source deployment/site_20260928/environment.sh`，与手册
"每个新终端都先执行 cd 和 source"一致。

### 2.1 启动顺序

1. 点「单实例检查」：看工程根/环境脚本/模型/录像空间是否就绪、板端是否有 roscore·roslaunch·
   gzserver·px4·mavros 残留、九个模块入口是否齐全，并在有 ROS master 时列出**与专项入口冲突的
   旧应用节点**（这些必须先退出）。
2. 点「一键启动设备」（可选舵机）：按 roscore → MAVROS → 雷达 → 相机 顺序逐个启动并等待就绪
   （`ROS master`、`connected=true`、`/livox/lidar`、`/camera/image_raw` 新鲜）。已经在跑的
   设备节点会被识别为"已在运行"而跳过，不重复叠加。舵机两个终端默认不在自动流程里，需单独点击
   （会复位机构，界面会弹确认）。
3. 对应卡片先点「选择此组」，选投递方式/路线等选项，点「配置检查（不启动节点）」确认有效。
   再选「飞行 flight」模式、勾未解锁/起飞点确认；实投组输入「实投」，
   再点卡片底部「飞行（flight）」及弹窗「确认启动 flight」。设备启动本身不启动专项。
   随后等 READY。右栏阶段灯会走 `启动中 → 初始化中（定位/相机/坐标一致）→ 地图就绪（MAPPING_READY）
   → 就绪（READY）`，并实时显示 `pose_samples`、`camera_info`、`image_seen`、`compressed_fresh`、
   定位一致性原因、`distinct_clouds` 等关键量。**`INITIALIZING`、`MAPPING_READY` 都不等于 READY。**
4. 出现 `fc_lio_disagreement` 时按手册处理：等飞控与 LIO 自行收敛，不要转动机身追数值。
5. READY 后由飞手**人工解锁并拨入 OFFBOARD**，按低空稳定条件自动启动任务
   （时间线显示 `AUTO_MISSION_START True`）。03/04/08也使用此时序；03按实际1.0m起飞高度判稳定。
   自动时序下不再点「启动任务」。接管改模式后不会自动抢回控制或恢复任务。
6. 结束时按手册落地停机：点「停止（Ctrl+C）」让 `run_trial.py` 走既有收尾（等 `BAG_CLOSED` 和
   应用退出），再断电。

### 2.2 任务组

- **现场组号 1–6**：1 单投中断、2 连续两投、
  3 仅记忆、4 整圈重访投递、5 提前中断重访、6 高速拍摄采集。
  工作台现在统一走对应模块入口，投递组可选模拟/实投；现场组原实投默认值保留。
  `start_test.sh` 仍兼容旧命令，但不承担工作台新增参数。
- **模块目录 01–09**（`deployment/board_trials_4x4/<目录>/start.sh`）：用于 H 降落（03）、
  走廊（04）、整场（08）等专项。04/08在各自独立 `*_test_area.yaml` 中填写实测走廊航点与H坐标，
  **出厂留空时拒绝启动**。08默认模拟投递、可显式选择实投；三个H流程均不使用30cm终点悬停。
- 每个卡片都能展开「命令预览」，看到将要执行的完整命令（可复制）。独立「配置检查（不启动节点）」
  跑 `--check-config`，无需飞行确认；「预览（preview）」会启动地面应用链，二者不同。
- 运动优化默认不加参数；搜索路线与高位续扫默认继承现场 YAML，未开启时不隐式改变飞行策略。
  续扫仅第五组/整场可选。高速采集支持速度与光照标签（标签不改变曝光）。
- 防护：`flight` 必须勾选"飞机已回到起飞点、未解锁、机头朝场内"；实投必须额外输入确认词
  **实投**；同一时刻只允许一个专项入口，重复启动会被拒绝。界面预览的命令会随启动请求一起回传
  后端做一致性校验，**不一致直接拒绝启动**，避免"给人看的命令"和"真正执行的命令"漂移。

### 2.3 初始化 / READY 监控与回报

- 阶段机 + 事件时间线 + 告警（含"应该怎么做"的提示），全部来自 `run_trial.py` 的真实输出
  （`INITIALIZING`/`MAPPING_READY`/`READY`/`FLIGHT_STATUS`/`AUTO_*`/异常栈）。
- 新鲜任务遥测区分 COMPLETE/ABORTED；飞控上锁不会被当作任务成功或已落地。显示30cm悬停交接和
  LIO输出年龄/队列；缺失或过期显示未观测，不推断健康。诊断话题通过 `workbench.yaml` 配置。
- READY、失败、上报事件都会即时提示；「声音提醒」打开后 READY/失败会有提示音。
- 「生成回报」把当前阶段、时间线、告警、遥测整理成 Markdown，落到
  `~/.config/liftrace-flight-workbench/reports/`，可复制或下载，用于现场留档/群内回报。
- 操作审计写在 `~/.config/liftrace-flight-workbench/ops.jsonl`（谁在什么时刻启动了哪条命令）。

### 2.4 飞行日志与产物

底部抽屉三个页签：

- **运行日志**：专项入口终端的实时输出（可过滤 `READY`/`FLIGHT_STATUS`/`ERROR` 等关键字）。
- **板端产物**：板端 `logs/board_<专项>_<时间>/` 列表（run_metadata.json、camera_info.json、
  supervisor_result.json、vision_events.jsonl、navigation_pose.csv、bag 索引…），可 tail 预览、
  小文件直接下载。**大 bag（数百 MB）请用 `scp`/`rsync` 在板端原包留存后回传**，工作台不搬大包。
- **操作时间线**：本机侧的操作与阶段事件流水。

---

## 3. 离线自检（不需要板端、不需要 ROS）

```bash
cd tools/flight_workbench
python3 tests/test_status.py     # 阶段解析、告警节流、就绪判定、命令拼装、地址清单
python3 tests/test_probe.py      # 41项：假rospy、typed摘要、NaN/时钟/类型故障及只读边界
python3 tests/test_review.py     # 实投请求、速度、编排失败/旧遥测、并发与收尾回归
python3 tests/test_workbench_options.py # 新参数/实投约束/配置检查/ABORT与过期遥测
python3 tests/test_board_version.py # 板端源码与生成消息接口一致性（全mock）
python3 tests/selfcheck.py       # 23 项：纸板工程端到端（会话→编排→READY→回报→产物→SSE）
python3 tests/smoke_http.py      # 20项：接口/静态页/SSE/确认拒绝/地址切换
node tests/test_observe.js      # 只读曲线、映射、分组/截断、导出和过期数据
```

`tests/fake_board/` 是纸板工程，按 `run_trial.py` 的真实输出格式回放一遍
（`INITIALIZING → MAPPING_READY → READY → FLIGHT_STATUS → STOPPED`），
`tests/fake_board_setup.sh` 可重新生成其中的模块入口与占位文件。

---

## 4. 常见问题

| 现象 | 处理 |
|---|---|
| 连接失败 / `ROOT=MISSING` | 换网络后地址变了；确认 `board_root` 是现场实际部署目录（当前 `/home/orangepi/liftrace_board_trials_20260928`）。 |
| `SSH 层失败：Permission denied (publickey,password)` | 这块板没有我们任何免密公钥，且工作台没拿到口令。在「连接」里填 SSH 口令（勾「记住」存到本机 profile，0600），或把公钥装进板端 `~/.ssh/authorized_keys`。**没口令时工作台用 `BatchMode=yes` 立即失败并如实报错，不会再挂在口令提示上被误判成"板端文件缺失"。** |
| `SSH 层失败：REMOTE HOST IDENTIFICATION HAS CHANGED` | 换板后 known_hosts 里还是旧指纹：确认是新板后 `ssh-keygen -R 192.168.43.59` 删掉旧记录再连。 |
| `SSH 层失败：No route to host / Connection timed out` | 板端未上电或不同网段：先 ping 板子；地址在下拉里选对（当前 3.126）。 |
| 探针不上线（右上角灰） | 探针需要板端 ROS Python 与已 source 的环境；先确认 `环境脚本` 存在。探针未起来不影响终端操作，只是没有遥测。 |
| 一键启动设备某步失败 | 看该步详情与对应终端输出；roscore 已存在会被跳过，MAVROS 串口按实际接线核对。 |
| 初始化一直不过 | 检查飞机是否**未解锁且静置**、机头是否朝场内 +X、相机原始图/压缩图/CameraInfo 是否齐全；`fc_lio_disagreement` 时等收敛，不要转动机身。 |
| READY 后飞机没动 | READY 仅应用就绪，仍需飞手**人工解锁并拨入 OFFBOARD**；稳定后入口自动启动任务。核对板端版本。 |
| 04/08 组一启动就退出 | 走廊航点/H 坐标留空，入口按设计拒绝；补实测坐标后再启动。 |
| 舵机按钮点了没反应 | 5a/5b 需要单独确认；`sudo` 提示会自动填口令（若配置了），也可在终端里手动输入。 |
| 收尾 | 先落地上锁，再「停止（Ctrl+C）」，等应用退出与 `BAG_CLOSED`，确认无 `.bag.active` 再断电。现场1–6组末段是30cm悬停，须飞手落地。 |

---

## 5. 文件与接口

```
tools/flight_workbench/
  workbench.yaml        现场配置：连接、终端表、任务组表、单实例检查、探针话题
  server.py             HTTP API + SSE + 顺序启动编排 + 回报
  wb_ssh.py             SSH/本地会话（常驻终端 + 一次性命令，自动回应口令提示）
  wb_board.py           板端操作：连接自检、单实例检查、探针上传、命令拼装、日志浏览
  wb_status.py          阶段解析、告警提示、就绪判定、回报文本（纯函数，可单测）
  board_probe.py        板端只读探针（只订阅话题/读节点与服务列表，每 1s 打一行 JSON）
  web/                  前端（原生 JS，无 CDN、无构建）
    observe.html/js/css /observe和/motor共用的只读大页，不加载启动动作代码
  tests/                离线自检与纸板工程
  start_workbench.sh    启动脚本
```

主要接口（前端消费，便于二次开发）：

- `GET /api/snapshot`：全量状态（连接、终端、任务组、阶段、遥测、编排、告警、时间线、产物）
- `GET /api/events`：SSE，首帧 `hello` 带快照，之后 `out/session/stage/telemetry/alert/timeline/
  trial/orchestration/board/toast`
- `POST /api/connect|config|disconnect`、`POST /api/action/{preflight,start_all,stop_all,
  mission_start,report}`、`POST /api/session/{open,input,close,clear,resize,key}`、
  `POST /api/trial/{start,stop}`、`POST /api/logs/{refresh,tail}`、`GET /api/logs/download`

配置改动（换板、换串口、换视频节点、增减任务组）只改 `workbench.yaml`；不要把口令写进仓库。

## 6. 已知限制

- 界面上的"命令预览"是前端按同一规则复刻的字符串（启动时会与后端逐字校验，不一致即拒绝启动）；
  若后端 `build_group_command` 改了写法，前端预览会先被拒绝而不是静默执行错误命令，此时更新
  `web/app.js` 的 `groupCommandBody()` 即可。
- 大 bag（数百 MB）仍需 `scp`/`rsync` 回传；工作台只做 tail 与小文件下载。
- 终端输出按帧批量解析，单帧超两万行的暴输出会短暂卡顿；每会话缓冲 5000 行。
- 尚未接 PTY resize（面板尺寸变化不会同步 `stty`）；声音提示需要一次用户点击后才能播放。
- 板端大文件与 ROS 日志仍由现场既有流程收集，工作台会上传只读探针；启用手动坐标后还会在
  `logs/flight_workbench/site_overlays/`写独立运行配置，不覆盖原现场YAML。

## 2026-10-03 界面修复与离线复核

- `flight` 主终端与下方运行日志各有自己的显示节点，二者可同时显示同一份输出；设备日志不混入任务日志。
- 选择任务、切换 preview/flight、编辑参数后保留卡片滚动位置；状态刷新保留右栏位置。
- 终端/运行日志分别按不区分大小写的关键词筛选，筛选输入不被遥测刷新抢焦点。当前筛选是文本行匹配，不是 JSON 字段查询。
- 向上翻阅日志时暂停跟随；回到末尾或重新开启“自动滚动”后继续跟随。历史缓冲有上限，达到上限后最旧行会淘汰。
- SSH 状态消息按字段合并，避免局部状态覆盖整份地址配置。`save_password=false` 不再误保存输入口令。
- 离线预览的“连接”不发起 SSH；实机连接时使用默认模式启动服务。2026-10-03曾在 `http://127.0.0.1:8793` 预览，未开启 `--allow-local-commands`；检查后按用户要求关闭服务。

本轮验证（均没有连接板端或启动 ROS）：

| 检查 | 结果 |
|---|---|
| `tests/test_review.py` | 18 项通过；设备、SSH、任务操作均 mock |
| `tests/test_frontend.js` | 44 条命令一致性、22 次模拟请求、输入焦点通过 |
| `tests/test_group_selection.js` | 10 个卡片入口与 7 类嵌套控件通过；九种专项，第五组多一个 mock 入口 |
| `tests/browser_regression.mjs` | Chromium 22 项通过：可见输出、滚动、筛选、历史/自定义地址、密码仅送内存接口、离线连接拦截 |

浏览器检查复用 Node 24 的内置 WebSocket 和本机 Chromium，无新依赖。先启动上述离线服务，再运行：

```text
node tools/flight_workbench/tests/browser_regression.mjs http://127.0.0.1:8793 <Chrome或Edge可执行文件路径>
```

浏览器测试在独立临时配置中运行，连接 API 被替换为本地桩，不启动试飞。WSL 无 Node 时可从 Windows 调用已有 Node；不要为此更改 ROS Python。

### 2026-10-03 当前板端与舵机入口

当前SSH为 `orangepi@192.168.3.126`，Orange Pi 5。舵机实体包在部署根的 `patrol_uav_ws-patrol_planner/src/actuator_pwm`，二进制在同工作区 `devel/lib/actuator_pwm/pwm_node1`；当前电脑没有 `hardware_ws`。工作台默认已同步这两个路径，换电脑时必须按实际映射选择，不能套用5 Plus的2/3/4槽通道。历史SSH、自定义地址及密码入口保留；已保存的个人连接配置可能覆盖默认值，使用时选取当前地址。

5a只准备权限和禁用输出；5b服务启动才依次复位三槽。两次地面工程链测试已得到三个真实PWM ACK，禁止将ACK写成有机械位置传感器的反馈。工作台服务仍关闭，本轮没有通过工作台启动飞行。

## 2026-10-06 手工现场坐标与H结果推广

04走廊接H、08整场卡片支持手填有序走廊点（每行x,y,agl）、H中心、墙面几何和可选flight_area JSON。米制、相对起飞点：+X朝场内，+Y向左；agl是FC中心离地高度。原工程自动生成H观察/下降尾段，不要重复加入走廊点。04本身就是走廊接H，未增加纯走廊任务。

勾选手动输入 → 填实测值 → 本机生成并预览坐标命令 → 连接后的配置检查 → preview → 按原流程flight。配置检查在板端使用正式生成器展开并验证，生成logs/flight_workbench/site_overlays独立文件，原现场YAML不变。可选几何/范围留空继承原值。基础文件变化后需重新生成草稿版本，不能静默改写已预览的overlay。

离线命令生成不连接飞机；页面草稿可跨卡片/状态更新保留，刷新整页不持久保存。新API POST /api/trial/command只生成字符串，启动仍需原确认及预览命令一致性检查。修改坐标或任务选项后请重新生成。

03/04/08本地硬件末段为H对准下降后POSCTL，飞手完成落地；本轮未上板。第五组与08运动优化默认开启但巡航上限仍0.5m/s。工作台优化复选框是显式开启，未勾选继承配置；09新卡片默认1.2m/s，可选1.0/0.5对照。完整说明见[九组速度与推广报告](../../docs/deployment/h_promotion_20261006/REPORT.md)。

新增验证：tests/test_geometry.py（8项）与tests/browser_geometry.mjs（9项Edge离线检查）。
