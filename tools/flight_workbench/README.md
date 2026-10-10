# 试飞验证看板（flight_workbench）

前线可分发工作台 ZIP 说明见 [README_DISTRIBUTION.md](README_DISTRIBUTION.md)。在既有
`rl_drone` 环境运行 `python tools/flight_workbench/build_zip.py` 即可打包当前工作台文件；
使用明确文件清单，不包含个人状态、日志或板端工程，不要求提交后才能打包。
Windows 通过 `start_windows.bat` / `start_windows.ps1` 使用原生 Python + Paramiko，无需 WSL；Linux 保留原 OpenSSH/PTY 实现。默认端口8771，日志/观察链接使用当前页面同源地址。
独立正赛卡片指向 `deployment/competition/field.example.yaml`，保留比赛FC高位2.6m、规划1.2m/s/1.0m/s²、搜索/接近前视1.0/0.4m、走廊前视0.6/0.4m、投递FC AGL 0.35m与FC软件限高3.2m；场地实测与确认必填，测试场地成功不是10×10正赛验收。运动优化/高位续扫/障碍柱的“继承/开/关”分别省略覆盖或传对应on/off CLI。正式模板默认开启运动优化和高位续扫，历史测试复现保留原值。两项独立，不能从运动优化开关推断续扫。配置检查与运行生成结果分别显示实际值；尚无监督器输出或已改变选择时显示“尚未确认”。报告见 [本轮验证](../../docs/verification/workbench_release_20261008/REPORT.md)。
离线解压启动检查：`python tools/flight_workbench/tests/test_distribution.py`。

2026-10-10：主页和日志页新增 [Bag 离线回放](/replay) 独立入口，详见
[回放集成与验证](REPLAY_INTEGRATION.md)。默认发现本仓 `试飞产物/`、工作台 `data/` 和本机
状态目录的已生成结果；`logs/` 不默认扫描，需在启动 YAML 的 `replay.data_roots` 显式加入。
已有 1008 等 `analysis/` 播放器直接打开，不重新生成。生成仅在本机 ROS Noetic/WSL/Linux
环境可用，Windows 原生只查看已有结果；CPU 为默认编码，NVENC 只加速编码，不加速 OpenCV。
不需 SSH 连接，`--transport local` 和 `--logs-only` 均支持本地回放；原 ULog 功能保留。

2026-10-07：SSH 登录与 sudo 是两次独立认证，SSH 成功不会给新终端继承 sudo 的认证缓存。
旧应答器不识别中文 sudo 提示，故 5a 中文密码提示需要手输；5a `exit 0`、5b
`Servo ready / initialization verified` 均为正常完成标志，不应因提示或重复显示判失败。
现在仅在用户已确认的 5a 初始化会话中复用内存连接口令，识别 `[sudo] password for ...:`
及中文「密码/密碼」提示（含全角冒号、ANSI 和分片）；sudo 每个会话最多自动应答一次。
没有内存口令、关闭自动应答、口令不同/错误、PTY 回显未关闭时仍需人工处理；不改 sudoers。
SSH 登录最多自动应答三次；任意 shell、探针、其他终端、一次性检查以及私钥 passphrase
不会得到 sudo 自动口令。自动输入的口令不会放入命令行、日志、历史缓冲或 SSE 输出。

用户确认执行了两次 5a 初始化，因此重复日志正常，不改执行/输出链路。
5a 为一次性任务：未执行灰色「未初始化」、执行中黄色「初始化中」、exit 0 绿色
「初始化成功」、非零退出红色「初始化失败」，未知退出结果仍不判成功。
绿色仅表示软件初始化成功，无物理动作反馈；常驻 5b 退出仍灰色「已退出」。
终端 tab 与一键启动进度中的 5a 灯共用同一个状态 class，不改任务 READY 阶段。
状态修复兼容现有快照和 session 事件，刷新页面即可加载；无需重启服务或重连 SSH。
sudo 自动应答属于 Python 服务改动，**仅网页刷新不会生效**，必须在设备全部完成正常收尾后
由操作者另行重启本地工作台，再按既有确认启动 5a；不能为了应用修复断开正在运行的会话。
本轮只用模拟 PTY 和隔离实例验证，没有现场 PWM/SSH 操作。

定向自检：`python tools/flight_workbench/tests/test_auth_prompts.py`、
`node tools/flight_workbench/tests/test_terminal_status.js`；无需板端。

2026-10-06新增独立低空观察区，顺序为FC AGL 0.60m悬停、短程前移、矩形；分别原样调用
`deployment/low_hover_observation/start.sh flight hover|forward|square`。先选择观察卡片再点一键设备，
仅编排ROS、MAVROS、MID360与独立定位/EV终端，不启动相机、视觉、规划、任务管理器或舵机。
配置检查和preview均调用原入口的离线preview，不使用九组的`--check-config`。

flight沿用确认框及命令一致性检查，输出进入专项flight终端与运行日志。只有
`READY_FOR_MANUAL_ARM_AND_OFFBOARD`表示预发完成；status JSON用于展示动作/保持/接管，
不会采用常规Mission的相机/地图READY门槛。READY后飞手人工解锁并重新拨入OFFBOARD；
结束继续悬停，飞手手动降落上锁。Ctrl+C仅请求保持接管，不超时强杀；观察入口尚未退出时
拒绝全停、断连和关闭定位/MAVROS等依赖，也拒绝重跑，等待`OBSERVATION_CLOSED`后再操作。
保留原诊断录包默认，不新增JPEG、图像或点云录制。矩形UI名称对应既有`square`，不修改profiles。

新增离线回归为`tests/test_low_observation.py`。用既有rl_drone运行工作台测试时，如缺pexpect，
可临时设置`PYTHONPATH=/usr/lib/python3/dist-packages`复用本机已有纯Python包，无需pip安装。
实际运行仍依赖板端已有low_hover入口、现场ROS环境与FAST-LIO/EV；源码更新后本地工作台
需重新加载配置，已有板端进程不会因刷新网页自动更新。此集成未初始化飞行或实机验证。

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

电机页新增话题诊断：区分无发布者、已订阅但没有消息、消息过期、类型/解析失败及AUX输出bank缺失。
探针在MAVROS离线时也预订阅配置话题，正常订阅由rospy在发布者出现或重启后接续；仅注册失败才重试。
`/mavros/target_actuator_control`独立展示group及8个原始controls，不映射为四台电机，不替代RC OUT或ESC电流/RPM。
已有工作台个人profile若缺此新增项，需在其`probe.observe_topics`添加
`actuator_target: /mavros/target_actuator_control`。现存探针进程不会自动加载新代码和启动参数；重连只读探针后才生效，不能据源码更新宣称上游有电机遥测。

只读采样清单（先在现场终端source原0928环境，不更改参数、消息间隔或启停设备）：

```bash
rosnode info /flight_workbench_probe
rosnode info /mavros
rostopic info /mavros/rc/out
rostopic info /mavros/target_actuator_control
rostopic info /mavros/esc_status
rostopic info /mavros/esc_telemetry
# 每条限时10秒；timeout=124代表窗口内未收到，不代表话题没登记
timeout 10s rostopic echo -n 1 /mavros/rc/out
timeout 10s rostopic echo -n 1 /mavros/target_actuator_control
timeout 10s rostopic echo -n 1 /mavros/esc_status
timeout 10s rostopic echo -n 1 /mavros/esc_telemetry
```

核对RC OUT的数组长度：当前已登记接线使用raw17/20/18/19，需要完整AUX bank；只收到8/16路不能改用MAIN曲线。
若独立echo与探针都收不到，先按飞控到MAVROS的数据流缺失处理；有publisher登记不能证明有包。
ESC硬件/固件没有回传时保留缺项。这里不调用set_message_interval/set_stream_rate或其他飞控服务。

---

## 1. 快速开始

在 WSL（开发机）里运行服务端，浏览器打开提示的地址：

```bash
cd /home/xhj/liftrace-worktrees/r2026-board-vision-tests
bash tools/flight_workbench/start_workbench.sh              # 默认 127.0.0.1:8771
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
| 4 相机 | 终端 tab「4 · 下视相机」 | `bash deployment/competition/start_camera.sh /dev/video0` |
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
   （5a 不输出初始化；5b 默认仅检查并提供服务，不自动复位，界面分别弹确认）。
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

当时5a只准备权限和禁用输出，旧5b服务启动会依次复位三槽；该旧启动行为已于2026-10-07要求取消，当前5b约定为仅检查/提供服务、不自动复位。两次当时的地面工程链测试已得到三个真实PWM ACK，禁止将ACK写成有机械位置传感器的反馈。该历史轮工作台服务关闭，没有通过工作台启动飞行。

## 2026-10-06 手工现场坐标与H结果推广

04走廊接H、08整场卡片支持手填有序走廊点（每行x,y,agl）、H中心、墙面几何和可选flight_area JSON。米制、相对起飞点：+X朝场内，+Y向左；agl是FC中心离地高度。原工程自动生成H观察/下降尾段，不要重复加入走廊点。04本身就是走廊接H，未增加纯走廊任务。

勾选手动输入 → 填实测值 → 本机生成并预览坐标命令 → 连接后的配置检查 → preview → 按原流程flight。配置检查在板端使用正式生成器展开并验证，生成logs/flight_workbench/site_overlays独立文件，原现场YAML不变。可选几何/范围留空继承原值。基础文件变化后需重新生成草稿版本，不能静默改写已预览的overlay。

离线命令生成不连接飞机；页面草稿可跨卡片/状态更新保留，刷新整页不持久保存。新API POST /api/trial/command只生成字符串，启动仍需原确认及预览命令一致性检查。修改坐标或任务选项后请重新生成。

03/04/08本地硬件末段为H对准下降后POSCTL，飞手完成落地；本轮未上板。第五组与08运动优化默认开启但巡航上限仍0.5m/s。工作台优化复选框是显式开启，未勾选继承配置；09新卡片默认1.2m/s，可选1.0/0.5对照。完整说明见[九组速度与推广报告](../../docs/deployment/h_promotion_20261006/REPORT.md)。

新增验证：tests/test_geometry.py（8项）与tests/browser_geometry.mjs（9项Edge离线检查）。

## 2026-10-06 自动搜索航线与理想FOV图

02/06/07/08/09任务卡片可展开实测坐标后勾选“按搜索区自动生成”。填写搜索矩形、内收量，选择矩形/两线/三线，输入镜头离地高度、相对FC的Z偏移及任务X/Y方向有效FOV。生成按钮同时提供蓝色覆盖、橙色盲区和FC目标高度。FOV可由实测地面宽度按2*atan(D/(2*h))换算为度，需核对相机安装方向。

默认现场仍限高2m：偏移−0.16m时，镜头1.84m对应FC2m；镜头2m需要FC2.16m，会被原限高检查拒绝。扩大搜索区不自动扩大允许边界和地图；由可选flight_area按实测值配置。门口、走廊引导点和H不能由场地范围推断，08仍需手填。

新API POST /api/survey/plan只算路线与理想几何覆盖；/api/trial/command生成运行overlay命令。板端检查会复核安装偏移和正式任务配置。范围内覆盖率不是避障后实际覆盖或完整靶标召回率。操作和部署记录见[完整报告](../../docs/deployment/survey_workbench_20261006/REPORT.md)。

## 2026-10-07 日志操作页（本地实现，未现场运行）

主工作台的「录制与下载」打开 `/logs`，沿用主工作台当前连接及 profile 内保存的密码。新增 `orangepi@10.75.120.193` 下拉候选，不覆盖当前 host 或个人连接档案。先在主工作台连接，再刷新状态/文件；页面不会自动启动录制、设备或任务。

- **常规轻量 bag**：复用板端 `deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_bag.py` 的 `TrialBag`，压缩图默认5Hz、相机信息、任务/视觉/位姿状态及小范围 `/sdf_map/occupancy_inflate`；不录原始雷达或全图建图云。相机输入/节流输出通过 `workbench.yaml` 的 `logging.routine_settings` 配置。只是独立录制模块，不调用 `run_trial.py` 或飞行入口。
- **纯诊断 bag**：复用 `deployment/low_hover_observation/record_diagnostics.py --output … --config deployment/low_hover_observation/recording.yaml --duration 秒`；继承现有类型白名单/队列/空间限制，无图像、点云、JPEG。
- **开始/停止**：用户点击并确认后执行。默认300秒，可选1–900秒；工程已有recorder（包括旧任务内置rosbag）在场时拒绝重复开包。用板端独立锁串行创建，登记PID、启动时间和完整argv。停止仅对核对一致的工作台进程发SIGINT；常规bag模块随后只关闭自己创建的rosbag/throttle。未匹配的PID、旧任务bag及其他ROS节点不发送停止信号。
- **ULog**：FC SD内部录制由既有logger/SDLOG设置控制，页面不启动/停止FC logger，不写参数，不发`logging_start`。页面未接入MAVLink streaming录制，也不声称实时得知FC内部logger状态。用户报告2026-10-07只读核实`SDLOG_MODE=0`、`SDLOG_PROFILE=1`，是该次现场信息，不是页面实时采样；mode0解锁时录制。
- **飞控日志取回**：显式查询索引调用板端新版`fetch_px4_ulog.py --list --format json`；旧版不支持索引时提示手填已核实ID，不把FTP目录文件名冒称日志编号。取回调用`--log-id ID --output logs/flight_workbench_recordings/ulog_…/flight.ulg --timeout 600`，同时传配置中的`--namespace`；兼容旧/新下载CLI，不覆盖板端脚本。沿用其connected/disarmed门控、LOG_REQUEST_DATA和finally END，可能影响FC日志传输模式。最大编号不自动选中，不等同“本轮试飞”。取消也只向登记的该下载进程发SIGINT，保留`.part`。
- **状态/下载**：工作台自有操作显示启动中、录制中、传输中、封闭或失败，保留原模块PASS/INCOMPLETE含义及输出；旧recorder只列进程，不冒称其READY。文件列表递归浏览工程`logs/`（150文件、600目录、深度5层），支持专项子目录及既有bag/ULog。筛选后点击下载到浏览器本地目录；工作台自有bag须有closed摘要，活动`.active`/`.part`和仍被打开写入的文件拒绝下载。仅允许安全相对路径并检查resolve仍位于logs内。

下载沿用`wb_ssh.run_bytes`接口，改为stdout/stderr独立pipe；SSH登录通过临时私有Unix socket的askpass助手读取内存/profile密码，助手文件和环境变量不含密码。保留原PTY sudo/SSH修复；下载助手不应答sudo、应用密码或私钥passphrase。文件字节不做文本替换，另外用随机标记+长度+结束标记隔离shell启动输出，校验传输完整及下载期间文件未变化。失败/超时返回错误，不以部分数据报成功，不把凭据放入URL/错误日志。

新API为`POST /api/recording/{status,start,stop,index}`及`GET /api/recording/download?path=相对路径`。同源/Host校验、连接状态、确认词与输入校验在服务端执行；查询与下载不自动开启设备。工作台操作尚未确认结束时拒绝切换连接；用户退出页面/断开SSH会话不会杀这些独立进程，bag时限和ULog下载时限仍生效。进程重启后可从板端登记文件核对状态，须先刷新再操作。

本轮只在隔离本地fixture验证，没有SSH现场连接、录制、舵机操作、仿真或服务重启。probe集成依赖`board_probe.py`/`wb_status.py`当前接口；现场最新日志另行核查。只读比较`/tmp/fetch_px4_ulog_board_20261007.py`，未以本地旧版覆盖板端新版。

最新现场澄清：8791的`/proc/5438/cmdline`为B/tools/flight_workbench/server.py，`--profile-dir /tmp/liftrace-workbench-frontline-ainjon1n/profile`；源码并非/tmp解压副本，Python内存仍是旧版。静态文件修改会立即影响该进程的页面，刷新不会更新Python API。日志入口和“只重连probe”默认隐藏，仅snapshot明确提供对应capability时显示；旧后端刷新页面不会展示这些未支持的操作。直接打开新日志静态页也会先检查snapshot，缺capability时只提示独立后端，不请求录制API。本次已把`wb_logs.py`、日志页三资源及本README加入打包白名单。此轮没有重启旧后端或硬件会话。5b描述/确认默认不输出，当前直接node命令显式传`_initialize_on_startup:=false`。现场launch也已部署支持`initialize_on_startup:=false`；两种CLI语法不能混用。被动启动版已编译并原子替换，原PID12271未重启，后仓现场确认锁止。不新增复位按钮。

如需立即使用日志页，最简方式是从新解压目录**另开日志专用后端**（此处只是交接命令，本轮未执行）：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
bash tools/flight_workbench/start_workbench.sh --logs-only --host 127.0.0.1 --port 8792
```

解压包根目录下入口改为`bash start_workbench.sh …`。浏览器访问新打印的端口（预期`http://127.0.0.1:8792/`）。该模式只开放连接/认证、日志状态/索引/录制/下载，不调用`ensure_probe`、不刷新设备preflight、拒绝设备/任务/舵机/stop_all/重连probe API；启动时不自动SSH。沿用本机profile及工程配置；若旧服务使用临时profile，可加`--profile-dir /已核实的旧profile目录`复用已保存认证，保持页面“保存密码”不勾选。操作者点击连接才执行只读工程/ROS Python检查。若密码未保存、仅存在于旧工作台进程内存，独立页需重新输入；不会读取旧进程内存，默认不保存新密码。停止独立日志后端不关闭旧8791的SSH会话，也不停止旧ROS节点；本页创建的录制/取回进程仍按既有时限/明确停止按钮收尾。也可等待旧工作台按现场流程正常退出后，下一次用新包完整启动。

完整工作台另外接入只读探针watchdog：仅非人工退出以2/5/10/20/30秒退避重连probe；设备/任务/舵机常驻服务不会被watchdog重启。stop_all、断开、probe Ctrl+C/关闭/退出130均取消恢复；退出75提示独占锁冲突，保留原owner并停止自动重试。顶部「只重连 probe」是独立明确操作。snapshot、状态监视、实时曲线和ready判定使用`wb_status.probe_link_status()`的链路/时间状态；缓存过期或会话退出后不继续显示当前设备绿色状态。5a前后端均拒绝对已运行的5b服务再次初始化；提示先按现场流程退出旧服务，不自动杀进程。

用户提供的现场说明：常驻服务启动一次，此轮保持原PID，没有通过本改动重启。15:38 stop_all/Ctrl+C与15:51板端重启造成的SSH中断应区分，后者静态eth0恢复稍晚；probe重连次数不等于常驻服务启动次数。现场雷达已恢复10Hz，statepose已恢复；本功能不把旧缓存或软件ACK当当前物理状态。大bag已有多次SSH断流，密码/二进制完整性修复不代表网络问题已解决；没有断点续传，失败/超时不给部分文件报成功。ULog55已另行取回`/tmp/fcu_log55.ulg`，本轮不读取或改写该现场文件。

独立验证（已有conda + 系统已安装的纯Python依赖，不安装新包）：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
PYTHONPATH=/usr/lib/python3/dist-packages python -m unittest discover -s tools/flight_workbench/tests -p 'test_logs*.py' -v
PYTHONPATH=/usr/lib/python3/dist-packages python -m unittest discover -s tools/flight_workbench/tests -p test_binary_download.py -v
PYTHONPATH=/usr/lib/python3/dist-packages python -m unittest discover -s tools/flight_workbench/tests -p test_auth_prompts.py -v
```

## 2026-10-08 第08组速度与独立工程入口

第08组固定使用 `deployment/board_trials_4x4/08_full_mission/site_20261007_221730.yaml`；
来源是昨日最后221730轮 metadata，不是旧 namedsite。下拉可选有限空间规划0.5/0.35，
或显式传 `--speed-profile competition` 选择1.2/1.0和前视1.0/0.4/0.2m。
两档保留同一现场几何、FC高位2m和本组H观察高度；昨日现场不是10×10正赛场地。
0.6/0.4是走廊前视距离m，1.2/1.0是规划速度/加速度上限，不是实测速度。
仅配置检查走 `--check-config`，普通preview仍会启动节点。

独立正赛下拉包含 `deployment/competition/candidates/rectangle_motion.yaml`、
`snake_motion.yaml` 和 `snake3_motion.yaml`，以各文件配置和场地确认状态为准。

相机按钮共用 `bash deployment/competition/start_camera.sh {video_device}`；
该入口仍启动同一标定launch。默认0928工程的 `connection.env_script` 保留。
若连接独立工程根目录，在连接设置中将此项改为
`deployment/competition/environment.sh`，同时填写实际工程根目录；不自动切根目录。
