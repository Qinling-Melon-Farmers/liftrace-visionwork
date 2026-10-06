# 0928工程更新与低空观察部署（2026-10-06）

已部署到 `orangepi@192.168.3.126:/home/orangepi/liftrace_board_trials_20260928` 原工程，并完成ARM整包构建及离线验证。本轮没有启动ROS master、实时雷达、飞控、相机、舵机或飞行任务；没有改动PX4参数。部署前试飞分支为 `7a7cb6c0`，本报告所在提交包含本轮默认值、入口、观察套件和部署记录。

## 已部署内容

| 内容 | 当前状态 |
|---|---|
| 低空动力观察 | 独立0.60m悬停、前移0.5/1.0/1.5m、1m四边形；结束悬停人工降落 |
| 任务/投递补丁 | 异步舵机、执行事实及槽位锁定、版本许可与乱序回执、未执行时序误拒绝局部重试、新图像不吞旧动作成功回执；消息、服务、控制器与消费者成套重编 |
| 导航补丁 | 速度坐标转换、过期降档恢复、共同限高及pending就绪/超时、搜索及投后走廊收尾同ID重发、全局直线偏好、完成状态修复 |
| 高位与视觉 | 当前类别新鲜度/低位复核交接修复、高位续扫代码、H反光与笔画兜底、H阶段YOLO门控、人工OFFBOARD及接管收口；现有投递圆环0.75配置保留 |
| 运动优化 | 九组settings均默认enabled=true，保留各专项的禁用项及原子配置；H/走廊/记忆/采集不因此增加投递或moving recovery |
| 航线 | 新现场入口可选rectangle、snake2、snake3；使用原现场小场地范围和FC 2m，不冒充正赛10×10m全场覆盖 |
| FAST-LIO | 三线程并行匹配、并行容器/地图盒/时间戳/协方差及按订阅输出修复、实时诊断；保留现场OMP绑核/等待设置 |
| EV预测/reset | 独立观察套件及可选完整LIO快照已部署；默认不启动，未接回正式EV/控制，未标定reset路径不放行 |
| 工具与录制 | 新ULog分析工具、工作台工具已复制；低空仅诊断bag＋飞控SD ULog，无相机、JPEG编码、视频或点云 |

这些是已入试飞分支的可靠候选与本轮默认配置，不表示全部已经完成同版实飞验收。真实超高回入、纯虚拟柱退出、可信在线FC reset事件和低层HOLD接续尚未实现完整验收，不宣称已经根治高度重置。

## 三线程确认

板端实际构建缓存为 `FAST_LIO_MATCH_THREADS:STRING=3`；生产二进制编译参数含 `MP_EN`、`MP_PROC_NUM=3`、`-fopenmp`，链接 `libgomp`，source现场环境后所有动态依赖解析正常。rospack定位至0928内FAST_LIO。

在ARM上直接调用生产匹配函数、实际ikd-tree与OpenMP的离线测试通过：

```text
PASS: 10 production matching cases; workers=3; maximum scan=100019; checked vector indexing
```

因此已验证编译配置和实际三线程执行，不只是设置环境变量；尚未启动实时LIO，不能声称本轮传感器输入/在线时延已验收。启动时继续选择0928环境，勿启动其他旧目录的R64程序。`top_level_scripts/build_competition.sh`现默认显式传3，防止以后ARM干净构建退回自动单线程。

现场OMP设置为 `OMP_WAIT_POLICY=PASSIVE`、`OMP_PROC_BIND=close`、`OMP_PLACES=cores`，保留并同步回本地。预测Python入口单独设置OMP_NUM_THREADS=1只约束该观察进程，不改变LIO生产匹配线程数。

## 默认启动设置

| 入口 | 默认行为 |
|---|---|
| `deployment/low_hover_observation/start.sh` | 先preview；flight需飞手手动解锁并重新拨入OFFBOARD；FC估计AGL0.60m，上升0.15m/s、水平0.20m/s；无自主降落/舵机 |
| 原 `deployment/site_20260928/start_test.sh` | 保留原实测航点和实投语义，投递组flight仍走真实许可舵机；九组运动优化现在默认开启 |
| 新 `deployment/site_20261006/start_test.sh` | 默认preview、rectangle、motion开启；flight默认mock，实投必须显式`--real-release`；采集速度先0.5m/s，可显式选1.0/1.2 |
| 高位续扫 | 代码已部署；新入口06/08可`--resume-survey on`，默认不打开 |
| EV观察 | 默认preview不连master；显式shadow/reset --start才启动观察；正式EV桥保持原链 |
| 完整状态快照 | 同一LIO的`prediction_state_enabled`默认false，观察时在地面启动定位入口显式true |

现场普通搜索/飞行范围沿用前方6m、左右±1.5m，高位FC中心AGL/上限2m，收尾0.3m悬停；H专项沿用H自动降落。四边形/双线/三线在原survey范围X[0.8,5.5]、Y[-1.1,1.1]生成，不修改现有相机、TF、舵机映射或位姿保护策略。

04走廊、08整机的现场YAML仍缺真实走廊点/H坐标，因此配置检查应拒绝；不能把代码上板说成这两组现在可以起飞。正赛10×10m模板未拿来覆盖这份狭小场地档案。九组模块是01单投、02高位整圈重访、03H降落、04走廊+H、05低空多投、06高位集齐中断、07记忆、08整机、09高速采集；现场编号与目录编号不同，见新入口说明。

## 操作入口

低空完整五终端顺序、手动OFFBOARD和收尾：[低空操作手册](../../../deployment/low_hover_observation/README.md)。本轮只做下面无连接预览，没有执行flight：

```bash
cd ~/liftrace_board_trials_20260928
bash deployment/low_hover_observation/start.sh preview hover
bash deployment/low_hover_observation/start.sh preview forward
bash deployment/low_hover_observation/start.sh preview square
bash deployment/site_20261006/start_test.sh 5 preview --survey-pattern snake3 --check-config
bash deployment/ev_observation_20261006/start.sh check
```

路线、motion和实投参数：[现场入口](../../../deployment/site_20261006/README.md)。预测/raw/smooth对照及独立轻量采集：[EV观察说明](../../../deployment/ev_observation_20261006/README.md)。预测输出只到 `/ev_shadow/*`，reset候选只到 `/ev_task_boundary/*`；没有向MAVROS或舵机的正式输出接线。无权威reset/独立LIO健康生产者及已验收标定，reset观察默认只能报告未就绪。

## 验证与恢复

- ARM视觉及导航整包构建通过；可选快照二次增量构建通过，新消息由当前0928 devel加载。
- 专项105项：103通过、2跳过；低空46项通过；现场入口10项通过；三种航线离线配置检查通过。
- ARM EV数值/隔离65项：64通过、1跳过（板端无原始研究工作树）；实际生成ROS消息18项：17通过、1跳过（仅适用临时fixture的拒绝用例）。本机相应用例65/65及18/18通过。
- ARM生产匹配10项通过，workers=3。没有运行完整ROS图、SITL或飞行。末次检查未发现ttyACM设备，飞控连接需在正式初始化时核查；本报告不代表飞行READY。
- 小型结果见[checks.json](checks.json)；全量构建/回归日志在本地`logs/board_refresh_20261006/`和板端`deployment_results/refresh_20261006/`。
- 板端备份`~/board_deploy_backups/20261006_updates/`保留更新前文件、现场档案及旧控制完整快照。现场位姿策略、相机模型/外参、舵机包没有整包覆盖。
- 沿用板端既有排除：trial_recorder.py、finish_recording.py、render_trial_replay.py不部署；它们留在本机做离线回放。普通九组仍按原轻量bag方案记录压缩图/小范围地图，低空观察完全不启用JPEG链。


### 2026-10-06现场绑核补充
现场实测OMP_PLACES=cores会把LIO主线程及匹配工作线程固定到RK3588的0/1/2号A55小核，出现约1.01s输出年龄及8帧积压。改为可配置lio_omp_places={4},{5},{6},{7}后，实测输出年龄约0.039s、队列0，工作线程位于4/5/6；这是单次地面样本，不是长期时延分位数。两个板端定位launch现默认大核范围，可按硬件通过同名参数覆盖；匹配线程数仍为3。当前初始化未降低300ms时效或5度对齐门槛。

2026-10-06补充：[H专项FC 1.2m与板上待实飞更新清单](H_1P2_AND_PENDING_FLIGHTS.md)。03识别高度已部署为1.2m，接近仍为1.0m，其他专项高度不变。
