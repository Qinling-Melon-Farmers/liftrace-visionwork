# 独立整机入口交接（2026-10-08）

F=/home/xhj/liftrace-worktrees/r2026-board-frame-fix，起点7b6ca237。仅修改F，未连接板端、未启动ROS/仿真/机构、未提交、未改共享变更记录。main整合、导航同步和提交由主代理负责。

运行实现按已部署且用户确认08整场链路验收的本地存档选择性对齐。比赛参数保持原样：正式field.example与rectangle/snake候选的全部解析值逐项等于修改前；只增加中文注释。成功有限空间配置独立归档，不作为比赛默认，不以生成文件数值完全一致作为目标。

## 主代理传播清单

- [RUNTIME_FILES.txt](RUNTIME_FILES.txt)：本轮实际修改/新增的运行文件；包含deployment/competition、实体actuator、限高控制补丁和硬件任务适配器。
- [UNCHANGED_REQUIRED_FILES.txt](UNCHANGED_REQUIRED_FILES.txt)：独立入口未改但必需的环境/launch/共有session/配置闭包。B有些对应文件不存在，不能以B HEAD整目录覆盖F；目标缺失时从F选择性补齐。
- [VERIFICATION_FILES.txt](VERIFICATION_FILES.txt)：离线接线检查及回归。
- [SOURCE_COMMITS.json](SOURCE_COMMITS.json)：来源提交、原作者与日期；保留所有贡献分支与作者，本次无删除分支/squash/改作者。
- [CLI_CONTRACT.md](CLI_CONTRACT.md)：工作台使用的确切接口。

这些清单不是删除清单。主代理在root先保存自己的旧控制快照，再选择性overlay；不得整目录清空原资产。构建产物为本机x86_64，不能当ARM部署程序复制上板。

## 实现与原值保护

competition_supervisor加入--motion-optimization on/off、--obstacle-columns on/off和可选--motion-action-timeout。省略时继承YAML，动作预算未配置时仍用原正式90s，目标120s。--check-config仅离线；--output-dir与--fc-reference同时提供可离线生成。运动优化开关开启仍按既有要求校验实测corridor_speed_schedule。显式关闭写入enabled=false，避免继承旧ROS参数。拒绝把新恢复开关带入正式配置。

environment.sh修复source继承入口位置参数导致catkin setup错误的问题。真实start.sh --help与模板preview --check-config现已通过；不是只检查Python函数。

硬件任务适配器采用已验收08的fixed-board-session初始化与完整任务时限；共有FullManager只增加runtime类型挂钩，仿真默认仍是HighViewFull。任务/证据/释放/规划行为继续复用共有实现，运行不导入board_trials。原90/120预算、前视距离、路径阶段、膨胀、比赛航线/高度/速度/加速度均保留。

patrol_control仅回收d42c5a57的已部署限高补丁：数值边界1e-9，首次拒绝锁定整个XYZ，拒绝非法轨迹及最终限幅产生的非法结果，合格新轨迹通过原门槛后解开保持。不新增自主退出/下降恢复。F修改前源文件及头文件与B/legacy_baseline/20261007_height_hold/patrol_control对应文件逐字相同；该快照包含FILES.txt/SHA256SUMS并有CATKIN_IGNORE，本次引用它，不创建散落备份或重算无关哈希。部署依据为B/docs/deployment/height_margin_20261007/README.md记录的实际0928编译/替换。

actuator_pwm回收B/deployment/actuator_startup_20261007/source/actuator_pwm实体包。后/右/左硬件地址分别febf0020.pwm、febf0030.pwm、fd8b0030.pwm，不使用旧pwmchip编号；后仓release2100us/lock1700us，右/左脉宽保留。initialize_on_startup默认false，三槽只读被动检查合格才注册raw服务，装载时启动不发送复位脉冲。原完整回收快照引用B/legacy_baseline/20261007_servo_startup，来源7日两次servo_field_pull与后续部署记录可追溯。未编译旧诊断/原文件保留，不删除资产。

FAST-LIO生产匹配器已在F，源码无需回拷：与B的差异是EV快照/PredictionState扩展，未纳入。完整构建产物flags.make明确含MP_EN与MP_PROC_NUM=3；定位launch保留OMP_WAIT_POLICY=PASSIVE、OMP_PROC_BIND=close、OMP_PLACES={4},{5},{6},{7}。这是本机编译/配置核对，不虚构当前板端线程采样。

hardware_startup与B mapping_startup逐字相同，hardware_session实际启动主体与成功08共有编排相同，保留静止未解锁参考→定位一致→新鲜FreeDOM→视觉/控制READY→人工解锁与OFFBOARD→稳定起飞后请求任务。轻量bag保留已有压缩图节流/局部膨胀云；不新增视频编码、原始雷达录制或全图录制。十二厘米槽位只核对，未修改；见[SLOT_AUDIT.json](SLOT_AUDIT.json)。

**稳定运行配置不接EV预测。EV预测观察分支冻结保留；本轮不推广完整投影、FC reset接线或新恢复，也不以完成reset方案作为验收前提。** 新恢复由高位代理另行负责，不进入本次F基线。

## 比赛与测试配置的刻意差异

[PARAMETER_COMPARISON.json](PARAMETER_COMPARISON.json)记录正式默认与成功轮差异，以下全保留比赛侧既有值：

| 项目 | 正式field.example | 成功有限测试场地 |
| --- | --- | --- |
| FC高位搜索/软件限高 | 2.6 / 3.0 m | 2.0 / 2.3 m |
| FC释放/H捕获 | 0.45 / 0.9 m | 0.35 / 1.2 m |
| 规划速度/加速度上限 | 1.2 m/s / 1.0 m/s² | 0.5 m/s / 0.35 m/s² |
| 巡航/精密/走廊前视 | 1.0 / 0.4 / 0.15 m | 0.5 / 0.25 / 0.25 m |
| 运动/目标动作预算 | 90 / 120 s | 60 / 60 s |
| 运动优化 | field.example未启用；候选显式启用 | 启用 |
| 场地与搜索区域 | 原约10×10待测量模板 | 现场有限空间测量值 |
| H末段 | 原POSCTL交接 | POSCTL交接 |

两个运动candidates仍保留原相机2.6m、对应FC2.76m设计及全部运动/搜索参数，不能因软件链验收就称该候选速度/完整比赛场地已实跑。

field.example明确中文说明XY坐标、FC与相机AGL、同一ground_z、规则4m与软件限高、门前/门后通过点而非门中心停车点、现场必填和显式开关。门/H仍留空、site_confirmed仍false。

[成功测试原settings](validated_test/settings.yaml)取自B/试飞产物/board_full_mission_20261007_221730/run_metadata.json，而非本地旧namedsite；两门前后Y为1.0/-0.7、-1.1/-2.5。deployment/competition/field_20261007_validated.yaml是独立入口可读取的测试参考，须显式选择。它为兼容独立生成器，将原target_bounds用作外层flight_bounds，原飞行中心范围保留在search_center_bounds；完整原settings单独保留，不伪称适配后逐字相同。独立生成器继承原正式走廊限高阶段/区域及正式任务储备等行为，因此即使选测试参考也不宣称所有生成值完全等同历史08。

## 验证与边界

- 最终完整构建：BUILD_JOBS=2 bash deployment/competition/build.sh，exit0；视觉、导航、LIO、Livox、控制与实体舵机均由F自身overlay构建。日志logs/competition_release_20261008/build_final.log。
- Python相关回归61项PASS：原competition16、新入口9、生产限高5、人工OFFBOARD6、H交接22、既有投后保持2、起飞XY1。
- actuator_startup_tests CTest PASS；只操作临时mock sysfs，生产helper覆盖被动检查/零写入/错误配置/脉宽和读回失败等路径，未执行真实PWM或pwm_node1。
- 四个profile离线launch展开PASS：正式模板、两个原候选、独立测试参考；每个包括preview/flight应用、两种定位对齐和单独建图，无ROS节点启动。
- start.sh --help、模板preview --check-config、bash -n与git diff --check通过；未确认模板flight在启动session前拒绝。
- 原比赛三个YAML解析值不变、十二厘米槽位不变。新增开关on/off、预算缺省/显式覆盖及非法值、硬件session初始化均已测试。

完整结果见[TEST_RESULTS.json](TEST_RESULTS.json)。编译仅有既有依赖/测试夹具告警，没有编译或测试失败。

成功轮metadata中的DEPLOYMENT_MANIFEST仍引用3cc460ba及编译基线e1f067b，它不是所有后续现场改动的逐文件清单。限高与舵机后续改动有本地回收包、补丁、部署说明与来源提交支持，但本轮无板端连接，无法证明当前整个源码目录逐字等同22:17成功轮，也不编造新ARM构建/当前运行一致性。用户确认的08链路验收保持有效；正式新场地/候选参数的动态表现按各自既有验收边界阅读。此次迁移没有额外reset方案验收门槛。

后续由主代理统一记录与提交F、选择性传播到mainintegration并保持来源贡献历史；本代理不提交、不改共享changelog、不合main、不连接或启动设备。


## 最新授权后的提交与导航回流
用户已认可报告和关键diff，并明确授权本代理独占F变更记录、提交/push当前feature及导航两个feature。此前“未提交/不改共享台账”描述的是第一阶段授权边界；本节和2026-10-08台账为后续执行记录。
competition_config仅去除两处重复motion赋值；四profile生成的四个文件前后逐字相同，见CLEANUP_EQUIVALENCE.json；入口9项回归再次通过。运行参数和算法语义未变。
本次F提交文件逐项列于F_COMMIT_FILES.txt；不stage日志、编译产物或无关未跟踪物。导航回流记录将在各目标feature注明本F来源commit及目标自身验证，保留原作者来源，不改main、分支历史或现场YAML。
