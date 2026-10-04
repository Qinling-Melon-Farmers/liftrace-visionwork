# 10月4日现场更新与入口区分

SSH为 `orangepi@192.168.3.126`。本次部署目标是 `/home/orangepi/liftrace_board_trials_20260928`，不是今天日志实际使用的 `/home/orangepi/liftrace_r64_onboard_405bda42`。后者本轮没有被更新；继续运行旧r64入口不会自动获得0928补丁。

## 已部署内容

以视觉板端分支 `daa1488e` 为来源，应用 `61e54388..daa1488e` 中的34个源码、配置和测试文件差异：

- H反光检测强化：自适应窗口81、闭运算7；原有结构与质量判断保留。
- `landing` 阶段暂停类别YOLO/RKNN推理并丢弃跨阶段的在途结果；走廊阶段不等于landing。
- 监督器不再因解锁主动请求OFFBOARD；飞手解锁并手动拨入OFFBOARD，稳定悬停后才自动请求任务。
- H降落交接要求新鲜飞控状态；人工切走后取消旧LAND及旧轨迹的恢复，不自动抢回。
- 投递像素偏移使用真实观测时间，与镜头离地高度和内参配套换算；相关控制头文件、专项生成器和launch一并部署。
- 统一现场 `h/landing` 快捷入口。

采用现场源码上应用差异的方式，保留试飞组新增的RK3588大核绑定。没有整目录覆盖，也没有修改现场FAST-LIO、位姿跳变保护、场地YAML、舵机或轻量录包配置。投递测高修改与H识别修改是两个独立更新，不能把前者遗漏后只复制控制器。

板端回滚目录：`/home/orangepi/liftrace_patch_20261004_public`，含manifest、逐文件before、新文件以及旧控制包快照。旧包快照不参与编译。本地部署记录：`logs/field_update_20261004/`。

## 验证结果与边界

- 板端 `uav_vision` 与 `patrol_control` 实际编译通过。
- 51项离线检查通过：人工模式启动6、现场路由8、类别阶段门控10、观测时间5、测高7、H接管15。
- H入口 `--check-config` 返回 `CONFIG_VALID; no ROS nodes started`。
- 34个部署文件与准备版本一致；13个现场保留文件与部署前一致。
- 检查结束未运行ROS/控制/舵机节点。本轮没有解锁、PWM、模式服务调用或试飞，不把编译和离线测试称为实飞验收。

现场可先执行纯配置检查：

```bash
cd /home/orangepi/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
bash deployment/site_20260928/start_test.sh h preview --check-config
```

本记录不授权因脚本更新而立即起飞。今日故障仍需按[坠机复盘](CRASH_REVIEW_20261004.md)处理。

## 正赛入口核查

本机独立正赛入口为 `feat/r2026-competition-integrated` 的 `deployment/competition/start.sh`，调用完整 `HighViewFull`；没有整套加载H专项或现场2m/0.5m/s/30cm悬停档。正赛模板原保留2.6m高位、1.2m/s、三投接走廊及H降落。

但正赛示例的H捕获高度1.8m和默认关闭笔画兜底，与已通过seed31/38研究入口约0.9m/开启兜底不同；本轮在整机分支单独修正该入口参数并验证生成，不把现场专项的1.8m整体替换。修复 `cb19c20f` 已推送视觉远端 `feat/r2026-competition-integrated`，14项配置、18项工作台检查及实际launch参数展开通过。既有正赛field.yaml需显式补入 `landing_enable_h_stroke_fallback: true`。参数对齐不是独立实机整场验收。

此次板端0928补丁没有安装新的独立正赛发行包，也没有更改H专项现场观察高度或全局笔画兜底默认值。不能把“公共补丁已部署”理解为“旧r64已变成最新正赛工程”。

## FAST-LIO取回与运行身份

11套板内源码已归档到本地 `logs/field_update_20261004/board_snapshot/`，配套构建信息在 `board_build_inventory.json`。0928的构建参数包含 `MP_EN / MP_PROC_NUM=3`；今天ROS日志实际指向旧r64可执行文件，其构建记录是 `MP_PROC_NUM=1`、没有 `MP_EN`。两者必须分开评价，编译记录也不能替代运行时负载测量。

前线修改是原有最近邻/平面残差逐点循环的OpenMP并行及CPU绑定，没有发现新增独立IMU预测或异步EKF。专用 `feat/ev-continuity-20261004` 已支持显式匹配线程数；本轮修复匹配点云clear后按索引写入的越界及固定100000点缓存边界，提交 `df29c381` 已推送视觉远端。三线程构建、10组实际匹配场景及CTest 4/4通过；没有直接复制前线整个定位包，也没有把实验性EV预测器切到飞控输入。该修复仍只在EV实验分支，不在本次0928部署中。

## 日志和LIO门槛

已取回今天三批旧r64的ROS文字日志。用户随后提供70/71/72号ULog并指定72为坠机。72号包含24条MAVLink时间同步记录，以 `板端epoch秒 = PX4启动秒 - estimated_offset/1e6` 换算后，启动133.356944秒的1.8→0.8m高度目标切换对应板端17:54:07.551，与6d663ab0轮ROS接受该目标的17:54:07.549仅差约2ms。这支持关联两份记录；它不等于独立校准了真实UTC，也不能用ULog文件名时间直接匹配。报告以相对时间描述因果顺序。原始日志保留、不入Git。

本轮没有放宽LIO源数据过期门槛。先区分输入年龄、桥接拒收、PX4融合超时、任务位姿保护与电机/电源事件，具体分析见坠机复盘。旧日志中的高度重置结论不能直接当作今日事故原因。
