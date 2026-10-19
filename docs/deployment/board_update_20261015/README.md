# 2026-10-15 板端0928更新与现场默认入口

板端：`orangepi@10.189.79.232`。工程根目录：`/home/orangepi/liftrace_board_trials_20260928`。

## Git部署结果

- 原板端HEAD `4e880b1e`，包含未在本机0928主分支里的舵机初始化锁修复和GridCost格数上限20000修复，全部保留。
- 本轮合入 `2f0c932a` 和 `a811fd91`，板端最终合并HEAD `23100bcb`。
- 板端直连GitHub TLS失败，使用本机刚推送的Git bundle，经SSH传送后执行Git fetch和merge；不是文件夹覆盖。
- 现场未提交的08下降速度1.5m/s、释放上限0.45m保留。投递目标按用户要求变为0.25m。现场改动仍为本机配置差异，没有静默提交到公共默认。
- 更新前快照、原配置diff和stash引用：`/home/orangepi/deployment_backups/0928_20261015_low_coverage/`。
- 本轮改动为Python、入口和YAML，未更换控制器/LIO二进制，没有启飞、PWM输出或停止其他节点。

## 默认现场与三套路线

无需手填或开启“实测航点”。以下文件缺失会明确报错，不退回旧小场地。显式指定其他现场文件仍优先。

| 路线 | 默认现场YAML（相对工程根目录） | FC搜索高度 | 默认速度/加速度 |
| --- | --- | --- | --- |
| 08高位四边形 | `deployment/board_trials_4x4/08_full_mission/site_waypoint_map_20261015.yaml` | 2.6m | 1.2m/s、1.0m/s² |
| 10牛耕 | `deployment/board_trials_4x4/10_low_coverage_full/site_waypoint_map_20261015.yaml` | 1.8m，可选1.9m | 同上 |
| 10回字形 | 同上，入口选择`nested_rectangles` | 1.8m，可选1.9m | 同上 |

高位四边形为(1,-3.5)→(5.5,-3.5)→(5.5,3.5)→(1,3.5)闭合；低位牛耕6点、回字形11点。几何来自用户确认的waypoint_map。保留5个走廊点及H(8.3,-3.5)，没有猜测新的门前门后点。

三套共同：巡航前视1.0m，其余导航0.6m；H转场1.0m，最终捕获/对准升至1.1m；H下降/交接策略继承现有vh、D2、POSCTL，下降目标0.35m、交接上限0.50m。本轮只改H捕获高度，不把投递0.25m误写成H下降高度。高位软件上限3.2m，低位备份2.3m。低位备份关闭额外虚拟柱和顶棚，保留实体膨胀/避障；高位继承原障碍柱策略。

板端实际无`--site-config`、无`--speed-profile`生成检查：08四边形，以及10两图案×两高度，共5组全部`CONFIG_VALID`。每组实际输出规划1.2/1.0、H捕获1.1、投递0.25、现场释放上限0.45。验证原始输出位于备份目录`verification/`，并已取回本机交付目录同名位置。离线参考FC=(0,0,0)，local Z换算仅用于检查，实际入口仍重新测量起飞参考。

## H旧阻塞检查

板端生产调用向`LandingHighAlignmentWindow`显式传递`external_landing/mark_max_age_sec=0.5`。H观测年龄和相邻H观测间隔均遵循0.5秒；里程计0.2秒新鲜度独立保留。高位确认仍为水平误差≤6cm、水平速度<0.1m/s、连续1秒。

板端控制二进制与对象编译于10月14日07:10，晚于该修复头文件05:25，修复后源码未被本轮覆盖。使用板端生产头文件做4Hz H+100ms传输的独立数值检查：旧0.2秒条件零次通过，当前0.5秒条件41次满足。该检查不启动ROS节点，不代表真实落点验收。

## LIO优化核验边界

- 板端实际CMakeCache：`FAST_LIO_MATCH_THREADS=3`。
- fastlio_mapping编译参数：`MP_EN`、`MP_PROC_NUM=3`、`-fopenmp`、`-O3`；二进制链接`libgomp`。
- 正常定位入口：`OMP_PLACES={4},{5},{6},{7}`，`OMP_PROC_BIND=close`，`OMP_WAIT_POLICY=PASSIVE`。
- board_load保留特征提取、点过滤3、最大迭代3、0.15m体素、20m局部立方体与6m检测范围；关闭位置调试日志。
- 实际MID360硬件配置：关闭path与PCD保存，地图发布1Hz；代码保留有订阅者才发布、匹配缓冲区复用等优化。
- 后续现场预览启动后，再次只读核验PID43046：实际可执行文件来自当前0928目录；环境为PASSIVE/close和大核4～7；主计算线程43046允许CPU4，OpenMP工作线程43166/43167允许CPU5/6。此次已确认实际3线程并行和大核绑定，不能据此保证后续负载下永不积压；没有放宽EV时效门槛。

## 工作台与舵机

工作台运行于http://127.0.0.1:8771/，目标10.189.79.232；只读探针实查live/fresh。08与10默认现场命令均指向上述文件，四组低位图案/高度组合和08在浏览器的预览命令与后端一致。前端热更新不停止已有设备会话。地图搜索路线已修复：牛耕6点、回字形11点和连接段按生产生成器结果显示，自动路线不会写成显式覆盖点。两套工作台均完成浏览器验证，最终包已随提交7f80a762推送。

完整工作台目录：`/home/xhj/liftrace-deliverables/workbench_review_20261015/fixed/flight_workbench`。最终ZIP：`/home/xhj/liftrace-deliverables/workbench_review_20261015/flight_workbench_20261015_low_coverage_shapes.zip`；远端存于本分支`docs/deployment/low_coverage_backup_20261015/`同名文件。共126项、493242字节，不含凭据和会话日志。

现场实际preview会话因SSH连接退出记录为255；不能据此宣称整轮运行READY。该轮主管已退出，定位launch42999与建图launch45412成为孤儿，子进程LIO43046、FreeDOM45439仍在；定位明确`enable_control_output:=false`。用户确认已主动停止设备终端，但另有外部MAVROS实例，收尾时飞控状态读取为空，故没有停止定位子树或重启任何设备。再次运行preview前须确认旧预览残留已结束，避免实例冲突。探针live/fresh只代表探针连接正常，不代表设备全部健康或可飞。

现场已有`pwm_node1`持有舵机独占锁并提供`/legacy/Servo_raw`。5a初始化和第二个5b实例被拒绝是避免重复占用，不是SSH权限问题。本轮未运行舵机初始化或驱动舵机。