# Orange Pi 5 现场舵机参考包

来源：`orangepi@192.168.3.126:~/liftrace_board_trials_20260928/patrol_uav_ws-patrol_planner/src/actuator_pwm`，实体源码，不依赖其他工程的符号链接。此目录是 5 的接线档案；5 Plus 仍使用其独立参考包，禁止混用。

2026-10-07 更新：左仓改用 **PWM3_M0 / Pin15**，三路 pwmchip 编号改为**按硬件地址运行时解析**。

|槽号|位置|PWM与设备地址|初始/释放脉宽ns|
|---|---|---|---|
|1|后仓，现场Pin11标记|febf0020.pwm|1700000 / 700000（现场确认伸出锁止、缩回开仓）|
|2|右仓，现场Pin7标记|febf0030.pwm|1000000 / 2100000|
|3|左仓，现场Pin15标记|fd8b0030.pwm（PWM3_M0）|1100000 / 2100000|

周期 20000000ns、normal 极性、每次脉冲 1 秒。设备地址和三槽映射来自现场源文件；不是根据 5 Plus 源码猜测。未经核对不要在其他机型上使用。

**表内不再给出 pwmchip 编号**：编号会随设备树叠加层变化，节点与 init_pwm.sh 都在运行时按 device 地址反查。

## 左仓为什么以前“初始化不动”

左仓原来是按 `fd8b0010.pwm`（PWM1 / Pin16 / GPIO1_D3）驱动的，但现场左仓舵机实际接在
**Pin15 = GPIO0_D4 = PWM3_M0 = fd8b0030.pwm**。PWM3 默认在设备树里未启用，
所以该引脚一直没有波形输出，左仓在启动复位和投放时都不动，而后仓、右仓正常。

启用方式：把 `pwm3-m0` 加入 `/boot/orangepiEnv.txt` 的 `overlays` 并重启
（当前该文件为 `overlays=uart1-m1 wifi-ap6275p pwm15-m2 pwm14-m1 pwm1-m1 pwm3-m0`，
改动前备份 `/boot/orangepiEnv.txt.before_ground_pwm3_20261007`）。

两个必须知道的连带影响：

1. **pwmchip 编号全部改变**。当前枚举为：后仓 `febf0020.pwm`=pwmchip5、右仓 `febf0030.pwm`=pwmchip6、
   左仓 `fd8b0030.pwm`=pwmchip2（另有 pwmchip0=`fd8b0010.pwm`、pwmchip1=`fd8b0020.pwm` 背光、
   pwmchip3=`febd0020.pwm` 背光、pwmchip4=`febf0000.pwm` 风扇）。
   过去写死的 4/5/0 在此配置下分别指向风扇/后仓/旧左仓，**完全错误**；
   本包改为按地址解析后，叠加层开与关两种配置都能正确工作。
2. **恢复旧 `orangepiEnv.txt` 会同时去掉 pwm3-m0**，左仓立即失效，且编号回到旧值。
   本包仍能按地址正确定位后仓/右仓，但左仓会因 `fd8b0030.pwm` 缺失而拒绝启动（fail-closed）。

补回已有的 CheckedPulse 修复：每一步 disable/duty/enable/disable 检查返回；sysfs 写入后读回；
三槽启动检查全部成功才开放 raw 服务；被动路径退出零写入，执行过写事务才在退出禁用输出；不 unexport；错误返回 False。
此反馈确认的是 PWM 配置/操作，不是机械位置或实际载荷离机。

## 服务接口（不可改）

`launch_all.launch` 把节点的 `Servo` 重映射到 **`/legacy/Servo_raw`**，类型 `patrol_control/Servo`
（`int32 req` → `bool res`）。任务链的对外 `/Servo` 由 `uav_mission` 的 `guarded_servo_proxy`
独占，只有拿到 `/mission/release_permission` 才会转发到本节点。

因此：

- 必须以 `roslaunch actuator_pwm launch_all.launch` 启动。直接 `rosrun actuator_pwm pwm_node1`
  不会应用重映射，节点会去抢注 `/Servo`，与许可代理冲突。
- 地面单测目录 `~/servo_ground_test_pwm3/` 用的是 `/ground_test/Servo`，那是地面测试专用名字，
  **不要**带进飞行链。

## 启动被动化（2026-10-07）

`~initialize_on_startup` 默认 `false`，launch 同名参数也默认 `false`。
三槽依次只读检查 `period=20000000`、`polarity=normal`、`enable=0`，
并检查 period/duty_cycle/polarity/enable 的可写权限。启动不改 duty，不输出脉冲；
任一读检查、权限、设备地址解析失败即退出，不注册 raw 服务，析构亦不写。
被动模式不要求 duty=0，保留现有占空值；不会因 duty 保留后仓700000而开启输出。
C++构造函数的第四参数仅为无硬件测试注入临时 sysfs 根目录，节点使用系统默认根目录。

已装载并机械锁止时必须保持默认 `false`。只有显式设为 `true` 才执行旧的
逐步返回值检查复位事务（disable/duty0/period/polarity/disable/initial/enable/1s/disable）；
该选项仅用于非装载的人工机构准备，不能作为装载后的启动或恢复方法。
后仓锁止值已据现场反馈改为1700000ns，释放值改为700000ns；左右未改。
`init_pwm.sh` 原样保留，脚本会配置/写 sysfs，并非被动启动的一部分；
本次只部署源码并构建，不执行该脚本，不执行节点或任何服务请求。

**当前旧服务PID不重启。** 覆盖源码、编译或修改ROS参数不会把新常量切入当前oldservice；
默认被动启动和后仓新方向只对下次使用新二进制创建的进程生效。
当前oldservice仍携带旧后仓释放1700000ns/启动700000ns，不可据此次编译视为已切换。

构建：加载上述环境后执行 `cmake --build patrol_uav_ws-patrol_planner/build --target pwm_node1 -j2`，
本次只构建该target，不执行整工程构建或服务重启。本包复用当前工程生成的 `patrol_control/Servo`，调用客户端同样需要 source 该工程环境。

## 验证与限制

- 2026-10-03 板端重新编译成功，CheckedPulse 本地故障注入通过；两轮地面未解锁验收使用隔离的合成视觉/对齐证据
  进入真实 strict arbiter→guarded proxy→本包 PWM，1/2/3 成功，无许可、错槽与重复槽拒绝。
- 2026-10-07 左仓切到 PWM3_M0（Pin15）并加入设备树叠加层；`pwm3-m0` 生效后
  `fd8b0030.pwm` 枚举为 pwmchip2，启动复位的三路输出最终均为 disabled，未发送投放请求。
- 本包尚未在 PWM3_M0 配置下做过完整投放验收；机械作动需要现场观察确认。
- 本轮没有改变飞行配置、没有改 `patrol_control` 状态机。

## 本补丁本地验证

独立 CMake 只编译 pwm_node1 和 actuator_startup_tests；CTest通过。
无硬件测试覆盖只读配置/权限检查、默认zero writes、错误配置拒绝、
显式复位事务与逐步故障、三槽投放常量及checkedPulse故障、生产控制器析构。
生产控制器仅访问测试临时目录，inotify验证被动构造/启动/失败/析构没有IN_MODIFY，
并实际观测配置文件读取。未运行实际ROS节点、SSH、PWM或仿真；未做真实释放测试。
