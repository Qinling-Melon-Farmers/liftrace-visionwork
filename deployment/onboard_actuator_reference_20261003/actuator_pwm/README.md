# Orange Pi 5现场舵机参考包（2026-10-03）

来源：`orangepi@192.168.3.126:~/liftrace_board_trials_20260928/patrol_uav_ws-patrol_planner/src/actuator_pwm`，实体源码，不依赖其他工程的符号链接。此目录是5的接线档案；5 Plus仍使用其独立参考包，禁止混用。

|槽号|位置|PWM与设备地址|初始/释放脉宽ns|
|---|---|---|---|
|1|后仓，现场Pin11标记|pwmchip4 / febf0020.pwm|700000 / 1700000|
|2|右仓，现场Pin7标记|pwmchip5 / febf0030.pwm|1000000 / 2100000|
|3|左仓，现场Pin16标记|pwmchip0 / fd8b0010.pwm|1100000 / 2100000|

周期20000000ns、normal极性、每次脉冲1秒。设备地址和三槽映射来自现场源文件；不是根据5 Plus源码猜测。未经核对不要在其他机型上使用。

补回已有的CheckedPulse修复：每一步disable/duty/enable/disable检查返回；sysfs写入后读回；三槽初始化全部成功才开放raw服务；退出只禁用输出、不unexport；错误返回False。此反馈确认的是PWM配置/操作，不是机械位置或实际载荷离机。

当前板端原包快照：`~/liftrace_board_trials_20260928/legacy_baseline/20261003_servo_feedback/`，含原文件清单及SHA256；本地取回源文件在本轮忽略的logs目录。没有改旧patrol_control状态机。

## 现场启动

明确允许机构复位后，先启动roscore，然后在终端执行：

```bash
cd ~/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
sudo bash patrol_uav_ws-patrol_planner/src/actuator_pwm/init_pwm.sh
roslaunch actuator_pwm launch_all.launch
```

初始化脚本只导出通道、设置权限/周期/极性并关闭输出；**ROS服务启动会依次复位三个槽**。不要用反复重启服务的方式重试投递，重启本来就会复位。`launch_all.launch`把Servo重映射到 `/legacy/Servo_raw`，正常任务必须经过许可代理，不应在飞行中手动调用raw。

构建：加载上述环境后执行 `cmake --build patrol_uav_ws-patrol_planner/build --target pwm_node1 -j2`。本包复用当前工程生成的 `patrol_control/Servo`，调用客户端同样需要source该工程环境。

## 验证与限制

10月3日板端重新编译成功，CheckedPulse本地故障注入通过。两轮地面未解锁验收使用**隔离的合成视觉/对齐证据**进入真实strict arbiter→guarded proxy→本包PWM，1/2/3成功，无许可、错槽与重复槽拒绝。测试结束三路enable=0，节点退出；现场第二轮确认三槽均正常作动，回位来自机构后方填充物顶回，按用户要求保持时序。本轮不是飞行/视觉识别验收，没有改变飞行配置。
