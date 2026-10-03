# 第五组现场启动命令（2026-10-03，Orange Pi 5）

本页对应 `orangepi@192.168.43.59`、实体部署 `/home/orangepi/liftrace_board_trials_20260928`。现场第五组是 **06_high_priority**，不是目录05。使用当前现场外参、静态TF、关闭虚拟顶棚和障碍柱、0.25/0.20/0.10m膨胀、0.5m/s、高位2m、低位1.4m、投递0.6m。真实舵机走许可链。

本页提供下一次安排试飞时使用的命令；本轮诊断没有重新启动试飞。17:14这一轮的失败见[本轮复盘](HIGH_PRIORITY_REVIEW_20261003.md)。用户要求保留本机当前“位姿阶跃打印而不抛异常”的实现，本次未改动；不能将它记成与Git内保留连续性异常的版本完全一致。

> 10月3日晚本地修订（尚未上板）：删除解锁后自动请求 OFFBOARD 的行为。以下新版流程须部署后使用；飞机现已断电，本轮不进行部署。H 专项见 [H 启动说明](H_STARTUP_20261003.md)。

## 终端数量和共同准备

从全部进程停止开始，使用 **6个常驻终端，另开1个查看状态**。可以对应tmux的7个窗口；当前命令不会自行创建一键任务。已经运行的设备不重复启动。飞手解锁前必须看到应用READY；本组由飞手人工解锁并拨 OFFBOARD，达到起飞高度且稳定后自动进入任务，无需另发任务启动服务。

每个SSH终端先执行：

```bash
ssh orangepi@192.168.43.59
cd /home/orangepi/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
```

顺序如下。以下均为实机入口，不在笔记本SITL执行。

## 1：ROS主节点

```bash
roscore
```

其他终端依赖此主节点。若已经存在，不再开第二个。

## 2：飞控通信

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

确认 `/mavros/state` 中connected为true。此命令本身不解锁或起飞。

## 3：MID360输入

```bash
roslaunch uav_mission mid360_driver2.launch \
  user_config_path:="$PWD/deployment/site_20260928/MID360_config.json"
```

本机雷达网口为192.168.1.100、雷达192.168.1.175；与Wi-Fi SSH网段分开。任务入口管理后续定位与地图，不在其他终端重复启动一套FAST-LIO。

## 4：相机

```bash
bash deployment/board_trials_4x4/start_camera.sh /dev/video0
```

使用既有相机入口，输出用于视觉和轻量bag；不运行额外MP4编码脚本。

## 5：舵机实体包

飞机地面未解锁，且机构允许复位后执行：

```bash
sudo bash patrol_uav_ws-patrol_planner/src/actuator_pwm/init_pwm.sh
roslaunch actuator_pwm launch_all.launch
```

`roslaunch`会执行三槽初始化/复位。本机已使用主工作区实体源码，不使用旧 `hardware_ws` 或跨目录软链接。此机三槽映射PWM4/5/0。日常试飞不用手工调用三个raw释放服务。

## 6：第五组应用和轻量录包

先只检查配置：

```bash
bash deployment/board_trials_4x4/06_high_priority/start.sh flight \
  --real-release \
  --site-config deployment/site_20260928/test_area.yaml \
  --check-config
```

检查通过且现场准备好后启动：

```bash
bash deployment/site_20260928/start_test.sh 5 flight
```

此命令选择真实投递入口，同时启动任务需要的定位、地图、规划、视觉与许可代理，并使用轻量bag。正常流程为：READY → 飞手解锁并拨 OFFBOARD → 爬升/起飞条件满足 → 高位巡航 → 合格优先线索允许提前中断 → 低位重访、确认、对准与投递 → 回起飞点 → 下降至30cm悬停 → 飞手落地上锁。

**异常路径与正常收尾不同：** 17:14轮对准证据超时后 `mission_failed=true`、0/3投递，返回起飞点后没有满足30cm悬停入口的成功条件，保持在较高位置等待人工处置。不能因状态显示LAND就假定飞机已经进入自动下降。当前诊断没有修改这一逻辑。

## 7：查看状态（逐条运行）

```bash
rosservice type /legacy/Servo_raw
# 期望 patrol_control/Servo
rosnode info /servo_controller1
# Services中应有 /legacy/Servo_raw
rostopic echo /mavros/state
```

`rostopic echo`会持续运行，Ctrl+C后可改看下一项，或再开tmux窗口：

```bash
rostopic echo /navigation/mission_status
rostopic echo /uav_high_view/probe_status
rostopic echo /board_trials/terminal_hover_status
```

终端6出现READY并不代表已经起飞；飞手决定何时解锁和切入 OFFBOARD。阶段、目标数、committed、mission_failed、reason一起看，不能只看COMPLETE或LAND。

## 收尾与ULog

飞手落地上锁后，等待录包正常关闭；保留 `logs/board_high_priority_<时间>/`，再停止应用及各设备终端。不要在空中关闭MAVROS。

PX4当前 `SDLOG_MODE=0`，解锁时已有机内ULog。新工具只读取已封闭日志，不改日志参数或飞控参数。停桨未解锁、保留ROS和MAVROS通信时，可列出目录：

```bash
python3 tools/flight_logs/fetch_px4_ulog.py --list-dir /fs/microsd/log
```

本轮核实的日志编号为781；下一轮不能照抄编号，须按新索引与解锁时序确认：

```bash
python3 tools/flight_logs/fetch_px4_ulog.py --log-id 781 \
  --output /tmp/flight_781.ulg
```

输出或同名`.part`已存在时工具拒绝覆盖；换输出名称即可。工具要求新鲜的未解锁状态，结束/中断后发送LOG_REQUEST_END。飞控文件日期可能不准，回传后需按解锁、模式变化和高度曲线匹配bag。

17:14轮的bag和ULog均已取回本机；本轮下载结束已关闭临时MAVROS与ROS主节点，未重启任务或舵机。
