> **2026-10-05 当前操作以[九组新版手册](../board_redeploy_20261001/NINE_TRIALS_20261005.md)为准。** 下文为按原日期保留的历史配置/部署记录；不得据旧段落启用自动OFFBOARD、完整点云或独立视频录制。新版优化选项、实投入口、现场高度及待实飞内容已统一说明。

# 现场试飞操作手册（2026-10-01）

2026-10-02本地更新：[看板review与现场点击操作](../flight_workbench_20261002/REVIEW_AND_OPERATIONS.md)。
H的1.0m自动任务高度已适配，走廊/整场有独立自动时序配置；**尚未上板**。以下10-01手动H流程保留供旧板端使用。

适用：香橙派 `/home/orangepi/liftrace_board_trials_20260928` 的现场专项入口。不是完整比赛启动说明，也不使用 Desktop 历史副本。本文仅整理既有操作，没有修改程序。

## 到底开几个终端

**从板端所有节点都未启动开始，有实投时需要6个常驻终端；建议另开第7个用于监测。H专项不需要舵机，只需5个常驻终端，另开1个监测/发送任务开始命令。** 每个终端可以是独立SSH窗口，也可以是一个tmux会话里的窗口。已经运行的设备节点不重复启动。起飞后不需要另外开终端发送任务开始服务。

| 终端 | 常驻内容 | 意义 |
|---|---|---|
| 1 | roscore | ROS通信主节点 |
| 2 | MAVROS | 与飞控通信 |
| 3 | MID360驱动 | 雷达数据 |
| 4 | 相机 | 下视图像 |
| 5 | 舵机服务 | 提供真实释放接口；当前现场版本启动会依次复位三只舵机 |
| 6 | 专项flight入口 | 定位、地图、视觉、规划、任务、控制及bag |
| 7（建议） | 状态监测 | READY、飞控模式、任务进度 |

飞机应已回到起飞点、未解锁、机头朝场内+X；有投递的组别重新装好对应槽位。不要同时运行旧整机launch与本入口。

## 每个终端的公共准备

电脑通过当前IP登录；最后使用的地址如下，换网络后以实际地址为准：

```bash
ssh orangepi@192.168.3.15
cd /home/orangepi/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
```

以下命令均在香橙派执行。每个新终端都先执行cd和source。环境脚本使用板内127.0.0.1通信，不将Wi-Fi地址写入ROS节点通信。

## 按顺序启动

终端1：

```bash
roscore
```

终端2：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

终端3：

```bash
roslaunch uav_mission mid360_driver2.launch \
  user_config_path:="$PWD/deployment/site_20260928/MID360_config.json"
```

雷达专网是板端192.168.1.100、雷达192.168.1.175，不要替换为Wi-Fi网段。

终端4：

```bash
bash deployment/board_trials_4x4/start_camera.sh /dev/video0
```

终端5（实际投递组需要，H专项跳过）：

10月1日现场更新的驱动启动时会依次复位三只舵机，须先确认机构/载荷和人员位置允许动作。重启后先核对PWM芯片2/3/4分别对应febf0000/0010/0020，然后运行现场初始化脚本（导出、周期、权限、极性；不使能输出）：

```bash
sudo bash /home/orangepi/liftrace_deploy/liftrace-visionwork/init_pwm.sh
```

再启动服务：

```bash
/home/orangepi/liftrace_board_trials_20260928/hardware_ws/devel/lib/actuator_pwm/pwm_node1 \
  /Servo:=/legacy/Servo_raw
```

这是10月1日换回旧5Plus后重新编译的舵机可执行文件，本轮尚未通电动作验证。地面手动试舵机不属于普通起飞步骤，本文不安排自动释放测试。

终端6，以第五组为例：

```bash
bash deployment/site_20260928/start_test.sh 5 flight
```

**等待明确的READY后，人工解锁。** 现场配置随后自动请求OFFBOARD、起飞到低空稳定高度、启动任务。它不会自动解锁。无需再手动调用`/navigation/start_mission`。旧README前半部分关于手动服务的描述属于早期操作，不适用于当前现场自动时序。

`INITIALIZING`、`MAPPING_READY`都不等于最终READY。若显示`fc_lio_disagreement`，需等飞控和LIO坐标一致且稳定，不应通过转动机身追逐数值或绕过检查。

终端7，可依次检查；用Ctrl+C结束当前echo再执行下一条：

```bash
rostopic echo /mavros/state
rostopic echo /navigation/mission_status
rostopic echo /uav_high_view/probe_status
rosservice type /legacy/Servo_raw
```

最后一条应为`patrol_control/Servo`。查看第6终端日志能直接看到READY与任务状态。

## 现场组号（不是原八组目录编号）

| 参数 | 内容 | 现场flight释放方式 |
|---|---|---|
| 1 | 低空直飞、视觉中断、单投 | 真实舵机 |
| 2 | 低空连续多投，默认两投 | 真实舵机 |
| 3 | 高位整圈，只记忆 | 不投递 |
| 4 | 高位整圈，记忆后逐个重访 | 真实舵机 |
| 5 | 高位满足支持条件提前中断，低空重访 | 真实舵机 |
| 6 | 高速拍摄专项 | 不投递；另核对该专项速度档 |

例如第三组为`start_test.sh 3 flight`。需要preview时使用`start_test.sh 5 preview`，它不接通飞行控制出口；结束preview后再启动flight，不能两者同时运行。

## 当前现场行为与结束步骤

- 场地范围前方6m、左右±1.5m；高位航线内收到前5.5m、左右±1.1m。
- 高位2m、低位1.4m，均为飞控中心AGL；正常巡航0.5m/s。投递高度独立配置。
- 现场1～6组正常完成后在结束位置下降至飞控中心离地30cm悬停，由飞手落地；不是自动返回起飞点，也不是完整比赛H降落。
- 障碍柱关闭，三维避障保留；虚拟顶棚关闭，静态TF，水平膨胀0.25m。
- 默认录压缩相机、视觉、位姿、任务和轨迹bag，不录独立MP4；累计地图点云默认不录。需要地图诊断时应显式启用`record_map_clouds`，不能拿轻量包解释缺失的地图。
- 飞手改模式后程序不自动抢回。出现异常由飞手接管，不在空中重启应用。
- 落地停机后等待`BAG_CLOSED`及应用退出，再断电。bag位于`logs/board_<专项>_<时间>/`；原包留板端可用U盘拷贝。
- 下一轮放回起飞点并静置，重新运行第6终端入口；设备1～5仍正常时不必重启。换电后需重新检查设备进程，不重复叠加。

## tmux方式

可以只保留一个SSH窗口，在tmux内建立以上6～7个窗口：

```bash
tmux new -s flight
# Ctrl+b 后按 c：新建窗口；Ctrl+b 后按数字：切换窗口
# Ctrl+b 后按 d：脱离会话，进程保留
# 重新登录后恢复：
tmux attach -t flight
```

每个窗口分别执行上面的准备与启动命令。tmux保留远程终端，不替代飞控失联处理或现场接管。

建议未来封装“设备启动”和“trial 5”两个入口：检查单实例、分窗口、等待READY、保留日志、统一退出。**目前没有交付新的tmux一键编排脚本**，不要把上述方案理解为现有功能。

## 今日H识别—降落专项（目录03，不是现场第三组memory）

**本项检验真正的视觉H降落，末段允许程序请求AUTO.LAND，不执行30cm终点悬停。** 如果现场仍要求手动落地，只能验收此前的接近/升高/对齐并由飞手接管，不能将其记作自主降落通过。本文仅准备命令，没有启动飞机。

### 配置与预期路线

H平放在起飞点正前方约2m，即起飞固定坐标(2,0)，机头朝+X。继承已知相机外参和未解锁静置时的地面基准，地面FC Z约0时不手填高度外参。

使用独立 `deployment/site_20260928/h_landing_test_area.yaml`：保留6m×左右±1.5m范围、最高2m、0.5m/s、静态TF、关闭虚拟顶棚/障碍柱、0.25/0.20/0.10m膨胀、轻量bag。只移除30cm终点悬停选项，关闭自动解锁后任务序列；其他组test_area.yaml不改。

预期：原地到1.0m → (1.3,0) → H附近(2,0)，均1.0m → 同XY升至1.8m → LAND阶段识别H并视觉对齐 → 按控制器条件下降 → 约0.55m的AUTO.LAND交接高度配置 → 飞控落地。数值均为FC中心AGL；达到高度本身不构成降落许可，还需要相应视觉/状态条件。

采用手动启动任务的原因：03_h_landing起飞航点为1.0m，但通用自动序列检查reference.low_z对应1.4m。这次不修改控制代码，用明确的手动任务入口避免悬停等待。

### 启动顺序和命令

1. 每个终端按上文SSH、cd、source。启动终端1 roscore、2 MAVROS、3雷达、4相机。**跳过舵机终端**；H专项不投递，不能使用start_real.sh，也不要执行`start_test.sh 3`（那是memory）。
2. 新旧专项不能同时运行。飞机未解锁静置，在应用终端先做纯配置检查（不启动ROS节点）：

```bash
bash deployment/board_trials_4x4/03_h_landing/start.sh preview \
  --site-config deployment/site_20260928/h_landing_test_area.yaml --check-config
```

3. 如需先检查图像和定位，可去掉`--check-config`运行preview；退出并等录包关闭后再运行flight。实际试飞入口为：

```bash
bash deployment/board_trials_4x4/03_h_landing/start.sh flight \
  --site-config deployment/site_20260928/h_landing_test_area.yaml
```

4. 等最终`READY`。飞手按现场流程解锁、进入OFFBOARD，确认原地1.0m稳定；本配置不会自动请求OFFBOARD或调用任务开始。在监测终端执行一次：

```bash
rosservice call /navigation/start_mission "{}"
```

必须确认服务返回success=true。勿在仍INITIALIZING时调用，勿为了催促重复调用。实际起飞/降落动作由现场飞手决定执行时机。

5. 保持应用终端运行，监测终端可分别查看：

```bash
rostopic echo /navigation/mission_status
rostopic echo /uav_vision/detections_mapped
rostopic echo /mavros/state
rostopic echo /mavros/extended_state
```

H是几何检测链，不是五分类YOLO新增的第六类。预览/普通搜索阶段未出现H标记不等于H链故障，要结合进入LAND后的模式、新鲜H输出和对齐响应判断。

### 判定与收尾

记录是否完成：接近H、定点升至1.8m、有效H观测与对齐、下降/AUTO.LAND交接、ON_GROUND和未解锁。任务终态也单独记录；只有飞手落地或仅到达(2,0)，不算自主视觉降落通过。若任务仍LAND但已落地，记录为终态交接待查，不能把预设坐标到达当作CV成功。

飞手接管后程序不应抢回；不要空中重启。落地停机后等待BAG_CLOSED和应用退出。轻量包位于`logs/board_landing_<时间>/`，含相机、视觉和任务链；没有累计地图点云及独立MP4。

### 当前旧板设备差异

10月1日检查时有线口为enP4p65s0/enP3p49s0（均DOWN），不是eth0。接好雷达后确认实际接口与192.168.1.100地址；未确认接口前不要照抄旧eth0配置。飞控/dev/ttyACM0和相机/dev/video0也需接线后实际确认。

## 远端同步状态（10月1日直查）

视觉高位研究 `feat/high-view-search-research`：bc74a32；导航接替开发 `feat/high-view-liveness-20260919`：5d3f803（origin与fork均已推）。公共修复包含轨迹先于航点的交接竞态、下降净空接线；现场限高限速不覆盖研究配置。九组完整部署和本手册维护在视觉板端分支及导航“板端参考分支”。试飞组“板载代码”分支未被改写。


最新设备检查见[硬件检查记录](../board_redeploy_20261001/HARDWARE_CHECK.md)，包括雷达临时地址、现场舵机差异及尚未验证项。
