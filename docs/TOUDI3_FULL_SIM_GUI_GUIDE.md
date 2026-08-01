# toudi3 完整仿真与 GUI 傻瓜式指南

更新时间：2026-08-02
适用范围：WSL2 Ubuntu 20.04、ROS Noetic、PX4 SITL、Gazebo Classic、RViz

本文只启动仿真软件，不启动 `actuator_pwm`，不连接真实飞控，不执行真实舵机投递。

本文是逐步操作和排障手册，不是视觉算法指标定义。
`run_toudi3_full_competition_sim_gui_new.sh` 只用于新视觉节点与仿真输入的人工连通观察；
联合环境问题已由 `toudi3_combined_env.sh` 解决，独立 Gazebo 真值、自动评分和 shadow
最小入口也已落地，但不能仅凭 GUI 框、topic 有输出或一次航线完成宣称视觉有效。正式
L0-L3 验收层级、首轮阈值和当前结果见
[VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md) 与
[VISION_LAPTOP_SIM_BASELINE_20260715.md](/home/xhj/liftrace/docs/VISION_LAPTOP_SIM_BASELINE_20260715.md)。

原 2025 链完整赛程目前仍处于分层排障：自动起飞和三次软件 mock 投递已完成，原始
0.8 m Wall_15 门洞穿越尚未完成。最新证据、航点语义问题、planner 所有权和 world
派生规则见
[2025原始链完整仿真阻塞与WORLD改造方案_20260802.md](/home/xhj/liftrace/docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md)。

## 0. 推荐方式：进入 WSL 后使用 Linux 原生命令

下面正文默认已经进入 Ubuntu WSL。Windows PowerShell 只负责最开始打开 WSL：

```powershell
wsl -d Ubuntu-20.04
```

进入 WSL 后，所有 `source`、`roslaunch`、`rostopic`、日志和停止命令都直接在 Linux 终端执行，不再套 `wsl -e bash -c`。

无 GUI 旧链完整验收建议使用 3 个终端：

1. **终端 1：启动终端**，保持前台运行，看到 roslaunch 的完整日志和最终退出状态；
2. **终端 2：状态终端**，查看 PX4 状态、位姿、航点 setpoint 和关键话题；
3. **终端 3：诊断终端**，可选，用于 `rosnode list`、日志尾部和停止残留进程。

GUI 模式需要额外打开 Gazebo/RViz 窗口，但仍建议保留这 3 个终端。

本次无 GUI 长回归曾使用 `tmux` 将启动终端放到后台，以便跨越工具单次执行时限持续
观察；`tmux` 不是当前手册的必需依赖。普通用户直接按下面的三终端顺序执行即可。
仓库中旧的 `top_level_scripts/simulation.sh`、`vision.sh`、`location.sh` 等历史脚本
会启动实机视觉、`actuator_pwm` 或旧路径，不纳入当前 toudi3 仿真入口。

每个新终端都先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
cd /home/xhj/liftrace
```

第一行加载 ROS 命令；第二行加载主工程节点、消息和 launch；第三行把后续相对路径统一到仓库根目录。

### 0.1 先区分两个入口

这两个命令职责不同，不能把 `patrol_world.launch` 当作完整航路点启动命令：

| 命令 | 会启动什么 | 是否发布航路点 setpoint |
| --- | --- | --- |
| `roslaunch patrol_control patrol_world.launch` | world、PX4、Gazebo、MAVROS 和飞机 | 否；只用于检查场景/飞机/相机 |
| `bash ./top_level_scripts/run_toudi3_full_competition_sim_gui_old.sh` | 上述内容 + `uav_arming_node`、`patrol_control`、旧视觉、规划/建图 | 是；用于完整旧链航路回放 |

如果 Gazebo 中有飞机但 `/mavros/setpoint_position/local` 没有持续消息，说明只启动了
world 入口，尚未启动控制链。完整仿真必须使用第二行入口，或手动另起
`patrol_control_px4_sim.launch`。

还必须区分两种控制所有权：

| 参数 | 实际控制方式 | 能证明什么 |
| --- | --- | --- |
| `waypoint_mode=true` / `flag_planner_px4=true` | 旧控制直接向 MAVROS 发插值航点 | 状态机和预设路径可执行 |
| `waypoint_mode=false` / `flag_planner_px4=false` | 旧控制发 `/fastplanner/goal`，轨迹链给 MAVROS | Fast-Planner 在线规划/避障 |

不要因为 `fast_planner_node` 已启动就认定 planner 正在控制飞机。必须同时核对参数、目标
话题和最终 setpoint 发布者。

## 1. 两条启动路径

| 入口 | 控制/规划 | 视觉节点 | 适合用途 |
|---|---|---|---|
| `run_toudi3_full_competition_sim_gui_old.sh` | 旧 `patrol_control` + 航点模式 | 旧 `circle_detector`、`simple_cross_detect`、`landing_detector_node` | 先验收原有起飞、巡航、检测点等待、降落控制 |
| `run_toudi3_full_competition_sim_gui_new.sh` | 同一套旧控制/规划 | 新 `uav_vision` Phase D 全链 | 检查新视觉链输入、融合、圆环精修、地图投影和兼容输出 |
| `run_toudi3_full_competition_sim_headless_old.sh` | 旧 `patrol_control` + 航点模式 | 旧视觉链 | 推荐的无 GUI 完整赛程回归 |

两条路径不要同时启动。两者会使用同一 Gazebo/PX4/MAVROS 资源，并且部分视觉话题是全局绝对话题。

### 1.1 只观察当前视觉算法（推荐给视觉组）

若只想查看 toudi3、相机画面和当前 `uav_vision` 输出，不需要飞机起飞或旧控制链，使用：

```bash
cd /home/xhj/liftrace
./top_level_scripts/run_toudi3_visual_gui_safe.sh
```

该入口只启动 Gazebo world、固定评测相机、独立真值和 Phase-D 笔记本视觉节点；不会启动
PX4、MAVROS、`patrol_control`、arming 或 `actuator_pwm`。默认相机使用当前已知的 135°
tent 压力位姿，可用 `VISUAL_GUI_CAMERA_X/Y/Z/YAW` 环境变量覆盖。它只用于人工观察，
正式结论仍以 headless runner 的 CSV/JSON/report 为准。

若需要单独打开相机图像窗口，在安全 GUI 保持运行时另开终端执行：

```bash
./top_level_scripts/run_toudi3_visual_viewer_safe.sh
```

默认显示 `/camera/color/image_raw`。可在 `rqt_image_view` 下拉框切换到
`/uav_vision/circle_debug`、`/uav_vision/cross_debug` 等调试图；也可通过
`VISUAL_GUI_IMAGE_TOPIC` 指定初始话题。

新链目前是“新视觉观测链 + 旧控制链”的联合仿真入口。`detect_compat_bridge` 默认不把像素坐标伪装成世界坐标，因此新链的地图候选和 `drop_ready` 可观测，但不应宣称已经替代旧 `patrol_control` 的投递闭环。

节点总图：

- [toudi3_full_sim_nodes.svg](../vision_ws/toudi3_full_sim_nodes.svg)：完整仿真中所有主要节点和数据流。
- [toudi3_old_chain_nodes.svg](../vision_ws/toudi3_old_chain_nodes.svg)：旧视觉链。
- [toudi3_new_visual_chain_nodes.svg](../vision_ws/toudi3_new_visual_chain_nodes.svg)：新视觉 Phase D 链。

## 2. 一次性检查与编译

进入 WSL 后，在任意一个终端执行：

```bash
test -f /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world && echo world_ok
test -x /home/xhj/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo px4_ok
test -d /home/xhj/AstraDroneOpen/simulation/sim_workspace/devel/lib && echo astra_plugins_ok
```

如需重新编译：

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace/vision_ws
catkin_make -j1
cd /home/xhj/liftrace/patrol_uav_ws-patrol_planner
catkin_make -DROS_EDITION=ROS1 -j1
```

只有怀疑构建产物损坏时才清理可再生缓存：

```bash
cd /home/xhj/liftrace/patrol_uav_ws-patrol_planner
rm -rf build devel
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1 -j1
```

清理只针对可再生的 `build/devel`，不会删除源码、模型或权重。

## 2.1 航路点与 world 坐标契约

完整 toudi3 入口默认加载：

```text
/home/xhj/liftrace/patrol_uav_ws-patrol_planner/src/patrol_control/config/patrol_toudi3.yaml
```

它不再直接使用包含多段历史路线的 `patrol_sim.yaml`。当前测试路线使用 world 中
嵌套模型 `3` 的绝对靶标中心：

| 靶标 | world 中心（m） | 路线用途 |
| --- | --- | --- |
| `dibao` | `(-0.602, -1.041)` | 第 1 个 Detect_point |
| `qiaoliang` | `(-1.903, -0.023)` | 可作为第 2 个 Detect_point |
| `zhangpeng` | `(1.016, 0.256)` | 可作为第 3 个 Detect_point |
| `zhuangjiache` | `(-1.589, 3.022)` | 已在场景中，三舵机基线不再追加为第 4 次投递 |
| `tanke` | `(0.283, 3.856)` | 已在场景中，可作为普通观察点或后续候选 |
| `landing_h` | `(0, 0)` | 最终 Land_point |

`landing_h` 当前按 1×1 m 水平贴图板实现；规则书明确了降落区和 H 形状，但未给出
H 贴纸的独立尺寸，因此这是仿真工程假设，不应当当作正式规则尺寸。

旧 `patrol_control` 只有 Servo 1～3。完成 3 个检测点后是否跳转由
`detect_skip_enable` 和 `waypoint_skipping_index` 共同决定。关闭跳转可以继续执行普通
走廊航点，但不能把第 4 个点继续写成 `Detect_point`：最新运行已证实它会下降并调用
Servo 4，随后报错、等待超时。需要观察更多靶标时，将其写成普通观察点，由新视觉候选
记忆负责记录。

启动后用以下命令确认实际参数已加载：

```bash
rosparam get /waypoints
rosparam get /switch/flag_planner_px4
rosparam get /waypoint_skipping_index
rostopic hz /mavros/setpoint_position/local
```

预期结果：`/waypoints` 只显示 toudi3 专用路线的 8 个点，
`/switch/flag_planner_px4` 为 `1`，setpoint 持续发布。若仍看到 51 个点，说明使用了
旧的 `patrol_control_px4_sim.launch` 默认入口，而不是完整 toudi3 入口。

### 2.2 real.yaml 和派生 world 必须显式传入

`run_toudi3_full_competition_sim.sh` 与旧的 `launch_toudi3_full_sim.sh` 当前没有把
`patrol_toudi3_real.yaml` 固定传给 launch，默认仍是 `patrol_toudi3.yaml`。执行完整路线
专项时，不要只看文件编辑状态；启动后必须以 `/waypoints` 为准。

手动启动示例（仅 PX4 SITL）：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/vision_ws/devel/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
roslaunch patrol_control patrol_full_competition_sim.launch \
  world:=/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world \
  waypoint_config:=/home/xhj/liftrace/patrol_uav_ws-patrol_planner/src/patrol_control/config/patrol_toudi3_real.yaml \
  waypoint_mode:=true start_visual:=false gui:=true rviz:=true
```

需要软件类别、圆环和 Servo mock 时，在另一个已加载 ROS 环境的终端启动：

```bash
python3 /home/xhj/liftrace/top_level_scripts/sim_helpers.py
```

此时关闭旧视觉节点，避免 mock 与旧检测器竞争同一全局话题。这两条命令会启动仿真自动
解锁和纯软件投递，只能用于 SITL；不会启动 `actuator_pwm`。若选择派生开发 world，同时
替换 `world:=...`，并在报告中记录文件 SHA256。

### 2.3 原始门与开发门的用途

- 原始 `toudi3.world` 保持不变，Wall_15 门洞宽 0.8 m，用于最终规则对照；
- 先建立 1.2 m 派生开发门，隔离验证定位、planner、控制和双向穿越；
- 开发门通过不能冒充原始门通过；
- Gazebo 中开启 “Show Collisions”，分别保存俯视、门正视和侧视截图；
- 门区专项先关闭视觉和投递，不要每轮先跑完整三投。

详细尺寸预算和 Gate 顺序见阻塞专题文档。

## 3. 无 GUI 旧链完整赛程回归（先做这个）

### 终端 1：启动旧链

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
cd /home/xhj/liftrace
bash ./top_level_scripts/run_toudi3_full_competition_sim_headless_old.sh
```

命令作用：启动 `toudi3.world`、PX4 SITL、MAVROS、仿真自动解锁、旧 `patrol_control`、航点生成器、旧视觉节点、FAST-LIO/FreeDOM 和 Fast-Planner。`TOUDI3_GUI=false`、`TOUDI3_RVIZ=false` 只关闭窗口，不关闭仿真控制节点；`px4_max_distance=0.2` 用于限制直飞 setpoint 单步变化；完整 toudi3 SITL 入口还将旧检测点对准到达阈值设为 `0.20 m`，避免 WSL 仿真姿态的小稳态误差使状态机卡在 `Aligning`。

不要关闭这个终端，直到记录到最终降落和 `armed: false`。旧 roslaunch 在任务完成后通常仍会保持 Gazebo、MAVROS 和视觉节点运行，不会自行退出；完成记录后在终端 3 执行停止脚本即可。

### 终端 2：状态观察

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
rostopic echo /mavros/state
```

另开一个终端 2.1 查看位姿和目标点：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
rostopic echo /mavros/local_position/pose
```

需要同时看 setpoint 时，使用终端 3：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
rostopic echo /mavros/setpoint_position/local
```

终端 2 重点记录：`connected: true`、`armed: true`、`mode: OFFBOARD`；位姿 z 值从地面附近上升；setpoint 沿 `patrol_toudi3.yaml` 航点推进；末端出现 `AUTO.LAND` 日志或 `LandDetectDone timeout` 后的安全降落和 `armed: false`。

当前路线下控制台应依次看到 `Send point to take off`、`Have arrive Point number` 和
`Next Point Pose`。如果只看到 `Send point to take off`，重点检查
`/mavros/local_position/pose` 是否持续发布以及 PX4 是否进入 `OFFBOARD`；如果没有任何
`patrol_control` 日志，说明完整控制入口没有启动。

### 终端 3：节点和话题诊断

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
rosnode list
rostopic list
rostopic info /camera/color/image_raw
rostopic info /livox/lidar
```

看到 `/gazebo` 发布相机和点云、`circle_detector_node`/`simple_cross_detect`/`landing_detector_node` 订阅相机、`laserMapping` 订阅点云，即表示旧链节点接线完成。节点启动不等于赛程验收，仍以飞机位姿、模式和最终降落为准。

### 无 GUI 回归结束

记录到最终降落后，终端 1 不一定自然退出；在终端 3 执行以下命令收尾。若中途失败，也使用同一命令清理，不要用宽泛的 `pkill -f`：

```bash
/home/xhj/liftrace/top_level_scripts/stop_toudi3_sim.sh
```

无 GUI 入口不设置运行时限，适合让旧链跑完整段航点。工具或终端本身不要再包一层短 `timeout`。

## 3.1 H 降落模型专项检查

模型已安装到：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h/
```

启动完整仿真前可先检查文件：

```bash
test -f /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h/model.sdf
test -f /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h/materials/textures/Hjiangluo.png
file /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h/materials/textures/Hjiangluo.png
grep -n landing_h /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world
```

`file` 必须显示 `PNG image data`。若此前 Gazebo 已经打开，先执行停止脚本并重新
启动；旧的 `gzserver/gzclient` 可能仍保留错误贴图的 OGRE 材质缓存。

Gazebo 中应在起飞点 `(0,0)` 的地面看到黑色圆环和 H。起飞阶段降落检测是关闭的，
只有航路状态切换到最终 `Land_point` 后，`patrol_control` 才会发布
`/detect/landing_control=true`；因此 H 出现在画面中不应在起飞阶段直接触发降落。

专项观察命令：

```bash
rostopic echo /detect/landing_control
rostopic echo /detect/land_mark_point
rostopic echo /mavros/state
```

若已到 `(0,0,0.8)` 但没有 `land_mark_point`，检查相机是否朝向地面、
`landing_detector_node` 是否订阅 `/camera/color/image_raw`，以及 TF `map -> camera_link`
是否存在。当前旧 `landing_detector_node` 仍使用历史硬编码相机模型，H 模型只能验证
触发链和控制时序，不能作为地图投影精度验收。

## 4. 启动旧完整链条 GUI

进入 WSL 后，在终端 1 执行下面命令。不要给它加 `timeout`，保持该窗口运行；Gazebo 和 RViz 会在 WSLg 中打开 GUI：

```bash
cd /home/xhj/liftrace
bash ./top_level_scripts/run_toudi3_full_competition_sim_gui_old.sh
```

它会启动：

- `toudi3.world`、Gazebo GUI、`iris_mid360`；
- PX4 SITL、MAVROS、`uav_arming_node`；
- 旧 `patrol_control`、`waypoint_generator`、`fast_planner_node`、`traj_server`；
- FAST-LIO、FreeDOM；
- 旧圆环、红十字、降落检测；
- 一个控制/规划 RViz。

启动后先等待以下状态出现：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
```

正常重点看：`connected: True`、随后 `armed: True`、模式进入 `OFFBOARD`，Gazebo 中飞机离地并沿 `patrol_toudi3.yaml` 航点移动。

## 5. 启动新视觉链完整仿真

V-SIM-00 联合环境 Gate 已通过。先关闭旧仿真，再执行：

```bash
cd /home/xhj/liftrace
bash ./top_level_scripts/run_toudi3_full_competition_sim_gui_new.sh
```

新链使用：

```text
/camera/color/image_raw
/camera/color/camera_info
/mavros/local_position/odom
```

并启动：

```text
target_detector (dev/sim，优先使用 rl_drone Python)
cross_detector
circle_detector
landing_detector
detection_fusion
target_refiner
target_map_projector
target_memory
drop_aligner
detect_compat_bridge
```

如果终端出现 `ultralytics` 或模型加载异常，几何检测节点仍可运行，但标准目标 YOLO 结果为空；此时不要把“新链无标准目标”误判为 Gazebo 相机没有发布。检查：

```bash
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
rosnode list
rostopic hz /camera/color/image_raw
rostopic hz /uav_vision/detections_resolved
```

新链结果检查：

```bash
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
rostopic echo -n 1 /uav_vision/detections_refined
rostopic echo -n 1 /uav_vision/detections_mapped
rostopic echo -n 1 /uav_vision/targets
rostopic echo -n 1 /uav_vision/drop_ready
rostopic echo -n 1 /uav_vision/release_evidence
```

其中重点字段是：

- `center_refined`：是否使用蓝色圆环中心；
- `map_valid`：是否有有效 CameraInfo、TF 和地面求交；
- `map_point/map_frame`：地图记忆所使用的点；
- `drop_ready`：视觉对准成立，不等于舵机释放许可。
- `release_evidence`：可审计视觉证据，仍不等于任务/安全层最终许可。

定量检查优先运行视觉-only 固定场景 suite：

```bash
cd /home/xhj/liftrace
./top_level_scripts/run_toudi3_visual_suite.sh
```

该 suite 不启动 PX4、MAVROS、旧控制、解锁或执行机构；当前全部推理仍运行在笔记本，
不是 OrangePi 板端验收。

## 6. RViz 中手动订阅

启动脚本会自动打开一个 RViz，并加载 `patrol_pc.rviz`。如果 RViz 没有打开，可以在 WSL 中手动执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
rosrun rviz rviz -d /home/xhj/liftrace/patrol_uav_ws-patrol_planner/src/patrol_control/rviz_config/patrol_pc.rviz
```

在 RViz 左侧按 `Add`，按消息类型添加：

| RViz Display | Topic | 用途 |
|---|---|---|
| `Pose` | `/mavros/local_position/pose` | 飞机当前位姿 |
| `Pose` | `/mavros/setpoint_position/local` | 控制侧目标点 |
| `Pose` | `/fastplanner/goal` | Fast-Planner 目标 |
| `Marker` | `/planning_vis/trajectory` | 规划轨迹 |
| `PointCloud2` | `/cloud_registered` | FAST-LIO 当前点云 |
| `PointCloud2` | `/freedom/static_pointcloud` | FreeDOM 静态地图 |
| `PointCloud2` | `/sdf_map/occupancy_inflate` | 局部膨胀障碍地图 |
| `PointCloud2` | `/sdf_map/esdf` | ESDF 距离场（若有发布） |
| `Image` | `/camera/color/image_raw` | Gazebo 相机原始画面 |
| `Image` | `/uav_vision/circle_debug` | 新链圆环调试图 |
| `Image` | `/uav_vision/cross_debug` | 新链红十字调试图 |
| `Image` | `/uav_vision/landing_debug` | 新链降落标识调试图 |

固定坐标系优先使用当前配置中的 `camera_init`。若 RViz 顶部显示 `No tf data`，先用以下命令查看可用 frame，再把 `Global Options → Fixed Frame` 改成实际存在且能连接飞机的 frame：

```bash
source /opt/ros/noetic/setup.bash
rosrun tf view_frames
rosrun tf tf_echo camera_init body
```

`TargetDetectionArray`、`TargetCandidateArray` 是自定义消息，RViz 默认没有对应 Display。它们用 `rostopic echo` 查看；若需要可视化，应观察对应 debug image，或后续增加专门的 `visualization_msgs/MarkerArray` 可视化节点。

## 7. 完整赛程验收顺序

在 Gazebo 和 RViz 同时可见的情况下，按下面顺序记录：

1. Gazebo 是否加载 `toudi3.world`、飞机、相机和 MID360；
2. `/mavros/state.connected` 是否为 `True`；
3. 是否自动解锁、进入 `OFFBOARD`、离地；
4. `/mavros/setpoint_position/local` 是否沿 `patrol_toudi3.yaml` 变化；
5. 经过巡航和检测航点后，飞机是否保持在场地附近，没有飞出场景；
6. 末端是否出现 `AUTO.LAND` 或降落高度变化；
7. 记录最终位置、模式和日志，不以“节点启动成功”代替整段赛程通过。

建议在终端 2 或终端 3 观察：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /mavros/state
rostopic echo /mavros/local_position/pose
rostopic echo /mavros/setpoint_position/local
```

2026-07-13 的无 GUI 旧链长回归已实际完成：全链节点启动，飞机进入 `OFFBOARD`，完成起飞、首段巡航、两个检测航点的无目标超时切换，抵达最终着陆航点；随后出现 `LandDetectDone timeout`、`Have Land!`、`Vehicle DisArmed!`，最终状态为 `armed: false`，位姿约 `(0.13, 0.06, 0.24)`。本次控制台没有捕获到 `Simulation AUTO.LAND mode enabled` 的显式成功行，因此“最终降落/解锁状态”已验证，但 PX4 `AUTO.LAND` 服务响应仍应作为单独诊断项复核。GUI 脚本仍不设置运行时限，便于观察；完成后使用停止脚本收尾。

## 8. 结束仿真与故障清理

优先在启动终端按 `Ctrl+C`。若 GUI 或 roslaunch 残留，可执行：

```bash
exec /home/xhj/liftrace/top_level_scripts/stop_toudi3_sim.sh
```

仅在确认没有其他 ROS 任务时清理日志：

```bash
rm -rf /home/xhj/.ros/log
mkdir -p /home/xhj/.ros/log
```

常见现象：

- `spawn_model service timeout`：Gazebo 启动早期服务尚未出现，先看后续是否完成模型和插件加载；当前入口已带自动重试；
- `px4_config.yaml contains invalid YAML`：本机系统 MAVROS 配置存在非法 `//` 注释；仓库入口已改用 `patrol_control/config/mavros_px4_sim.yaml`，不要直接修改 `/opt/ros/noetic/share/mavros`；
- 当前 toudi3 入口已在仓库内对 PX4 `posix_sitl` 做“立即生成 + 失败重试”适配；若仍失败，先看 Gazebo 是否在启动阶段崩溃，再调整 `spawn_delay`、`spawn_retry_delay` 或 `spawn_retries`；
- `ultralytics missing`：新链的 dev detector 使用了错误 Python 或环境未加载，先确认 `VISION_PYTHON`；
- RViz 无点云：检查 FAST-LIO/FreeDOM 节点和 Fixed Frame，不要立即判断 Gazebo 没有飞机；
- 飞机逐渐飞出场景：记录 `/mavros/setpoint_position/local` 与当前位置，优先确认是否误启用了历史 planner 路径。
