# 视觉组仿真分层与环境入口（ROS Noetic + PX4 + Gazebo）

更新时间：2026-08-26

## 1. 用途和安全边界

本文帮助视觉组选择正确的仿真入口，并明确每种测试能证明什么。逐终端的 toudi3 操作、GUI 和排障见 [docs/TOUDI3_FULL_SIM_GUI_GUIDE.md](/home/xhj/liftrace/docs/TOUDI3_FULL_SIM_GUI_GUIDE.md)；指标、场景和任务顺序见 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。原 2025 链完整赛程的窄门阻塞和 world 分级改造见 [docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md](/home/xhj/liftrace/docs/2025原始链完整仿真阻塞与WORLD改造方案_20260802.md)。

本文所有完整仿真入口只连接 PX4 SITL，不连接真实飞控，不启动 `actuator_pwm`。任何实机解锁、起飞、投递或 PWM 操作均不在本文范围内。

## 2. 当前结论

- 外部仿真底座：`AstraDroneOpen + PX4 SITL + Gazebo Classic + iris_mid360`；
- 当前比赛场景：仓库根目录 `toudi4_copy.world` 与 `patrol_world.launch`；
- 当前机架：`patrol_control/models/iris_mid360_downward_camera/model.sdf`，
  相机话题为 `/downward_camera/image_raw`、`/downward_camera/camera_info`；
- 新视觉人工连通入口：`run_toudi3_full_competition_sim_gui_new.sh`；联合环境使用
  `toudi3_combined_env.sh`；
- 确定性软件回归：`uav_vision` 的 map/patrol mock launch；
- L1 定量入口：`run_toudi3_visual_suite.sh`，复用 world 五类标准靶/H，红十字按场景插入；
- L2 隔离入口：`uav_vision_eval/toudi3_full_shadow.launch`；隔离契约已通过，10 min 未验收；
- 旧内部 `patrol_control_sim.launch` 仍受 Fast-Planner 运行时问题影响，不作为视觉验收基线；
- 原 2025 外部 SITL 链已完成自动起飞和三次软件 mock 投递，但原始 0.8 m 门洞穿越、
  北区巡航、返航和降落尚未完成；当前直达航点模式不等于在线避障；
- 当前独立真值、自动评分和 shadow 最小基建已存在，但 30-seed、正式召回/时延阈值和
  10 min 稳定性仍未通过。
- 导航组 `liftrace-controlwork@5144aa8` 的搜索 manager 已接入
  `navigation_search_delivery_toudi4.launch`；2026-08-26 headless 完成两投后在 600 s
  超时，属于 L3 联调入口，尚未通过三投/返航/落地 Gate。

因此新视觉 GUI 仿真仍只算“链路烟测”。定量结论必须来自 `uav_vision_eval` 报告，且
当前所有推理结果都属于笔记本，不属于 OrangePi 板端。

### 2.1 Fast-Planner 仿真地图参数

面向 `10 m × 10 m × 4 m` 比赛场地，正式 PX4/Gazebo 和内部仿真入口统一使用以下
SDF 参数；这些值只修改仿真配置，`patrol_planner_real.xml` 的实机膨胀参数保持原样：

| 参数 | 仿真值 | 含义 |
| --- | ---: | --- |
| `sdf_map/map_size_x` | 20.0 m | 以 `camera_init` 为中心，x 方向约覆盖 ±10 m |
| `sdf_map/map_size_y` | 20.0 m | 以 `camera_init` 为中心，y 方向约覆盖 ±10 m |
| `sdf_map/map_size_z` | 5.0 m | 从 `ground_height=-0.2 m` 覆盖至约 4.8 m |
| `sdf_map/obstacles_inflation` | 0.25 m | 水平膨胀 |
| `sdf_map/obstacles_inflation_up` | 0.20 m | 向上膨胀 |
| `sdf_map/obstacles_inflation_down` | 0.10 m | 向下膨胀 |

分辨率为 `0.1 m` 时，仓库原正式仿真默认 `100 × 100 × 50 m` 对应约 5 亿体素；
`20 × 20 × 5 m` 对应约 200 万体素，体素数减少 250 倍。实际进程内存还包含 ESDF、
占据概率、膨胀缓存和规划器其他数组，不能仅凭体素数承诺固定 RSS；需以一次 headless
实跑测量为准。2026-08-23 的 planner-only 15 s headless 烟测记录到进程族最大 RSS
`184064 kB`（约 180 MiB），但该数值不包含 PX4、Gazebo、MAVROS、视觉推理和完整任务链。

`uav_mission/launch/legacy_mode_regression.launch` 的 `14 × 14 × 6 m` 是历史回归显式覆盖，
`topo_replan.launch` 是独立拓扑算法示例，均不代表正式比赛仿真默认值。

## 3. 四层验证入口

| 层级 | 当前入口 | 主要检查 | 通过依据 |
| --- | --- | --- | --- |
| L0 mock | `phase_d_map_mock.launch`、`phase_d_mock_patrol_regression.launch` | 消息、TF、地图投影、模式和兼容接口 | assertion 退出码 0 |
| L1 Gazebo 静态真值 | `run_toudi3_visual_suite.sh` | 检测、中心、实例关联、地图误差 | 自动 CSV/JSON/report；正式阈值待全过 |
| L2 toudi3 shadow 飞行 | `toudi3_full_shadow.launch` | 连续观测、ID、时延、稳定性 | 隔离已验证；固定 seed + 10 min 待完成 |
| L3 任务闭环 | `coverage_r6.launch`（历史临时 manager）；`navigation_search_delivery_toudi4.launch`（导航组 manager） | 搜索、接近、恢复、对准证据 | 临时 manager 有三投证据；导航组 manager 当前 2/3、600 s 超时 |

实拍视频/rosbag 回放与 OrangePi RKNN 分别是域差和部署门禁，不属于 Gazebo 的替代品。

## 4. 环境准备

### 4.1 Windows 进入 WSL

```powershell
wsl -d Ubuntu-20.04
```

以下命令默认在 WSL Ubuntu 终端运行：

```bash
source /opt/ros/noetic/setup.bash
export PROJECT_ROOT=/home/xhj/liftrace
export VISION_WS=$PROJECT_ROOT/vision_ws
export UAV_WS=$PROJECT_ROOT/patrol_uav_ws-patrol_planner
export PX4_ROOT=/home/xhj/PX4-Autopilot
```

### 4.2 一次性资产检查

```bash
test -f "$PROJECT_ROOT/toudi4_copy.world" && echo world_ok
test -x "$PX4_ROOT/build/px4_sitl_default/bin/px4" && echo px4_ok
test -d /home/xhj/AstraDroneOpen/simulation/sim_workspace/devel/lib && echo astra_plugins_ok
test -f "$VISION_WS/src/uav_vision/launch/phase_d.launch" && echo vision_ok
```

### 4.3 编译

视觉工作区：

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace/vision_ws
catkin_make -j1
```

主集成工作区需要先叠加视觉消息，并清空可能残留的白名单：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/vision_ws/devel/setup.bash
cd /home/xhj/liftrace/patrol_uav_ws-patrol_planner
catkin_make -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES="" -j1
```

## 5. L0：先跑确定性回归

### 5.1 地图投影、记忆和对准

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/vision_ws/devel/setup.bash
roslaunch uav_vision phase_d_map_mock.launch
```

### 5.2 新视觉与旧主控接口回归

该回归同时依赖两个工作区，使用已验证的联合环境入口：

```bash
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
roslaunch uav_vision phase_d_mock_patrol_regression.launch
```

验收要求：assertion 节点正常结束且退出码为 0，连续运行 3 次没有偶发失败。mock 通过只说明软件契约成立，不说明真实图像可识别。

推荐统一运行器：

```bash
/home/xhj/liftrace/top_level_scripts/run_visual_mock_regressions.sh
```

该脚本除运行 launch 外还检查 assertion 的 PASS marker；原因是 ROS1 `roslaunch` 在 required
节点以非零码退出时仍可能返回外层 0，不能只看 shell 返回值。

## 6. 外部仿真底座检查

只检查 PX4/Gazebo/MAVROS 时使用：

```bash
source /opt/ros/noetic/setup.bash
export PX4_ROOT=/home/xhj/PX4-Autopilot
export SITL_GAZEBO=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export ROS_PACKAGE_PATH=/opt/ros/noetic/share:$ROS_PACKAGE_PATH:$PX4_ROOT:$SITL_GAZEBO
export GAZEBO_MODEL_PATH=$SITL_GAZEBO/models:${GAZEBO_MODEL_PATH}
roslaunch px4 astra_example.launch vehicle:=iris_mid360
```

基础检查：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /mavros/local_position/odom
```

该入口不包含 toudi3 视觉任务，不用于算法评测。

## 7. toudi3 入口选择

| 命令 | 用途 | 重要限制 |
| --- | --- | --- |
| `roslaunch patrol_control patrol_world.launch` | 查看 world、飞机和相机 | 不启动完整任务航路 |
| `run_toudi3_full_competition_sim_headless_old.sh` | 旧链赛程回归 | 不验证新视觉 |
| `run_toudi3_full_competition_sim_gui_old.sh` | 旧链 GUI 排障 | 不验证新视觉 |
| `run_toudi3_full_competition_sim_gui_new.sh` | 新视觉 Phase D 连通烟测 | 当前不是 shadow，会和旧主控接线 |
| `run_toudi3_visual_suite.sh` | 八个固定 L1 场景 | 视觉-only；不启动 PX4/MAVROS/控制/执行机构 |

旧、新完整入口不能同时运行，它们共享 Gazebo、PX4、MAVROS 和全局视觉话题。

### 7.0 旧链完整赛程必须按 Gate 拆分

旧链排障不再用“一次跑全场”判断所有问题：

1. R0：定点悬停 10 min，先保证 `/clock`、PX4 heartbeat 和 odom 连续；
2. R1：原始主厅完成起飞、3 次 mock 投递、返航和降落；
3. R2：1.2 m 派生开发门，直达模式双向通过；
4. R3：1.2 m 开发门，Fast-Planner 拥有控制权并双向通过；
5. R4：回归 0.8 m 原始门；
6. R5：再组合成旧完整链；
7. R6：最后接新视觉检测、地图记忆、候选接近和 guarded 三投。

R2 使用 `waypoint_mode=true`，只证明控制/几何；R3/R4 使用
`waypoint_mode=false`，才检查 planner。视觉组可在这些 Gate 上做 shadow 记录，但旧链
是否穿门不计入视觉算法完成度。

### 7.1 新视觉 GUI 烟测

先检查联合环境：

```bash
rospack find uav_vision
rospack find patrol_control
python3 -c "from uav_vision.msg import TargetDetectionArray"
```

先加载联合环境，三项必须同时通过：

```bash
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
```

GUI 烟测命令：

```bash
cd /home/xhj/liftrace
bash ./top_level_scripts/run_toudi3_full_competition_sim_gui_new.sh
```

另一个 WSL 终端检查：

```bash
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env

rostopic hz /downward_camera/image_raw
rostopic echo -n 1 /camera/color/camera_info
rostopic echo -n 1 /uav_vision/detections_resolved
rostopic echo -n 1 /uav_vision/detections_refined
rostopic echo -n 1 /uav_vision/detections_mapped
rostopic echo -n 1 /uav_vision/targets
rostopic echo -n 1 /uav_vision/drop_ready
```

检查字段：

- 输入 header 时间是否单调、frame 是否正确；
- `center_refined` 的来源是否是期望的几何/圆环中心；
- CameraInfo 和 TF 有效时 `map_valid` 是否成立；
- `map_point` 和 `map_frame` 是否合理；
- `selected_target` 是否携带当前观测，而不是仅因回调重发旧记忆；
- `drop_ready` 是否只在正确 `align_mode` 下出现。

以上均正常仍只代表烟测通过。没有独立真值时，不记录“召回率”“中心误差”“地图误差”等算法结论。

### 7.2 结束仿真

优先在启动终端按 `Ctrl+C`。需要清理当前 toudi3 仿真残留时：

```bash
/home/xhj/liftrace/top_level_scripts/stop_toudi3_sim.sh
```

不要在存在其他 ROS 任务时使用宽泛的 `pkill -f`。

### 7.3 导航组 manager + 新视觉 headless 联调

该入口使用导航组原始 manager 生成覆盖/候选抵近目标，外围适配器承接现有视觉和旧控制
投递链。必须显式提供模型；空路径会启动“发布空检测”的开发兼容模式。

```bash
cd /home/xhj/liftrace
export UAV_VISION_MODEL_PATH=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt
SIM_RUN_AUTHORIZED=1 SIM_NO_RECORD=1 bash top_level_scripts/sim_run.sh \
  navigation_upstream_visual_delivery_headless \
  roslaunch uav_mission navigation_search_delivery_toudi4.launch \
  gui:=false rviz:=false enable_debug_image:=false \
  spawn_red_cross:=true red_cross_seed:=0
```

`SIM_RUN_AUTHORIZED=1` 只允许写在本次明确获准的启动命令前，禁止导出为长期环境变量。包装器会拒绝
已有 ROS/Gazebo/PX4/RViz 进程或第二个 `sim_run.sh`，并在正常结束、失败、超时和信号中断后调用
`stop_toudi3_sim.sh`；只有清理复查为零残留时才算完成收尾。

必须检查 run 目录中的：

- `target_search_status.json`：覆盖索引、候选、失败原因、投递槽位和任务终态；
- `gate_status.json`：raw/planner goal 所有权、旧 manager 缺席和三投断言；
- `red_cross_truth.yaml`：只用于复盘，不能进入任务；
- `roslog/target_search_manager_py-*.log`：导航组原状态转换和实际三维到达误差。

当前已知基线是 `navigation_upstream_visual_delivery_headless_model_20260826_023411`：600 s
到 9/16、tent/bridge 两投成功、pillbox capture timeout、第三投未完成。GUI 必须等同一
headless Gate 三投/返航/落地通过后再开；2026-08-26 用户已明确暂缓 GUI 二轮。

## 8. 正式视觉仿真入口与剩余完成定义

当前 L1/L2 最小入口已满足 headless、独立真值、自动记录、失败非零退出和 shadow 输出隔离。
继续扩为正式 Gate 时必须满足：

- headless 可运行，固定随机 seed；
- 不启动 `actuator_pwm`；
- shadow 模式下视觉结果不改变飞行状态；
- 场景 manifest 记录目标类别、姿态、相机姿态、光照和运动参数；
- 真值来自 Gazebo model state + CameraInfo + TF，不来自检测器；
- 自动记录检测、关联、地图点、观测年龄、模式和延迟；
- 自动生成 `summary.json`、逐样本 CSV 和 `report.md`；
- Gate 失败时进程退出非零；
- 同一 seed 可复现同一失败。

V-SIM-00 已通过；首轮指标阈值、30-seed 场景矩阵和任务顺序统一见
`VISION_2026_ROADMAP.md`。当前笔记本定量结果见
`docs/VISION_LAPTOP_SIM_BASELINE_20260715.md`。

## 9. 常见误区

- Gazebo 中看到框，不等于召回率达标；
- topic 有频率，不等于延迟低，必须比较源图像时间戳和输出时间；
- `map_valid=true` 不等于地图点准确，必须与独立世界真值比较；
- 目标保存在 memory 中，不等于它仍是当前可控观测；
- `drop_ready=true` 不等于允许投递；
- 一条固定航线看到全部已知靶，不等于具备全场自主搜索；
- 仿真清晰贴图通过，不等于真实光照、模糊、遮挡和 NPU 后处理通过。
