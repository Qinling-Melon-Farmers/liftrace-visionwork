# P1-3：FC reset 与稳定任务坐标候选接口（2026-10-05）

修改工作树：`r2026-ev-continuity`；基线：`feat/ev-continuity-20261004@df29c381`。本轮只准备生产组件、观察接线和可重复离线用例。未连接板端，未启动 ROS 节点、仿真或硬件；未修改其他工作树、共享 AGENTS/联调日志，未 commit/push，交主代理记录和提交。

**本轮没有把 P1-3 宣告为整机已根治。** 新组件不进入现有竞争/板端入口，不能默认启用或跨分支投放；真正控制连续性仍待可信 reset 生产者、完整标定及低层 HOLD 联动验证。

## 已有能力与缺口

现有 FAST-LIO 已具备预处理/匹配容器/地图盒/输出性能修复、可显式三线程匹配和完整预测状态。独立 IMU 预测只发布 shadow 位姿，能在受限输入条件下接续小校正，不直接替代 MAVROS EV。详见[既有专项计划](../../planning/ev_continuity_20261004/README.md)。

`map_camera_alignment.py` 根据两路共同机体位姿估计 camera_init↔map，允许小变化融合及大变化重新初始化；它没有消费可信 FC reset 计数，没有联锁释放，也没有保证在重新初始化期间最终设定点已原子接续。`navigation_frame_adapter.py` 支持正反向 TF 转换，但正式现场 `legacy_static` 入口沿用静态对齐和启动地面基准。不能由这些已有组件推导出 reset 已完整闭环。

普通 `/mavros/local_position/pose`、`odom` 不含本轮所需的 reset 计数、delta 和 estimator 启动代次。当前仓库没有已验收的该事件生产者。不能从一个高度差推断是 FC 重置、LIO 变化或真实运动，也不能用 ULog 事后发现的字段冒充在线输入已存在。

## 本轮实现与边界

| 文件 | 内容 |
|---|---|
| `src/uav_mission/src/uav_mission/task_frame_continuity.py` | 纯生产组件；统一变换、可信 reset 接续、反馈/几何/最终 FC 设定点、HOLD 请求与许可门控 |
| `src/uav_mission/scripts/ev_task_boundary.py` | ROS 观察适配器；共同机体两路输入、显式事件/健康契约、协方差转换、候选输出；不调用 mode/arming/Servo，不广播现有 TF |
| `src/uav_mission/launch/ev_task_boundary.launch` | 必须提供 reference_file 的独立观察入口；没有任何正式入口 include 它 |
| `src/uav_mission/config/ev_task_boundary.example.yaml` | 标为 SYNTHETIC 的禁用示例；`calibration_verified=false`，不是现场标定 |
| `src/uav_mission/test/test_task_frame_continuity.py` | 22 项实际组件回归 |
| `src/uav_mission/test/test_ev_task_boundary.py` | 10 项真实 ROS 消息/适配回归，无 ROS master |
| `tools/ev_continuity/task_boundary_case.py` | 无 ROS/硬件的 0.524m 复现入口，产生[数值记录](FC_RESET_CASE.json) |

上表 `src/` 根指 `patrol_uav_ws-patrol_planner/`。Catkin 已登记节点安装和两组测试。

### 坐标算法

固定任务坐标 T、任务地面 `ground_z`、测得的 LIO→T 对齐及 body→camera 完整外参，不随 FC reset 修改。FC→T 变换为 `T_task_fc`。

可信事件必须定义 **`p_new = T_new_old * p_old`**。接续计算：

```text
T_task_new = T_task_old * inverse(T_new_old)
p_task_feedback = T_task_new * p_fc_new
p_fc_command = inverse(T_task_new) * p_task_command
p_task_camera = T_task_body * T_body_camera
camera_AGL = camera_task_z - ground_z_task
```

候选组件用同一代变换生成机体反馈、相机投影和最终 FC 设定点；不滤掉真实运动，不修改传感器时间戳，不给每个任务航点机械地加 delta_z。

接收事件后立即换代、撤销旧许可；先等待 reset 后时间戳的共同机体观测，并累计至少 3 个独立新帧、跨度至少 0.10s，才恢复候选投影/下降/释放。已验证一对 reset 后观测但窗口尚未填满时，输出以新 FC 坐标表示的上一个可信物理 HOLD 点；不输出新的下降目标。缺可信映射时只有 `hold_required` 请求，没有猜测出来的本地 HOLD 点。

同一 reset 回放不重复叠加；冲突回放、计数漏项、FC 启动代次变化、LIO 代次/健康失效、源时间回退、断连及空中人工接管均拒绝或锁存，不能自动夺回控制。陈旧输入须重新积累新帧。真正同步上升保留原幅度，并不触发 reset。AUTO.LAND 可保留反馈，但不放行释放或 OFFBOARD 设定点；本组件未修改 PX4 AUTO.LAND 内部目标。

### 观察输出与参数

默认输出为 `/ev_task_boundary/{task_pose,task_odom,camera_pose,fc_setpoint,fc_hold_request,release_permission,status}`。输出必须留在 `/ev_task_boundary` 或 `/ev_task_boundary_*` 的独立命名空间内；禁止把观察 `fc_setpoint` 直接改成 MAVROS 输出。**这些都是候选结果，当前没有消费者将它们用来控制飞机或操作执行器。**

所有输入话题、参考坐标和候选阈值由节点参数提供。默认 FC 观测输入 `/mavros/local_position/odom`，LIO 共同机体输入 `/mavros/vision_pose/pose`，飞控状态 `/mavros/state`。独立健康/事件输入缺失时绝不 READY。LIO 输入必须已经经过完整 IMU→body 安装变换，不能把雷达参考点和 FC 中心直接比较。

候选门槛：输入/健康/FC state 年龄 0.20s、同步差 0.04s、未来容差 0.02s、事件年龄 0.20s、两路位姿残差 0.08m/3°、恢复 3 帧且跨度 0.10s、相邻间隔 ≤0.20s。这些只是观察用例参数，**没有用来修改现有 LIO 300ms、任务过期或位姿跳变保护阈值，也没有作为实飞安全标定值。**

完整标定必须提供 task/FC/LIO frame、两路启动代次、权威初始 reset 序号、两个完整刚体对齐、任务地面和完整相机安装外参；示例的零地面/单位变换/−0.16m 不应直接用于实机。`calibration_verified` 必须是真正布尔值，默认 false；不能靠把它改成 true 完成标定。

### 两项缺失生产者的在线契约

观察节点以 `std_msgs/String` 接受版本化 JSON；必须由经过核验的生产者输出，不可手工填零后进入飞行。

`reset_input` 示例（纯注入数据）：

```json
{"version":1,"stamp":1.2,"fc_epoch":"synthetic_fc_boot","previous_counter":0,"counter":1,"frame_id":"map","authoritative":true,"translation_new_from_previous":[0.0,0.0,0.524],"quaternion_new_from_previous_xyzw":[0.0,0.0,0.0,1.0]}
```

stamp 是 reset 在 FC 坐标中生效的测量时刻，需转为统一 ROS 时间基准；fc_epoch 唯一标识飞控/估计器启动代次；counter 是生产者按本契约提供的单调事件序号，不应直接把可回卷的某个 PX4 uint8 counter 填进来。MAVROS ENU 与 PX4 NED 的 delta 符号/坐标必须在生产者中确认。初版板端生产者建议先限定权威 Z reset；姿态 reset 不应在未确认完整坐标定义时直接当成世界刚体旋转。本组件全旋转测试仅说明数学接口一致。

`lio_health_input` 示例：

```json
{"version":1,"stamp":1.21,"lio_epoch":"synthetic_lio_boot","frame_id":"synthetic_lio","healthy":true,"body_calibrated":true}
```

stamp 对应实际 LIO body 观测，不是健康节点的当前墙钟；lio_epoch 在源重建/重置时变更。healthy 必须来自真实校正健康/时间覆盖/重建状态，不是“有订阅到 pose 就 true”。当前 FAST-LIO PredictionState 已有 epoch，但普通 EV PoseStamped 没携带此健康契约；生产者尚待接入。

## 本轮验证

| 验证 | 结果与实际范围 |
|---|---|
| E 的 `uav_mission` Catkin 构建 | PASS；依赖只读借用整机 overlay，生成/安装输出在 E；未重构其他包或启动节点 |
| 生产组件检查 | 22/22 PASS；0.524m reset、真实上升、LIO 重置/跳变/失效/陈旧、模式接管、断连、源时间/重复/两帧、event 代次/回放、完整外参/变换、许可与视觉证据换代 |
| ROS 实际消息/节点方法检查 | 10/10 PASS；位姿/相机/最终 FC 指令一致、消息序列化、pose 协方差、child twist 保持、HOLD/许可撤销、旧证据不能生成新许可、观察输出隔离 |
| Catkin CTest 两组 | 2/2 PASS，包含上述 32 项用例 |
| 可重复生产组件 case | PASS；[FC_RESET_CASE.json](FC_RESET_CASE.json) 为合成注入，不是实飞回放 |

[构建日志](/tmp/ev_task_boundary_build_20261005.log)，CTest 输出在 E 的 `patrol_uav_ws-patrol_planner/build/Testing/Temporary/LastTest.log`，两份 xunit 在 `build/test_results/uav_mission/`。初次测试集成时缺待生成测试文件/测试 fixture 搜索路径的错误已修正；表格只列最终结果。

| 0.524m reset 指标 | reset 前 | 接续后 |
|---|---:|---:|
| FC 观测 Z | 0.600m | 1.124m |
| 稳定任务机体 AGL | 0.600m | 0.600m |
| 镜头 AGL | 0.440m | 0.440m |
| 最终 FC 设定点 Z | 0.600m | 1.124m |
| FC 中的高度误差 | 0m | 0m |
| 100px 偏移在平面上的投影 | −0.088m | −0.088m |

如果仍发旧 0.600m FC 目标，则高度误差为 −0.524m。这是接口注入算例；没有据此证明真实电机不会下沉、reset 通知足够早或板端调度无缺口。

## 复现命令（只读输入、离线执行）

从 Windows 使用 `wsl -e bash -c '...'` 包装。以下在 WSL 内执行，不启动 ROS master：

```bash
source /home/xhj/liftrace-worktrees/r2026-board-frame-fix/patrol_uav_ws-patrol_planner/devel/setup.bash
cd /home/xhj/liftrace-worktrees/r2026-ev-continuity/patrol_uav_ws-patrol_planner
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES=uav_mission -DFAST_LIO_MATCH_THREADS=3 -j2
source devel/setup.bash
cd build
ctest -R 'task_frame_continuity|ev_task_boundary' --output-on-failure

cd /home/xhj/liftrace-worktrees/r2026-ev-continuity
/home/xhj/miniconda3/envs/rl_drone/bin/python tools/ev_continuity/task_boundary_case.py --out /tmp/fc_reset_case.json
```

## 待板端验证与正式接线条件

1. 核验飞控在线 reset 计数/delta/epoch 传输及共同时间基准，特别是消息先后顺序、ENU/NED 转换、丢失/回卷/估计器重启。只有残差或普通 local_position 时保持禁止自动重映射。
2. 由实际 FAST-LIO 校正/PredictionState 给出独立健康代次；量测完整 FC↔LIO body 安装旋转/平移、重力轴、相机外参与固定任务地面。前述约 4° 相对倾斜仍需标定，不能由本轮猜测抵消。
3. 在板端仅观察对照 raw FC、稳定 task、真实 LIO、相机投影和候选最终 FC 指令。验证真实上升、停流、重置及人工接管，不直接替换现有 EV 输入。
4. 正式接入时，规划/地图/任务/限高/许可统一消费稳定任务坐标；相机 TF 使用该任务 body 及原安装外参，不能同时再套用旧动态 FC→task TF。最终边界只做一次逆变换，禁止旧 FC 地面/裁高值再裁剪接续后的 FC setpoint。
5. 低层控制器必须消费原子 reference generation、HOLD_REQUIRED 和恢复回执，在 reset 生效/确认窗口内持续输出经确认的新坐标 HOLD；没有可信映射时需现有飞控保持/失效链接管。**本轮观察节点没有接入这项实际控制握手，因此不能证明输出连续或暂停下降已经在飞机上生效。**
6. 许可需在候选帧/视觉证据换代后重新获取，raw actuator 调用仍经现有工程许可/事务链。AUTO.LAND 内部目标恢复需要飞控自身契约单独确认。

以上条件及动态故障注入通过后，才考虑主代理合入正式链并安排板端验证。本轮保留 P1-3 为“候选接口与离线验证完成，在线生产者/最终执行握手及动态验收未完成”。
