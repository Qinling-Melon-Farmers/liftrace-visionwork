# EV连续性观察套件（2026-10-06）

本目录是可运行的独立测试套件，放在0928原工程内，与 `deployment/low_hover_observation` 共用 `deployment/site_20260928/environment.sh` 选择的视觉/导航devel。不新建工程、Catkin工作区或ROS master。默认只预览；不自动启动、连接板端、发布正式EV/控制、调整飞控或执行投递。

EV来源和原样复制文件见 `manifest.json`。纯Python预测/任务坐标组件、上游ROS观察适配器及回归保持原样，放在 `source/` 中保留原相对路径。suite wrapper使用明确Python路径导入它们；生成的ROS消息始终来自当前0928工作区。**不要直接运行source目录中的原上游ROS入口**，使用本目录start.sh；status漏防重映射由wrapper补齐。

## 文件和依赖

- `start.sh`：统一入口，无参数是preview。
- `source.sh`：仅选择与低空suite相同的工作区依赖，不启动节点；不将vendored uav_mission抢在当前生成消息包前面。
- `observer.py` / `ev_suite/`：严格输入配置、当前消息检查、全部publisher白名单、显式启动和图检查。
- `offline.py`：无master的纯数值/ROS消息测试、合成故障用例、bag只读重放。
- `record_shadow.py` / `config/recording_shadow.yaml`：可选独立小消息bag，复用现有低空有界诊断录制实现。
- `parallel_check.py`：只读三线程构建/运行证据，绝不启动LIO或matching测试。
- `integration/`：已冻结快照4文件补丁、原始FAST-LIO比较和集成说明。4文件已经由主代理集成，不要刷新patch。
- `verification/`：本机测试报告及合成数据。`.tmp/`是忽略的临时测试文件，不是部署依赖。

非ROS测试：既有conda rl_drone、NumPy；合成图另用Matplotlib。ROS入口：系统ROS Python、NumPy/PyYAML、rospy/rosbag/tf、标准消息、mavros_msgs，且当前0928 devel中的最新uav_mission/ReleasePermission（含mission_id/decision_seq/attempt/target_first_seen/permission_epoch/permission_revision）和新编译fast_lio/PredictionState。没有系统pip安装、旧消息回退或EV工作区overlay兜底。

主代理负责同一工程内构建和部署。本地devel仍旧时检查明确BLOCKED，不能用suite内消息文件冒充已编译。`contracts/ReleasePermission.msg`只保存0928契约来源，绝不替换运行消息。

## 默认预览、离线测试

下面为WSL/板端bash内容；Windows实际执行请统一 `wsl -e bash -c '...'` 包装，中文文件通过UTF-8文件消费。

```bash
cd /home/xhj/liftrace-worktrees/r2026-board-vision-tests
bash deployment/ev_observation_20261006/start.sh

source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
bash deployment/ev_observation_20261006/start.sh offline --demo
```

离线测试不会init_node或启动master；结果写本目录verification。合成reset例子在离线内存使用已知真值，绝不给在线节点注入假健康/假事件。

当前工作区消息构建完成后，可做无master的实际devel检查与测试：

```bash
bash deployment/ev_observation_20261006/start.sh check
bash deployment/ev_observation_20261006/start.sh ros-test --out deployment/ev_observation_20261006/verification/ros_result.json
```

`check --kind reset`只要求新版ReleasePermission；shadow/all还要求PredictionState。两个入口都不访问master、不init_node。为本机旧devel提供的 `verify_ros_schema.py` 使用ROS genpy从真实消息定义生成suite内临时序列化测试fixture，无master；**仅验证消息/适配器算法，不是Catkin/ARM构建**。正式preflight主动拒绝此fixture路径，不能用于运行观察。

```bash
source /opt/ros/noetic/setup.bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 deployment/ev_observation_20261006/verify_ros_schema.py
```

已有bag输入只读：

```bash
bash deployment/ev_observation_20261006/start.sh ros-test --bag /absolute/input.bag --timing-only --out deployment/ev_observation_20261006/verification/timing_result.json
bash deployment/ev_observation_20261006/start.sh ros-test --bag /absolute/full_state.bag --out deployment/ev_observation_20261006/verification/replay_result.json
```

时序工具可读旧bag；预测重放必须有/livox/imu和完整PredictionState，否则明确拒绝。离线重放位置是LIO IMU参考点，不是PX4融合位姿。不要rosbag play向正式master回放MAVROS话题。

## 之后获准的shadow观察

默认执行下列命令仍只检查依赖/config，只有显式--start才运行观察节点。已有ROS master/LIO由现场原入口管理，本套件绝不启动它们。

```bash
bash deployment/ev_observation_20261006/start.sh shadow
# 当前地面观察轮次获得明确启动授权后：
bash deployment/ev_observation_20261006/start.sh shadow --start
```

需要同一个既有LIO在地面按主代理修改的低空定位入口使用 `prediction_state_enabled:=true`，默认false。见integration/README.md；不要运行第二个LIO。快照源时间/epoch/IMU frame必须一致。

预测参考点明确为LIO IMU，`imu_to_body_xyz=[0,0,0]`；不猜安装外参。limits仍是离线候选，不改正式过期、ABORT或300ms桥接阈值，不替换lio_external_pose输入。

**全部输出是精确白名单**：

```text
/ev_shadow/raw_pose
/ev_shadow/smooth_pose
/ev_shadow/status
/ev_task_boundary/task_pose
/ev_task_boundary/task_odom
/ev_task_boundary/camera_pose
/ev_task_boundary/fc_setpoint
/ev_task_boundary/fc_hold_request
/ev_task_boundary/release_permission
/ev_task_boundary/status
```

wrapper拒绝任意CLI ROS remap、ROS_NAMESPACE、output/observer_namespace参数，逐项检查重映射前/后的精确话题及消息类型，包括status；连观察命名空间内改名或交换输出都拒绝。在创建publisher前检查全组名字。只读取validated config，不继承master旧私有输出参数。`disable_rosout=True`避免额外/rosout publisher破坏自检。候选setpoint/许可不得桥接正式消费者；本套件没有mode/arming/Servo/TF接口。

启动前后可用 `bash deployment/ev_observation_20261006/start.sh graph-check` 只读检查已有本机master，比较formal_ev_publishers；观察publisher必须恰好等于对应白名单。缺master就拒绝，不自动创建。此检查不证明外部消费者没有另行桥接输出，正式控制链应持续沿用原EV桥和控制源。

## reset观察保持禁用标定

```bash
bash deployment/ev_observation_20261006/start.sh reset
# 后续获准后，可仅观察拒绝/等待状态：
bash deployment/ev_observation_20261006/start.sh reset --start
```

默认使用原上游SYNTHETIC示例，calibration_verified保持真正的false；wrapper连改成true也拒绝。未知真实标定、权威reset生产者、独立LIO健康生产者均不造假。status明确列出三项未满足条件，ready只能false，不产生可信投影/下降/释放。/mavros/vision_pose/pose在reset节点中是订阅输入。离线0.524m用例不能证明在线事件/健康或实际低层HOLD接续已实现。

## 可选轻量shadow录制

低空原录制器默认拒绝livox话题及PredictionState；不能仅把新话题加到其正式recording.yaml。本套件独立wrapper只在自己的模块实例扩展精确/livox/imu例外和fast_lio/PredictionState类型，并再用11话题/精确类型白名单复核。原录制器及原配置不变。

```bash
# 只预览，不创建文件、不连接master：
bash deployment/ev_observation_20261006/start.sh record-preview --output logs/ev_shadow_EXPLICIT_RUN/recording
# 后续明确授权采集、且已有master/LIO/shadow：
bash deployment/ev_observation_20261006/start.sh record --start --duration 600 --output logs/ev_shadow_EXPLICIT_RUN/recording
```

可与低空suite原诊断录制并行，写不同run目录。额外bag只录capture_topics.txt列出的11个小消息；不录原始雷达点云、图像或视频。最长600秒，4096条/16MiB写队列、单消息64KiB、订阅队列100，最低1GiB空间，沿用低空原实现的写入/丢弃统计和优雅关包。READY表示writer可用，不保证所有源话题已有数据；验收必须查看summary/topic_manifest是否收到IMU及快照且无丢弃/源时间异常。其Publisher和ServiceProxy被禁用，只订阅和写bag；Ctrl+C仅结束自己的录制，不停止定位或低空飞行。

第二轮ARM构建/production matching回归、板端消息preflight、在线图检查和负载/真实故障验证由主代理补验。本轮没有启动任何真实节点、录制、仿真或板端连接，也没有提交或修改共享变更记录。
