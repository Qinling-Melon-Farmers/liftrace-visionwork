# 导航组接入、运行与仿真手册（2026-08-07）

适用交付：`uav_vision_navigation_handoff_20260807_beta1.zip`

本文假定接收方只有板端原始工程和本 ZIP，但允许先把原始工程复制到自己的笔记本完成
联调开发。命令中的目录、相机话题和相机 frame 必须替换为接收方实际值，不要把示例路径
写回源码。

## 1. 能力边界

| 目标 | 仅用板端原始工程和 ZIP | 还需要什么 |
| --- | --- | --- |
| 编译视觉消息和节点 | 支持 | ROS Noetic 开发依赖、OpenCV |
| OrangePi 启动 RKNN 视觉链 | 支持接线 | RKNNLite、相机、CameraInfo、TF；整机尚未验收 |
| 原导航节点订阅视觉候选 | 支持 | 按本文配置 workspace overlay 或 ROS 话题 |
| 运行视觉 mock | 支持 | ROS Noetic；不需要 PX4/Gazebo/模型推理 |
| 接入已有仿真相机运行 PT 视觉 | 条件支持 | 笔记本 ML 环境、Image、CameraInfo 和 TF |
| 复现完整 toudi3/PX4/Fast-Planner 阶段 4 | 不支持单包运行 | 开发机完整仓库功能分支、PX4/Gazebo 和场景资产 |

### 1.1 推荐推进顺序

```text
笔记本原始工程副本
  -> 编译 uav_vision + 视觉 mock
  -> PT 模型接入笔记本仿真相机
  -> 导航消费者接入 targets/map_point
  -> 搜索、接近、恢复和整机无 GUI 仿真
  -> 固化话题、frame、候选和任务契约
  -> OrangePi 上切换 FP32 RKNN、真实相机和真实 TF
  -> 板端 shadow/整机联调
```

**整机仿真未通过前，不进入板端部署 Gate。** 笔记本使用包内 PT 模型；OrangePi 使用包内
FP32 RKNN。两条推理入口必须保持相同消息、类别顺序和候选字段，导航代码不应区分 PT/RKNN。

`reference_integration/launch/coverage_navigation.launch` 依赖完整主工程中的 `uav_mission`、
`patrol_control`、Fast-Planner 和仿真 wrapper。它是接口参考，直接在只有原工程和 ZIP 的
环境中运行会出现缺包或缺 include，这是预期边界。

## 2. 解压后的目录关系

```text
uav_vision_navigation_handoff_20260807_beta1/
  README_FIRST.md                 首要边界和话题摘要
  INSTALL_AND_SIMULATION.md       本手册
  SOURCE_REVISION.txt             打包源码提交
  MANIFEST.sha256                 包内文件校验

  vision_ws/                      唯一可直接 Catkin 编译的工作区
    src/uav_vision/
      CMakeLists.txt/package.xml  ROS 包入口和系统依赖
      msg/                        导航侧要消费的消息
      launch/                     dev/PT、board/RKNN 和 mock 入口
      config/                     模型、投影、记忆和阈值
      models/                     PT、FP32 RKNN、metadata
      docs/                       完整字段契约

  reference_integration/          不参与 Catkin 编译
  evidence/                       只读验证结果，不参与运行
```

推荐把整个解压目录放在板端原工程旁边，不覆盖原工程：

```text
<work_root>/original_patrol_ws/       板端原始工程
<work_root>/uav_vision_.../vision_ws  新视觉独立工作区
```

不要把 `reference_integration/` 放进任何工作区的 `src/`。如果团队决定复用其中代码，应先
按自己的包结构、消息和规划入口做正式移植。

## 3. 首次校验和系统依赖

```bash
sha256sum -c uav_vision_navigation_handoff_20260807_beta1.zip.sha256
unzip uav_vision_navigation_handoff_20260807_beta1.zip
cd uav_vision_navigation_handoff_20260807_beta1
sha256sum -c MANIFEST.sha256
```

`uav_vision/package.xml` 声明 ROS 依赖。板端至少还要确认：

```bash
python3 -c 'import rospy, cv2, numpy, yaml, cv_bridge'
python3 -c 'from rknnlite.api import RKNNLite'
```

第二条失败表示 RKNNLite 尚未进入当前板端 Python，不应改用 PyTorch 绕过。

## 4. 编译视觉工作区

```bash
source /opt/ros/noetic/setup.bash
cd <解压目录>/vision_ws
catkin_make --pkg uav_vision -j1
source devel/setup.bash

rospack find uav_vision
rosmsg show uav_vision/TargetCandidate
rosmsg show uav_vision/TargetCandidateArray
```

### 4.1 只通过 ROS 话题联调

如果原工程不在编译期引用 `uav_vision` 消息，可以不重编原工程：

- 终端 A 按原方法 source 并启动相机、定位和导航；
- 终端 B source `vision_ws/devel/setup.bash` 并启动视觉；
- 两边使用同一个 `ROS_MASTER_URI`，通过 ROS 话题通信。

### 4.2 原导航源码直接依赖视觉消息

此时 `vision_ws` 是原工程的 underlay。先编视觉，再在同一终端重编原工程：

```bash
source /opt/ros/noetic/setup.bash
source <解压目录>/vision_ws/devel/setup.bash
cd <板端原始工程的catkin工作区>
catkin_make -j1
source devel/setup.bash
```

消费者包需要按实际语言增加依赖：

```xml
<!-- package.xml -->
<depend>uav_vision</depend>
```

```cmake
# CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  uav_vision
)
catkin_package(CATKIN_DEPENDS uav_vision)
add_dependencies(your_navigation_node ${catkin_EXPORTED_TARGETS})
```

将 `uav_vision` 加入消费者现有的 component/dependency 列表，不要删除其原有依赖；把
`your_navigation_node` 替换为实际 target。

不要复制消息内容形成第二套同名/不同名结构；直接依赖 `uav_vision` 生成的消息。

## 5. 板端接线检查

视觉必须同时获得图像、匹配的 CameraInfo 和图像时刻 TF：

```bash
rostopic list | grep -E 'image|camera_info|camera|mavros|lio'
rostopic echo -n 1 <image_topic>/header
rostopic echo -n 1 <camera_info_topic>/header
rosrun tf2_ros tf2_echo camera_init <camera_optical_frame>
```

要求：

- 图像和 CameraInfo 分辨率、时间及光学 frame 一致；
- `camera_init <- camera_optical_frame` 在图像时间戳可查询；
- `ground_z` 是 `camera_init` 中真实靶面高度；
- 相机外参来自实测标定，不使用仿真固定外参。

缺 CameraInfo 或 TF 时 `map_valid=false` 是正确的失败关闭行为，不能通过放宽
`allow_latest_tf_fallback` 掩盖接线问题。

## 6. 整机仿真通过后的 OrangePi RKNN 部署

只有第 9/10 节的笔记本整机仿真通过后，才进入本节。包内
`control_handoff_board.launch` 默认使用随包 FP32 RKNN 和 metadata：

```bash
source /opt/ros/noetic/setup.bash
source <解压目录>/vision_ws/devel/setup.bash

roslaunch uav_vision control_handoff_board.launch \
  image_topic:=<真实下视图像话题> \
  camera_info_topic:=<匹配的CameraInfo话题> \
  map_frame:=camera_init \
  ground_z:=<camera_init中的靶面高度> \
  start_legacy_compat:=false
```

搜索/观察阶段默认 `align_mode=disabled`，仍保留标准靶、红十字和圆环候选，只屏蔽 H。
进入标准靶末端圆环对准时再发布 `drop_circle`：

```bash
rostopic pub -1 /uav_vision/align_mode std_msgs/String 'data: drop_circle'
```

任务开始、地图重置或任务结束时清理长期记忆：

```bash
rosservice call /uav_vision/reset_memory
```

## 7. 运行后检查

```bash
rostopic hz /uav_vision/detections
rostopic echo /uav_vision/targets
rostopic echo /uav_vision/selected_target
rostopic echo /uav_vision/perf
```

导航动作候选至少满足：

```text
state >= 2
map_valid == true
map_frame == camera_init        # 或双方约定的任务 frame
association_valid == true
reject_reason == ""
now - last_seen <= 0.5 s
```

`selected_target` 是排序建议，不是 planner goal；`drop_offset` 是像素偏差，不是地图坐标；
`drop_ready/release_evidence` 也不是舵机许可。

## 8. 不依赖飞行器的视觉 mock

这些入口不启动 PX4、Gazebo、规划或执行机构：

```bash
source /opt/ros/noetic/setup.bash
source <解压目录>/vision_ws/devel/setup.bash

roslaunch uav_vision phase_d_map_mock.launch
roslaunch uav_vision target_memory_physical_mock.launch
roslaunch uav_vision map_rejection_mock.launch
```

三个入口分别验证地图投影/候选链、stable ID/类别投票、缺 TF 失败关闭。required assertion
正常结束并触发 roslaunch 收尾即为通过。

## 9. 笔记本接入已有仿真相机

这是导航组的首选开发路径。准备一份板端原始工程副本，并在笔记本安装与本项目等价的
Ubuntu 20.04/WSL2、ROS Noetic、Gazebo Classic、PX4/MAVROS 和规划环境。PT 推理只在
笔记本 ML conda 环境运行，不在 OrangePi 上运行。

先按第 4 节编译视觉工作区；导航源码若直接引用视觉消息，再以视觉工作区为 underlay
重编原工程副本。启动顺序建议分终端执行：

```text
终端 A：roscore 或原整机 launch
终端 B：PX4/Gazebo/MAVROS/定位/规划
终端 C：control_handoff_dev.launch（PT 视觉）
终端 D：导航组自己的候选消费者与任务节点
终端 E：rostopic/TF/日志检查
```

视觉启动示例：

```bash
source /opt/ros/noetic/setup.bash
source <解压目录>/vision_ws/devel/setup.bash

roslaunch uav_vision control_handoff_dev.launch \
  image_topic:=<仿真下视图像> \
  camera_info_topic:=<仿真CameraInfo> \
  map_frame:=camera_init \
  ground_z:=<仿真靶面高度> \
  python_launch_prefix:=<含PyTorch和Ultralytics的python绝对路径>
```

该入口只要求仿真端提供 Image、CameraInfo 和 TF，可以接导航组自己的仿真；它不会自动
启动 Gazebo、PX4 或规划器。

导航组可以选择两条整机仿真路径：

1. 在自己的原工程副本和场景中接入 `control_handoff_dev.launch`，自行实现导航任务；
2. 使用第 10 节完整功能分支，复现本项目已经通过的 `target_area_navigation` Gate。

第一条不要求采用参考 coverage manager；第二条用于复核我们交付的接口和已知结果。

### 9.1 上板前最小 Gate

```text
[ ] uav_vision 独立编译、消息可见、三个视觉 mock 通过
[ ] PT 链能从仿真 Image/CameraInfo/TF 产生五类合法地图候选
[ ] 导航只消费 map_valid/association_valid/reject_reason 合法候选
[ ] stable ID、last_seen、reset 和任务终态去重行为明确
[ ] 搜索/接近/恢复期间 planner goal 发布者和超时策略明确
[ ] 整机无 GUI 仿真正常收尾并保存日志
[ ] 未连接真实 Servo/actuator
```

只有以上项目通过，才把源码和参数带到 OrangePi。不要复制笔记本的 `build/devel`，不要把
PT/Ultralytics 作为板端主路径。

## 10. 完整阶段 4 SITL

复现 `target_area_navigation` 还需要：

- `liftrace-visionwork` 的 `feat/external-mission-coverage` 分支；
- 至少包含阶段 4 运行提交 `942ba1d`，交付文档提交为 `4156171`；
- PX4 SITL、Gazebo Classic、toudi3 world/model、MAVROS；
- 修改后的 `patrol_control`、`uav_mission`、Fast-Planner 和统一 `sim_run.sh`；
- 笔记本 PT 环境及模型。

应在笔记本使用单独 clone/worktree，不覆盖板端原始工程或其联调副本。完整环境就绪后
入口为：

```bash
SIM_NO_RECORD=1 \
UAV_VISION_MODEL_PATH=<best.pt绝对路径> \
top_level_scripts/sim_run.sh target_area_navigation \
roslaunch uav_mission coverage_navigation.launch \
gui:=false rviz:=false wall_timeout:=1800
```

ZIP 中 `reference_integration/` 可帮助对照接口，但不能替代上述完整分支和环境。

## 11. 常见故障

| 现象 | 优先检查 |
| --- | --- |
| `rospack find uav_vision` 失败 | 是否 source 正确 `vision_ws/devel/setup.bash` |
| RKNN 节点启动但无检测 | RKNNLite、模型路径、输入话题、`/uav_vision/perf` |
| 有类别但没有合法候选 | 同帧圆环关联、连续 3 帧、置信度和拒绝原因 |
| `map_valid=false` | CameraInfo、frame_id、图像时间 TF、`ground_z` |
| 候选一直存在但目标已离开视野 | 检查 `last_seen`，长期地图记忆不会自动删除 |
| reference launch 报缺包/include | 该目录不是独立运行包，需完整功能分支或自行移植 |

首次联调不要连接真实释放动作；先完成消息、TF、地图点、新鲜度和 reset 生命周期检查。
