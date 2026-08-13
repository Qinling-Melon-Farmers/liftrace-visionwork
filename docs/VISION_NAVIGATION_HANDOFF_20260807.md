# 新视觉链导航组联调包说明（2026-08-07）

交付级别：`20260807-beta1 / 导航接口联调包`
交付对象：导航组
可编译主体：纯 `uav_vision` ROS 包

## 1. 交付边界

本 ZIP 用于让导航组编译视觉消息、启动新视觉链、读取地图候选，并在自己的导航任务中
完成搜索、接近和恢复策略。视觉包本身不会发布 `/fastplanner/goal`，不会控制飞机，也不会
调用释放机构。

阶段 4 为验证视觉接口可被导航闭环消费，临时实现并验证了 `MissionCommand.msg`、
`coverage_search_manager.py` 和 `coverage_navigation.launch`。这些文件作为
`reference_integration/` 参考材料随包提供，但不属于 `uav_vision`、不是强制接口，也不能
从 ZIP 中直接独立运行。导航组可自行决定是否复用其候选过滤、权重排序、超时和恢复逻辑。

包内不包含可运行的 `patrol_control`、`uav_mission`、Fast-Planner、PX4/MAVROS、舵机服务
或真实执行机构实现。

### 接收方只有板端原始工程时

推荐先把原始工程复制到导航组自己的笔记本，用包内 PT 模型完成视觉、导航和整机仿真，
通过后再上 OrangePi 切换 FP32 RKNN。导航代码应只依赖 ROS 消息和话题，不区分推理后端。

| 能力 | 当前支持情况 |
| --- | --- |
| 独立编译/启动纯视觉链 | 支持，需板端已有 ROS/OpenCV/RKNNLite 和正确相机输入 |
| 原导航工程订阅视觉消息 | 支持，需按 overlay 或纯话题方式接入 |
| 视觉 mock | 支持，不需要 PX4/Gazebo |
| 接入已有仿真相机 | 条件支持，仿真必须提供 Image/CameraInfo/TF |
| 完整 toudi3 阶段 4 SITL | ZIP 单独不支持，需完整开发机功能分支和 PX4/Gazebo 资产 |

解压目录位置、Catkin overlay、板端 RKNN、话题/TF 检查、视觉 mock 和完整 SITL 依赖见
同级 `INSTALL_AND_SIMULATION.md`。

## 2. 包内内容

```text
vision_ws/src/uav_vision/            可独立编译的纯视觉包
  msg/                               视觉消息定义
  launch/control_handoff_*.launch    笔记本 PT / OrangePi RKNN 入口
  config/                            话题、模型、投影、记忆和对准参数
  docs/NAVIGATION_GROUP_HANDOFF.md   导航接口与字段契约
  models/                            PT、FP32 RKNN 和六分类 metadata

reference_integration/               阶段 4 参考实现，不参与视觉包编译
  MissionCommand.msg
  ReleaseResult.msg
  coverage_policy.py
  coverage_search_manager.py
  coverage_navigation_assertion.py
  coverage_toudi3.yaml
  coverage_navigation.launch

evidence/                            阶段 4 与既有模型评测摘要
INSTALL_AND_SIMULATION.md            原工程接入、板端运行与仿真依赖
SOURCE_REVISION.txt                  打包源码版本
MANIFEST.sha256                      ZIP 内文件校验
```

## 3. 导航侧必须提供给视觉的输入

| 输入 | 类型 | 要求 |
| --- | --- | --- |
| 相机图像 | `sensor_msgs/Image` | 真实 `header.stamp` 和光学 `frame_id` |
| 相机参数 | `sensor_msgs/CameraInfo` | 与图像分辨率及去畸变状态一致 |
| TF | TF2 | 图像时刻存在 `map_frame <- camera optical frame` |
| `/uav_vision/align_mode` | `std_msgs/String` | `disabled/drop_circle/drop_cross/landing` |
| `/uav_vision/reset_memory` | `std_srvs/Empty` | 任务或地图生命周期切换时调用 |

## 4. 导航侧主要消费的话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/uav_vision/targets` | `TargetCandidateArray` | 全部目标记忆和地图候选 |
| `/uav_vision/selected_target` | `TargetCandidate` | 视觉权重排序建议，不是导航命令 |
| `/uav_vision/drop_offset` | `DropOffset` | 末端图像偏差，不是地图 Pose |
| `/uav_vision/drop_ready` | `DropReady` | 视觉对准状态，不是动作许可 |
| `/uav_vision/release_evidence` | `ReleaseEvidence` | 身份、几何、新鲜度和对准证据 |

将候选用于当前导航动作前，至少检查：

```text
state >= 2
map_valid == true
map_frame == 导航使用的 frame
association_valid == true
reject_reason == ""
now - last_seen <= 0.5 s
```

地图记忆会持续发布，topic 新消息不等于目标刚被重新看到。目标投递成功或失败后的终态
去重属于任务消费者职责，不能只依赖视觉 stable ID 自动删除。

## 5. 当前权重和阶段 4 证据

标准类权重为 `tank=5、panzer=2.5、bridge=2、pillbox=1.5、tent=1`；红十字为 10。
`selected_target` 是视觉建议，导航组仍可结合可达性、任务状态和剩余时间重新选择。

`target_area_navigation_20260807_190817` 已完成：

- 12/12 非靶标坐标覆盖端点和安全返航；
- 五类恰好五个 stable ID，候选顺序为 `tank -> panzer -> bridge -> pillbox -> tent`；
- `/coverage_search_manager` 是该验证中的唯一 planner goal 发布者；
- 零碰撞、零越界、零 Servo 调用；
- 最终地图点中四类误差为 2.18-5.72 cm，`pillbox` 离群误差为 22.97 cm。

该 Gate 证明接口能支撑搜索、地图记忆和候选排序，不证明导航组必须采用参考 manager，
也不证明末端对准、三次投递、OrangePi ROS 链或实机已经验收。

## 6. 编译和启动

```bash
source /opt/ros/noetic/setup.bash
cd <解压目录>/vision_ws
catkin_make --pkg uav_vision -j1
source devel/setup.bash

roslaunch uav_vision control_handoff_dev.launch \
  image_topic:=/your/down_camera/image_raw \
  camera_info_topic:=/your/down_camera/camera_info \
  map_frame:=camera_init \
  ground_z:=0.0 \
  python_launch_prefix:=<ML环境中的python绝对路径>
```

OrangePi 使用 `control_handoff_board.launch` 和包内 FP32 RKNN；板端 ROS 相机/TF 与 10 分钟
稳定性仍未验收，不能把笔记本 PT 仿真结果当作板端结论。
