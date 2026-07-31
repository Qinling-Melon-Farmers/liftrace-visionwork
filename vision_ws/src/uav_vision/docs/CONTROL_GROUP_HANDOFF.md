# uav_vision 控制组联调说明

交付版本：`20260717-alpha1`  
包版本：`uav_vision 0.2.1`  
边界：只包含视觉运行链、消息、参数、模型和视觉 mock；不包含控制、规划、任务仲裁、
MAVROS 指令、舵机服务或执行机构代码。

## 1. 新视觉链路

```text
image + CameraInfo + TF + align_mode
  -> six-class detector (PT or RKNN)
  -> red-cross / blue-ring / landing-H geometry
  -> detection_fusion
  -> target_refiner
  -> target_map_projector
  -> target_memory
  -> targets / selected_target
  -> drop_offset / drop_ready / release_evidence
```

六分类顺序固定为：

```text
bridge, panzer, pillbox, tent, tank, red_cross
```

标准靶必须与同帧蓝色圆环完成一对一关联才可进入 operational 候选。红十字使用自身
几何中心；H 需要外圈与内部结构，并且只在 `landing` 阶段进入 operational 链。

## 2. 单目相机如何计算地图点

当前实现不依赖深度图。它使用“标定单目相机 + 图像时刻位姿 + 已知目标平面”约束：

1. 检测/几何节点得到精修像素中心 `p=(u,v)`；
2. `image_geometry::PinholeCameraModel` 使用当前 `CameraInfo.K/P` 将像素反投影为相机
   光学坐标系射线 `r_c=(x,y,z)`；
3. TF2 在源图像 `header.stamp` 查询 `map_frame <- camera_optical_frame`，得到相机原点
   `O_m` 和地图系射线方向 `d_m`；
4. 假定靶标位于参数给定的平面 `Z=ground_z`，计算：

```text
t = (ground_z - O_m.z) / d_m.z
P_m = O_m + t * d_m
```

只有 `|d_m.z|` 足够大且 `t>0` 时地图点有效。结果写入：

```text
map_valid=true
map_point=P_m
map_frame=<launch map_frame>
transform_age_sec=<图像时间与实际 TF 时间差>
```

所以普通单目下视相机可以工作，条件是：

- 图像已经去畸变，或图像内容与 CameraInfo 的针孔模型严格一致；
- 相机内参对应当前分辨率；
- `header.frame_id` 是真实相机光学 frame；
- 相机外参与 LIO/导航 TF 正确；
- TF 能覆盖源图像时间戳；
- 靶面近似位于已知平面。

当前实机为单个下视相机 + MID360。MID360/FAST-LIO 提供机体地图位姿；视觉通过 TF 使用
该位姿，不要求相机本身提供深度。当前代码尚未使用 MID360 在线拟合局部地面；如果场地
明显不平、靶标不在地面或 `ground_z` 不确定，应由定位侧提供局部地面平面，或改用雷达
平面、测距、深度相机、多视角三角化，不能继续套固定 `ground_z`。

### 当前误差边界

- `map_quality` 当前主要继承几何置信度，不是统计协方差；
- 地面高度误差、相机外参误差、内参/去畸变误差和 TF 时间误差都会直接进入地图误差；
- 近水平射线、交点在相机后方、缺 CameraInfo、缺 TF 或 TF 过旧均 fail-closed；
- `allow_latest_tf_fallback=false` 是交付默认值，不用最新 TF 冒充图像时刻位姿。

## 3. 控制组必须提供的输入

| 输入 | 类型 | 契约 |
|---|---|---|
| 相机图像 | `sensor_msgs/Image` | `header.stamp/frame_id` 必须真实 |
| 相机参数 | `sensor_msgs/CameraInfo` | 与图像分辨率、去畸变状态一致 |
| TF | TF2 | 必须存在 `map_frame <- camera optical frame` |
| `/uav_vision/align_mode` | `std_msgs/String` | `disabled/drop_circle/drop_cross/landing` |
| `/uav_vision/reset_memory` | `std_srvs/Empty` | 任务开始、结束或地图重置时调用 |

控制组拥有任务阶段、导航目标、飞行速度/高度、释放许可和执行机构；视觉包不会发布飞行
指令，也不会调用舵机。

## 4. 控制组建议消费的输出

### `/uav_vision/targets`

`uav_vision/TargetCandidateArray`，包含全部活跃候选。每个候选至少检查：

```text
state >= 2                 # CONFIRMED
map_valid == true
map_frame == 导航使用的 frame
association_valid == true  # 标准靶必须成立
reject_reason == ""
now - last_seen <= 0.5 s   # 当前接近/对准时
```

地图记忆可长期保存到 reset，不能因为 topic 刚收到就把历史候选当成当前观测。

### `/uav_vision/selected_target`

视觉按赛委会确认的规则权重给出的单个建议，不是导航命令。当前排序为 red_cross=10、
panzer=2.5、bridge=2、pillbox=1.5、tent=1、tank=5（tank 权重已确认）。
Mission Manager 可以结合剩余时间、电量、可达性和 delivered/failed 集合重新排序。

### `/uav_vision/drop_offset`

图像坐标偏差：

```text
dx_px = target_u - camera_cx
dy_px = target_v - camera_cy
```

它适合末端控制器使用，但不是地图 Pose。若控制器需要地图目标，优先使用候选
`map_point`；不要把像素值直接写入 `/detect/waypoint_mark_point`。

### `/uav_vision/drop_ready`

仅表示像素偏差和连续帧满足视觉阈值，不是最终动作许可。

### `/uav_vision/release_evidence`

包含目标 ID、类别、确认、几何、中心、新鲜度、对准和拒绝原因。控制/安全层仍需组合：

- 任务阶段和目标身份；
- 飞机高度、速度和姿态；
- 载荷槽位和机构健康；
- 防重放、重复目标和比赛规则；

之后才能形成最终 `release_permission`。

## 5. 编译与启动

### Catkin 编译

```bash
source /opt/ros/noetic/setup.bash
cd <解压目录>/vision_ws
catkin_make --pkg uav_vision -j1
source devel/setup.bash
```

### 笔记本/dev

PyTorch/Ultralytics 应运行在已有 ML Python 环境。`python_launch_prefix` 由接收方显式填写：

```bash
roslaunch uav_vision control_handoff_dev.launch \
  image_topic:=/your/down_camera/image_raw \
  camera_info_topic:=/your/down_camera/camera_info \
  map_frame:=camera_init \
  ground_z:=0.0 \
  python_launch_prefix:=<ML环境中的python绝对路径>
```

### OrangePi/RK3588

需要板端系统 Python 可导入 ROS、OpenCV、NumPy、PyYAML、cv_bridge 和 RKNNLite：

```bash
roslaunch uav_vision control_handoff_board.launch \
  image_topic:=/your/down_camera/image_raw \
  camera_info_topic:=/your/down_camera/camera_info \
  map_frame:=camera_init \
  ground_z:=0.0
```

交付的 `merged_standard_fp32.rknn` 是当前有效主候选；INT8 当前零有效检测，未放入 ZIP。
板端离线模型有效不等于 ROS 相机/TF 和 10 分钟稳定性已经通过。

## 6. 交付范围

ZIP 包含：

- `uav_vision` 源码、消息、配置和纯视觉 launch；
- dev/sim PT 与 OrangePi FP32 RKNN 主候选；
- 六分类 metadata；
- 本说明、模型评测摘要和 SHA256 清单；
- 视觉 L0 mock/assertion。

ZIP 明确不包含：

- `patrol_control`、`uav_mission`、`actuator_pwm`；
- PX4/MAVROS 控制、Fast-Planner、任务状态机或释放代理；
- 历史 `Visual/detect_ws/yolov5_detect` 包；
- 数据集、bag、视频、build/devel、日志或 Python cache；
- 当前不可用 INT8 和旧 standard+tank 模型。

## 7. 已验证与未验证

已验证：视觉 L0 圆环、实例关联、新鲜度、物理 stable ID、无效 TF 拒绝、地图投影和释放
证据；完整 toudi3/SITL 中能够产生 `camera_init` 地图候选；FP32 RKNN 板端离线有效。

未验证：控制组原始工程直接接入、地图候选驱动飞机闭环、跨视角 30-seed、10 分钟 ROS
板端稳定性、真实相机内外参归属、三次仿真投递和任何实机动作。
