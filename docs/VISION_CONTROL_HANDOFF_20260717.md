# 视觉组向控制组交付说明（2026-07-17）

交付级别：`Alpha 1 / 接口联调包`  
交付对象：导航、任务与控制组  
运行边界：纯视觉，不包含任何控制、规划、解锁、投递或执行机构实现。

## 1. 请先确认交付边界

本 ZIP 用于让控制组在其原始工程中编译视觉消息、启动完整视觉链、读取地图候选并开发
Mission Manager/导航消费者。它不是把当前改造过的控制代码交给控制组，也不要求覆盖其
原始 `patrol_control`。

包内不包含：

- `patrol_control`、`uav_mission`、`actuator_pwm`；
- Fast-Planner、FAST-LIO、PX4 或 MAVROS 控制实现；
- 舵机服务、释放许可或任务状态机；
- 旧 `Visual`、`detect_ws` 和 `yolov5_detect` 工程；
- 数据集、视频、bag、build/devel 和日志。

## 2. 包内内容

```text
vision_ws/src/uav_vision/       完整视觉运行包
  msg/                          视觉消息
  config/                       阈值、投影、记忆和模型输入配置
  launch/control_handoff_*.launch
                                纯视觉 dev/board 入口
  scripts/ + src/ + include/    检测、融合、精修、投影、记忆、对准
  docs/CONTROL_GROUP_HANDOFF.md  详细接口和单目地图投影说明
  models/
    merged_standard_best.pt      笔记本/仿真模型
    merged_standard_fp32.rknn    OrangePi FP32 主候选
    merged_standard_6cls_metadata.yaml
evidence/                       评测结论快照，不参与运行
MANIFEST.sha256                 ZIP 内逐文件校验
```

## 3. 当前视觉链能力

```text
六分类粗检测
  -> 红十字/蓝环/H 几何验证
  -> 同帧融合与标准靶—圆环实例关联
  -> 单目射线与地面平面求交得到地图点
  -> 连续帧确认、stable ID、类别投票、地图融合与长期记忆
  -> 候选排序、像素对准偏差和结构化释放证据
```

地图点不需要深度相机。视觉使用 CameraInfo 将精修像素变成三维射线，使用图像时刻 TF
把射线变换到 `camera_init` 等导航坐标系，再与 `z=ground_z` 地面平面求交。普通下视单目
相机可以使用，但必须有正确内参、去畸变状态、相机外参、图像时间戳和 LIO/导航 TF。

更完整的公式、失败原因、话题和控制消费规则见包内：

`vision_ws/src/uav_vision/docs/CONTROL_GROUP_HANDOFF.md`

## 4. 控制组接入原则

控制组应直接消费 `/uav_vision/targets` 或 `/uav_vision/selected_target` 中的：

```text
id / class_name / map_point / map_frame / map_valid
map_quality / last_seen / state / association_valid / reject_reason
```

控制侧必须自行决定候选接近、避障、任务切换、恢复搜索和最终投递。`selected_target` 只是
视觉排序建议；`drop_ready` 和 `release_evidence` 都不是舵机许可。

禁止把 `/uav_vision/drop_offset` 的像素值伪装成地图 Pose。需要导航目标时使用经过 TF
投影的 `map_point`。

## 5. 当前证据和限制

- 视觉 L0 圆环、关联、新鲜度、stable ID、无效 TF 拒绝、地图投影和释放证据回归通过；
- 完整 toudi3/SITL 中能够产生 `camera_init` 地图候选；
- `merged_standard_fp32.rknn` 已在 OrangePi 做离线图片/视频有效性验证；
- 尚未完成控制组原始工程接入、视觉地图点驱动飞机、30-seed 跨视角、10 分钟板端 ROS、
  真实相机内外参归属和三次闭环投递；
- 当前 `map_quality` 是工程质量分，不是带协方差的定位不确定度；
- 当前 INT8 和旧双 RKNN 不可用，未放入 ZIP。

## 6. 验收建议

控制组首次接入只做以下安全检查：

1. 编译 `uav_vision`，确认自定义消息可见；
2. 用录制图像或仿真相机启动 `control_handoff_dev.launch`；
3. 发布 `align_mode`，检查 targets、selected_target 和地图 frame；
4. 故意断开 CameraInfo/TF，确认 `map_valid=false` 且无导航候选；
5. 只把合法 `map_point` 送入控制组自己的 mock/导航目标入口；
6. 在完成坐标、时间、新鲜度和停止条件验收前，不连接真实释放动作。
