# uav_vision 导航组接口说明

交付版本：`20260807-beta1`
包版本：`uav_vision 0.2.1`
边界：视觉只输出观测、地图候选、记忆和像素对准证据，不输出飞行或执行机构命令。

## 1. 运行链

```text
Image + CameraInfo + TF + align_mode
  -> six-class detector
  -> geometry detectors + fusion + target refiner
  -> target map projector
  -> target memory
  -> /uav_vision/targets + /uav_vision/selected_target
  -> drop aligner outputs
```

标准目标必须完成同帧类别与蓝环一对一关联，且地图投影有效，才能进入 operational 候选。

## 2. 输入接口

| 输入 | 类型 | 约束 |
| --- | --- | --- |
| 图像 | `sensor_msgs/Image` | queue 1；时间戳和光学 frame 必须真实 |
| CameraInfo | `sensor_msgs/CameraInfo` | 与图像分辨率、裁剪和去畸变状态一致 |
| TF | TF2 | 图像时间存在 `map_frame <- camera optical frame` |
| `/uav_vision/align_mode` | `std_msgs/String` | `disabled/drop_circle/drop_cross/landing` |
| `/uav_vision/reset_memory` | `std_srvs/Empty` | 任务开始/结束或地图重置时调用 |

## 3. 输出接口

### `/uav_vision/targets`

类型为 `uav_vision/TargetCandidateArray`，包含视觉记忆中的全部候选。用于动作前检查：

```text
state >= 2
map_valid == true
map_frame == expected mission frame
association_valid == true
reject_reason == ""
now - last_seen <= 0.5 s
```

`first_seen` 用于确定发现顺序，`last_seen` 是最后真实观测时间；长期地图记忆重发不会刷新
`last_seen`。`map_quality` 是工程质量分，不是定位协方差。

### `/uav_vision/selected_target`

单个视觉排序建议。标准类权重：

```text
tank=5, panzer=2.5, bridge=2, pillbox=1.5, tent=1
```

导航消费者可以结合可达性、任务终态、剩余时间和自身规则改选目标。该话题不是规划目标，
`uav_vision` 不发布 `/fastplanner/goal`。

### `/uav_vision/drop_offset`

输出目标精修中心相对 CameraInfo 主点的 `dx_px/dy_px`。它是像素偏差，不是地图 Pose。

### `/uav_vision/drop_ready` 与 `/uav_vision/release_evidence`

前者只表示视觉像素对准，后者提供目标身份、几何、观测年龄、稳定帧和拒绝原因；二者都
不是释放许可。

## 4. stable ID 和地图记忆

- 地图候选默认保留到 `/uav_vision/reset_memory`；
- 相同物理目标使用地图距离、类别投票和连续帧维持 stable ID；
- 收敛到 0.6 m 内的重复标准目标记录会合并并保留最早 ID；
- 目标成功、失败或不可达后的任务终态由消费者维护，视觉不会代替任务层删除目标；
- 缺 CameraInfo、无效关联、缺 TF 或 TF 过旧时地图候选失败关闭。

## 5. 参考导航代码

ZIP 根目录 `reference_integration/` 包含阶段 4 使用的消息草案、候选策略和 coverage manager。
它们依赖主工程中的 `patrol_control/uav_mission/Fast-Planner`，不在视觉工作区参与编译，也不
能从交付 ZIP 直接运行。它们只用于展示以下已验证行为：

- 非靶标坐标蛇形覆盖；
- 候选有效性过滤和规则权重排序；
- stable ID 的 delivered/failed 终态去重；
- 规划目标重试、20 秒不可达和恢复索引；
- Mission Manager 独占 planner goal 的集成方式。

导航组可以选择复用、改写或完全不用这些参考文件。视觉正式接口仅为本文件列出的 ROS
输入、输出、消息字段和服务。

## 6. 已验证与限制

阶段 4 无 GUI SITL 完成靶标区域 12/12 覆盖、五类五 ID、权重队列和返航，零碰撞、越界
和 Servo 调用。最终地图误差四类为 2.18-5.72 cm，`pillbox` 为 22.97 cm 离群值。

尚未验收：搜索后的末端对准误差收敛、三次投递、30-seed、OrangePi ROS 相机/TF 和实机。
仿真结果不能描述为 RKNN 板端验收。
