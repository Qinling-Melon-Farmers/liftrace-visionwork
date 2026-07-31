# OrangePi 5 Plus 视觉性能记录模板

本页用于 Phase D 板端验证时记录 `target_detector_rknn.py` 的真实运行表现。当前仓库内的 detector 已开始发布 `/uav_vision/perf`，建议结合 `rostopic echo`、`rostopic hz`、`top` 一起记录。

## 1. 记录环境

| 项目 | 记录值 |
| --- | --- |
| 日期 | |
| 板端主机 | `orangepi@192.168.3.15` |
| 相机 / 图像源 | |
| 分辨率 | |
| 模型路径 | |
| launch | `phase_d_board.launch` |
| 是否 unified / split assets | |

## 2. 建议采集命令

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace/vision_ws
source devel/setup.bash

roslaunch uav_vision phase_d_board.launch
```

另开终端：

```bash
rostopic hz /uav_vision/detections
rostopic echo -n 5 /uav_vision/perf
top
free -m
```

## 3. `/uav_vision/perf` 关键字段

当前 detector 节点会在 `DiagnosticArray` 中输出：

- `backend`
- `image_topic`
- `frames`
- `detections`
- `processing_ms`
- `inference_ms`
- `fps_ema`
- `model_path` 或 `std_model/tank_model`

## 4. 记录表

| 指标 | 结果 |
| --- | --- |
| `detections` 频率 | |
| `fps_ema` | |
| `processing_ms` 平均值 | |
| `inference_ms` 平均值 | |
| CPU 占用 | |
| 内存占用 | |
| 是否出现队列积压 | |
| 是否稳定输出 `/uav_vision/detections` | |

## 5. 结论

- 是否达到搜索阶段 `5-10 Hz`：
- 是否达到精修阶段 `15-20 Hz`：
- 是否需要继续优化：
- 备注：
