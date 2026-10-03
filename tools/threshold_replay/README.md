# 阈值离线复跑

仅在本机读取现成bag_replay/data.json，重新执行当前TargetMemory的候选逻辑；不重跑YOLO，不启动ROS Master/服务/控制。circle单独比较0.80与0.75，H不变；仅已解锁OFFBOARD drop_circle窗口进入结果统计。参数取当前target_memory.yaml与板端公共覆盖，不假定完全还原旧机载未归档参数。

先source现有ROS/双工作区，再激活rl_drone，使用当前视觉板端完整工程（研究分支只读复跑也需可导入生成的ROS消息）：

```bash
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash --extend
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python tools/threshold_replay/replay.py --root "$PWD" --replay /absolute/path/replay/data.json --output /absolute/path/threshold_result.json
```

多份输入可重复--replay；--coarse接收既有抽样粗投影JSON（poses、events、reference），--coarse-end限制原任务结束时间。本轮来源是本机/home/xhj/third_memory_replay.json，使用--coarse-end 1790606942.77；这是历史抽样数据，不是完整视频检测召回率评测。

样本只改变circle准入，后续同一候选链会产生不同ID/确认时序；未复跑投递上下文、飞行响应或机构，不能将其写成释放成功。中心差是邻时高分投影的一致性，不是实测靶心误差；无邻时参照必须保留unpaired计数。结果范围见docs/verification/high_speed_capture_20260929/REPORT.md。

2026-10-03：对照通过实际参数`drop_circle_geometry_confidence`运行，不再临时改共享辅助门槛。该参数仅在drop_circle生效；搜索圆环/H仍用aux_geometry_confidence=0.80。
