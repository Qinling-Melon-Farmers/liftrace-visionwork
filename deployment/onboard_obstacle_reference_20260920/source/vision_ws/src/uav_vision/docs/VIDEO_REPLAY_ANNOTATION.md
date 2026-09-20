# 当前 ROS 像素链 MP4 回放与标注

`video_replay_annotation.launch` 用真实运行节点逐帧处理 MP4，并输出一份带标注的 MP4。
它用于人工检查 YOLO 原框、红十字/H/蓝环几何中心、融合裁决及标准靶蓝环精修中心。

## 边界

该入口只启动：

```text
MP4 publisher
  -> target_detector + cross/circle/landing detectors
  -> production detection_fusion (search view + landing view)
  -> production target_refiner
  -> annotation recorder
```

- 不启动 `target_map_projector`、`target_memory`、`drop_aligner`、导航、控制、PX4、MAVROS
  或执行机构。
- 不发布 `/detections_mapped`、`/targets` 或 `/selected_target`，也不会伪造 stable ID、TF、
  地图点和投递许可。
- MP4 没有 CameraInfo/pose。publisher 只发布一份中性 CameraInfo 以初始化 C++ 像素检测器；
  本 launch 不消费它做投影，不能把它当作相机标定结果。
- landing detector 在回放中逐帧运行，便于人工看片；它的 operational 输出仍经过独立
  `landing` 模式 fusion。搜索与降落结果没有被混成一个虚构的新模式。
- `class_profile=r2026` 时 tank 的 YOLO 原框仍可见，但不会获得标准靶精修中心。

publisher 每发一帧，都会等待四个原始检测源、search fusion、landing fusion 和 refiner
全部返回，再发布下一帧。因此不会因 `queue_size=1` 丢帧；处理墙钟速度可以低于实时，
输出仍按源视频 FPS 写入，每个已解码输入帧只写一次，媒体时长保持不变。

默认 `orientation_auto=false`，保持解码器原始像素坐标。对 `real_target.mp4`，本机 conda
OpenCV 实测为 `2560x1080`；若打开自动旋转则变成 `1080x2560`，会改变 YOLO 框和几何
中心的坐标系。只有明确要验证旋转后的独立链路时才传 `orientation_auto:=true`。

## 运行

先编译所用 worktree 的视觉工作区：

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace-worktrees/vsim04-video-cd-validation/vision_ws
catkin_make
source devel/setup.bash
```

先用 30 帧检查接线和画面图例：

```bash
roslaunch uav_vision video_replay_annotation.launch \
  video_path:=/home/xhj/liftrace/vision_ws/test_data/real_target.mp4 \
  target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt \
  output_video:=/tmp/real_target_ros_pixel_chain_smoke.mp4 \
  max_frames:=30
```

全量输出使用一个新的目标文件名：

```bash
roslaunch uav_vision video_replay_annotation.launch \
  video_path:=/home/xhj/liftrace/vision_ws/test_data/real_target.mp4 \
  target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt \
  output_video:=/home/xhj/liftrace/vision_ws/test_data/real_target_ros_pixel_chain_20260902.mp4 \
  class_profile:=r2026
```

默认拒绝覆盖已有输出。明确重跑同一路径时才传 `overwrite:=true`。

## 画面图例

- 绿色：YOLO 原始框和框中心；
- 青色：circle/cross/H 检测器的原始几何候选；
- 品红色：search/observation fusion 保留的中心；
- 蓝色：landing fusion 保留的中心；
- 橙色：`target_refiner` 完成蓝环一对一关联后的标准靶中心；
- 黄色：仍只有原框、缺少有效关联或被 profile 禁止的标准靶。

顶部固定写明 `PIXEL ONLY`，避免把该视频误当作地图、stable ID 或选靶验收。
