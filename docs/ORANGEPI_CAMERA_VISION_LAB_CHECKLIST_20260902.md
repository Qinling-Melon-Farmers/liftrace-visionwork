# OrangePi 新相机视觉链实验室清单（2026-09-02）

## 本轮边界

- 接受 `calibration_1280x720.yaml` 为已完成标定结果，不重做标定正确性复核。
- 只验证 OrangePi、USB/V4L2、1280×720 图像、CameraInfo、RKNN 推理与显示。
- 不启动解锁、起飞、舵机、PWM、真实投递或任务控制节点。
- 实验室到位后，由用户提供当前网络/IP/账号状态，再开始 SSH；优先使用 SSH key。
  必须使用密码时只交互输入，或从仓库外 `chmod 600` 的文件通过
  `ORANGEPI_SSH_PASSWORD_FILE` 读取，不写入源码、命令参数或日志。

## 上板前冻结输入

- 代码：本轮通过验证的视觉 feature revision，不使用根工作树旧 VCL05 快照。
- 模型：`merged_standard_fp32.rknn`，通过 `UAV_VISION_RKNN_MODEL_PATH` 或 launch 的
  `model_path:=...` 显式传入；模型缺失必须启动失败。
- 元数据：`merged_standard_6cls_metadata.yaml`。
- 相机标定：`vision_ws/src/camera_sdk/param/calibration_1280x720.yaml`。
- 相机设备优先使用 `/dev/v4l/by-id/...` 稳定路径，不依赖可漂移的 `/dev/video0`。

## 安全测试顺序

1. 只读检查 SSH、系统、磁盘、内存、ROS、OpenCV、RKNNLite。
2. `lsusb`、`lsusb -t`、`v4l2-ctl --list-devices` 和
   `v4l2-ctl --list-formats-ext` 确认 USB 拓扑与 1280×720 MJPG@30 能力。
3. 用 V4L2 抓一帧 1280×720 图像，拉回笔记本审片；记录实际协商分辨率、FPS、FOURCC。
4. 单独启动标定相机 ROS 节点，核对 Image/CompressedImage/CameraInfo 同 stamp、
   同 `downward_camera_optical_frame`、分辨率严格为 1280×720。
5. 单独运行板载 RKNN 查看器，确认真实相机画面、六分类框、置信度、推理耗时与板载显示。
6. 启动统一视觉入口，检查 detections、resolved/refined/mapped、perf 和 debug image；
   没有安装外参/TF 时，mapped 必须明确无效，不得伪造地图点。
7. 补齐相机安装外参后，再做地图点和 10 分钟 CPU/NPU/RSS/温度稳定性 Gate。

统一入口：

```bash
unzip orangepi_vision_camera_20260902.zip
cd orangepi_vision_camera_20260902/vision_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch uav_vision board_camera_vision.launch \
  video_devices:=/dev/v4l/by-id/<camera-id> \
  model_path:=$(pwd)/src/uav_vision/models/merged_standard_fp32.rknn \
  enable_debug_image:=true
```

板载直接显示入口：

```bash
cd /home/orangepi/<deploy>/orangepi_vision_camera_20260902
export DISPLAY=:0
export XAUTHORITY=/home/orangepi/.Xauthority
python3 top_level_scripts/board_realtime_rknn_viewer.py \
  --camera /dev/v4l/by-id/<camera-id> \
  --camera-width 1280 --camera-height 720 --camera-fps 30 \
  --calibration vision_ws/src/camera_sdk/param/calibration_1280x720.yaml \
  vision_ws/src/uav_vision/models/merged_standard_fp32.rknn
```

## 通过条件

- USB 无反复断连或降速，设备稳定可识别。
- 实际采集为 1280×720，协商 FPS 可见且与设备能力一致。
- Image 与 CameraInfo 的尺寸、stamp、frame 一致。
- 缺模型/RKNNLite/相机时 fail-fast，不发布伪健康空检测。
- 实时画面可见，目标框与类别可人工检查，性能话题持续更新且无帧积压。
- raw 图中心在地图投影前使用 CameraInfo 畸变校正；没有有效 TF 时不输出有效地图点。
