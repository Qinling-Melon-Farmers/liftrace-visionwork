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
   TF 链不完整时，mapped 必须明确无效，不得伪造地图点。
7. Image+CameraInfo+RKNN 的 10 分钟资源/心跳稳定性已先行完成；2026-09-05 机械平移接线
   后，再做有效地图点与 `optical→body→mission` TF Gate。

统一入口：

```bash
unzip orangepi_vision_camera_20260902.zip
cd orangepi_vision_camera_20260902/vision_ws
source /opt/ros/noetic/setup.bash
# 板端 /usr/local 还保留历史 OpenCV 3.4；ROS Noetic 的 cv_bridge 使用系统
# OpenCV 4.2，必须显式选择同一 ABI，避免把 3.4/4.2 同时链接进视觉节点。
catkin_make \
  -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -DCMAKE_BUILD_TYPE=Release \
  -j4 -l4
source devel/setup.bash
roslaunch uav_vision board_camera_vision.launch \
  video_devices:=/dev/v4l/by-id/<camera-id> \
  model_path:=$(pwd)/src/uav_vision/models/merged_standard_fp32.rknn \
  enable_debug_image:=true
```

默认外参为当前可连通链路的 `body → downward_camera_optical_frame`，平移
`[0,0,-0.16] m`；雷达 IMU 到相机的主机械测量是正下方 `0.20 m`，不能把它直接填给
当前由飞控 IMU 驱动的 `body`。若上层整机 launch 已发布同一 TF，追加
`publish_camera_extrinsic:=false`。详见
[2026 实机安装外参基线](/home/xhj/liftrace/docs/2026实机相机与投递机构安装外参基线_20260905.md)。

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

## 2026-09-02 现场结果

- 受测视觉 revision：`abc5103bcd0d4c8d3331f64d9f3bf0cce1bab177`。
- 新相机严格发布 `1280×720 bgr8` Image 和同 stamp/frame 的 CameraInfo；K/D 与当前
  `calibration_1280x720.yaml` 一致。
- 无人触碰清洁复跑 wall 601 秒：raw/info 为 17,498/17,499 条、29.164/29.163 Hz；统一
  RKNN 为 8,223 条、13.705 Hz，processing P50/P95 `62.186/73.478 ms`，inference
  P50/P95 `54.684/63.894 ms`。
- `errors=[]`、`forbidden_nodes=[]`、新增 USB/UVC 内核日志 0 B；roslaunch/rosmaster
  60/60 个健康采样存活。无安装外参，本轮没有有效 mapped 检测。
- 后 213 秒旁路：NPU 22/22 个点均为 100%@1 GHz，10 个视觉节点 CPU 均值求和约
  3.66 核，RSS 均值求和约 952.5 MiB（含共享页重复，非 USS）；最高温度 69.307 °C，
  所有可见 cooling state 均为 0。
- 冻结 revision 的 15 秒 compressed 回归同时得到 raw/info/JPEG 438/439/439 条、约
  29.17 Hz，438 组三话题同 stamp，确认按需 JPEG 没有破坏远程显示入口。
- 首轮 600 秒的人为碰线造成一次真实 USB disconnect，节点约 2.81 秒后自动重开；该轮只
  作为故障恢复证据，不作为清洁稳定性 PASS。

完整指标、边界和证据索引见
[OrangePi板端视觉性能报告_20260902.md](/home/xhj/liftrace/docs/OrangePi板端视觉性能报告_20260902.md)。
现场测试时相机主要朝顶棚且尚未取得安装外参；上述结果只关闭设备、CameraInfo、RKNN
像素链和板端 10 分钟稳定性。2026-09-05 已取得并配置机械平移，但尚未在完整
`camera_init → body → optical` 树上验证，仍不关闭实景识别精度、地图定位、联合导航或飞行。

## 通过条件

- USB 无反复断连或降速，设备稳定可识别。
- 实际采集为 1280×720，协商 FPS 可见且与设备能力一致。
- Image 与 CameraInfo 的尺寸、stamp、frame 一致。
- 缺模型/RKNNLite/相机时 fail-fast，不发布伪健康空检测。
- 实时画面可见，目标框与类别可人工检查，性能话题持续更新且无帧积压。
- raw 图中心在地图投影前使用 CameraInfo 畸变校正；没有有效 TF 时不输出有效地图点。
