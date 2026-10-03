# USB 相机与 YOLO 实时显示

在 Ubuntu / ROS Noetic 环境运行；每个终端先 source 本机已构建的
`vision_ws/devel/setup.bash`。相机应支持 MJPG 1280×720，标定须对应实际镜头。

## 仅相机

```bash
roslaunch camera_sdk camera_calibrated_1280x720.launch video_devices:=/dev/video0
# 另一个已 source 的终端
rosrun image_view image_view image:=/camera/image_raw
```

实现是 `camera_sdk/script/camera_sdk.py`，发布原图、CameraInfo 及按需编码的压缩图。
旧 `usb_cam.launch` 的显示订阅 `/camera/color/image_raw` 与 usb_cam 默认输出不一致，
推荐使用上面的标定入口。`uav_vision.launch` 只是 placeholder，不能启动 YOLO。

## OrangePi / RK3588

以下统一入口已经启动相机，不要再启动上一节的相机进程：

```bash
roslaunch uav_vision board_camera_vision.launch \
  video_devices:=/dev/video0 \
  model_path:=/absolute/path/merged_standard_fp32.rknn \
  metadata_path:=$(rospack find uav_vision)/config/merged_standard_6cls_metadata.yaml
```

RKNN Lite2、NPU 驱动、librknnrt 必须与现有板端运行环境匹配。
此入口的地图投影仍需外部定位 TF；仅看 YOLO 像素检测不依赖地图 TF。

## 笔记本

先启动“仅相机”入口，在已具备 ROS Python 模块与 Ultralytics 的既有 ML 环境运行：

```bash
python "$(rospack find uav_vision)/scripts/target_detector.py" \
  _model_path:=/absolute/path/best.pt _device:=cpu
```

有可用 CUDA 时可将 cpu 改为 0。不要在系统 Python 临时安装 ML 包。

## 显示识别结果（两个后端通用）

在系统 ROS Python 终端运行新增入口：

```bash
roslaunch uav_vision yolo_live_view.launch
```

新脚本已登记到 Catkin 安装列表。如尚未重新配置工作区，可直接运行源码
（无需可执行位），再单独打开显示器：

```bash
python3 "$(rospack find uav_vision)/scripts/yolo_live_view.py"
# 另一个 ROS 终端
rosrun image_view image_view image:=/uav_vision/yolo_debug
```

输出为 `/uav_vision/yolo_debug`，标注框、类别、置信度、检测数量。
按原始时间戳及 frame_id 精确匹配，只接受 source=target_detector；
不把几何分支当成 YOLO，不把旧框叠到最新帧。默认每路最多缓存30帧；
推理延迟超过缓存覆盖时间时会丢弃无法匹配的帧，可通过 buffer_frames 调整。
无检测时输出该帧及数量0；无推理结果则不制造新结果画面。
没有显示订阅者时清空缓存并跳过绘图。

无桌面板端可设置 `show_window:=false`，在配置好 ROS 网络的地面站
用 image_view / rqt_image_view 订阅该输出。默认发布 raw 图，注意网络带宽。
实际显示帧率受推理及传输限制，30 Hz 相机并不意味着30 Hz识别。

排查：`rostopic hz /camera/image_raw`、`rostopic echo /uav_vision/perf`、
`rostopic hz /uav_vision/yolo_debug`。原 detections 话题有多个来源，
其总频率不等于 YOLO 频率。已有整机应用运行时，只附加显示节点，不重复启动视觉入口。

本次仅源码审阅，未编译、未连接 USB/NPU 实测，未运行仿真或飞行。
