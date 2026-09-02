# vision_ws

更新时间：2026-08-31

## 1. 定位

这是视觉组日常开发与评测工作区。在线视觉主包为 `src/uav_vision`；
`src/uav_vision_eval` 专门承载 Gazebo 真值、场景、记录和评分，避免测试代码污染运行包。
笔记本 PT/ONNX 与 Gazebo camera-only 600 秒已有证据；OrangePi FP32 RKNN 已完成离线
图片/视频验证，板端 ROS CameraInfo/TF 和 10 分钟仍未验收。

当前业务优先级、架构和仿真 Gate 统一见 [../VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。

## 2. 目录

| 路径 | 用途 |
| --- | --- |
| `src/uav_vision` | 2026 在线视觉链 |
| `src/uav_vision_eval` | 仿真/回放评测包 |
| `src/camera_sdk` | 相机发布节点与新旧标定 profile |
| `src/detect_pkg` | 历史检测资产 |
| `src/yolov5_detect` | 历史 YOLO 参考 |
| `migration_refs` | 旧视觉快照，不参与编译 |
| `test_data` | 数据集、回放、评测和证据 |
| `runs` | 训练与模型产物 |

## 3. 当前链

```text
image + CameraInfo + TF
  -> target/cross/circle/landing detectors
  -> detection_fusion
  -> target_refiner
  -> target_map_projector
  -> target_memory
  -> drop_aligner
  -> detect_compat_bridge
```

现有主接口：

```text
/uav_vision/detections
/uav_vision/detections_resolved
/uav_vision/detections_refined
/uav_vision/detections_mapped
/uav_vision/targets
/uav_vision/selected_target
/uav_vision/drop_offset
/uav_vision/drop_ready
/uav_vision/release_evidence
/uav_vision/align_mode
```

重要语义：`selected_target` 是候选建议，`drop_ready` 是视觉对准观测，二者都不是飞行动作或舵机许可。

## 4. 构建

```bash
source /opt/ros/noetic/setup.bash
cd /home/xhj/liftrace/vision_ws
catkin_make -j1
source devel/setup.bash
```

训练或 dev/sim PyTorch 推理使用既有环境：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
```

不要在系统 Python 中安装 ML 包。

## 5. 常用入口

```bash
# Phase D dev/sim 在线链（模型必须显式存在）
UAV_VISION_MODEL_PATH=/absolute/path/to/best.pt \
  roslaunch uav_vision phase_d.launch

# Phase D 板端 RKNN 入口
roslaunch uav_vision phase_d_board.launch

# 确定性地图/记忆/对准回归
roslaunch uav_vision phase_d_map_mock.launch

# 新视觉—旧主控接口 assertion
roslaunch uav_vision phase_d_mock_patrol_regression.launch
```

预定的 toudi3 新视觉 GUI 烟测：

```bash
cd /home/xhj/liftrace
bash ./top_level_scripts/run_toudi3_full_competition_sim_gui_new.sh
```

该命令只证明链路贯通，不是视觉精度验收或 shadow 模式。联合工作区应使用：

```bash
source /home/xhj/liftrace/top_level_scripts/toudi3_combined_env.sh
liftrace_setup_toudi3_combined_env
liftrace_assert_toudi3_combined_env
```

## 6. 当前 Gate

- L0 mock：圆环坐标、全局关联、地图/新鲜度、H 门控和释放证据有 assertion；
- L1 Gazebo 真值：formal23 22/23、static25 24/25、sparse30 25/30，2 m/s 与 3.6 m
  不作为通用工作域；
- L2 camera-only：隔离契约与笔记本/Gazebo 600 秒已通过，联合导航和板端 10 min 未完成；
- 圆环实例关联：`real_target.mp4` 整段模型/完整链回放、板端抽样和人工审片已完成；58.80%
  是预测框与算法圆环候选的自洽率，仍缺独立人工中心/实例真值；
- 地图投影/ID：固定 Gazebo 和 mock 已有，30-seed 跨视角/同步实拍 pose 未完成；
- H：内部结构与 landing 阶段门控已实现，实拍负样本待扩；
- 释放：`release_evidence` 已实现，任务/安全层最终许可未实现；
- 部署：PT/ONNX fixed-letterbox 对照已通过；FP32 RKNN 离线有效，OrangePi ROS 链未验收。

新相机标定源文件为 `calibration.yaml`，ROS profile 为
`src/camera_sdk/param/calibration_1280x720.yaml`。连接新相机时使用：

```bash
roslaunch camera_sdk camera_calibrated_1280x720.launch \
  video_devices:=/dev/videoX \
  frame_id:=downward_camera_optical_frame
```

该入口严格要求实际输出 1280×720、rotation=0，并统一三个相机话题的 frame。安装外参仍须
在机架安装后独立标定；不要把这份内参注入 Gazebo 或旧 2560×1080 手机视频。

详细状态见 [../VISION_MIGRATION_CHECKLIST.md](/home/xhj/liftrace/VISION_MIGRATION_CHECKLIST.md)。

## 7. 数据与模型

- dev/sim 模型必须由 launch 参数或 `UAV_VISION_MODEL_PATH` 显式提供；PT/RKNN 节点缺模型或
  运行时会立即退出，不再用 completed 空检测维持伪健康链路；
- 模型训练历史、压力集和候选对照保存在 `test_data`、`runs` 及日期评测文档；
- 不因某个训练指标更高就直接切换默认模型；
- 下一次模型/数据迭代必须由固定仿真或实拍失败样本驱动，并在同一评测集上给出前后对照；
- OrangePi 只使用 RKNN/NPU 主路径。

## 8. 下一步

当前先恢复 VCL06 地图链并带有效模型完成 90 秒 typed smoke；视觉侧随后补实拍圆环/H/
红十字人工定量真值、同步 pose 与 30-seed。新相机完成实际 1280×720 CameraInfo/安装外参后，
进入统一六分类 RKNN/OrangePi ROS 5–10 Hz 和 10 分钟验收。

## 9. 相关文档

- 包接口：[src/uav_vision/README.md](/home/xhj/liftrace/vision_ws/src/uav_vision/README.md)
- 工作区规则：[../VISION_WORKSPACE_GUIDE.md](/home/xhj/liftrace/VISION_WORKSPACE_GUIDE.md)
- 仿真分层：[../SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md](/home/xhj/liftrace/SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md)
- 主路线：[../VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)
- 板端部署：[../VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md](/home/xhj/liftrace/VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md)
