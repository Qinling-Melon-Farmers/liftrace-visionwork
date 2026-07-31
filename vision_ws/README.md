# vision_ws

更新时间：2026-07-15

## 1. 定位

这是视觉组日常开发与评测工作区。在线视觉主包为 `src/uav_vision`；
`src/uav_vision_eval` 专门承载 Gazebo 真值、场景、记录和评分，避免测试代码污染运行包。
当前推理/识别均在笔记本运行，OrangePi 尚未验收。

当前业务优先级、架构和仿真 Gate 统一见 [../VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)。

## 2. 目录

| 路径 | 用途 |
| --- | --- |
| `src/uav_vision` | 2026 在线视觉链 |
| `src/uav_vision_eval` | 仿真/回放评测包 |
| `src/camera_sdk` | 历史相机资产 |
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
# Phase D dev/sim 在线链
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
- L1 Gazebo 真值：五类标准靶、红十字、H、背景已有固定基线，正式召回/时延 Gate 未全过；
- L2 headless shadow：隔离契约已通过，10 min 稳定性未完成；
- 圆环实例关联：单目标 Gazebo 可用，实拍仅有 58.80% 自洽率且缺人工真值；
- 地图投影/ID：固定 Gazebo 和 mock 已有，30-seed 跨视角/同步实拍 pose 未完成；
- H：内部结构与 landing 阶段门控已实现，实拍负样本待扩；
- 释放：`release_evidence` 已实现，任务/安全层最终许可未实现；
- 部署：PT/ONNX 对照未通过；统一六分类 RKNN 和 OrangePi 实测未完成。

详细状态见 [../VISION_MIGRATION_CHECKLIST.md](/home/xhj/liftrace/VISION_MIGRATION_CHECKLIST.md)。

## 7. 数据与模型

- 当前默认 dev/sim 模型和参数以 `src/uav_vision/config` 为准；
- 模型训练历史、压力集和候选对照保存在 `test_data`、`runs` 及日期评测文档；
- 不因某个训练指标更高就直接切换默认模型；
- 下一次模型/数据迭代必须由固定仿真或实拍失败样本驱动，并在同一评测集上给出前后对照；
- OrangePi 只使用 RKNN/NPU 主路径。

## 8. 下一步

当前先完成 30-seed/10 min 正式回归、降低笔记本融合时延并提高低召回类；随后补实拍
圆环/H/红十字人工真值和同步 pose，修复 PT/ONNX 差异。只有这些前置满足并获得板端条件后，
才开始统一六分类 RKNN/OrangePi 验收。

## 9. 相关文档

- 包接口：[src/uav_vision/README.md](/home/xhj/liftrace/vision_ws/src/uav_vision/README.md)
- 工作区规则：[../VISION_WORKSPACE_GUIDE.md](/home/xhj/liftrace/VISION_WORKSPACE_GUIDE.md)
- 仿真分层：[../SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md](/home/xhj/liftrace/SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md)
- 主路线：[../VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)
- 板端部署：[../VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md](/home/xhj/liftrace/VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md)
