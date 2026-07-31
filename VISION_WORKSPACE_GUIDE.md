# 视觉组工作区与代码归属

更新时间：2026-07-15

## 1. 固定结论

- `vision_ws/src/uav_vision`：视觉运行主包；
- `vision_ws/src/uav_vision_eval`：仿真真值、记录和评分包；
- `patrol_uav_ws-patrol_planner`：整机集成，只接受必要的任务阶段和接口接线；
- `Visual`、`detect_ws`、`vision_ws/migration_refs`：历史参考，不作为新功能入口；
- 当前任务顺序只在 [VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md) 维护。

## 2. 目录职责

```text
vision_ws/
  src/
    uav_vision/                 # 在线视觉链
      msg/                      # 运行消息
      config/                   # 可部署参数
      launch/                   # mock/dev/sim/board 入口
      scripts/                  # Python ROS 节点
      src/ + include/           # C++ 几何节点
    uav_vision_eval/            # 只用于测试与评分
      config/scenarios/         # 固定 seed 场景
      scripts/                  # 真值、记录、报告
      launch/                   # L1/L2 评测入口
    camera_sdk/                 # 历史相机包
    detect_pkg/                 # 历史检测资产
    yolov5_detect/              # 历史 YOLO 资产
  test_data/                    # 数据集、回放、报告和证据
  migration_refs/               # 旧代码快照，不编译
```

## 3. 什么改在哪里

| 变更 | 位置 |
| --- | --- |
| 检测、融合、精修、投影、记忆、对准 | `uav_vision` |
| 消息和运行时参数 | `uav_vision/msg`、`uav_vision/config` |
| Gazebo 真值、场景、记录器、评分器 | `uav_vision_eval` |
| 实拍视频/rosbag 评测工具 | 优先 `uav_vision_eval`，证据进 `test_data` |
| RKNN 运行入口 | `uav_vision`，部署记录进板端文档 |
| 任务阶段、搜索中断许可、最终动作仲裁 | 主集成工作区对应任务/控制包 |
| LIO、地图、局部规划 | 主集成工作区，不在视觉任务中顺手修改 |

不要在 `patrol_control` 中继续堆检测算法；不要把搜索航线规划塞进 `uav_vision`；不要让评测工具成为实机运行依赖。

## 4. 运行包的当前链

```text
target_detector / cross_detector / circle_detector / landing_detector
  -> detection_fusion
  -> target_refiner
  -> target_map_projector
  -> target_memory
  -> drop_aligner
  -> detect_compat_bridge
```

主接口：

- `/uav_vision/detections`
- `/uav_vision/detections_resolved`
- `/uav_vision/detections_refined`
- `/uav_vision/detections_mapped`
- `/uav_vision/targets`
- `/uav_vision/selected_target`
- `/uav_vision/drop_offset`
- `/uav_vision/drop_ready`
- `/uav_vision/release_evidence`
- `/uav_vision/align_mode`

旧 `/detect/*` 仅用于迁移兼容。Pose 兼容输出默认不得把像素点伪装成世界坐标。

## 5. 开发循环

每个视觉改动按以下顺序：

1. 明确要改善的逐场景指标和失败样本；
2. 修改 `uav_vision` 代码/参数；
3. 编译并跑 L0 assertion；
4. 跑同一固定 seed 的 L1 Gazebo 真值集；
5. 跑相关实拍回放，检查域差；
6. 接口或时序改动再跑 L2 shadow；
7. 保存参数快照、summary、逐样本证据；
8. 更新包 README、迁移 Gate 和联调变更记录。

没有新失败证据时，不启动新一轮数据扩充或训练。

## 6. Python 与构建约定

- ROS 系统脚本使用 `/usr/bin/python3`；
- 训练和 dev/sim PyTorch 推理使用 `rl_drone`：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
```

- 不在系统 Python 中安装 ML 包；
- OrangePi 主路径使用 RKNN/NPU；
- 当前完整 ROS 运行与评测仍是笔记本 PyTorch/CPU/C++ 路径；统一六分类 FP32 RKNN 已有
  OrangePi 离线证据，但板端 ROS 相机/CameraInfo/TF 和 10 min 稳定性仍未验收；
- 图像订阅 `queue_size=1`，debug image 默认关闭；
- 话题、CameraInfo、TF、模型路径和阈值全部参数化；
- 修改 `CMakeLists.txt` 或消息后必须实际编译。

## 7. 数据与产物

- 冻结数据集不原地修改，新版本使用新目录和 manifest；
- 训练、bag、大视频、build/devel 不进入版本管理建议；
- 每次评测保存代码/模型标识、参数、场景 seed 和输入清单；
- `summary.json` 用于机器比较，`report.md` 用于人审，逐样本 CSV 用于追错；
- 日期评测文档是历史证据，不承担当前任务排序。

## 8. 相关文档

- 视觉主路线：[VISION_2026_ROADMAP.md](/home/xhj/liftrace/VISION_2026_ROADMAP.md)
- 运行包接口：[vision_ws/src/uav_vision/README.md](/home/xhj/liftrace/vision_ws/src/uav_vision/README.md)
- 仿真分层：[SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md](/home/xhj/liftrace/SIMULATION_GUIDE_NOETIC_PX4_GAZEBO_QGC.md)
- 迁移 Gate：[VISION_MIGRATION_CHECKLIST.md](/home/xhj/liftrace/VISION_MIGRATION_CHECKLIST.md)
- 板端部署：[VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md](/home/xhj/liftrace/VISION_2026_ORANGEPI5PLUS_EXECUTION_PLAN.md)
- 新旧链对照：[VISION_CHAIN_COMPARISON.md](/home/xhj/liftrace/VISION_CHAIN_COMPARISON.md)
