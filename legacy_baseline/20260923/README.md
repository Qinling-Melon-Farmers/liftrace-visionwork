# patrol_control 修改前快照

来源：本工作树 `patrol_uav_ws-patrol_planner/src/patrol_control`，2026-09-23 修改前。仅为审计旧控制链最小补丁；本目录不参与 catkin 编译。

| 快照文件 | 原路径 | SHA256 |
| --- | --- | --- |
| `patrol_control/patrol_control.cpp` | `src/patrol_control/src/patrol_control.cpp` | `3300d5227a829db6f0155052bc3e3bea4c1819b101bc35243548a8b759f27d43` |
| `patrol_control/patrol_control.h` | `src/patrol_control/include/patrol_control/patrol_control.h` | `caa040dc8c1fc3940978f1a2b17e50dca06ecca6155156b248386b5371eafd32` |

变更是在外部任务对准时约束飞控中心，投递许可另检查实时位置；默认未启用的旧控制行为保持原样。

# plan_env 修改前快照

来源：本工作树 `patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env`，2026-09-23 分阶段膨胀参数改动前。完整文件清单见 `plan_env_file_list.txt`，逐文件校验值见 `plan_env_sha256.txt`；`plan_env/` 不参与构建。本次仅为点云地图增加按阶段读取水平膨胀参数和重建完成回执。
