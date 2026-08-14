# uav_vision_eval 视觉评测包

本包只放仿真真值、隔离原型、记录器和 Gate，不作为实机运行依赖。当前生产视觉链仍在
`uav_vision`，任务/控制/规划仍在主集成工作区。

## 斜下辅助相机原型

- `launch/oblique_aux_projection_mock.launch`：单目、深度、回退、辅助记忆和零控制输出 L0；
- `launch/oblique_static_eval.launch`：45°/55°/60°固定姿态 L1；
- `launch/oblique_coverage_shadow.launch`：复用现有覆盖航线的双相机观察模式；
- `scripts/run_oblique_fixed_matrix.py`：固定姿态 smoke/full 矩阵；
- `scripts/run_oblique_coverage_shadow.sh`：生成派生机架并启动 SITL shadow；
- `scripts/generate_oblique_vehicle_sdf.py`：从未改动的单下视基线生成运行期派生 SDF；
- `config/oblique_camera_feasibility.yaml`：粗投影和辅助记忆参数。

详细架构、角度、算法、Gate 和当前证据见
[斜下辅助相机搜索可行性设计](/home/xhj/liftrace/docs/斜下辅助相机搜索可行性设计_20260815.md)。

辅助链只允许发布 `/uav_vision/aux/*`，不得发布 `/fastplanner/goal`、Servo、释放许可、
`/uav_vision/selected_target` 或旧 `/detect/*`。
