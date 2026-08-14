# uav_vision_eval 视觉评测包

本包只放仿真真值、隔离原型、记录器和 Gate，不作为实机运行依赖。当前生产视觉链仍在
`uav_vision`，任务/控制/规划仍在主集成工作区。

## 斜下辅助相机原型

- `launch/oblique_aux_projection_mock.launch`：单目、深度、回退、辅助记忆和零控制输出 L0；
- `launch/oblique_static_eval.launch`：45°/55°/60°固定姿态 L1；
- `launch/oblique_coverage_shadow.launch`：复用现有覆盖航线的双相机观察模式；
- `launch/oblique_aux_blue_chain.launch`：斜下 RGB 的 OpenCV 蓝环匿名粗发现；
- `launch/oblique_guided_search.launch`：评测专用主动 A/B，可比较原 12 点覆盖与粗候选裁路；
- `scripts/run_oblique_fixed_matrix.py`：固定姿态 smoke/full 矩阵；
- `scripts/run_oblique_coverage_shadow.sh`：生成派生机架并启动 SITL shadow；
- `scripts/run_oblique_guided_ab.sh`：无 GUI 重复运行主动搜索 A/B 并汇总；
- `scripts/generate_oblique_vehicle_sdf.py`：从未改动的单下视基线生成运行期派生 SDF；
- `config/oblique_camera_feasibility.yaml`：粗投影和辅助记忆参数。

详细架构、角度、算法、Gate 和当前证据见
[斜下辅助相机搜索可行性设计](/home/xhj/liftrace/docs/斜下辅助相机搜索可行性设计_20260815.md)。

辅助视觉链只允许发布 `/uav_vision/aux/*`，不得发布 Servo、释放许可、
`/uav_vision/selected_target` 或旧 `/detect/*`。唯一例外是
`aux_guided_search_manager.py`：它是 `uav_vision_eval` 内的临时导航外挂，只在主动 A/B
launch 中独占 `/fastplanner/goal`，用于测量实际路线收益，不属于视觉组生产交付接口。

推荐生产路线不是双 YOLO 常开，而是一个常驻 RKNN/YOLO 实例按
`AUX_SEARCH -> APPROACH -> DOWNWARD_VERIFY -> DROP_ALIGN` 阶段切换输入源。OpenCV 蓝环
只作为低资源基线；真实环境效果不足时允许辅助粗发现改用 YOLO，下视链始终负责最终类别、
地图点、stable ID 和投递证据。

主动 A/B（自动无 GUI，所有子运行经 `sim_run.sh` 归档）：

```bash
cd /home/xhj/liftrace
bash vision_ws/src/uav_vision_eval/scripts/run_oblique_guided_ab.sh --repeats 3
```

当前固定坐标首轮辅助搜索时间缩短 47.5%、路径缩短 40.2%，但第二次辅助运行受 planner
不可达影响而 FAIL，因此只能判定“有显著潜力，稳定性未通过”。详细证据和全随机下一 Gate
见设计文档。
