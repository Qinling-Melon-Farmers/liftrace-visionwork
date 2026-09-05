# VCL06 整机 ROS 运行拓扑快照

- 采集时间：`2026-09-06T00:48:43.206376+08:00`
- ROS Master：`http://localhost:11311`
- 节点：`33`；话题：`239`；发布边：`227`；订阅边：`187`；服务：`152`
- 正式接口审计：**FAIL**
- 未通过项：`legacy_actuator_topics_absent`

## 来源

- `uav_vision`: `/home/xhj/liftrace-worktrees/vdeploy-final-closeout-plan/vision_ws/src/uav_vision`
- `uav_mission`: `/home/xhj/liftrace-controlwork-worktrees/vcl06-local-full-mission/patrol_uav_ws-patrol_planner/src/uav_mission`

## 接口审计

| 检查 | 结果 | 现场值 |
| --- | --- | --- |
| `formal_legacy_topics_absent` | PASS | `present=[]` |
| `legacy_actuator_topics_absent` | FAIL | `present={'/control1': {'publishers': ['/patrol_control'], 'subscribers': []}, '/control2': {'publishers': ['/patrol_control'], 'subscribers': []}, '/control3': {'publishers': ['/patrol_control'], 'subscribers': []}, '/servo/complete': {'publishers': [], 'subscribers': ['/patrol_control']}}` |
| `point_class_has_live_contract` | PASS | `publishers=['/patrol_control'], subscribers=['/navigation/planner_bridge', '/release_permission_arbiter', '/visual_delivery_audit']` |
| `typed_detections_connected` | PASS | `publishers=['/target_map_projector'], subscribers=['/navigation_vcl06_assertion', '/patrol_control', '/target_memory']` |
| `single_planner_goal_publisher` | PASS | `publishers=['/navigation/planner_bridge']` |
| `legacy_bridge_node_absent` | PASS | `matching_nodes=[]` |
| `old_coverage_manager_absent` | PASS | `matching_nodes=[]` |
| `node_present:/navigation/mission_manager` | PASS | `present=True` |
| `node_present:/navigation/planner_bridge` | PASS | `present=True` |
| `guarded_servo_service_present` | PASS | `providers=['/guarded_servo_proxy']` |
| `mock_raw_servo_service_present` | PASS | `providers=['/mock_raw_servo_server']` |
| `single_visual_source_root` | PASS | `uav_vision=/home/xhj/liftrace-worktrees/vdeploy-final-closeout-plan/vision_ws/src/uav_vision` |
| `single_navigation_source_root` | PASS | `uav_mission=/home/xhj/liftrace-controlwork-worktrees/vcl06-local-full-mission/patrol_uav_ws-patrol_planner/src/uav_mission` |

## 图和原始数据

- `rqt_graph_nodes_only_core.svg/png`：主交付图；rqt_graph `Nodes only` 后端生成，椭圆为节点、边标签为话题、箭头为发布到订阅方向。
- `rqt_graph_nodes_only_full.svg/png`：同口径全量运行图。
- `ros_nodes.txt`、`ros_topics_verbose.txt`：分别由 `rosnode list`、`rostopic list -v` 直接生成。
- `nodes.csv`、`topics.csv`、`edges.csv`、`services.csv`、`one_sided_topics.csv`、`ros_system_state.json`：结构化快照。

Graphviz 渲染：`{"rqt_nodes_only": {"full": {"svg": {"ok": true, "path": "rqt_graph_nodes_only_full.svg", "output": ""}, "png": {"ok": true, "path": "rqt_graph_nodes_only_full.png", "output": ""}}, "core": {"svg": {"ok": true, "path": "rqt_graph_nodes_only_core.svg", "output": ""}, "png": {"ok": true, "path": "rqt_graph_nodes_only_core.png", "output": ""}}, "graph_mode": "node_node", "core_nodes": 28, "core_topics": 41}}`

该快照只描述采集时 ROS Master 中已注册的接口；未启动的 legacy 回归入口不在图中。
