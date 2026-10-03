#!/usr/bin/env python3
import json
from pathlib import Path

import roslaunch


REPORT_DIR = Path(__file__).resolve().parent
ROOT = REPORT_DIR.parents[2]
SCENE = ROOT / "docs/verification/history_31_40_20260920/seed_32"
LAUNCH = ROOT / "vision_ws/src/uav_high_view/launch/fov_inner_repair.launch"

config = roslaunch.config.load_config_default(
    [(str(LAUNCH), [
        "scene_dir:=%s" % SCENE,
        "field_seed:=32",
        "corridor_fast:=true",
        "high_agl:=2.6",
        "target_model_path:=/home/xhj/liftrace/vision_ws/runs/"
        "liftrace_6cls_v5_merged_standard_20260714/weights/best.pt",
    ])],
    11311,
    verbose=False,
)
values = {name: item.value for name, item in config.params.items()}

expected = {
    "/navigation/planner_bridge/execution/initial_plan_timeout": 12.0,
    "/navigation/planner_bridge/execution/search_initial_plan_timeout": 12.0,
    "/navigation/mission_manager/high_view_probe/config/staging_xy": [0.6, 0.05],
    "/fast_planner_node/fsm/server_hold_replan_enabled": True,
    "/fast_planner_node/fsm/server_hold_seconds": 0.25,
    "/fast_planner_node/fsm/server_progress_max_age": 0.5,
    "/fast_planner_node/fsm/liveness_enabled": True,
    "/fast_planner_node/progress/enabled": True,
    "/traj_server/progress/enabled": True,
    "/traj_server/traj_server/require_goal_identity": True,
    "/navigation_vcl06_assertion/startup_wall_timeout": 180.0,
    "/random_field_spawner/spawn/initial_model_states_timeout": 90.0,
    "/target_detector/device": 0,
}
for name, value in expected.items():
    if values.get(name) != value:
        raise AssertionError("%s: %r != %r" % (name, values.get(name), value))

result = {
    "launch": str(LAUNCH),
    "scene": str(SCENE),
    "parameter_count": len(values),
    "node_count": len(config.nodes),
    "expected": expected,
    "passed": True,
}
(REPORT_DIR / "launch_validation.json").write_text(
    json.dumps(result, indent=2) + "\n", encoding="utf-8")
print("Reliability launch expansion passed")
