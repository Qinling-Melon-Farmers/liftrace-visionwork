import ast
from pathlib import Path

import roslaunch.config
import roslib.packages

root = Path(__file__).resolve().parents[1]
for source in root.rglob("*.py"):
    ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
for enabled in (False, True):
    arguments = ["enable_control_output:=" + str(enabled).lower(),
                 "start_servo:=" + str(enabled).lower(), "start_camera:=true"]
    config = roslaunch.config.load_config_default([(str(root / "test.launch"), arguments)], None, verbose=False)
    nodes = {node.namespace.rstrip("/") + "/" + node.name: node for node in config.nodes}
    assert len(nodes) == len(config.nodes), "duplicate nodes"
    for node in config.nodes:
        assert roslib.packages.find_node(node.package, node.type), "missing executable: " + node.package + "/" + node.type
    params = {name: param.value for name, param in config.params.items()}
    assert params["/target_map_projector/ground_z"] == -.25
    assert params["/navigation_frame_adapter/enable_setpoints"] == enabled
    assert params["/fast_planner_node/sdf_map/map_size_y"] == 12.0
    assert params["/navigation/planner_bridge/execution/enabled"] == enabled
    assert params["/navigation/planner_bridge/target/recovery_height"] == .20
    assert params["/navigation/planner_bridge/execution/max_goal_z"] == .25
    for name in ("/patrol_control", "/navigation/mission_manager", "/servo_controller1", "/right_servo_once", "/guarded_servo_proxy"):
        assert (name in nodes) == enabled, name
    assert sum(node.name == "fast_planner_node" for node in config.nodes) == 1
    if enabled:
        assert params["/external_planner_max_command_z"] == .25
        assert params["/align_height"] == .25
        assert params["/drop_system/release_setpoint_height"] == .1
        assert params["/uav_vision/recovery_height"] == .20
        route = params["/navigation/mission_manager/mission/post_delivery_route"]
        assert len(route) == 7 and route[-1] == [1.4, 4.4, .25]
        assert all(point[2] == .25 for point in route)
        assert params["/guarded_servo_proxy/raw_service_name"] == "/test_4x5/right_once"
        assert params["/release_permission_arbiter/payload_slots"] == 1
        assert ["/mavros/setpoint_position/local", "/navigation/setpoint_mission"] in nodes["/patrol_control"].remap_args
        assert nodes["/navigation_frame_adapter"].remap_args == []
        assert ["Servo", "/legacy/Servo_raw"] in nodes["/servo_controller1"].remap_args
    print("PASS", "flight" if enabled else "preview", len(config.nodes), "nodes; parameters/remaps verified; no nodes started")
