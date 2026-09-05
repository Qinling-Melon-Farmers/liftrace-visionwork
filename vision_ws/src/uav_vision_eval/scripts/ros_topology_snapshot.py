#!/usr/bin/env python3
"""Capture a live ROS 1 node/topic graph and audit the formal VCL06 interfaces."""

import argparse
import csv
import datetime as dt
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import rosgraph


CORE_TOPICS = {
    "/detect/point_class",
    "/downward_camera/camera_info",
    "/downward_camera/image_raw",
    "/fastplanner/goal",
    "/fastplanner/setpoint_position/local",
    "/freedom/static_pointcloud",
    "/mavros/extended_state",
    "/mavros/local_position/odom",
    "/mavros/local_position/pose",
    "/mavros/setpoint_position/local",
    "/mavros/state",
    "/mission/command",
    "/mission/control_ready",
    "/mission/gazebo_contact_status",
    "/mission/planner_anchor_status",
    "/mission/random_field_status",
    "/mission/release_commitment_evidence",
    "/mission/release_permission",
    "/mission/release_permission_active",
    "/mission/release_result",
    "/mission/uav_contacts_raw",
    "/navigation/mission_command_raw",
    "/navigation/mission_result",
    "/navigation/mission_start_gate_status",
    "/navigation/mission_status",
    "/navigation/planner_bridge_status",
    "/planning/bspline",
    "/planning/goal_status",
    "/planning/pos_cmd",
    "/uav_vision/align_mode",
    "/uav_vision/alignment_target_context",
    "/uav_vision/detections",
    "/uav_vision/detections_mapped",
    "/uav_vision/detections_refined",
    "/uav_vision/detections_resolved",
    "/uav_vision/drop_offset",
    "/uav_vision/drop_ready",
    "/uav_vision/release_evidence",
    "/uav_vision/release_evidence_context",
    "/uav_vision/selected_target",
    "/uav_vision/targets",
}

# These names belong to the explicitly selected 2025 compatibility entry and
# must not be registered by the formal VCL06 launch.
FORBIDDEN_FORMAL_TOPICS = {
    "/cross/control",
    "/detect/class_control",
    "/detect/control",
    "/detect/cross_mark_point",
    "/detect/cross_status",
    "/detect/land_mark_point",
    "/detect/landing_control",
    "/detect/servo_complete",
    "/detect/servo_status",
    "/detect/tank_control",
    "/detect/tank_status",
    "/detect/waypoint_mark_point",
    "/yolo_detect",
}

# The service-based actuator contract supersedes these old topic controls.
# They are audited separately because they originate in patrol_control rather
# than the visual compatibility bridge.
LEGACY_ACTUATOR_TOPIC_CANDIDATES = {
    "/control1",
    "/control2",
    "/control3",
    "/servo/complete",
}


def run_command(argv):
    completed = subprocess.run(
        argv,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return completed.returncode, completed.stdout


def state_map(entries):
    return {name: sorted(set(peers)) for name, peers in entries}


def write_csv(path, header, rows):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(header)
        writer.writerows(rows)


def render_dot(dot_path):
    results = {}
    for output_type in ("svg", "png"):
        output_path = dot_path.with_suffix("." + output_type)
        code, output = run_command(
            ["dot", "-T" + output_type, str(dot_path), "-o", str(output_path)]
        )
        results[output_type] = {
            "ok": code == 0,
            "path": output_path.name,
            "output": output.strip(),
        }
    return results


def write_rqt_nodes_only_graphs(
    output_dir, publishers, subscribers, registered_topics
):
    """Generate the same node-to-node graph used by rqt_graph Nodes only."""
    import rosgraph.impl.graph
    from qt_dotgraph.pydotfactory import PydotFactory
    from rqt_graph.dotcode import NODE_NODE_GRAPH, RosGraphDotcodeGenerator

    graph = rosgraph.impl.graph.Graph()
    graph.set_master_stale(5.0)
    graph.set_node_stale(5.0)
    graph.update()

    # __init__ only subscribes to /statistics.  The topology snapshot does not
    # need traffic colouring and must not register its own ROS node.
    generator = RosGraphDotcodeGenerator.__new__(RosGraphDotcodeGenerator)
    factory = PydotFactory()

    common = {
        "rosgraphinst": graph,
        "graph_mode": NODE_NODE_GRAPH,
        "dotcode_factory": factory,
        "hide_single_connection_topics": False,
        "hide_dead_end_topics": False,
        "cluster_namespaces_level": 0,
        "accumulate_actions": False,
        "orientation": "LR",
        "rank": "same",
        "ranksep": 0.2,
        "rankdir": "TB",
        "simplify": False,
        "quiet": True,
        "unreachable": False,
        "hide_tf_nodes": False,
        "group_tf_nodes": False,
        "group_image_nodes": False,
        "hide_dynamic_reconfigure": True,
    }

    full_dot = output_dir / "rqt_graph_nodes_only_full.dot"
    full_dot.write_text(
        generator.generate_dotcode(ns_filter="/", topic_filter="/", **common),
        encoding="utf-8",
    )

    core_topics = sorted(CORE_TOPICS & set(registered_topics))
    core_nodes = sorted(
        {
            node
            for topic in core_topics
            for node in publishers.get(topic, []) + subscribers.get(topic, [])
        }
    )
    core_dot = output_dir / "rqt_graph_nodes_only_core.dot"
    core_dot.write_text(
        generator.generate_dotcode(
            ns_filter=",".join(core_nodes),
            topic_filter=",".join(core_topics),
            **common
        ),
        encoding="utf-8",
    )
    return {
        "full": render_dot(full_dot),
        "core": render_dot(core_dot),
        "graph_mode": NODE_NODE_GRAPH,
        "core_nodes": len(core_nodes),
        "core_topics": len(core_topics),
    }


def add_check(checks, check_id, passed, detail):
    checks.append({"id": check_id, "passed": bool(passed), "detail": detail})


def build_audit(publishers, subscribers, services, nodes, package_paths):
    checks = []
    registered_topics = set(publishers) | set(subscribers)
    forbidden_present = sorted(FORBIDDEN_FORMAL_TOPICS & registered_topics)
    add_check(
        checks,
        "formal_legacy_topics_absent",
        not forbidden_present,
        "present={}".format(forbidden_present),
    )
    actuator_topics_present = sorted(
        LEGACY_ACTUATOR_TOPIC_CANDIDATES & registered_topics
    )
    add_check(
        checks,
        "legacy_actuator_topics_absent",
        not actuator_topics_present,
        "present={}".format(
            {
                topic: {
                    "publishers": publishers.get(topic, []),
                    "subscribers": subscribers.get(topic, []),
                }
                for topic in actuator_topics_present
            }
        ),
    )

    point_publishers = publishers.get("/detect/point_class", [])
    point_subscribers = subscribers.get("/detect/point_class", [])
    add_check(
        checks,
        "point_class_has_live_contract",
        bool(point_publishers and point_subscribers),
        "publishers={}, subscribers={}".format(point_publishers, point_subscribers),
    )

    mapped_publishers = publishers.get("/uav_vision/detections_mapped", [])
    mapped_subscribers = subscribers.get("/uav_vision/detections_mapped", [])
    add_check(
        checks,
        "typed_detections_connected",
        bool(mapped_publishers and mapped_subscribers),
        "publishers={}, subscribers={}".format(mapped_publishers, mapped_subscribers),
    )

    goal_publishers = publishers.get("/fastplanner/goal", [])
    add_check(
        checks,
        "single_planner_goal_publisher",
        goal_publishers == ["/navigation/planner_bridge"],
        "publishers={}".format(goal_publishers),
    )
    add_check(
        checks,
        "legacy_bridge_node_absent",
        not any(node.endswith("/detect_compat_bridge") for node in nodes),
        "matching_nodes={}".format(
            sorted(node for node in nodes if node.endswith("/detect_compat_bridge"))
        ),
    )
    add_check(
        checks,
        "old_coverage_manager_absent",
        not any(node.endswith("/coverage_search_manager") for node in nodes),
        "matching_nodes={}".format(
            sorted(node for node in nodes if node.endswith("/coverage_search_manager"))
        ),
    )
    for node_name in ("/navigation/mission_manager", "/navigation/planner_bridge"):
        add_check(
            checks,
            "node_present:" + node_name,
            node_name in nodes,
            "present={}".format(node_name in nodes),
        )

    servo_providers = services.get("/Servo", [])
    raw_servo_providers = services.get("/legacy/Servo_raw", [])
    add_check(
        checks,
        "guarded_servo_service_present",
        len(servo_providers) == 1,
        "providers={}".format(servo_providers),
    )
    add_check(
        checks,
        "mock_raw_servo_service_present",
        len(raw_servo_providers) == 1,
        "providers={}".format(raw_servo_providers),
    )

    vision_path = package_paths.get("uav_vision", "")
    mission_path = package_paths.get("uav_mission", "")
    add_check(
        checks,
        "single_visual_source_root",
        "/liftrace-worktrees/vdeploy-final-closeout-plan/vision_ws/src/uav_vision" in vision_path,
        "uav_vision={}".format(vision_path),
    )
    add_check(
        checks,
        "single_navigation_source_root",
        "/liftrace-controlwork-worktrees/vcl06-local-full-mission/" in mission_path,
        "uav_mission={}".format(mission_path),
    )
    return {
        "passed": all(check["passed"] for check in checks),
        "checks": checks,
        "forbidden_topics": sorted(FORBIDDEN_FORMAL_TOPICS),
        "legacy_actuator_topic_candidates": sorted(
            LEGACY_ACTUATOR_TOPIC_CANDIDATES
        ),
    }


def make_report(snapshot, audit, render_results):
    failed_checks = [
        check["id"] for check in audit["checks"] if not check["passed"]
    ]
    lines = [
        "# VCL06 整机 ROS 运行拓扑快照",
        "",
        "- 采集时间：`{}`".format(snapshot["captured_at"]),
        "- ROS Master：`{}`".format(snapshot["master_uri"]),
        "- 节点：`{}`；话题：`{}`；发布边：`{}`；订阅边：`{}`；服务：`{}`".format(
            snapshot["counts"]["nodes"],
            snapshot["counts"]["topics"],
            snapshot["counts"]["publisher_edges"],
            snapshot["counts"]["subscriber_edges"],
            snapshot["counts"]["services"],
        ),
        "- 正式接口审计：**{}**".format("PASS" if audit["passed"] else "FAIL"),
        "- 未通过项：`{}`".format(
            ", ".join(failed_checks) if failed_checks else "无"
        ),
        "",
        "## 来源",
        "",
        "- `uav_vision`: `{}`".format(snapshot["package_paths"].get("uav_vision", "")),
        "- `uav_mission`: `{}`".format(snapshot["package_paths"].get("uav_mission", "")),
        "",
        "## 接口审计",
        "",
        "| 检查 | 结果 | 现场值 |",
        "| --- | --- | --- |",
    ]
    for check in audit["checks"]:
        detail = check["detail"].replace("|", "\\|")
        lines.append(
            "| `{}` | {} | `{}` |".format(
                check["id"], "PASS" if check["passed"] else "FAIL", detail
            )
        )
    lines.extend(
        [
            "",
            "## 图和原始数据",
            "",
            "- `rqt_graph_nodes_only_core.svg/png`：主交付图；rqt_graph `Nodes only` 后端生成，椭圆为节点、边标签为话题、箭头为发布到订阅方向。",
            "- `rqt_graph_nodes_only_full.svg/png`：同口径全量运行图。",
            "- `ros_nodes.txt`、`ros_topics_verbose.txt`：分别由 `rosnode list`、`rostopic list -v` 直接生成。",
            "- `nodes.csv`、`topics.csv`、`edges.csv`、`services.csv`、`one_sided_topics.csv`、`ros_system_state.json`：结构化快照。",
            "",
            "Graphviz 渲染：`{}`".format(json.dumps(render_results, ensure_ascii=False)),
            "",
            "该快照只描述采集时 ROS Master 中已注册的接口；未启动的 legacy 回归入口不在图中。",
        ]
    )
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--settle-seconds", type=float, default=0.0)
    parser.add_argument("--fail-on-audit", action="store_true")
    args = parser.parse_args()

    if args.settle_seconds > 0.0:
        time.sleep(args.settle_seconds)

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    master = rosgraph.Master("/uav_vision_eval/ros_topology_snapshot")
    publishers_raw, subscribers_raw, services_raw = master.getSystemState()
    publishers = state_map(publishers_raw)
    subscribers = state_map(subscribers_raw)
    services = state_map(services_raw)
    topic_types = dict(master.getTopicTypes())
    topics = sorted(set(publishers) | set(subscribers))
    nodes = sorted(
        {
            node
            for peers in list(publishers.values())
            + list(subscribers.values())
            + list(services.values())
            for node in peers
        }
    )

    cli_results = {}
    for name, command in (
        ("ros_nodes", ["rosnode", "list"]),
        ("ros_topics_verbose", ["rostopic", "list", "-v"]),
    ):
        code, output = run_command(command)
        cli_results[name] = {"command": command, "exit_code": code}
        (output_dir / (name + ".txt")).write_text(
            output.rstrip("\n") + "\n", encoding="utf-8"
        )

    package_paths = {}
    for package in ("uav_vision", "uav_mission"):
        code, output = run_command(["rospack", "find", package])
        package_paths[package] = output.strip() if code == 0 else "ERROR: " + output.strip()

    publisher_edges = [
        ("publish", node, topic, topic_types.get(topic, ""))
        for topic in topics
        for node in publishers.get(topic, [])
    ]
    subscriber_edges = [
        ("subscribe", topic, node, topic_types.get(topic, ""))
        for topic in topics
        for node in subscribers.get(topic, [])
    ]
    snapshot = {
        "captured_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "master_uri": os.environ.get("ROS_MASTER_URI", ""),
        "counts": {
            "nodes": len(nodes),
            "topics": len(topics),
            "publisher_edges": len(publisher_edges),
            "subscriber_edges": len(subscriber_edges),
            "services": len(services),
        },
        "nodes": nodes,
        "topic_types": topic_types,
        "publishers": publishers,
        "subscribers": subscribers,
        "services": services,
        "package_paths": package_paths,
        "cli_results": cli_results,
    }
    audit = build_audit(publishers, subscribers, services, set(nodes), package_paths)
    snapshot["interface_audit"] = audit
    (output_dir / "ros_system_state.json").write_text(
        json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "interface_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    write_csv(output_dir / "nodes.csv", ["node"], ((node,) for node in nodes))
    write_csv(
        output_dir / "topics.csv",
        ["topic", "type", "publishers", "subscribers"],
        (
            (
                topic,
                topic_types.get(topic, ""),
                ";".join(publishers.get(topic, [])),
                ";".join(subscribers.get(topic, [])),
            )
            for topic in topics
        ),
    )
    write_csv(
        output_dir / "edges.csv",
        ["kind", "source", "target", "topic_type"],
        publisher_edges + subscriber_edges,
    )
    write_csv(
        output_dir / "services.csv",
        ["service", "providers"],
        ((service, ";".join(providers)) for service, providers in sorted(services.items())),
    )
    write_csv(
        output_dir / "one_sided_topics.csv",
        ["topic", "kind", "publishers", "subscribers"],
        (
            (
                topic,
                "publisher_only" if publishers.get(topic) else "subscriber_only",
                ";".join(publishers.get(topic, [])),
                ";".join(subscribers.get(topic, [])),
            )
            for topic in topics
            if not publishers.get(topic) or not subscribers.get(topic)
        ),
    )

    render_results = {
        "rqt_nodes_only": write_rqt_nodes_only_graphs(
            output_dir, publishers, subscribers, topics
        ),
    }
    (output_dir / "REPORT.md").write_text(
        make_report(snapshot, audit, render_results), encoding="utf-8"
    )

    print(str(output_dir))
    print("nodes={} topics={} audit={}".format(
        len(nodes), len(topics), "PASS" if audit["passed"] else "FAIL"
    ))
    if args.fail_on_audit and not audit["passed"]:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
