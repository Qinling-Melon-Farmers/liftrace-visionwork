#!/usr/bin/env python3
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from coverage_policy import (
    CandidateData,
    CandidateQueue,
    GoalRetryPolicy,
    candidate_rank,
    candidate_valid,
    generate_serpentine,
    point_inside_safe_bounds,
    resolve_safe_waypoint,
    select_serpentine_entry,
)


class CoveragePolicyTest(unittest.TestCase):
    def test_serpentine_respects_margin_spacing_and_direction(self):
        points = generate_serpentine(
            -5.0, 5.0, -5.0, 5.0, 0.5, 1.0, 2.0)
        self.assertEqual(len(points), 20)
        self.assertEqual((points[0].x, points[0].y), (-4.5, -4.5))
        self.assertEqual((points[1].x, points[1].y), (4.5, -4.5))
        self.assertEqual((points[2].x, points[2].y), (4.5, -3.5))
        self.assertEqual((points[3].x, points[3].y), (-4.5, -3.5))
        self.assertTrue(all(point.z == 2.0 for point in points))
        self.assertTrue(all(point_inside_safe_bounds(
            (point.x, point.y, point.z), (-5.0, 5.0, -5.0, 5.0), 0.5)
            for point in points))

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            generate_serpentine(0, 0, -1, 1, 0.5, 1.0, 2.0)
        with self.assertRaises(ValueError):
            generate_serpentine(-1, 1, -1, 1, 1.0, 1.0, 2.0)

    def test_serpentine_entry_uses_nearest_corner_without_losing_points(self):
        points = generate_serpentine(
            -5.0, 5.0, -5.0, 5.0, 0.5, 1.0, 2.0)
        selected = select_serpentine_entry(points, 4.0, 4.0)
        self.assertEqual((selected[0].x, selected[0].y), (4.5, 4.5))
        self.assertEqual(len(selected), len(points))
        self.assertEqual(
            {(point.x, point.y) for point in selected},
            {(point.x, point.y) for point in points})
        self.assertEqual(
            [point.index for point in selected], list(range(len(points))))

    def test_target_area_route_has_twelve_non_target_coordinate_points(self):
        points = generate_serpentine(
            -2.5, 1.5, -1.5, 4.5, 0.5, 1.0, 2.0)
        self.assertEqual(len(points), 12)
        self.assertEqual((points[0].x, points[0].y), (-2.0, -1.0))
        self.assertEqual((points[-1].x, points[-1].y), (-2.0, 4.0))

    def test_obstacle_endpoint_is_adjusted_to_nearest_clear_point(self):
        occupied = [(4.5, -4.5, 2.0), (4.25, -4.5, 2.0)]
        resolved = resolve_safe_waypoint(
            (4.5, -4.5, 2.0), occupied,
            (-5.0, 5.0, -5.0, 5.0), 0.5,
            clearance=0.30, search_step=0.25, max_adjustment=1.0)
        self.assertIsNotNone(resolved)
        self.assertNotEqual(resolved, (4.5, -4.5, 2.0))
        self.assertTrue(point_inside_safe_bounds(
            resolved, (-5.0, 5.0, -5.0, 5.0), 0.5))

    def test_tracking_reserve_keeps_commanded_corner_inward(self):
        resolved = resolve_safe_waypoint(
            (-4.5, -4.5, 2.0), [],
            (-5.0, 5.0, -5.0, 5.0), 1.25,
            max_adjustment=1.5)
        self.assertIsNotNone(resolved)
        self.assertTrue(point_inside_safe_bounds(
            resolved, (-5.0, 5.0, -5.0, 5.0), 1.25))

    def test_candidate_filter_order_and_terminal_dedup(self):
        base = dict(
            first_seen=8.0, last_seen=10.0, state=2, map_valid=True,
            map_frame="camera_init", association_valid=True,
            reject_reason="", x=1.0, y=2.0)
        tent = CandidateData(1, "tent", 0.95, **base)
        tank = CandidateData(2, "tank", 0.60, **base)
        old = CandidateData(3, "bridge", 0.99,
                            **dict(base, last_seen=9.0))
        rejected = CandidateData(4, "panzer", 0.99,
                                 **dict(base, reject_reason="no_association"))
        self.assertTrue(candidate_valid(tent, 10.4))
        self.assertFalse(candidate_valid(old, 10.0))
        self.assertFalse(candidate_valid(rejected, 10.0))
        self.assertLess(candidate_rank(tank), candidate_rank(tent))

        queue = CandidateQueue()
        queue.update([tent, tank, old, rejected], 10.0)
        self.assertEqual(queue.pop().target_id, 2)
        queue.mark_terminal(2)
        queue.update([tank], 10.0)
        self.assertEqual(queue.pop().target_id, 1)
        queue.mark_terminal(1)
        queue.update([tent], 10.0)
        self.assertIsNone(queue.pop())

        queue = CandidateQueue()
        queue.update([tent, tank], 10.0)
        queue.retain([tent.target_id])
        self.assertEqual(queue.pop().target_id, tent.target_id)
        self.assertIsNone(queue.pop())

    def test_all_classes_are_popped_in_rule_weight_order(self):
        base = dict(
            confidence=0.9, first_seen=1.0, last_seen=10.0, state=2,
            map_valid=True, map_frame="camera_init", association_valid=True,
            reject_reason="", x=0.0, y=0.0)
        classes = ("tent", "pillbox", "bridge", "panzer", "tank")
        queue = CandidateQueue()
        queue.update([
            CandidateData(index, class_name, **base)
            for index, class_name in enumerate(classes)
        ], 10.0)
        self.assertEqual(
            [queue.pop().class_name for _ in classes],
            ["tank", "panzer", "bridge", "pillbox", "tent"])

    def test_goal_retry_and_timeout(self):
        policy = GoalRetryPolicy(
            retry_interval=5.0, unreachable_timeout=20.0, max_retries=2)
        policy.start(100.0)
        self.assertEqual(policy.decision(104.9), "wait")
        self.assertEqual(policy.decision(105.0), "retry")
        self.assertEqual(policy.decision(110.0), "retry")
        self.assertEqual(policy.decision(115.0), "wait")
        policy.note_progress(119.0)
        self.assertEqual(policy.decision(120.0), "wait")
        self.assertEqual(policy.decision(138.9), "wait")
        self.assertEqual(policy.decision(139.0), "timeout")

        policy.start(100.0)
        self.assertEqual(policy.decision(120.0), "timeout")

        policy.start(0.0)
        policy.note_progress(4.9)
        self.assertEqual(policy.decision(5.0), "wait")
        self.assertEqual(policy.decision(9.9), "retry")

    def test_coverage_runtime_has_no_target_coordinate_inputs(self):
        package_dir = Path(__file__).resolve().parents[1]
        runtime_text = "\n".join([
            (package_dir / "config" / "coverage_toudi4.yaml").read_text(),
            (package_dir / "scripts" /
             "coverage_search_manager.py").read_text(),
        ]).lower()
        forbidden = (
            "goal_list", "target_coordinates", "target_positions",
            "bridge_position", "panzer_position", "pillbox_position",
            "tent_position", "tank_position",
        )
        for token in forbidden:
            self.assertNotIn(token, runtime_text)

    def test_coverage_safety_overrides_preserve_planner_defaults(self):
        package_dir = Path(__file__).resolve().parents[1]
        workspace_src = package_dir.parent
        planner_launch = workspace_src / "Fast-Planner" / "fast_planner" / (
            "plan_manage/launch/patrol_planner_px4_sim.launch")
        planner_root = ET.parse(str(planner_launch)).getroot()
        defaults = {arg.attrib["name"]: arg.attrib.get("default")
                    for arg in planner_root.findall("arg")}
        self.assertEqual(defaults["max_vel"], "3.0")
        self.assertEqual(defaults["max_acc"], "2.0")
        self.assertEqual(defaults["obstacles_inflation"], "0.15")
        self.assertEqual(defaults["clearance_threshold"], "0.20")

        coverage_root = ET.parse(str(
            package_dir / "launch" / "coverage_navigation.launch")).getroot()
        overrides = {arg.attrib["name"]: arg.attrib.get("value")
                     for include in coverage_root.findall("include")
                     for arg in include.findall("arg")}
        self.assertEqual(overrides["planner_max_vel"], "1.5")
        self.assertEqual(overrides["planner_max_acc"], "1.0")
        self.assertEqual(overrides["planner_obstacles_inflation"], "0.30")
        self.assertEqual(overrides["planner_clearance_threshold"], "0.35")
        self.assertEqual(overrides["px4_max_distance"], "0.2")

        r6_root = ET.parse(str(
            package_dir / "launch" / "coverage_r6.launch")).getroot()
        manager = next(node for node in r6_root.findall("node")
                       if node.attrib.get("name") ==
                       "coverage_search_manager")
        manager_params = {param.attrib["name"]: param.attrib.get("value")
                          for param in manager.findall("param")}
        self.assertEqual(manager_params["navigation_only"], "false")
        self.assertEqual(manager_params["execute_candidates"], "true")
        self.assertEqual(manager_params["collect_before_delivery"], "true")
        self.assertEqual(manager_params["final_land"], "true")

    def test_external_land_enters_legacy_landing_point_mode(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir.parent / "patrol_control" / "src" /
                  "patrol_control.cpp").read_text(encoding="utf-8")
        land_case = source.split(
            "case patrol_control::MissionCommand::LAND:", 1)[1].split(
                "default:", 1)[0]
        self.assertIn("Point_mode = Land_point;", land_case)
        self.assertIn("Drone_mode = Land;", land_case)

    def test_external_align_preserves_manager_candidate_map_point(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir.parent / "patrol_control" / "src" /
                  "patrol_control.cpp").read_text(encoding="utf-8")
        detect_init = source.split(
            "bool LLController::WayPointDetectDone()", 1)[1].split(
                "// ROS_INFO(\"dis_tooooo", 1)[0]
        self.assertIn("if (external_mission_mode_)", detect_init)
        self.assertIn("waypoint_temp.pose = waypoint_mark_point.pose;",
                      detect_init)
        external_branch = detect_init.split(
            "if (external_mission_mode_)", 1)[1].split("} else {", 1)[0]
        self.assertNotIn("waypoint_list[waypoint_next]", external_branch)

    def test_release_attribution_uses_circle_and_candidate_neighborhood(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir / "scripts" /
                  "coverage_search_manager.py").read_text(encoding="utf-8")
        matcher = source.split(
            "def _release_matches_candidate", 1)[1].split(
                "def _finish_candidate", 1)[0]
        self.assertIn('target_class != "circle"', matcher)
        self.assertIn("release_attribution_distance", matcher)
        self.assertNotIn("selected_target.id", matcher)

    def test_sim_visual_tf_uses_mavros_mission_pose(self):
        package_dir = Path(__file__).resolve().parents[1]
        root = ET.parse(str(
            package_dir / "launch" /
            "toudi3_visual_delivery_guarded.launch")).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("arg")}
        self.assertEqual(
            args["camera_pose_topic"], "/mavros/local_position/pose")
        self.assertEqual(args["camera_parent_frame"], "vision_body")
        pose_tf = next(node for node in root.findall("node")
                       if node.attrib.get("name") == "vision_pose_tf")
        params = {param.attrib["name"]: param.attrib.get("value")
                  for param in pose_tf.findall("param")}
        self.assertEqual(params["parent_frame"], "$(arg map_frame)")
        self.assertEqual(params["child_frame"],
                         "$(arg camera_parent_frame)")


if __name__ == "__main__":
    unittest.main()
