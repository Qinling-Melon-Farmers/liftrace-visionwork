#!/usr/bin/env python3
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

from uav_mission.contact_policy import (
    contact_episode_transition,
    relevant_contact_pairs,
)
from uav_mission.navigation_ab_policy import compare_navigation_ab

from coverage_policy import (
    CandidateData,
    CandidateQueue,
    CaptureEvidence,
    GoalRetryPolicy,
    SelectorCandidate,
    accumulate_run_facts,
    adapter_candidate_accepting,
    build_command_event,
    candidate_rank,
    candidate_valid,
    capture_evidence_matches,
    expected_delivery_classes,
    generate_serpentine,
    interrupt_eligible,
    point_inside_safe_bounds,
    profile_allowed_classes,
    profile_standard_classes,
    resolve_safe_waypoint,
    select_current_candidate,
    select_serpentine_entry,
)
from uav_mission.random_field_policy import (
    Footprint,
    RED_CROSS_FOOTPRINT_RADIUS,
    STANDARD_FOOTPRINT_RADIUS,
    footprint_clear,
    footprint_inside_bounds,
    plan_footprint_layout,
    validate_seed,
    validate_standard_classes,
)
from uav_mission.planner_anchor_policy import (
    resolve_model_sdf,
    validate_anchor_profile,
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

    def test_class_profiles_define_r2026_without_tank(self):
        self.assertEqual(
            profile_standard_classes("full"),
            ("tent", "pillbox", "bridge", "panzer", "tank"))
        self.assertEqual(
            profile_standard_classes("r2026"),
            ("tent", "pillbox", "bridge", "panzer"))
        with self.assertRaises(ValueError):
            profile_standard_classes("unknown")
        # r2026 队列允许集合 = 四类标准靶 + 随机红十字；tank 始终排除。
        self.assertEqual(
            profile_allowed_classes("r2026"),
            ("tent", "pillbox", "bridge", "panzer", "red_cross"))
        self.assertNotIn("tank", profile_allowed_classes("r2026"))
        self.assertIn("tank", profile_allowed_classes("full"))

    def test_selector_requires_current_streak_and_profile(self):
        base = dict(
            confidence=0.9, geometry_confidence=0.8, map_quality=0.7,
            last_seen=10.0, state=2,
            consecutive_observe_count=3, map_valid=True,
            map_frame="camera_init", association_valid=True,
            reject_reason="", x=0.0, y=1.0, z=0.0)
        panzer = SelectorCandidate(1, "panzer", **base)
        cross = SelectorCandidate(2, "red_cross", **base)
        tank = SelectorCandidate(3, "tank", **base)
        stale = SelectorCandidate(4, "bridge",
                                  **dict(base, last_seen=9.0))
        broken_streak = SelectorCandidate(
            5, "bridge", **dict(base, consecutive_observe_count=0))
        selected = select_current_candidate(
            [panzer, cross, tank, stale, broken_streak], 10.2,
            allowed_classes=profile_allowed_classes("r2026"),
            bounds=(-3.0, 3.0, -1.0, 7.0))
        self.assertEqual(selected.target_id, cross.target_id)

    def test_command_event_records_actual_search_to_approach(self):
        event = build_command_event(
            7, 12.5, 1, "SEARCH", "APPROACH", 42, "panzer")
        self.assertEqual(event["sequence"], 7)
        self.assertEqual(event["from_state"], "SEARCH")
        self.assertEqual(event["to_state"], "APPROACH")
        self.assertEqual(event["target_id"], 42)

    def test_selector_rejects_nonfinite_out_of_bounds_and_high_z(self):
        base = dict(
            confidence=0.9, geometry_confidence=0.8, map_quality=0.7,
            last_seen=10.0, state=2,
            consecutive_observe_count=3, map_valid=True,
            map_frame="camera_init", association_valid=True,
            reject_reason="", x=0.0, y=1.0, z=0.0)
        candidates = [
            SelectorCandidate(1, "panzer", **dict(base, x=float("nan"))),
            SelectorCandidate(2, "bridge", **dict(base, y=9.0)),
            SelectorCandidate(3, "tent", **dict(base, z=4.1)),
            SelectorCandidate(4, "pillbox", **dict(base, state=3)),
        ]
        self.assertIsNone(select_current_candidate(
            candidates, 10.1,
            allowed_classes=profile_allowed_classes("r2026"),
            bounds=(-3.0, 3.0, -1.0, 7.0), max_z=4.0))

    def test_selector_rejects_future_stamp_and_nonfinite_quality(self):
        base = dict(
            confidence=0.9, geometry_confidence=0.8, map_quality=0.7,
            last_seen=10.0, state=2, consecutive_observe_count=3,
            map_valid=True, map_frame="camera_init", association_valid=True,
            reject_reason="", x=0.0, y=1.0, z=0.0)
        candidates = [
            SelectorCandidate(1, "panzer", **dict(base, last_seen=10.2)),
            SelectorCandidate(
                2, "bridge", **dict(base, confidence=float("nan"))),
            SelectorCandidate(
                3, "pillbox", **dict(base, map_quality=float("inf"))),
            SelectorCandidate(
                4, "tent", **dict(base,
                                  geometry_confidence=float("nan"))),
        ]
        self.assertIsNone(select_current_candidate(
            candidates, 10.1,
            allowed_classes=profile_allowed_classes("r2026"),
            bounds=(-3.0, 3.0, -1.0, 7.0)))

    def test_adapter_candidate_gate_is_closed_for_out_of_order_startup(self):
        profile = "r2026"
        waiting = {
            "status": "RUNNING", "state": "WAIT_TAKEOFF",
            "class_profile": profile, "field_ready": True,
            "anchor_ready": False, "operational_ready": False,
            "candidate_accepting": False,
        }
        search_unready = dict(
            waiting, state="SEARCH", anchor_ready=True,
            candidate_accepting=True)
        search_ready = dict(
            search_unready, operational_ready=True)
        approach = dict(
            search_ready, state="APPROACH", candidate_accepting=False)
        self.assertFalse(adapter_candidate_accepting(None, profile))
        self.assertFalse(adapter_candidate_accepting(waiting, profile))
        self.assertFalse(adapter_candidate_accepting(search_unready, profile))
        self.assertTrue(adapter_candidate_accepting(search_ready, profile))
        self.assertFalse(adapter_candidate_accepting(approach, profile))
        self.assertFalse(adapter_candidate_accepting(search_ready, "full"))

    def test_random_field_policy_uses_full_footprints(self):
        self.assertGreater(STANDARD_FOOTPRINT_RADIUS, 0.70)
        self.assertGreater(RED_CROSS_FOOTPRINT_RADIUS, 0.24)
        occupied = [Footprint("first", 0.0, 0.0,
                              STANDARD_FOOTPRINT_RADIUS)]
        self.assertFalse(footprint_clear(
            1.2, 0.0, STANDARD_FOOTPRINT_RADIUS, occupied, gap=0.15))
        self.assertTrue(footprint_clear(
            1.7, 0.0, STANDARD_FOOTPRINT_RADIUS, occupied, gap=0.15))
        self.assertFalse(footprint_inside_bounds(
            -1.5, 0.0, STANDARD_FOOTPRINT_RADIUS,
            (-2.0, 2.0, -2.0, 2.0), margin=0.1))
        self.assertTrue(footprint_inside_bounds(
            0.0, 0.0, STANDARD_FOOTPRINT_RADIUS,
            (-2.0, 2.0, -2.0, 2.0), margin=0.1))
        with self.assertRaises(ValueError):
            validate_seed(0)
        with self.assertRaises(ValueError):
            validate_standard_classes(
                ("tent", "tent"), ("tent", "bridge"))

    def test_random_field_layout_restarts_after_greedy_dead_end(self):
        class SequenceRng:
            def __init__(self, values):
                self._values = iter(values)

            def uniform(self, _lower, _upper):
                return next(self._values)

        # First layout puts A in the middle; both attempts for B collide.
        # The second full-layout attempt moves A aside and fits both targets.
        rng = SequenceRng([
            2.0, 1.0,
            2.5, 1.0,
            1.5, 1.0,
            0.5, 0.5,
            3.5, 1.5,
        ])
        layout = plan_footprint_layout(
            rng, [("a", 0.4), ("b", 0.4)], [],
            (0.0, 4.0, 0.0, 2.0), (0.0, 4.0, 0.0, 2.0),
            attempts_per_target=2, layout_attempts=2, pair_gap=0.1)
        self.assertEqual(layout, [
            ("a", 0.5, 0.5), ("b", 3.5, 1.5)])

    def test_random_field_config_covers_boxes_and_compound_wall(self):
        package_dir = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((
            package_dir / "config" / "coverage_toudi3_random.yaml"
        ).read_text(encoding="utf-8"))
        box_radius = (0.6 ** 2 + 0.4 ** 2) ** 0.5
        self.assertTrue(all(
            float(config["static_model_radii"][name]) >= box_radius
            for name in ("Big box 4", "big_box3", "big_box3_0",
                         "Big box 4_0")))
        walls = [item for item in config["static_exclusions"]
                 if item["name"].startswith("toudi2_wall15_")]
        self.assertEqual(len(walls), 4)
        occupied = [Footprint(
            item["name"], float(item["world_x"]),
            float(item["world_y"]), float(item["radius"]))
            for item in walls]
        self.assertFalse(footprint_clear(
            -2.0, 3.70, STANDARD_FOOTPRINT_RADIUS, occupied, gap=0.15))

    def test_contact_policy_filters_ground_and_debounces_episodes(self):
        pairs = relevant_contact_pairs([
            ("iris::guard", "ground_plane::link::collision"),
            ("iris::guard", "toudi2::Wall_15::collision"),
            ("toudi2::Wall_15::collision", "iris::guard"),
        ], ["ground_plane"])
        self.assertEqual(pairs, [(
            "iris::guard", "toudi2::Wall_15::collision")])
        active, increment = contact_episode_transition(False, pairs)
        self.assertTrue(active)
        self.assertEqual(increment, 1)
        active, increment = contact_episode_transition(active, pairs)
        self.assertEqual(increment, 0)
        active, increment = contact_episode_transition(active, [])
        self.assertFalse(active)
        self.assertEqual(increment, 0)

    def test_random_field_spawner_resolves_sibling_policy_when_installed(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir / "scripts" /
                  "random_field_spawner.py").read_text(encoding="utf-8")
        path_setup = source.index("sys.path.insert(0, SCRIPT_DIR)")
        policy_import = source.index(
            "from coverage_policy import profile_standard_classes")
        self.assertLess(path_setup, policy_import)

    def test_planner_anchor_profiles_are_explicit_and_baseline_empty(self):
        package_dir = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((
            package_dir / "config" / "planner_anchor_profiles.yaml"
        ).read_text())
        baseline = validate_anchor_profile("baseline", config["profiles"])
        candidate = validate_anchor_profile(
            "a68925d", config["profiles"])
        self.assertEqual(baseline["anchors"], [])
        self.assertFalse(baseline["external_feature_dependency"])
        self.assertEqual(len(candidate["anchors"]), 9)
        self.assertTrue(candidate["external_feature_dependency"])
        self.assertIn("a68925d", candidate["source_revision"])
        with self.assertRaises(ValueError):
            validate_anchor_profile("unknown", config["profiles"])

    def test_anchor_model_resolver_honors_model_config_filename(self):
        with tempfile.TemporaryDirectory() as root:
            model_dir = Path(root) / "radio_tower"
            model_dir.mkdir()
            (model_dir / "model.config").write_text(
                "<model><sdf version='1.6'>radio_tower.sdf</sdf></model>")
            expected = model_dir / "radio_tower.sdf"
            expected.write_text("<sdf version='1.6'/>")
            self.assertEqual(
                resolve_model_sdf("radio_tower", [root]), str(expected))

    def test_formal_random_launch_has_one_manager_and_profile_remaps(self):
        package_dir = Path(__file__).resolve().parents[1]
        root = ET.parse(str(
            package_dir / "launch" /
            "navigation_search_delivery_random_field.launch")).getroot()
        nodes = root.findall("node")
        node_types = [node.attrib.get("type") for node in nodes]
        self.assertEqual(node_types.count("target_search_manager_py.py"), 1)
        self.assertEqual(
            node_types.count("navigation_visual_delivery_adapter.py"), 1)
        self.assertNotIn("coverage_search_manager.py", node_types)
        manager = next(node for node in nodes
                       if node.attrib.get("type") ==
                       "target_search_manager_py.py")
        adapter = next(node for node in nodes
                       if node.attrib.get("type") ==
                       "navigation_visual_delivery_adapter.py")
        for node in (manager, adapter):
            remaps = {(item.attrib.get("from"), item.attrib.get("to"))
                      for item in node.findall("remap")}
            self.assertIn(("/uav_vision/selected_target",
                           "/mission/profile_selected_target"), remaps)
        goal_remaps = {(item.attrib.get("from"), item.attrib.get("to"))
                       for item in manager.findall("remap")}
        self.assertIn(("/fastplanner/goal", "/navigation/goal_raw"),
                      goal_remaps)
        includes = root.findall("include")
        guarded = next(item for item in includes
                       if "toudi3_visual_delivery_guarded.launch" in
                       item.attrib.get("file", ""))
        include_args = {item.attrib.get("name"): item.attrib.get("value")
                        for item in guarded.findall("arg")}
        self.assertEqual(include_args.get("class_profile"),
                         "$(arg class_profile)")
        launch_args = {item.attrib.get("name"): item.attrib.get("default")
                       for item in root.findall("arg")}
        self.assertEqual(launch_args.get("nav_feature_profile"), "baseline")
        self.assertIn("px4_model_root",
                      launch_args.get("random_model_roots", ""))
        selector = next(node for node in nodes
                        if node.attrib.get("type") ==
                        "profile_candidate_selector.py")
        selector_params = {item.attrib.get("name"): item.attrib.get("value")
                           for item in selector.findall("param")}
        self.assertEqual(
            selector_params.get("require_adapter_candidate_accepting"),
            "true")
        self.assertIn("gazebo_contact_monitor.py", node_types)
        adapter_params = {item.attrib.get("name"): item.attrib.get("value")
                          for item in adapter.findall("param")}
        self.assertEqual(adapter_params.get("require_contact_monitor"),
                         "true")

    @staticmethod
    def _ab_metrics(nav_profile, arrivals, failures, drift):
        return {
            "status": "PASS",
            "assets_ready": True,
            "field_seed": 11,
            "class_profile": "r2026",
            "nav_feature_profile": nav_profile,
            "world": "/same/world",
            "target_model_path": "/same/model",
            "truth_targets": [{"class": "tent", "x": 1.0, "y": 2.0}],
            "route_spec": [[0.0, 0.0, 2.2], [1.0, 0.0, 2.2]],
            "arrival_wall_times": arrivals,
            "planning_failure_count": failures,
            "height_drift_rms": drift,
            "pose_max_gap_wall": 0.1,
            "max_altitude": 2.5,
            "actual_collision_count": 0,
            "boundary_violation_count": 0,
            "invalid_pose_count": 0,
            "unexpected_goal_count": 0,
            "planner_goal_publishers": [
                "/navigation_visual_delivery_adapter"],
            "raw_goal_publishers": ["/target_search_manager_py"],
        }

    def test_navigation_ab_accepts_safe_same_identity_improvement(self):
        baseline = self._ab_metrics("baseline", [30.0, 70.0], 2, 0.20)
        candidate = self._ab_metrics("a68925d", [31.0, 75.0], 1, 0.20)
        report = compare_navigation_ab(baseline, candidate)
        self.assertEqual(report["status"], "PASS")
        self.assertTrue(report["promote_candidate"])
        self.assertTrue(report["checks"][
            "wall_time_regression_le_10pct"])

    def test_navigation_ab_rejects_regression_missing_metric_and_mismatch(self):
        baseline = self._ab_metrics("baseline", [30.0, 60.0], 0, 0.20)
        candidate = self._ab_metrics("a68925d", [35.0, 70.0], 0, None)
        candidate["field_seed"] = 12
        report = compare_navigation_ab(baseline, candidate)
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["promote_candidate"])
        self.assertFalse(report["checks"][
            "same_seed_world_truth_model_route"])
        self.assertFalse(report["checks"][
            "wall_time_regression_le_10pct"])
        self.assertFalse(report["checks"][
            "planning_failure_or_height_improved"])

    def test_preflight_and_ab_launches_disable_delivery_and_hard_gate(self):
        package_dir = Path(__file__).resolve().parents[1]
        formal = ET.parse(str(
            package_dir / "launch" /
            "navigation_search_delivery_random_field.launch")).getroot()
        formal_args = {item.attrib.get("name"): item.attrib.get("default")
                       for item in formal.findall("arg")}
        self.assertEqual(formal_args.get("enable_candidate_selector"), "true")
        self.assertEqual(formal_args.get("start_hard_gate"), "true")
        selector = next(node for node in formal.findall("node")
                        if node.attrib.get("type") ==
                        "profile_candidate_selector.py")
        hard_gate = next(node for node in formal.findall("node")
                         if node.attrib.get("type") ==
                         "navigation_random_field_assertion.py")
        self.assertEqual(selector.attrib.get("if"),
                         "$(arg enable_candidate_selector)")
        self.assertEqual(hard_gate.attrib.get("if"),
                         "$(arg start_hard_gate)")

        expectations = (
            ("navigation_random_field_preflight.launch",
             "navigation_random_field_preflight.py", "30.0"),
            ("navigation_random_field_ab.launch",
             "navigation_ab_observer.py", "90.0"),
        )
        for launch_name, assertion_type, duration in expectations:
            root = ET.parse(str(package_dir / "launch" / launch_name)).getroot()
            args = {item.attrib.get("name"): item.attrib.get("default")
                    for item in root.findall("arg")}
            self.assertEqual(args.get("duration_sec"), duration)
            include = root.find("include")
            include_args = {
                item.attrib.get("name"): item.attrib.get("value")
                for item in include.findall("arg")}
            self.assertEqual(include_args.get("enable_candidate_selector"),
                             "false")
            self.assertEqual(include_args.get("start_hard_gate"), "false")
            self.assertEqual(include_args.get("enable_debug_image"), "false")
            node = next(item for item in root.findall("node")
                        if item.attrib.get("type") == assertion_type)
            self.assertEqual(node.attrib.get("required"), "true")

    def test_preflight_checks_camera_tf_geometry_and_topic_ownership(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir / "scripts" /
                  "navigation_random_field_preflight.py").read_text(
                      encoding="utf-8")
        self.assertIn("CameraInfo", source)
        self.assertIn("lookup_transform", source)
        self.assertIn('"truth_footprints"', source)
        self.assertIn('"truth_models_match_gazebo"', source)
        self.assertIn('"adapter_only_planner_goal"', source)
        self.assertIn('"navigation_manager_only_raw_goal"', source)
        self.assertIn('"inside_field_bounds"', source)
        self.assertNotIn("route_progress_observed", source)
        self.assertIn("time.sleep(0.1)", source)
        observer = (package_dir / "scripts" /
                    "navigation_ab_observer.py").read_text(encoding="utf-8")
        self.assertIn('"boundary_violation_count"', observer)
        self.assertIn('"unexpected_goal_count"', observer)
        self.assertIn("time.sleep(0.1)", observer)

    def test_navigation_ab_runner_is_ordered_and_never_promotes_default(self):
        package_dir = Path(__file__).resolve().parents[4]
        source = (package_dir / "top_level_scripts" /
                  "run_navigation_ab.sh").read_text(encoding="utf-8")
        self.assertIn("sim_run.sh", source)
        self.assertLess(source.index("run_preflight baseline"),
                        source.index("run_preflight a68925d"))
        self.assertLess(source.index("run_sample baseline"),
                        source.index("run_sample a68925d"))
        self.assertIn("navigation_ab_compare.py", source)
        self.assertIn('python3 "${AB_COMPARE}"', source)
        self.assertNotIn("rosrun", source)
        self.assertNotIn("git commit", source)
        self.assertNotIn("planner_anchor_profiles.yaml", source)

    def test_sim_vehicle_declares_bumper_contact_sensor(self):
        package_dir = Path(__file__).resolve().parents[2]
        model_path = (
            package_dir / "patrol_control" / "models" /
            "iris_mid360_downward_camera" / "model.sdf")
        root = ET.parse(str(model_path)).getroot()
        sensors = root.findall(".//sensor")
        contact = next(sensor for sensor in sensors
                       if sensor.attrib.get("name") ==
                       "competition_contact_sensor")
        self.assertEqual(contact.attrib.get("type"), "contact")
        plugin = contact.find("plugin")
        self.assertEqual(plugin.attrib.get("filename"),
                         "libgazebo_ros_bumper.so")
        self.assertEqual(plugin.findtext("bumperTopicName"),
                         "/mission/uav_contacts_raw")

        mission_manifest = ET.parse(str(
            package_dir / "uav_mission" / "package.xml")).getroot()
        dependencies = [element.text for element in mission_manifest
                        if element.tag in ("depend", "exec_depend")]
        self.assertIn("gazebo_msgs", dependencies)
        self.assertIn("gazebo_plugins", dependencies)

    def test_formal_gate_uses_contact_fact_and_audits_raw_selection(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir / "scripts" /
                  "navigation_random_field_assertion.py").read_text(
                      encoding="utf-8")
        self.assertIn('"/mission/gazebo_contact_status"', source)
        self.assertIn('"zero_actual_collisions"', source)
        self.assertIn('"/uav_vision/selected_target"', source)
        self.assertIn('"tank" not in self._raw_selected_classes', source)
        self.assertNotIn(
            '"zero_collisions": manager.get("collision_count")', source)

    def test_adapter_fails_closed_when_upstream_approach_stalls(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir / "scripts" /
                  "navigation_visual_delivery_adapter.py").read_text(
                      encoding="utf-8")
        approach_block = source.split(
            'elif self._state == "APPROACH":', 1)[1].split(
                'elif self._state == "CAPTURE":', 1)[0]
        self.assertIn(
            "upstream_manager_approach_unreachable_no_result_interface",
            approach_block)
        self.assertNotIn("_resume_search", approach_block)
        self.assertIn('"candidate_accepting": self._candidate_accepting()',
                      source)
        self.assertIn('"mission_elapsed_wall":', source)

    def test_guarded_launch_preserves_full_profile_by_default(self):
        package_dir = Path(__file__).resolve().parents[1]
        root = ET.parse(str(
            package_dir / "launch" /
            "toudi3_visual_delivery_guarded.launch")).getroot()
        args = {item.attrib.get("name"): item.attrib.get("default")
                for item in root.findall("arg")}
        self.assertEqual(args.get("class_profile"), "full")
        wrapped = next(item for item in root.findall("include")
                       if "toudi3_full_competition_sim_new_vision.launch" in
                       item.attrib.get("file", ""))
        wrapped_args = {item.attrib.get("name"): item.attrib.get("value")
                        for item in wrapped.findall("arg")}
        self.assertEqual(wrapped_args.get("class_profile"),
                         "$(arg class_profile)")

    def test_candidate_valid_respects_allowed_classes(self):
        base = dict(
            first_seen=8.0, last_seen=10.0, state=2, map_valid=True,
            map_frame="camera_init", association_valid=True,
            reject_reason="", x=1.0, y=2.0)
        tank = CandidateData(2, "tank", 0.60, **base)
        cross = CandidateData(5, "red_cross", 0.90, **base)
        allowed = profile_allowed_classes("r2026")
        self.assertTrue(candidate_valid(tank, 10.4))
        self.assertFalse(
            candidate_valid(tank, 10.4, allowed_classes=allowed))
        self.assertTrue(
            candidate_valid(cross, 10.4, allowed_classes=allowed))

    def test_queue_update_drops_disallowed_semantic_candidates(self):
        base = dict(
            confidence=0.9, first_seen=1.0, last_seen=10.0, state=2,
            map_valid=True, map_frame="camera_init", association_valid=True,
            reject_reason="", x=0.0, y=0.0)
        tank = CandidateData(2, "tank", **base)
        panzer = CandidateData(3, "panzer", **base)
        queue = CandidateQueue()
        queue.update([tank, panzer], 10.0,
                     allowed_classes=profile_allowed_classes("r2026"))
        self.assertEqual(queue.pop().class_name, "panzer")
        self.assertIsNone(queue.pop())

    def test_expected_delivery_classes_follows_profile_discovery(self):
        base = dict(
            confidence=0.9, first_seen=1.0, last_seen=10.0, state=2,
            map_valid=True, map_frame="camera_init", association_valid=True,
            reject_reason="", x=0.0, y=0.0)
        queue = CandidateQueue()
        candidates = [
            CandidateData(index, class_name, **base)
            for index, class_name in enumerate(
                ("tent", "pillbox", "bridge", "panzer", "red_cross"))
        ]
        queue.update(candidates, 10.0,
                     allowed_classes=profile_allowed_classes("r2026"))
        discovered = {
            candidate.class_name: candidate.target_id
            for candidate in candidates}
        self.assertEqual(
            expected_delivery_classes(discovered),
            ["red_cross", "panzer", "bridge"])

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

    def test_standard_target_capture_requires_blue_circle(self):
        candidate = CandidateData(
            1, "tank", 0.9, 1.0, 10.0, 2, True, "camera_init",
            True, "", 1.0, 2.0)
        red_cross = CaptureEvidence("red_cross", 1.0, 2.0, 10.0, 0.9)
        circle = CaptureEvidence("circle", 1.1, 2.0, 10.0, 0.9)
        self.assertFalse(capture_evidence_matches(
            candidate, [red_cross], 10.2, 1.0, 0.75))
        self.assertTrue(capture_evidence_matches(
            candidate, [circle], 10.2, 1.0, 0.75))

    def test_red_cross_capture_uses_own_geometry_not_circle(self):
        candidate = CandidateData(
            2, "red_cross", 0.9, 1.0, 10.0, 2, True, "camera_init",
            True, "", 1.0, 2.0)
        circle = CaptureEvidence("circle", 1.0, 2.0, 10.0, 0.9)
        red_cross = CaptureEvidence("red_cross", 1.1, 2.0, 10.0, 0.9)
        self.assertFalse(capture_evidence_matches(
            candidate, [circle], 10.2, 1.0, 0.75))
        self.assertTrue(capture_evidence_matches(
            candidate, [red_cross], 10.2, 1.0, 0.75))
        self.assertFalse(capture_evidence_matches(
            candidate, [red_cross], 11.2, 1.0, 0.75))

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

    def test_toudi4_coverage_bounds_are_local_to_main_h(self):
        package_dir = Path(__file__).resolve().parents[1]
        config = yaml.safe_load(
            (package_dir / "config" / "coverage_toudi4.yaml").read_text())
        self.assertEqual(
            config["field"],
            {"min_x": -3.992, "max_x": 4.008,
             "min_y": -1.132, "max_y": 8.718},
        )
        self.assertEqual(
            config["search_region"],
            {"min_x": -2.007, "max_x": 1.993,
             "min_y": 0.273, "max_y": 6.273},
        )

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
        self.assertEqual(defaults["map_size_x"], "20.0")
        self.assertEqual(defaults["map_size_y"], "20.0")
        self.assertEqual(defaults["map_size_z"], "5.0")
        self.assertEqual(defaults["obstacles_inflation"], "0.25")
        self.assertEqual(defaults["clearance_threshold"], "0.20")

        coverage_root = ET.parse(str(
            package_dir / "launch" / "coverage_navigation.launch")).getroot()
        overrides = {arg.attrib["name"]: arg.attrib.get("value")
                     for include in coverage_root.findall("include")
                     for arg in include.findall("arg")}
        self.assertEqual(overrides["planner_max_vel"], "1.5")
        self.assertEqual(overrides["planner_max_acc"], "1.0")
        self.assertEqual(overrides["planner_obstacles_inflation"], "0.25")
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

    def test_release_attribution_uses_mode_specific_geometry(self):
        package_dir = Path(__file__).resolve().parents[1]
        source = (package_dir / "scripts" /
                  "coverage_search_manager.py").read_text(encoding="utf-8")
        matcher = source.split(
            "def _release_matches_candidate", 1)[1].split(
                "def _finish_candidate", 1)[0]
        # 标准靶释放证据来自蓝环；红十字释放证据来自红十字自身几何。
        self.assertIn('release_mode == "drop_circle" and release_class == "circle"',
                      matcher)
        self.assertIn('release_mode == "drop_cross"', matcher)
        self.assertIn('release_class == "red_cross"', matcher)
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


class AccumulateRunFactsTest(unittest.TestCase):
    def test_merges_facts_across_snapshots_and_keeps_early_data(self):
        by_class, ids, selection = {}, set(), []
        accumulate_run_facts(by_class, ids, selection, {
            "discovered": [
                {"class": "tank", "id": 14},
                {"class": "bridge", "id": 4},
            ],
            "selection_sequence": [
                {"class": "tank", "id": 14},
            ],
        })
        accumulate_run_facts(by_class, ids, selection, {
            "discovered": [
                {"class": "pillbox", "id": 0},
            ],
            "selection_sequence": [
                {"class": "tank", "id": 14},
                {"class": "pillbox", "id": 0},
            ],
        })
        # ???????landing ????????????????
        accumulate_run_facts(by_class, ids, selection, {
            "discovered": [],
            "selection_sequence": [],
        })
        self.assertEqual(by_class["tank"], 14)
        self.assertEqual(by_class["bridge"], 4)
        self.assertEqual(by_class["pillbox"], 0)
        self.assertEqual(ids, {14, 4, 0})
        self.assertEqual(selection, [(14, "tank"), (0, "pillbox")])

    def test_ignores_non_standard_classes_and_invalid_ids(self):
        by_class, ids, selection = {}, set(), []
        accumulate_run_facts(by_class, ids, selection, {
            "discovered": [
                {"class": "tank", "id": 7},
                {"class": "circle", "id": 99},
                {"class": "landing_pad", "id": 3},
                {"class": "bridge", "id": None},
            ],
            "selection_sequence": [],
        })
        self.assertEqual(set(by_class), {"tank"})
        self.assertEqual(by_class["tank"], 7)
        self.assertEqual(ids, {7})
        self.assertEqual(selection, [])

    def test_latest_id_wins_and_selection_dedupes(self):
        by_class, ids, selection = {}, set(), []
        accumulate_run_facts(by_class, ids, selection, {
            "discovered": [{"class": "tank", "id": 1}],
            "selection_sequence": [{"class": "tank", "id": 1}],
        })
        accumulate_run_facts(by_class, ids, selection, {
            "discovered": [{"class": "tank", "id": 5}],
            "selection_sequence": [{"class": "tank", "id": 1},
                                   {"class": "tank", "id": 5}],
        })
        self.assertEqual(by_class["tank"], 5)
        self.assertEqual(ids, {1, 5})
        self.assertEqual(selection, [(1, "tank"), (5, "tank")])


def _candidate(target_id, class_name):
    return CandidateData(
        target_id=target_id, class_name=class_name, confidence=0.9,
        first_seen=1.0, last_seen=2.0, state=2, map_valid=True,
        map_frame="camera_init", association_valid=True,
        reject_reason="", x=0.0, y=0.0)


class InterruptPolicyTest(unittest.TestCase):
    def test_interrupt_requires_top_weight_above_threshold(self):
        pending = [_candidate(1, "tank")]
        self.assertTrue(interrupt_eligible(pending, 4.0))
        self.assertFalse(interrupt_eligible(pending, 5.5))

    def test_interrupt_ignores_lower_weights_even_if_first(self):
        pending = [_candidate(1, "panzer")]
        self.assertFalse(interrupt_eligible(pending, 4.0))

    def test_interrupt_with_red_cross_first(self):
        pending = [_candidate(2, "red_cross"), _candidate(1, "tank")]
        self.assertTrue(interrupt_eligible(pending, 4.0))
        self.assertTrue(interrupt_eligible(pending, 9.9))
        self.assertFalse(interrupt_eligible(pending, 10.5))

    def test_interrupt_empty_queue(self):
        self.assertFalse(interrupt_eligible([], 4.0))


class ExpectedDeliveryClassesTest(unittest.TestCase):
    def test_weight_order_with_red_cross(self):
        by_class = {
            "tank": 7, "bridge": 2, "pillbox": 0, "tent": 5,
            "panzer": 9, "red_cross": 11,
        }
        self.assertEqual(
            expected_delivery_classes(by_class),
            ["red_cross", "tank", "panzer"])

    def test_weight_order_without_red_cross(self):
        by_class = {
            "tank": 7, "bridge": 2, "pillbox": 0, "tent": 5, "panzer": 9,
        }
        self.assertEqual(
            expected_delivery_classes(by_class),
            ["tank", "panzer", "bridge"])


class RedCrossAccumulationTest(unittest.TestCase):
    def test_red_cross_is_accumulated_as_discovery_fact(self):
        by_class, ids, selection = {}, set(), []
        accumulate_run_facts(by_class, ids, selection, {
            "discovered": [
                {"class": "red_cross", "id": 3},
                {"class": "tank", "id": 7},
            ],
            "selection_sequence": [],
        })
        self.assertEqual(by_class["red_cross"], 3)
        self.assertEqual(ids, {3, 7})


if __name__ == "__main__":
    unittest.main()
