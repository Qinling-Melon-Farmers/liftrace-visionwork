#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
PATROL = WORKSPACE_SRC / "patrol_control"
FORMAL_LAUNCH = PACKAGE / "launch" / "navigation_search_delivery_vcl06.launch"
MANAGER_LAUNCH = PACKAGE / "launch" / "navigation_mission_manager.launch"
KS2A543_MODEL = PATROL / "models" / "iris_mid360_downward_camera" / \
    "model_ks2a543.sdf"
RVIZ_CONFIG = PATROL / "rviz_config" / "patrol_pc.rviz"
RUNTIME_CONFIG = PACKAGE / "config" / "vcl06_random_field_runtime.yaml"
FAST_LIO = WORKSPACE_SRC / "FAST_LIO"
PLAN_MANAGE = WORKSPACE_SRC / "Fast-Planner" / "fast_planner" / "plan_manage"


class ExternalMissionContractTest(unittest.TestCase):
    def test_mission_command_contract_is_exact(self):
        message = (PATROL / "msg" / "MissionCommand.msg").read_text(
            encoding="utf-8").strip().splitlines()
        self.assertEqual(message, [
            "uint8 SEARCH=0",
            "uint8 APPROACH=1",
            "uint8 ALIGN=2",
            "uint8 RESUME=3",
            "uint8 RETURN_HOME=4",
            "uint8 LAND=5",
            "Header header",
            "uint8 command",
            "uint32 target_id",
            "string target_class",
            "geometry_msgs/PoseStamped goal",
        ])

    def test_external_mode_defaults_false_through_launch_chain(self):
        launch_files = [
            PATROL / "launch" / "patrol_control_px4_sim.launch",
            PATROL / "launch" / "patrol_full_competition_sim.launch",
            PATROL / "launch" / "toudi3_full_competition_sim_new_vision.launch",
            PACKAGE / "launch" / "toudi3_visual_delivery_guarded.launch",
        ]
        for launch_file in launch_files:
            with self.subTest(launch_file=launch_file.name):
                root = ET.parse(str(launch_file)).getroot()
                args = {element.attrib.get("name"): element.attrib
                        for element in root.findall("arg")}
                self.assertEqual(
                    args["external_mission_mode"].get("default"), "false")

    def test_patrol_goal_publisher_and_next_point_are_guarded(self):
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        self.assertIn("if (!external_mission_mode_)", source)
        self.assertIn("NextPoint disabled in external mission mode", source)
        self.assertIn("planner goal publisher disabled", source)
        self.assertIn("hasValidExternalPlannerCommand", source)
        self.assertIn("position.z <= 0.05", source)
        self.assertIn("external_planner_start_max_distance", source)
        self.assertIn(
            'nh_.param("switch/flag_planner_px4", true)', source)
        self.assertEqual(source.count(
            'advertise<geometry_msgs::PoseStamped>("/fastplanner/goal"'), 1)

    def test_external_alignment_ignores_legacy_policy_inputs(self):
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        for callback in (
                "TankStatusCallback", "waypointMarkCallback",
                "crossMarkCallback", "ClassCallback",
                "selectedTargetCallback", "crossStatusCallback"):
            start = source.index("void LLController::%s" % callback)
            body = source[start:source.index("\n}", start) + 2]
            self.assertIn("if (external_mission_mode_)", body)
            self.assertIn("return;", body)

        align_start = source.index(
            "case patrol_control::MissionCommand::ALIGN:")
        align_end = source.index(
            "case patrol_control::MissionCommand::LAND:", align_start)
        align_body = source[align_start:align_end]
        self.assertIn("geometry_msgs::PoseStamped alignment_target", align_body)
        self.assertIn("cross_mark_point = alignment_target", align_body)
        self.assertIn("waypoint_mark_point = alignment_target", align_body)
        self.assertIn("have_cross_mark = true", align_body)
        self.assertIn("have_waypoint_mark = true", align_body)

    def test_external_mode_has_one_control_tick_and_no_legacy_subscription(self):
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        position_start = source.index("void LLController::positionCallback")
        position_end = source.index(
            "void LLController::publishControlReady", position_start)
        position_body = source[position_start:position_end]
        self.assertNotIn("externalMissionTick();", position_body)
        self.assertEqual(source.count("externalMissionTick();"), 1)

        timer_start = source.index("void LLController::cmdCallback")
        timer_end = source.index("void LLController::Lock", timer_start)
        timer_body = source[timer_start:timer_end]
        self.assertIn("externalMissionTick();", timer_body)
        self.assertIn(
            "if (external_mission_mode_) {\n"
            "                ROS_INFO_THROTTLE(\n"
            "                    5, \"[PatrolControl] Forwarding external alignment setpoint\");\n"
            "                break;",
            timer_body,
        )
        external_run = timer_body[
            timer_body.index("if (external_mission_mode_)"):
            timer_body.index("if (current_task_type == MAIN_MISSION)")]
        self.assertIn("hasValidExternalPlannerCommand", external_run)
        self.assertNotIn("if (!flag_planner_px4)", external_run)
        self.assertNotIn("mavros_point_cmd = patrol_cmd;", external_run)

        init_start = source.index("void LLController::initializeNode")
        init_end = source.index("void LLController::positionCallback", init_start)
        init_body = source[init_start:init_end]
        for topic in (
                "/detect/waypoint_mark_point", "/detect/cross_mark_point",
                "/yolo_detect", "/detect/tank_status",
                "/detect/cross_status", "/uav_vision/selected_target"):
            topic_index = init_body.index(topic)
            guard_index = init_body.rfind(
                "if (!external_mission_mode_)", 0, topic_index)
            self.assertNotEqual(guard_index, -1, topic)

    def test_external_landing_uses_typed_detection_without_legacy_bridge(self):
        header = (PATROL / "include" / "patrol_control" /
                  "patrol_control.h").read_text(encoding="utf-8")
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        self.assertIn("uav_vision/TargetDetectionArray.h", header)
        self.assertIn("landingDetectionsCallback", header)
        self.assertIn("external_landing_detections_topic_", header)
        self.assertIn("landing_detections_sub_ = nh_.subscribe", source)
        self.assertIn('detection.class_name != "landing_pad"', source)
        self.assertIn("!detection.map_valid", source)
        self.assertIn("!detection.geometry_verified", source)
        self.assertIn("landMarkCallback(mark)", source)

        init_start = source.index("void LLController::initializeNode")
        init_end = source.index("void LLController::positionCallback",
                                init_start)
        init_body = source[init_start:init_end]
        legacy_land_index = init_body.index("/detect/land_mark_point")
        self.assertNotEqual(
            init_body.rfind("if (!external_mission_mode_)",
                            0, legacy_land_index), -1)
        for topic in (
                "/detect/servo_status", "/detect/class_control",
                "/detect/tank_control", "/detect/control",
                "/detect/landing_control", "/cross/control",
                "/servo/complete", "/control1", "/control2", "/control3"):
            topic_index = init_body.index(topic)
            self.assertNotEqual(
                init_body.rfind("if (!external_mission_mode_)",
                                0, topic_index), -1, topic)

        stop_start = source.index("void LLController::stopDropAction")
        stop_end = source.index("void LLController::resetDropState", stop_start)
        self.assertIn("if (external_mission_mode_)",
                      source[stop_start:stop_end])

    def test_formal_headless_disables_manual_and_visualization_nodes(self):
        def include_args(root, suffix):
            include = next(
                item for item in root.iter("include")
                if item.attrib.get("file", "").endswith(suffix))
            return {item.attrib["name"]: item.attrib.get("value")
                    for item in include.findall("arg")}

        formal = ET.parse(str(FORMAL_LAUNCH)).getroot()
        formal_guarded = include_args(
            formal, "toudi3_visual_delivery_guarded.launch")
        self.assertEqual(formal_guarded["start_waypoint_generator"], "false")
        self.assertEqual(formal_guarded["enable_static_pointcloud_viz"],
                         "false")

        guarded_path = PACKAGE / "launch" / \
            "toudi3_visual_delivery_guarded.launch"
        guarded = ET.parse(str(guarded_path)).getroot()
        guarded_nested = include_args(
            guarded, "toudi3_full_competition_sim_new_vision.launch")
        for name in ("start_waypoint_generator",
                     "enable_static_pointcloud_viz"):
            self.assertEqual(guarded_nested[name], "$(arg %s)" % name)

        visual_path = PATROL / "launch" / \
            "toudi3_full_competition_sim_new_vision.launch"
        visual = ET.parse(str(visual_path)).getroot()
        visual_full = include_args(
            visual, "patrol_full_competition_sim.launch")
        for name in ("start_waypoint_generator",
                     "enable_static_pointcloud_viz"):
            self.assertEqual(visual_full[name], "$(arg %s)" % name)

        full_path = PATROL / "launch" / "patrol_full_competition_sim.launch"
        full = ET.parse(str(full_path)).getroot()
        full_control = include_args(full, "patrol_control_px4_sim.launch")
        full_mapping = include_args(full, "mapping_mid360_sim.launch")
        self.assertEqual(full_control["start_waypoint_generator"],
                         "$(arg start_waypoint_generator)")
        self.assertEqual(full_mapping["enable_static_pointcloud_viz"],
                         "$(arg enable_static_pointcloud_viz)")

        control_path = PATROL / "launch" / "patrol_control_px4_sim.launch"
        control = ET.parse(str(control_path)).getroot()
        planner_args = include_args(control, "patrol_planner_px4_sim.launch")
        self.assertEqual(planner_args["start_waypoint_generator"],
                         "$(arg start_waypoint_generator)")

        planner_path = PLAN_MANAGE / "launch" / \
            "patrol_planner_px4_sim.launch"
        planner = ET.parse(str(planner_path)).getroot()
        waypoint = next(item for item in planner.findall("node")
                        if item.attrib.get("name") == "waypoint_generator")
        self.assertEqual(waypoint.attrib.get("if"),
                         "$(arg start_waypoint_generator)")

    def test_fast_lio_waits_for_catkin_generated_headers(self):
        source = (FAST_LIO / "CMakeLists.txt").read_text(encoding="utf-8")
        target_start = source.index("add_dependencies(fastlio_mapping")
        target_end = source.index(")", target_start)
        dependency_block = source[target_start:target_end]
        self.assertIn("${${PROJECT_NAME}_EXPORTED_TARGETS}",
                      dependency_block)
        self.assertIn("${catkin_EXPORTED_TARGETS}", dependency_block)

    def test_control_readiness_is_latched_after_takeoff(self):
        header = (PATROL / "include" / "patrol_control" /
                  "patrol_control.h").read_text(encoding="utf-8")
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        self.assertIn("ros::Publisher control_ready_pub_", header)
        self.assertIn("bool control_ready_latched_ = false", header)
        self.assertIn('control_ready_topic_ = "/mission/control_ready"',
                      header)
        self.assertIn(
            "advertise<std_msgs::Bool>(control_ready_topic_, 1, true)",
            source)
        self.assertIn("publishControlReady(false)", source)
        self.assertIn("flag_takeoff_done = 1", source)
        self.assertIn("publishControlReady(true)", source)
        self.assertLess(
            source.index("flag_takeoff_done = 1"),
            source.index("publishControlReady(true)"))
        ready_window = source[
            source.index("flag_takeoff_done = 1"):
            source.index("void LLController::publishControlReady")]
        self.assertIn("if (external_mission_mode_)", ready_window)

    def test_external_gate_enables_mode_and_manager_owns_goal(self):
        launch = ET.parse(str(FORMAL_LAUNCH)).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in launch.findall("arg")
        }
        includes = {
            element.attrib["file"]: {
                arg.attrib["name"]: arg.attrib.get("value")
                for arg in element.findall("arg")
            }
            for element in launch.findall("include")
        }
        patrol_args = next(
            args for filename, args in includes.items()
            if filename.endswith("toudi3_visual_delivery_guarded.launch"))
        bridge_args = next(
            args for filename, args in includes.items()
            if filename.endswith("navigation_planner_bridge.launch"))
        manager_args = next(
            args for filename, args in includes.items()
            if filename.endswith("navigation_mission_manager.launch"))
        self.assertEqual(patrol_args["external_mission_mode"], "true")
        self.assertEqual(patrol_args["waypoint_mode"], "false")
        self.assertEqual(patrol_args["start_legacy_compat"], "false")
        self.assertEqual(patrol_args["vehicle_sdf"], "$(arg vehicle_sdf)")
        self.assertTrue(arguments["vehicle_sdf"].endswith(
            "model_ks2a543.sdf"))
        self.assertEqual(arguments["search_altitude"], "2.4")
        self.assertEqual(manager_args["search_altitude"],
                         "$(arg search_altitude)")
        self.assertEqual(bridge_args["planner_goal_topic"],
                         "/fastplanner/goal")
        self.assertEqual(bridge_args["execution_enabled"], "true")
        self.assertEqual(bridge_args["allow_live_goal_output"], "true")
        node_types = {node.attrib.get("type")
                      for node in launch.findall("node")}
        self.assertNotIn("coverage_search_manager.py", node_types)

    def test_ks2a543_full_vehicle_preserves_topics_and_calibration(self):
        root = ET.parse(str(KS2A543_MODEL)).getroot()
        camera_link = root.find(".//link[@name='downward_camera_link']")
        self.assertIsNotNone(camera_link)
        self.assertEqual(camera_link.findtext("pose"),
                         "0 0 -0.05 0 1.57079632679 0")
        sensor = camera_link.find("sensor[@name='downward_camera']")
        self.assertIsNotNone(sensor)
        camera = sensor.find("camera")
        self.assertAlmostEqual(float(camera.findtext("horizontal_fov")),
                               1.44593453190313)
        self.assertEqual(camera.findtext("image/width"), "1280")
        self.assertEqual(camera.findtext("image/height"), "720")
        self.assertEqual(camera.findtext("image/format"), "RGB_INT8")
        expected_distortion = {
            "k1": 0.0058668600963917095,
            "k2": 0.017910549546758369,
            "p1": -0.0010064115869294274,
            "p2": 0.0014715593681005204,
            "k3": -0.026485100937585344,
        }
        for name, expected in expected_distortion.items():
            self.assertAlmostEqual(
                float(camera.findtext("distortion/%s" % name)), expected)
        plugin = sensor.find("plugin")
        self.assertEqual(plugin.findtext("cameraName"), "downward_camera")
        self.assertEqual(plugin.findtext("imageTopicName"), "image_raw")
        self.assertEqual(plugin.findtext("cameraInfoTopicName"),
                         "camera_info")
        self.assertEqual(plugin.findtext("frameName"),
                         "downward_camera_optical_frame")
        self.assertAlmostEqual(float(plugin.findtext("focalLength")),
                               725.3510059644434)
        self.assertEqual(plugin.findtext("autoDistortion"), "true")

    def test_formal_search_altitude_and_rviz_observability(self):
        manager = ET.parse(str(MANAGER_LAUNCH)).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in manager.findall("arg")
        }
        self.assertEqual(arguments["search_altitude"], "2.2")
        node = manager.find(".//node[@type='navigation_mission_manager.py']")
        altitude = next(
            item for item in node.findall("param")
            if item.attrib.get("name") == "search/altitude")
        self.assertEqual(altitude.attrib["value"], "$(arg search_altitude)")

        rviz = RVIZ_CONFIG.read_text(encoding="utf-8")
        for topic in (
                "/downward_camera/image_raw",
                "/uav_vision/debug_image",
                "/uav_vision/circle_debug",
                "/freedom/static_pointcloud_viz",
                "/sdf_map/occupancy_inflate",
                "/planning_vis/trajectory",
                "/fastplanner/goal"):
            self.assertIn(topic, rviz)

    def test_legacy_coverage_policy_is_not_a_formal_policy_source(self):
        self.assertFalse((PACKAGE / "scripts" / "coverage_policy.py").exists())
        field_config = yaml.safe_load(
            (PACKAGE / "config" / "coverage_toudi3_random.yaml").read_text(
                encoding="utf-8"))
        for legacy_key in (
                "coverage", "navigation", "candidate", "interrupt",
                "execute_candidates", "collect_before_delivery"):
            self.assertNotIn(legacy_key, field_config)
        self.assertGreaterEqual(
            field_config["spawn"]["initial_model_states_timeout"], 60.0)
        spawner = (PACKAGE / "scripts" / "random_field_spawner.py").read_text(
            encoding="utf-8")
        self.assertIn(
            "from uav_mission.random_field_policy import (", spawner)
        self.assertNotIn("from coverage_policy import", spawner)

    def test_external_landing_is_fresh_h_gated_and_fails_closed(self):
        header = (PATROL / "include" / "patrol_control" /
                  "patrol_control.h").read_text(encoding="utf-8")
        source = (PATROL / "src" / "patrol_control.cpp").read_text(
            encoding="utf-8")
        self.assertIn("externalLandingMarkFresh", header)
        self.assertIn("externalLandingTick", header)
        self.assertIn("clearExternalLandingState", header)
        self.assertIn("msg.header.stamp <= external_landing_command_stamp_",
                      source)
        self.assertIn("source_age > external_landing_mark_max_age_sec_",
                      source)
        self.assertIn("anchor_error > external_landing_max_mark_offset_",
                      source)
        self.assertIn("external_landing_stable_frames_", source)
        self.assertIn(
            "if (!mark_fresh && !external_landing_alignment_complete_)",
            source)
        self.assertIn(
            "mark_fresh && !external_landing_alignment_complete_", source)
        self.assertIn("fresh H alignment latched", source)
        self.assertNotIn(
            "if (!mark_fresh) {\n"
            "        external_landing_stable_count_ = 0;\n"
            "        external_landing_alignment_complete_ = false;",
            source)
        self.assertIn("controller_landing_watchdog_timeout", source)
        self.assertIn("failed closed and holding position", source)
        self.assertIn("if (external_mission_mode_ && !mode_accepted)", source)
        self.assertIn("flag_land = false;", source)
        self.assertIn("duplicate LAND command ignored", source)
        self.assertIn(
            "if (external_mission_mode_ && external_landing_active_)",
            source)
        self.assertIn(
            "if(!external_mission_mode_ && Drone_mode == Land", source)

    def test_formal_configs_define_external_h_landing_contract(self):
        for filename in ("patrol_toudi3_new_vision.yaml",
                         "patrol_toudi4_new_vision.yaml"):
            with self.subTest(filename=filename):
                config = yaml.safe_load(
                    (PATROL / "config" / filename).read_text(
                        encoding="utf-8"))
                landing = config["external_landing"]
                self.assertEqual(landing["frame"], "camera_init")
                self.assertEqual(
                    landing["detections_topic"],
                    "/uav_vision/detections_mapped")
                self.assertGreater(landing["capture_height"],
                                   landing["auto_land_height"])
                self.assertGreaterEqual(landing["auto_land_height"],
                                        config["land_height"])
                self.assertLessEqual(landing["mark_max_age_sec"], 0.5)
                self.assertGreaterEqual(landing["stable_frames"], 1)
                self.assertNotIn("timeout_sec", landing)
                self.assertGreater(landing["watchdog_timeout_sec"], 0.0)
                runtime = yaml.safe_load(
                    RUNTIME_CONFIG.read_text(encoding="utf-8"))
                self.assertNotIn(
                    "landing_action_timeout", runtime["mission"])
                self.assertIs(config["switch"]["auto_land"], True)


if __name__ == "__main__":
    unittest.main()
