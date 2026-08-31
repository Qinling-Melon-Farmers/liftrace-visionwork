#!/usr/bin/env python3
"""Regression contract for the VCL06 simulated LiDAR-to-map input."""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
FAST_LIO_SIM = WORKSPACE / "src/FAST_LIO/config/mid360_sim.yaml"
FAST_LIO_REAL = WORKSPACE / "src/FAST_LIO/config/mid360.yaml"
FAST_LIO_PREPROCESS = WORKSPACE / "src/FAST_LIO/src/preprocess.cpp"
FREEDOM_VISUALIZATION = WORKSPACE / "src/FreeDOM/src/visualization.cpp"
MODEL_DIR = (
    WORKSPACE
    / "src/patrol_control/models/iris_mid360_downward_camera"
)
MODEL_SDF = MODEL_DIR / "model.sdf"
GUARD_MESH = MODEL_DIR / "meshes/competition_guard_ring.obj"
PATROL_WORLD_LAUNCH = (
    WORKSPACE / "src/patrol_control/launch/patrol_world.launch"
)


class SimLidarMapContractTest(unittest.TestCase):
    def test_sim_uses_xyz_parser_without_changing_real_mid360(self):
        sim = yaml.safe_load(FAST_LIO_SIM.read_text(encoding="utf-8"))
        real = yaml.safe_load(FAST_LIO_REAL.read_text(encoding="utf-8"))
        self.assertEqual(sim["preprocess"]["lidar_type"], 4)
        self.assertEqual(real["preprocess"]["lidar_type"], 1)
        source = FAST_LIO_PREPROCESS.read_text(encoding="utf-8")
        start = source.index("void Preprocess::sim_handler")
        end = source.index("void Preprocess::give_feature", start)
        handler = source[start:end]
        self.assertIn("pcl::PointCloud<pcl::PointXYZ> pl_orig", handler)
        self.assertIn("added_pt.intensity = 0.0", handler)
        self.assertNotIn("pcl::PointCloud<pcl::PointXYZI> pl_orig", handler)

    def test_contact_guard_is_a_ring_and_keeps_bumper_topic(self):
        root = ET.parse(str(MODEL_SDF)).getroot()
        collision = root.find(
            ".//link[@name='competition_contact_link']"
            "/collision[@name='competition_guard_collision']"
        )
        self.assertIsNotNone(collision)
        self.assertIsNone(collision.find("./geometry/cylinder"))
        uri = collision.findtext("./geometry/mesh/uri")
        self.assertEqual(
            uri,
            "model://iris_mid360_downward_camera/meshes/competition_guard_ring.obj",
        )
        self.assertEqual(
            root.findtext(
                ".//sensor[@name='competition_contact_sensor']"
                "/plugin/bumperTopicName"
            ),
            "/mission/uav_contacts_raw",
        )

    def test_guard_mesh_clears_the_lidar_origin_and_has_an_aperture(self):
        root = ET.parse(str(MODEL_SDF)).getroot()
        guard_pose = root.findtext(
            ".//link[@name='competition_contact_link']/pose"
        )
        self.assertIsNotNone(guard_pose)
        guard_z = float(guard_pose.split()[2])
        vertices = []
        faces = []
        for line in GUARD_MESH.read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "v":
                vertices.append(tuple(float(value) for value in fields[1:4]))
            elif fields[0] == "f":
                faces.append(tuple(int(value) for value in fields[1:]))

        self.assertEqual(len(vertices), 32)
        self.assertEqual(len(faces), 32)
        radial_distances = [math.hypot(x, y) for x, y, _ in vertices]
        self.assertGreaterEqual(min(radial_distances), 0.249)
        self.assertLessEqual(max(radial_distances), 0.301)
        self.assertEqual({round(abs(z), 3) for _, _, z in vertices}, {0.02})
        lidar_ray_origin_z = 0.13
        self.assertGreater(
            guard_z + min(z for _, _, z in vertices), lidar_ray_origin_z
        )

    def test_repo_local_guard_mesh_is_on_gazebo_model_path(self):
        root = ET.parse(str(PATROL_WORLD_LAUNCH)).getroot()
        model_path_arg = root.find("./arg[@name='gazebo_model_path']")
        self.assertIsNotNone(model_path_arg)
        value = model_path_arg.attrib.get("default", "")
        self.assertIn("$(find patrol_control)/models", value)
        self.assertIn("$(arg px4_model_root)", value)

    def test_static_map_publication_has_a_ros_timestamp(self):
        source = FREEDOM_VISUALIZATION.read_text(encoding="utf-8")
        start = source.index("void Visualizer::visualize_static_pointcloud")
        end = source.index("void Visualizer::visualize_raycast_map_range", start)
        publisher = source[start:end]
        self.assertIn(
            "pointcloud_msg.header.stamp = ros::Time::now();", publisher
        )


if __name__ == "__main__":
    unittest.main()
