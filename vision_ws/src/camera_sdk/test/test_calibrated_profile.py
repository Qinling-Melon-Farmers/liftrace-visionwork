#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import cv2
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE_ROOT / "param" / "calibration_1280x720.yaml"
LAUNCH = PACKAGE_ROOT / "launch" / "camera_calibrated_1280x720.launch"
SCRIPT = PACKAGE_ROOT / "script" / "camera_sdk.py"
SOURCE_CALIBRATION = PACKAGE_ROOT.parent.parent / "calibration.yaml"


class CalibratedCameraProfileTest(unittest.TestCase):
    def test_ros_camera_info_profile_matches_calibration(self):
        profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
        self.assertEqual(profile["image_width"], 1280)
        self.assertEqual(profile["image_height"], 720)
        self.assertEqual(profile["distortion_model"], "plumb_bob")
        self.assertEqual(len(profile["camera_matrix"]["data"]), 9)
        self.assertEqual(
            len(profile["distortion_coefficients"]["data"]), 5)
        self.assertEqual(len(profile["rectification_matrix"]["data"]), 9)
        self.assertEqual(len(profile["projection_matrix"]["data"]), 12)
        self.assertAlmostEqual(
            profile["camera_matrix"]["data"][0], 725.3510059644434)
        self.assertAlmostEqual(
            profile["camera_matrix"]["data"][4], 723.34035628450874)

        source = cv2.FileStorage(
            str(SOURCE_CALIBRATION), cv2.FILE_STORAGE_READ)
        self.assertTrue(source.isOpened())
        try:
            source_k = source.getNode("camera_matrix").mat().reshape(-1)
            source_d = source.getNode(
                "distortion_coefficients").mat().reshape(-1)
        finally:
            source.release()
        self.assertEqual(profile["camera_matrix"]["data"],
                         source_k.tolist())
        self.assertEqual(profile["distortion_coefficients"]["data"],
                         source_d.tolist())

    def test_launch_preserves_calibrated_pixel_geometry(self):
        root = ET.parse(str(LAUNCH)).getroot()
        node = root.find("node")
        self.assertIsNotNone(node)
        self.assertEqual(node.attrib.get("required"), "true")
        params = {
            item.attrib["name"]: item.attrib.get("value")
            for item in node.findall("param")
        }
        self.assertEqual(params["frame_width"], "1280")
        self.assertEqual(params["frame_height"], "720")
        self.assertEqual(params["rotation_angle"], "0")
        self.assertIn("calibration_1280x720.yaml",
                      params["camera_param_yaml"])
        self.assertEqual(params["frame_id"], "$(arg frame_id)")

    def test_image_and_camera_info_share_one_frame(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("img_msg.header.frame_id = frame_id", source)
        self.assertIn("compressed_msg.header.frame_id = frame_id", source)
        self.assertIn("camera_info.header.frame_id = frame_id", source)
        self.assertIn("actual_width != camera_info.width", source)
        self.assertIn("actual_height != camera_info.height", source)


if __name__ == "__main__":
    unittest.main()
