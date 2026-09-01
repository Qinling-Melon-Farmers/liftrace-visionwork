#!/usr/bin/env python3

import json
import os
import sys
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from uav_vision.video_replay_contract import (  # noqa: E402
    REQUIRED_RAW_SOURCES,
    frame_is_complete,
    validate_video_metadata,
)


class VideoReplayContractTest(unittest.TestCase):
    def test_frame_waits_for_every_real_pixel_stage(self):
        state = {
            "image": object(),
            "raw": {name: object() for name in REQUIRED_RAW_SOURCES},
            "resolved_search": object(),
            "resolved_landing": object(),
            "refined": object(),
        }
        self.assertTrue(frame_is_complete(state))
        state["raw"].pop("landing_detector")
        self.assertFalse(frame_is_complete(state))

    def test_metadata_rejects_missing_media_timing(self):
        self.assertEqual(
            validate_video_metadata({"width": 1080, "height": 1920, "fps": 60}),
            (1080, 1920, 60.0),
        )
        with self.assertRaises(ValueError):
            validate_video_metadata(json.loads('{"width":1080,"height":1920,"fps":0}'))

    def test_launch_excludes_map_memory_and_control_nodes(self):
        launch_path = os.path.join(
            PACKAGE_ROOT, "launch", "video_replay_annotation.launch")
        root = ET.parse(launch_path).getroot()
        node_types = {node.attrib.get("type") for node in root.iter("node")}
        self.assertIn("video_replay_publisher.py", node_types)
        self.assertIn("video_annotation_recorder.py", node_types)
        self.assertIn("detection_fusion.py", node_types)
        self.assertIn("target_refiner.py", node_types)
        self.assertNotIn("target_map_projector.py", node_types)
        self.assertNotIn("target_memory.py", node_types)
        self.assertNotIn("drop_aligner.py", node_types)

        with open(launch_path, "r", encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        self.assertNotIn("/selected_target", launch_text)
        self.assertNotIn("detections_mapped", launch_text)


if __name__ == "__main__":
    unittest.main()
