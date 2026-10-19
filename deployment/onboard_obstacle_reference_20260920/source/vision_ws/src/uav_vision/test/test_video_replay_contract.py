#!/usr/bin/env python3

import json
import os
import sys
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from uav_vision.video_replay_contract import (  # noqa: E402
    REQUIRED_RAW_SOURCES,
    frame_is_complete,
    mapped_message_is_fail_closed,
    validate_perf_status,
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

    def test_extended_frame_waits_for_mapped_and_perf(self):
        state = {
            "image": object(),
            "raw": {name: object() for name in REQUIRED_RAW_SOURCES},
            "resolved_search": object(),
            "resolved_landing": object(),
            "refined": object(),
            "mapped": None,
            "perf": None,
        }
        # Historical PT behavior remains complete at the refined stage.
        self.assertTrue(frame_is_complete(state))
        self.assertFalse(frame_is_complete(state, require_mapped_perf=True))
        state["mapped"] = object()
        self.assertFalse(frame_is_complete(state, require_mapped_perf=True))
        state["perf"] = object()
        self.assertTrue(frame_is_complete(state, require_mapped_perf=True))

    def test_mapped_replay_must_be_fail_closed(self):
        invalid = SimpleNamespace(map_valid=False)
        valid = SimpleNamespace(map_valid=True)
        self.assertTrue(mapped_message_is_fail_closed(
            SimpleNamespace(detections=[])))
        self.assertTrue(mapped_message_is_fail_closed(
            SimpleNamespace(detections=[invalid, invalid])))
        self.assertFalse(mapped_message_is_fail_closed(
            SimpleNamespace(detections=[invalid, valid])))
        self.assertFalse(mapped_message_is_fail_closed(None))

    def test_perf_evidence_requires_ok_rknn_backend(self):
        values = [SimpleNamespace(key="backend", value="rknn_unified")]
        status = SimpleNamespace(
            name="uav_vision/target_detector_rknn",
            level=0,
            message="rknn_unified",
            values=values,
        )
        message = SimpleNamespace(status=[status])
        self.assertEqual(
            validate_perf_status(
                message,
                expected_name="uav_vision/target_detector_rknn",
                expected_backend="rknn_unified"),
            "rknn_unified",
        )
        status.level = 1
        with self.assertRaises(ValueError):
            validate_perf_status(message)
        status.level = 0
        with self.assertRaises(ValueError):
            validate_perf_status(message, expected_backend="ultralytics")

    def test_metadata_rejects_missing_media_timing(self):
        self.assertEqual(
            validate_video_metadata({"width": 1080, "height": 1920, "fps": 60}),
            (1080, 1920, 60.0),
        )
        with self.assertRaises(ValueError):
            validate_video_metadata(json.loads('{"width":1080,"height":1920,"fps":0}'))

    def test_launch_defaults_to_pt_and_rknn_adds_only_evidence_stages(self):
        launch_path = os.path.join(
            PACKAGE_ROOT, "launch", "video_replay_annotation.launch")
        root = ET.parse(launch_path).getroot()
        args = {
            arg.attrib["name"]: arg.attrib.get("default", "")
            for arg in root.findall("arg")
        }
        self.assertEqual(args["detector_backend"], "pt")

        node_types = {node.attrib.get("type") for node in root.iter("node")}
        self.assertIn("video_replay_publisher.py", node_types)
        self.assertIn("video_annotation_recorder.py", node_types)
        self.assertIn("detection_fusion.py", node_types)
        self.assertIn("target_refiner.py", node_types)
        self.assertIn("target_detector.py", node_types)
        self.assertIn("target_detector_rknn.py", node_types)
        self.assertIn("target_map_projector.py", node_types)
        self.assertNotIn("target_memory.py", node_types)
        self.assertNotIn("drop_aligner.py", node_types)

        conditional_types = {}
        for group in root.iter("group"):
            condition = group.attrib.get("if", "")
            if not condition:
                continue
            conditional_types.setdefault(condition, set()).update(
                node.attrib.get("type") for node in group.findall("node"))
        pt_condition = "$(eval arg('detector_backend') == 'pt')"
        rknn_condition = "$(eval arg('detector_backend') == 'rknn')"
        self.assertIn("target_detector.py", conditional_types[pt_condition])
        self.assertIn(
            "target_detector_rknn.py", conditional_types[rknn_condition])
        self.assertIn(
            "target_map_projector.py", conditional_types[rknn_condition])

        recorder = next(
            node for node in root.iter("node")
            if node.attrib.get("type") == "video_annotation_recorder.py")
        recorder_params = {
            param.attrib.get("name"): param.attrib.get("value")
            for param in recorder.findall("param")
        }
        self.assertTrue({
            "raw_topic",
            "resolved_search_topic",
            "resolved_landing_topic",
            "refined_topic",
            "mapped_topic",
            "perf_topic",
            "require_mapped_perf",
            "require_map_fail_closed",
        }.issubset(recorder_params))

        with open(launch_path, "r", encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        self.assertNotIn("/selected_target", launch_text)
        self.assertIn("detections_mapped", launch_text)
        self.assertIn("video_replay_unavailable_map", launch_text)
        self.assertIn("require_map_fail_closed", launch_text)
        self.assertIn("allow_latest_tf_fallback", launch_text)
        self.assertIn("fail_closed_no_tf", launch_text)
        self.assertNotIn("static_transform_publisher", launch_text)


if __name__ == "__main__":
    unittest.main()
