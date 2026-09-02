#!/usr/bin/env python3

from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from uav_vision.model_contract import require_local_model_file


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PT_SCRIPT = PACKAGE_ROOT / "scripts" / "target_detector.py"
RKNN_SCRIPT = PACKAGE_ROOT / "scripts" / "target_detector_rknn.py"
PT_LAUNCH = PACKAGE_ROOT / "launch" / "phase_d.launch"
RKNN_LAUNCH = PACKAGE_ROOT / "launch" / "phase_d_board.launch"


class DetectorModelContractTest(unittest.TestCase):
    def test_empty_and_missing_model_paths_fail(self):
        for value in ("", "   ", None):
            with self.assertRaises(ValueError):
                require_local_model_file(value, "test")
        with self.assertRaises(ValueError):
            require_local_model_file("/definitely/missing/model.pt", "test")

    def test_existing_model_file_resolves(self):
        with tempfile.NamedTemporaryFile(suffix=".pt") as model:
            self.assertEqual(
                require_local_model_file(model.name, "test"),
                str(Path(model.name).resolve()))

    def test_full_chain_detector_processes_are_required(self):
        for launch_path, node_name in (
                (PT_LAUNCH, "target_detector"),
                (RKNN_LAUNCH, "target_detector_rknn")):
            root = ET.parse(str(launch_path)).getroot()
            node = next(item for item in root.findall("node")
                        if item.attrib.get("name") == node_name)
            self.assertEqual(node.attrib.get("required"), "true")

    def test_detectors_do_not_advertise_missing_runtime_as_empty_backend(self):
        pt_source = PT_SCRIPT.read_text(encoding="utf-8")
        rknn_source = RKNN_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("publishing empty detections for dev/sim", pt_source)
        self.assertNotIn("empty_no_rknnlite", rknn_source)
        self.assertNotIn("empty_no_runtime", rknn_source)
        self.assertIn("no usable RKNN runtime/model found", rknn_source)
        self.assertIn("_restore_standard_logging_levels()", rknn_source)


if __name__ == "__main__":
    unittest.main()
