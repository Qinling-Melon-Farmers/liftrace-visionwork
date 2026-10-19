"""Meaningful contract/package guards; fixtures never represent a training handoff."""
import argparse
import copy
import json
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest import mock

import numpy as np
import yaml
import release


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.metadata = release.ROOT / "vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml"
        self.config = release.ROOT / "vision_ws/src/uav_vision/config/target_detector_rknn.yaml"
        self.decoder = release.ROOT / "vision_ws/src/uav_vision/scripts/target_detector_rknn.py"

    def test_actual_board_preprocessing_and_red_cross_id(self):
        ns, contract = release.board_contract(self.metadata, self.config, self.decoder)
        output = np.zeros((1, 9, 8400), np.float32)
        output[0, :4, 0] = [320, 320, 100, 100]
        output[0, 8, 0] = 0.9
        ns["_validate_output_contract"]([output], release.CONTRACT)
        found = ns["_decode_outputs"]([output], 5, 0.5, 640, (640, 640, 3), 1, (0, 0), box_format="xywh")
        self.assertEqual(found[0]["class_id"], 4)
        self.assertFalse(contract["hardware_validated"])
        with self.assertRaises(ValueError):
            ns["_validate_output_contract"]([np.zeros((1, 10, 8400))], release.CONTRACT)

    def test_legacy_four_classes_and_reordered_classes_rejected(self):
        for names in [{0: "bridge", 1: "panzer", 2: "pillbox", 3: "tent"},
                      {0: "red_cross", 1: "panzer", 2: "pillbox", 3: "tent", 4: "bridge"}]:
            data = copy.deepcopy(release.CONTRACT)
            data["names"] = names
            path = self.root / "wrong.yaml"
            path.write_text(yaml.safe_dump(data))
            with self.assertRaisesRegex(ValueError, "names"):
                release.board_contract(path, self.config, self.decoder)

    def test_runtime_double_normalization_contract_rejected(self):
        config = yaml.safe_load(self.config.read_text())
        config["input_dtype"] = "uint8"
        path = self.root / "wrong_config.yaml"
        path.write_text(yaml.safe_dump(config))
        with self.assertRaisesRegex(ValueError, "input_dtype"):
            release.board_contract(self.metadata, path, self.decoder)

    def test_unconfirmed_handoff_rejected(self):
        path = self.root / "handoff.json"
        path.write_text(json.dumps({"final": False}))
        with self.assertRaisesRegex(ValueError, "marked these weights final"):
            release.read_handoff(path, self.root / "best.pt")

    def test_fp32_failure_has_no_fallback_config_call(self):
        runtime = mock.Mock()
        runtime.config.side_effect = ValueError("FP32 unavailable")
        with self.assertRaises(ValueError):
            release.rknn_config(runtime, "fp32")
        self.assertEqual(runtime.config.call_count, 1)
        self.assertEqual(runtime.config.call_args.kwargs["float_dtype"], "float32")


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / "fixture"
        self.source.mkdir()
        self.report = {
            "status": "PASS", "rknn_status": "exported",
            "purpose": "final_candidate", "final_model_confirmed": True,
            "hardware_validated": False, "names": release.NAMES,
            "model_id": "fixture_only", "effective_precision": "fp16",
            "architecture": {"fixture": True},
            "files": {"pt": "fixture_only.pt", "onnx": "fixture_only.onnx",
                      "rknn": "fixture_only_fp16.rknn", "metadata": "metadata.yaml",
                      "training_recipe": "training_recipe.yaml",
                      "source_statistics": "source_statistics.json",
                      "test_summary": "test_summary.md"},
        }
        for name in self.report["files"].values():
            (self.source / name).write_text("UNIT TEST FIXTURE, NOT A MODEL")
        meta = dict(release.CONTRACT, model_file="fixture_only_fp16.rknn",
                    candidate=True, hardware_validated=False)
        (self.source / "metadata.yaml").write_text(yaml.safe_dump(meta))
        release.write_json(self.source / "validation.json", {"status": "PASS", "hardware_validated": False})
        release.write_json(self.source / "rknn_simulator.json", {"status": "PASS", "hardware_tested": False})
        self.args = argparse.Namespace(export=self.source, output=self.root / "fixture_only.tar.gz",
                                       zip_output=self.root / "fixture_only.zip")
        self.update_report()

    def update_report(self):
        release.write_json(self.source / "export.json", self.report)

    def test_smoke_cannot_become_final_package(self):
        self.report.update(purpose="workflow_smoke", final_model_confirmed=False)
        self.update_report()
        with self.assertRaisesRegex(ValueError, "Smoke/historical/unconfirmed"):
            release.package_command(self.args)
        self.assertFalse(self.args.output.exists())

    def test_missing_training_summary_rejected(self):
        (self.source / "test_summary.md").unlink()
        with self.assertRaisesRegex(ValueError, "test_summary"):
            release.package_command(self.args)

    def test_wrong_model_metadata_pair_rejected(self):
        meta = dict(release.CONTRACT, model_file="old.rknn")
        (self.source / "metadata.yaml").write_text(yaml.safe_dump(meta))
        with self.assertRaisesRegex(ValueError, "another model"):
            release.package_command(self.args)

    def test_provisional_evaluation_notice_rejected(self):
        notice = self.root / "PT_TRAINING_COMPLETE.txt"
        notice.write_text("Training complete; evaluation pending")
        self.report["handoff"] = {
            "test_summary_status": "pending_805_regression_do_not_package_yet",
            "test_summary": str(notice),
            "final_test_summary_expected": str(self.root / "TRAINING_REPORT.txt"),
        }
        self.args.test_summary = notice
        self.update_report()
        with self.assertRaisesRegex(ValueError, "not a completed evaluation summary"):
            release.package_command(self.args)
        self.assertFalse(self.args.output.exists())

    def test_attach_completed_summary_at_package_time(self):
        original = self.source / "test_summary.md"
        original.unlink()
        self.report["files"].pop("test_summary")
        self.report["package_ready"] = False
        self.report["pending_attachments"] = ["test_summary"]
        summary = self.root / "TRAINING_REPORT.txt"
        summary.write_text("UNIT TEST: completed evaluation fixture")
        self.report["handoff"] = {
            "test_summary_status": "pending_805_regression_do_not_package_yet",
            "test_summary": str(self.root / "PT_TRAINING_COMPLETE.txt"),
            "final_test_summary_expected": str(summary),
        }
        self.args.test_summary = summary
        self.update_report()
        with mock.patch.object(release, "load_model", return_value=(None, {"fixture": True})), \
             mock.patch.object(release, "validate_onnx"):
            release.package_command(self.args)
        updated = json.loads((self.source / "export.json").read_text())
        self.assertTrue(updated["package_ready"])
        self.assertEqual(updated["pending_attachments"], [])
        with zipfile.ZipFile(self.args.zip_output) as archive:
            self.assertIn("fixture_only/test_summary.txt", archive.namelist())

    def test_completed_extra_evaluation_is_in_both_archives(self):
        extra = self.root / "bag_1444_summary.json"
        extra.write_text('{"fixture": true, "frames": 403}')
        self.args.evaluation_attachment = [extra]
        with mock.patch.object(release, "load_model", return_value=(None, {"fixture": True})), \
             mock.patch.object(release, "validate_onnx"):
            release.package_command(self.args)
        name = "fixture_only/evaluation_bag_1444_summary.json"
        with tarfile.open(self.args.output) as archive:
            self.assertIn(name, archive.getnames())
        with zipfile.ZipFile(self.args.zip_output) as archive:
            self.assertIn(name, archive.namelist())

    def test_credential_text_rejected(self):
        (self.source / "training_recipe.yaml").write_text("api_key: fixture_secret")
        with mock.patch.object(release, "load_model", return_value=(None, {"fixture": True})), \
             mock.patch.object(release, "validate_onnx"):
            with self.assertRaisesRegex(ValueError, "Credential-like"):
                release.package_command(self.args)
        self.assertFalse(self.args.output.exists())
        self.assertFalse(self.args.zip_output.exists())

    def test_local_symlink_rejected(self):
        path = self.source / "training_recipe.yaml"
        path.unlink()
        target = self.root / "external.yaml"
        target.write_text("outside_fixture: true")
        path.symlink_to(target)
        with self.assertRaisesRegex(ValueError, "unsafe package entry"):
            release.package_command(self.args)

    def test_existing_zip_prevents_partial_tar(self):
        self.args.zip_output.write_bytes(b"do not overwrite")
        with mock.patch.object(release, "load_model", return_value=(None, {"fixture": True})), \
             mock.patch.object(release, "validate_onnx"):
            with self.assertRaisesRegex(ValueError, "Output already exists"):
                release.package_command(self.args)
        self.assertFalse(self.args.output.exists())

    def test_complete_archive_and_no_overwrite(self):
        # Actual PT/ONNX execution is exercised by the separate historical smoke.
        # Mock only these heavy reads here; no RKNN/PT fixture is presented as usable.
        with mock.patch.object(release, "load_model", return_value=(None, {"fixture": True})), \
             mock.patch.object(release, "validate_onnx"):
            release.package_command(self.args)
            with tarfile.open(self.args.output) as archive:
                names = set(archive.getnames())
                for name in list(self.report["files"].values()) + [
                        "export.json", "validation.json", "rknn_simulator.json",
                        "classes.txt", "README_替换与回滚.md"]:
                    self.assertIn("fixture_only/" + name, names)
                text = archive.extractfile("fixture_only/README_替换与回滚.md").read().decode()
                self.assertNotIn("@MODEL@", text)
                self.assertIn("fixture_only_fp16.rknn", text)
                self.assertIn("尚未进行RK3588 NPU", text)
            with zipfile.ZipFile(self.args.zip_output) as archive:
                self.assertEqual(set(archive.namelist()), names)
                self.assertTrue(all(not name.startswith("/") and ".." not in name.split("/")
                                    for name in archive.namelist()))
                self.assertTrue(all((info.external_attr >> 16) & 0o170000 != 0o120000
                                    for info in archive.infolist()))
            previous = self.args.output.read_bytes()
            with self.assertRaisesRegex(ValueError, "Output already exists"):
                release.package_command(self.args)
            self.assertEqual(self.args.output.read_bytes(), previous)


if __name__ == "__main__":
    unittest.main()
