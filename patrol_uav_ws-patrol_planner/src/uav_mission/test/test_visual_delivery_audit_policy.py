#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from uav_mission.visual_delivery_audit_policy import (
    permission_matches_audit_view, resolve_audit_evidence,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GUARDED_LAUNCH = PACKAGE_ROOT / "launch" / \
    "toudi3_visual_delivery_guarded.launch"


def geometry_evidence(target_id=42, stable_frames=5):
    return {
        "align_mode": "drop_circle",
        "target_id": target_id,
        "target_class": "circle",
        "stable_frames": stable_frames,
        "evidence_valid": True,
        "stamp_nsec": 123,
    }


def strict_context():
    return {
        "context_valid": True,
        "context_active": True,
        "has_semantic_target": True,
        "align_mode": "drop_circle",
        "semantic_target_id": 7,
        "semantic_target_class": "tent",
        "evidence": geometry_evidence(),
    }


class VisualDeliveryAuditPolicyTest(unittest.TestCase):
    def test_strict_circle_uses_semantic_identity_and_nested_geometry(self):
        view, reason = resolve_audit_evidence(
            None, strict_context(), True)

        self.assertEqual(reason, "")
        self.assertEqual(view["target_id"], 7)
        self.assertEqual(view["target_class"], "tent")
        self.assertTrue(view["compare_target_class"])
        self.assertEqual(view["geometry_evidence"]["target_id"], 42)
        self.assertEqual(view["geometry_evidence"]["target_class"], "circle")
        self.assertEqual(view["geometry_evidence"]["stable_frames"], 5)
        self.assertEqual(view["geometry_evidence"]["stamp_nsec"], 123)
        semantic_permission = {
            "align_mode": "drop_circle",
            "target_id": 7,
            "target_class": "tent",
        }
        geometry_permission = dict(
            semantic_permission, target_id=42, target_class="circle")
        self.assertTrue(permission_matches_audit_view(
            semantic_permission, view))
        self.assertFalse(permission_matches_audit_view(
            geometry_permission, view))

    def test_legacy_mode_keeps_geometry_identity(self):
        evidence = geometry_evidence()
        view, reason = resolve_audit_evidence(evidence, None, False)

        self.assertEqual(reason, "")
        self.assertEqual(view["target_id"], 42)
        self.assertEqual(view["target_class"], "circle")
        self.assertFalse(view["compare_target_class"])
        self.assertIs(view["geometry_evidence"], evidence)
        self.assertTrue(permission_matches_audit_view({
            "align_mode": "drop_circle",
            "target_id": 42,
            "target_class": "legacy_class_is_not_compared",
        }, view))

    def test_strict_mode_rejects_missing_or_inactive_context(self):
        view, reason = resolve_audit_evidence(None, None, True)
        self.assertIsNone(view)
        self.assertEqual(reason, "evidence_context_missing")

        context = strict_context()
        context["context_active"] = False
        view, reason = resolve_audit_evidence(None, context, True)
        self.assertIsNone(view)
        self.assertEqual(reason, "evidence_context_invalid")

    def test_guarded_launch_forwards_existing_strict_flag_to_audit(self):
        root = ET.parse(str(GUARDED_LAUNCH)).getroot()
        audit = next(node for node in root.findall("node")
                     if node.attrib.get("name") == "visual_delivery_audit")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in audit.findall("param")}
        self.assertEqual(
            params["require_evidence_context"],
            "$(arg require_release_evidence_context)")


if __name__ == "__main__":
    unittest.main()
