#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def message_statements(name):
    path = PACKAGE_ROOT / "msg" / name
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


class CompetitionProfileContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        profile_path = PACKAGE_ROOT / "config" / "competition_profiles.yaml"
        cls.document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        cls.profiles = cls.document["profiles"]

    def test_r2026_is_exactly_the_frozen_five_class_profile(self):
        r2026 = self.profiles["r2026"]
        self.assertEqual(
            {
                "tent": 1.0,
                "pillbox": 1.5,
                "bridge": 2.0,
                "panzer": 2.5,
                "red_cross": 10.0,
            },
            r2026["classes"],
        )
        self.assertEqual(3, r2026["interrupt_top_k"])
        self.assertEqual(3, r2026["required_deliveries"])

    def test_r2026_excludes_tank_and_unknown_profiles(self):
        self.assertNotIn("tank", self.profiles["r2026"]["classes"])
        self.assertIsNone(self.profiles.get("unknown_profile"))

    def test_interrupt_order_is_deterministic(self):
        classes = self.profiles["r2026"]["classes"]
        ordered = sorted(classes, key=lambda name: (-classes[name], name))
        self.assertEqual(["red_cross", "panzer", "bridge"], ordered[:3])

class NavigationMessageContractTest(unittest.TestCase):
    def test_decision_schema_and_commands_are_frozen(self):
        statements = message_statements("NavigationDecision.msg")
        self.assertIn("uint8 SCHEMA_VERSION=1", statements)
        self.assertEqual(
            [
                "uint8 SEARCH=0",
                "uint8 APPROACH=1",
                "uint8 ALIGN=2",
                "uint8 RESUME=3",
                "uint8 RETURN_HOME=4",
                "uint8 LAND=5",
                "uint8 HOLD=6",
                "uint8 ABORT=7",
            ],
            [
                line
                for line in statements
                if line.startswith("uint8 ")
                and "=" in line
                and "SCHEMA_VERSION" not in line
            ],
        )
        for field in (
            "std_msgs/Header header",
            "uint8 schema_version",
            "string mission_id",
            "uint32 decision_seq",
            "time deadline",
            "string class_profile",
            "bool has_goal",
            "bool has_target",
            "uint32 target_id",
            "time target_first_seen",
            "time target_observation_stamp",
            "uint16 attempt",
            "uint8 payload_slot",
            "geometry_msgs/PoseStamped goal",
        ):
            self.assertIn(field, statements)

    def test_result_schema_statuses_stages_and_commit_evidence_are_frozen(self):
        statements = message_statements("NavigationResult.msg")
        self.assertIn("uint8 SCHEMA_VERSION=1", statements)
        for constant in (
            "uint8 ACCEPTED=0",
            "uint8 TIMED_OUT=7",
            "uint8 DISPATCH=0",
            "uint8 RELEASE=4",
            "uint8 LANDING=6",
        ):
            self.assertIn(constant, statements)
        for field in (
            "string executor_id",
            "std_msgs/Header header",
            "uint32 event_seq",
            "uint32 decision_seq",
            "bool terminal",
            "bool retryable",
            "bool payload_committed",
            "bool has_target",
            "uint32 target_id",
            "time target_first_seen",
            "uint16 attempt",
            "uint8 payload_slot",
            "string evidence_source",
        ):
            self.assertIn(field, statements)


if __name__ == "__main__":
    unittest.main()
