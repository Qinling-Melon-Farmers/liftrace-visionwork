"""Offline wrapper and actual generated-profile checks; no ROS/hardware imports."""
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "deployment/board_trials_4x4/common/uav_board_trials/scripts"))
sys.path.insert(0, str(ROOT / "patrol_uav_ws-patrol_planner/src/uav_mission/src"))
sys.path.insert(0, str(ROOT / "vision_ws/src/uav_high_view/src"))
import start_test
from trial_config import apply_site_profile, flight_geometry, generate, validate_settings
from trial_motion import motion_options


class SiteEntryPoint(unittest.TestCase):
    def settings(self, trial, pattern=None):
        argv = [trial, "flight", "--print-command"]
        if pattern:
            argv += ["--survey-pattern", pattern]
        _, cmd = start_test.command(argv)
        base = ROOT / "deployment/board_trials_4x4" / start_test.FOLDERS[trial]
        settings = yaml.safe_load((base / "settings.yaml").read_text())
        site = Path(cmd[cmd.index("--site-config") + 1])
        apply_site_profile(settings, yaml.safe_load(site.read_text()))
        motion = settings.get("motion_optimization", {})
        settings["motion_optimization"] = {**motion, "enabled": True}
        if "--survey-pattern" in cmd:
            settings["survey_pattern"] = cmd[cmd.index("--survey-pattern") + 1]
        if "--capture-speed" in cmd:
            settings["cruise_speed"] = float(cmd[cmd.index("--capture-speed") + 1])
        return settings, cmd

    def test_all_nine_and_field_aliases_delegate_to_expected_module(self):
        for trial, folder in start_test.FOLDERS.items():
            _, cmd = start_test.command([trial, "flight"])
            self.assertEqual(Path(cmd[1]).parent.name, folder)
            self.assertEqual(cmd[2], "flight")
            self.assertIn("--motion-optimized", cmd)
            self.assertNotIn("--real-release", cmd)
        for alias, trial in start_test.ALIASES.items():
            _, cmd = start_test.command([alias])
            self.assertEqual(Path(cmd[1]).parent.name, start_test.FOLDERS[trial])
            self.assertEqual(cmd[2], "preview")

    def test_dedicated_h_corridor_and_full_profiles(self):
        for trial, name in start_test.SITE_FILES.items():
            _, cmd = start_test.command([trial])
            self.assertEqual(Path(cmd[cmd.index("--site-config") + 1]).name, name)
            self.assertEqual("--survey-pattern" in cmd, trial == "full_mission")

    def test_generated_high_routes_use_site_2m_and_lab_geometry(self):
        rig = yaml.safe_load((ROOT / "deployment/board_trials_4x4/common/uav_board_trials/config/known_rig.yaml").read_text())
        for trial in sorted(start_test.HIGH_TRIALS):
            for pattern, count in (("rect", 5), ("snake2", 4), ("snake3", 6)):
                with self.subTest(trial=trial, pattern=pattern):
                    settings, _ = self.settings(trial, pattern)
                    self.assertEqual(settings["high_agl"], 2.0)
                    self.assertEqual(settings["max_agl"], 2.0)
                    area = flight_geometry(settings)
                    self.assertEqual(area["center_bounds"], [-0.35, 6.0, -1.5, 1.5])
                    points = area["survey_xy"]
                    self.assertEqual(len(points), count)
                    self.assertTrue(all(0.8 <= x <= 5.5 and -1.1 <= y <= 1.1 for x, y in points))
                    self.assertEqual(settings["obstacle_columns_enabled"], trial == "full_mission")
                    # Only the test fixture supplies missing measured geometry.
                    if trial == "full_mission":
                        settings["landing_xy"] = [2.8, 0]
                        settings["corridor_waypoints"] = [dict(x=0.6, y=0, agl=0.9), dict(x=1.2, y=0, agl=0.9)]
                    validate_settings(settings)
                    with tempfile.TemporaryDirectory() as temporary:
                        generate(ROOT, temporary, settings, (0.1, 0.1, 0.22), rig)
                        out = Path(temporary)
                        runtime = yaml.safe_load((out / "runtime.yaml").read_text())
                        overrides = yaml.safe_load((out / "overrides.yaml").read_text())
                        self.assertTrue(runtime["motion_optimization"]["enabled"])
                        self.assertEqual(runtime["high_view_probe"]["config"]["high_agl"], 2.0)
                        actual = runtime["high_view_probe"]["config"]["survey_xy"]
                        for point, generated in zip(points, actual):
                            self.assertAlmostEqual(generated[0], point[0] + 0.1)
                            self.assertAlmostEqual(generated[1], point[1] + 0.1)
                        self.assertAlmostEqual(overrides["/external_planner_max_command_z"], 2.0, places=5)

    def test_capture_defaults_to_half_metre_per_second(self):
        _, cmd = start_test.command(["6", "flight"])
        self.assertEqual(cmd[cmd.index("--capture-speed") + 1], "0.5")
        _, cmd = start_test.command(["capture", "flight", "--capture-speed", "1.0"])
        self.assertEqual(cmd[cmd.index("--capture-speed") + 1], "1.0")

    def test_missing_measured_geometry_still_refused(self):
        for trial in ("corridor_landing", "full_mission"):
            settings, _ = self.settings(trial)
            with self.assertRaises(ValueError):
                validate_settings(settings)

    def test_disallowed_options_rejected_before_any_process(self):
        for argv in (["h", "flight", "--real-release"], ["3", "flight", "--real-release"],
                     ["6", "flight", "--real-release"], ["5", "preview", "--real-release"],
                     ["1", "--survey-pattern", "snake2"], ["h", "--survey-pattern", "rect"],
                     ["5", "--capture-speed", "1.0"], ["3", "--resume-survey", "on"],
                     ["5", "--site-config", "competition.yaml"]):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit), mock.patch.object(start_test.subprocess, "call") as call:
                    start_test.main(argv)
                call.assert_not_called()

    def test_print_is_offline_and_check_delegates_exact_command(self):
        with mock.patch.object(start_test.subprocess, "call") as call, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(start_test.main(["5", "flight", "--print-command"]), 0)
            call.assert_not_called()
        _, expected = start_test.command(["5", "preview", "--check-config"])
        with mock.patch.object(start_test.subprocess, "call", return_value=0) as call:
            self.assertEqual(start_test.main(["5", "preview", "--check-config"]), 0)
            call.assert_called_once_with(expected)

    def test_site_whitelist_requires_motion_and_pattern_via_cli(self):
        settings, _ = self.settings("high_priority")
        for key, value in (("motion_optimization", {"enabled": True}), ("survey_pattern", "snake2")):
            with self.assertRaises(ValueError):
                apply_site_profile(copy.deepcopy(settings), {key: value})

    def test_old_entrypoint_inherits_enabled_settings_without_replacing_site_route(self):
        # Read the actual nine settings without injecting wrapper CLI overrides.
        for trial, folder in start_test.FOLDERS.items():
            with self.subTest(trial=trial):
                settings = yaml.safe_load((ROOT / "deployment/board_trials_4x4" / folder / "settings.yaml").read_text())
                site = ROOT / "deployment/site_20260928" / start_test.SITE_FILES.get(trial, "test_area.yaml")
                profile = yaml.safe_load(site.read_text())
                apply_site_profile(settings, profile)
                self.assertTrue(settings["motion_optimization"]["enabled"])
                self.assertNotIn("survey_pattern", settings)
                self.assertEqual(flight_geometry(settings)["survey_xy"], profile["flight_area"]["survey_xy"])
                options = motion_options(settings)
                self.assertTrue(options.enabled)
                if trial in {"landing", "corridor_landing", "memory_only", "high_speed_capture"}:
                    self.assertFalse(options.moving_recovery)
                self.assertEqual(settings.get("actuator_mode", "mock"), "mock")

    def test_cli_motion_enable_preserves_existing_suboptions(self):
        # Exercise the production CLI branch through its offline check path.
        import run_trial
        settings = yaml.safe_load((ROOT / "deployment/board_trials_4x4/06_high_priority/settings.yaml").read_text())
        settings["motion_optimization"].update(moving_recovery=False, recovery_min_samples=5)
        original_load = run_trial.yaml.safe_load
        captured = []

        def load(value):
            result = original_load(value)
            if isinstance(result, dict) and result.get("mode") == "high_priority":
                return copy.deepcopy(settings)
            return result

        def validate(result):
            captured.append(copy.deepcopy(result))
            validate_settings(result)

        with mock.patch.object(sys, "argv", ["run_trial.py", "high_priority", "preview", "--root", str(ROOT),
                                              "--site-config", str(ROOT / "deployment/site_20260928/test_area.yaml"),
                                              "--motion-optimized", "--check-config"]), \
                mock.patch.object(run_trial.yaml, "safe_load", side_effect=load), \
                mock.patch.object(run_trial, "validate_settings", side_effect=validate), \
                contextlib.redirect_stdout(io.StringIO()):
            run_trial.main()
        self.assertEqual(len(captured), 1)
        self.assertTrue(captured[0]["motion_optimization"]["enabled"])
        self.assertFalse(captured[0]["motion_optimization"]["moving_recovery"])
        self.assertEqual(captured[0]["motion_optimization"]["recovery_min_samples"], 5)


if __name__ == "__main__":
    unittest.main()
