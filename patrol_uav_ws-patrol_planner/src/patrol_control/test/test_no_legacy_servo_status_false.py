#!/usr/bin/env python3
"""Guard the fixed-drop log contract against the legacy false-status text."""

from pathlib import Path


def test_legacy_servo_status_false_text_is_absent():
    source = Path(__file__).resolve().parents[1] / "src" / "patrol_control.cpp"
    text = source.read_text(encoding="utf-8")
    assert "servo status false" not in text
    assert text.count("Awaiting positive Servo ACK; release remains blocked") == 3


def test_new_vision_config_uses_zero_dynamic_slot_offsets():
    config = (Path(__file__).resolve().parents[1] /
              "config" / "patrol_toudi3_new_vision.yaml").read_text(
                  encoding="utf-8")
    assert "dynamic_slot_offsets: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]" in config

    launch = (Path(__file__).resolve().parents[1] /
              "launch" / "toudi3_full_competition_sim_new_vision.launch").read_text(
                  encoding="utf-8")
    assert "patrol_toudi3_new_vision.yaml" in launch


if __name__ == "__main__":
    test_legacy_servo_status_false_text_is_absent()
    test_new_vision_config_uses_zero_dynamic_slot_offsets()
