#!/usr/bin/env python3
"""Guard the fixed-drop log contract against the legacy false-status text."""

from pathlib import Path


def test_legacy_servo_status_false_text_is_absent():
    source = Path(__file__).resolve().parents[1] / "src" / "patrol_control.cpp"
    text = source.read_text(encoding="utf-8")
    assert "servo status false" not in text
    assert text.count("Awaiting positive Servo ACK; release remains blocked") == 3


if __name__ == "__main__":
    test_legacy_servo_status_false_text_is_absent()
