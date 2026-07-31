#!/usr/bin/env python3
"""Deterministic unit assertion for Aligning-triggered scene capture."""
import importlib.util
import os
import sys


def _load_recorder_module():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "aligning_scene_recorder.py")
    spec = importlib.util.spec_from_file_location(
        "aligning_scene_recorder_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    capture_gate = _load_recorder_module().CaptureGate(
        trigger_modes={"drop_circle"},
        required_control_state=2,
        frames_per_episode=3,
        min_interval_sec=0.5,
    )

    capture_gate.update("disabled", 1)
    assert capture_gate.request_capture(9.0) is None

    # A visual mode without the legacy Aligning state must not arm capture.
    capture_gate.update("drop_circle", 1)
    assert capture_gate.request_capture(9.5) is None

    # The transition to the complete old-control context starts episode 1.
    capture_gate.update("drop_circle", 2)
    assert capture_gate.request_capture(10.0) == (1, 1)
    assert capture_gate.request_capture(10.1) is None
    assert capture_gate.request_capture(10.5) == (1, 2)
    assert capture_gate.request_capture(11.0) == (1, 3)
    assert capture_gate.request_capture(11.5) is None

    # Remaining active does not create a second episode. Re-entering after
    # disabled does, and a backward /clock reset must not suppress frame 1.
    capture_gate.update("drop_circle", 2)
    assert capture_gate.request_capture(12.0) is None
    capture_gate.update("disabled", 1)
    capture_gate.update("drop_circle", 2)
    assert capture_gate.request_capture(1.0) == (2, 1)

    print("V-CL Aligning capture gate PASS")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # deterministic command-line failure
        print("V-CL Aligning capture gate FAIL: %s" % error, file=sys.stderr)
        sys.exit(1)
