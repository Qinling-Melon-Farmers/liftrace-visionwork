#!/usr/bin/env python3
"""Offline contract tests for the RKNN input and raw-output adapters."""
import sys

import cv2
import numpy as np

sys.path.insert(0, "/home/xhj/liftrace/vision_ws/src/uav_vision/scripts")
from target_detector_rknn import (  # noqa: E402
    _candidate_arrays,
    _decode_outputs,
    _to_model_input,
)


def check(name, condition, detail=""):
    if not condition:
        raise AssertionError("%s: %s" % (name, detail))
    print("[PASS]", name)


def main():
    raw = np.zeros((1, 10, 8400), dtype=np.float32)
    candidates = _candidate_arrays([raw])
    check("channel-first raw output is transposed",
          len(candidates) == 1 and candidates[0].shape == (8400, 10),
          "got %s" % ([tuple(x.shape) for x in candidates],))

    raw[0, 0:4, 0] = [320.0, 320.0, 100.0, 100.0]
    raw[0, 4 + 2, 0] = 0.9
    detections = _decode_outputs(
        [raw],
        num_classes=6,
        conf_threshold=0.5,
        imgsz=640,
        orig_shape=(640, 640, 3),
        scale=1.0,
        pad=(0.0, 0.0),
    )
    check("channel-first output reaches YOLO decoder",
          len(detections) == 1 and detections[0]["class_id"] == 2,
          "got %s" % detections)

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :, 0] = 255
    tensor, _, _, _ = _to_model_input(
        image,
        640,
        layout="NHWC",
        color_space="RGB",
        input_dtype="uint8",
        normalize=False,
    )
    check("quantized input keeps uint8 bytes", tensor.dtype == np.uint8,
          "got %s" % tensor.dtype)
    check("quantized input is not divided by 255", int(tensor.max()) == 255,
          "max=%s" % tensor.max())
    check("input shape is NHWC batch", tensor.shape == (1, 640, 640, 3),
          "got %s" % (tensor.shape,))


if __name__ == "__main__":
    main()
