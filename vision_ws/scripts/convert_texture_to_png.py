#!/usr/bin/env python3
"""Convert an image to a real PNG file.

This is intentionally small and dependency-light so it can be used for
Gazebo model assets whose filename extension does not match their MIME type.
Run it from the project's rl_drone conda environment.
"""

import argparse
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    image = cv2.imread(str(args.input), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"cannot decode image: {args.input}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(
        str(args.output), image, [cv2.IMWRITE_PNG_COMPRESSION, 9]
    ):
        raise RuntimeError(f"cannot write image: {args.output}")

    print(f"converted {args.input} -> {args.output}: shape={image.shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
