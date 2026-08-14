#!/usr/bin/env python3
"""从未改动的单下视基线机架生成带斜下辅助相机的运行期派生 SDF。"""

import argparse
import math
import os
from pathlib import Path


ALLOWED_ANGLES = (45.0, 55.0, 60.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-sdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--angle-deg", required=True, type=float)
    parser.add_argument(
        "--sensor-mode", choices=("mono", "depth"), default="depth",
        help="mono 使用轻量 RGB 插件；depth 使用 RGB-D/点云插件")
    parser.add_argument(
        "--fragment",
        default="")
    args = parser.parse_args()

    if not any(abs(args.angle_deg - value) < 1.0e-6 for value in ALLOWED_ANGLES):
        parser.error("angle-deg must be one of 45, 55, 60")
    base_path = Path(args.base_sdf).resolve()
    model_dir = (Path(os.path.realpath(__file__)).parents[1] /
                 "models/iris_mid360_downward_aux_camera")
    fragment_path = Path(
        args.fragment or str(model_dir / (
            "aux_camera_fragment_rgb.sdf.inc"
            if args.sensor_mode == "mono" else
            "aux_camera_fragment.sdf.inc"))).resolve()
    output_path = Path(args.output).resolve()
    base = base_path.read_text(encoding="utf-8")
    if base.count("</model>") != 1:
        raise RuntimeError("base SDF must contain exactly one top-level model")
    if "downward_camera_link" not in base or "mid360_joint" not in base:
        raise RuntimeError("base SDF is not the expected downward-camera MID360 rack")
    fragment = fragment_path.read_text(encoding="utf-8").format(
        pitch_rad="{:.12f}".format(math.radians(args.angle_deg)))
    derived = base.replace(
        "  </model>", fragment.rstrip() + "\n  </model>", 1)
    derived = derived.replace(
        '<model name="iris_mid360_downward_camera">',
        '<model name="iris_mid360_downward_aux_camera_{:02d}">'.format(
            int(round(args.angle_deg))), 1)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text(derived, encoding="utf-8")
    os.replace(str(temporary), str(output_path))
    print(str(output_path))


if __name__ == "__main__":
    main()
