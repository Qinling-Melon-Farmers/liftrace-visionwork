#!/usr/bin/env python3
"""输出斜下相机视场的可复算几何表。"""

import argparse
import json
import math


def geometry(angle_deg, height, horizontal_fov_deg=60.0, aspect=4.0 / 3.0):
    horizontal_fov = math.radians(horizontal_fov_deg)
    vertical_fov = 2.0 * math.atan(math.tan(horizontal_fov / 2.0) / aspect)
    depression = math.radians(angle_deg)
    near_angle = min(math.pi / 2.0 - 1.0e-6, depression + vertical_fov / 2.0)
    far_angle = depression - vertical_fov / 2.0
    if far_angle <= 0.0:
        far = None
    else:
        far = height / math.tan(far_angle)
    near = height / math.tan(near_angle)
    center_distance = height / math.tan(depression)
    center_width = 2.0 * (height / math.sin(depression)) * math.tan(horizontal_fov / 2.0)
    return {
        "angle_deg": angle_deg,
        "height_m": height,
        "horizontal_fov_deg": horizontal_fov_deg,
        "vertical_fov_deg": math.degrees(vertical_fov),
        "forward_near_m": near,
        "forward_far_m": far,
        "center_ground_distance_m": center_distance,
        "center_row_width_m": center_width,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--angles", nargs="+", type=float, default=[45, 55, 60])
    parser.add_argument("--heights", nargs="+", type=float, default=[2.0, 3.0, 3.5])
    parser.add_argument("--output", default="-")
    args = parser.parse_args()
    rows = [geometry(angle, height)
            for angle in args.angles for height in args.heights]
    text = json.dumps({"rows": rows}, ensure_ascii=False, indent=2) + "\n"
    if args.output == "-":
        print(text, end="")
    else:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(text)
