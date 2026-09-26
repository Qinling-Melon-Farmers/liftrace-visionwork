"""Offline parameter diagrams; no ROS, simulator, or flight configuration writes."""
import importlib.util
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Patch
import yaml

D = Path(__file__).resolve().parent
ROOT = D.parents[2]
MODULES = ROOT / "patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission"

def module(name):
    spec = importlib.util.spec_from_file_location(name, MODULES / (name + ".py"))
    loaded = importlib.util.module_from_spec(spec)
    sys.modules[name] = loaded
    spec.loader.exec_module(loaded)
    return loaded

def main():
    boundary_module = module("boundary_revisit")
    corridor_module = module("corridor_speed")
    speed_module = module("execution_speed")
    scene = ROOT / "docs/verification/fov_landing_inner_20260919/seed_2672"
    runtime = yaml.safe_load((scene / "fast_runtime.yaml").read_text())
    overrides = yaml.safe_load((scene / "frame_overrides.yaml").read_text())
    bounds = overrides["/navigation/mission_manager/high_view_full/boundary_policy/bounds"]
    boundary = boundary_module.BoundaryRevisit(enabled=True, bounds=tuple(bounds))
    config = corridor_module.CorridorSpeedConfig(**runtime["corridor_speed_schedule"])
    corridor = corridor_module.CorridorSpeed(config)
    following = speed_module.FollowingSpeed()
    near_distance = boundary.margin + boundary.max_view_offset_m
    landing = tuple(runtime["mission"]["landing_xy"])
    examples = []
    for name, point, completed in [
        ("entry descent", (6.7, 4.05), 1),
        ("open approach", (8.35, 2.7), 2),
        ("hysteresis retains fast", (8.35, 2.45), 3),
        ("door entered", (8.35, 2.3), 3),
        ("hysteresis retains slow", (8.35, 2.45), 3),
        ("door cleared", (8.35, 2.6), 3),
        ("H approach", (8.5, -3.6), 8),
    ]:
        phase, lead = corridor.select(point, landing, completed)
        examples.append(dict(name=name, xy=point, completed=completed,
                             phase=phase, lead_m=lead))
    data = {
        "source_commit": "c04cc48",
        "scope": "Current pure selectors and retained seed2672 fast scene; not flight validation",
        "bounds": bounds,
        "boundary_margin_m": boundary.margin,
        "boundary_near_distance_m": near_distance,
        "boundary_goal_example": {
            "xy": [6.8, 0], "near": boundary.near((6.8, 0)),
            "far_revisit_profile": following.select("SEARCH", "", 0, boundary.near((6.8, 0))),
            "low_coverage_far_is_slow": boundary.slow_coverage((2, 0), (6.8, 0)),
            "low_coverage_near_wall_is_slow": boundary.slow_coverage((6.75, 0), (6.8, 0)),
            "low_coverage_last_meter_is_slow": boundary.slow_coverage((5.8, 0), (6.7, 0)),
        },
        "corridor_config": runtime["corridor_speed_schedule"],
        "corridor_sequential_examples": examples,
    }
    (D / "policy_examples.json").write_text(json.dumps(data, indent=2) + "\n")
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 6.9), gridspec_kw={"width_ratios": [1.35, 1]})
    a, b, c, e = bounds
    ax = axes[0]
    ax.add_patch(Rectangle((a, c), b-a, e-c, facecolor="#fbd3c8", edgecolor="#555555", lw=1.5))
    ax.add_patch(Rectangle((a+near_distance, c+near_distance),
                           b-a-2*near_distance, e-c-2*near_distance,
                           facecolor="#ebf4fb", edgecolor="#bf654d", ls="--"))
    ax.plot(2, 0, "o", color="#145c86", ms=7)
    ax.plot(6.8, 0, "X", color="#a53027", ms=9)
    ax.annotate("", xy=(6.65, 0), xytext=(2.2, 0),
                arrowprops=dict(arrowstyle="->", color="#ad5423", lw=2))
    ax.text(2, -.5, "Start (example)", fontsize=10, ha="center")
    ax.text(6.8, .90, "Edge goal", fontsize=10, ha="right")
    ax.text(4.3, .35, "Entire REVISIT: L=0.20m", fontsize=9, ha="center")
    ax.text((a+b)/2, 2.6, "Interior REVISIT: L=1.00m", ha="center", fontsize=11)
    ax.text((a+b)/2, 4.15, f"Goal near boundary: clearance < {near_distance:.4f}m",
            ha="center", fontsize=9)
    ax.plot(0, 0, "s", color="#333333", ms=4)
    ax.annotate("", xy=(2.4, -2.6), xytext=(.7, -2.6),
                arrowprops=dict(arrowstyle="->"))
    ax.text(2.5, -2.6, "+X / initial heading", fontsize=9, va="center")
    ax.annotate("", xy=(.7, -1.2), xytext=(.7, -2.6),
                arrowprops=dict(arrowstyle="->"))
    ax.text(.7, -1.05, "+Y", fontsize=9, ha="center")
    ax.set_xlim(a-.4, b+.4); ax.set_ylim(c-.35, e+.35)
    ax.set_aspect("equal"); ax.set_xlabel("Fixed launch-frame X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("REVISIT: goal-based slow classification", fontsize=11)
    ax.grid(alpha=.2); ax.set_axisbelow(True)
    ay = axes[1]
    ay.set_facecolor("#dceffc")
    for wall in config.wall_coordinates:
        ay.axhspan(wall-config.exit_distance_m, wall+config.exit_distance_m, color="#ffe6b3")
        ay.axhspan(wall-config.enter_distance_m, wall+config.enter_distance_m, color="#f6beb7")
        ay.axhline(wall, color="#333333", lw=1.4, ls="--")
        ay.text(.03, wall+.06, f"Wall plane Y={wall:+.1f}m", fontsize=10)
    ay.text(.5, 3.5, "OPEN: L=0.40m", ha="center", fontsize=11)
    ay.text(.5, 0, "OPEN: L=0.40m", ha="center", fontsize=11)
    ay.text(.5, -3.5, "H within 0.80m takes priority\n(two-dimensional distance)", ha="center", fontsize=10)
    ay.set_ylim(c-.35, e+.35); ay.set_xlim(0, 1); ay.set_xticks([])
    ay.set_ylabel("Corridor Y (m)"); ay.set_title("After staging + descent: pose-based selection", fontsize=11)
    ay.legend(handles=[
        Patch(facecolor="#f6beb7", label="d <=0.75m: DOOR, L=0.15m"),
        Patch(facecolor="#ffe6b3", label="0.75<d<0.95m: retain prior state"),
        Patch(facecolor="#dceffc", label="d >=0.95m: OPEN, L=0.40m")],
        loc="upper center", bbox_to_anchor=(.5, -.08), fontsize=9)
    fig.suptitle("Current speed-selection geometry | L is a distance, not m/s", fontsize=13)
    fig.text(.5, .016, "Policy diagram only; no obstacles or flyability assessment. LOW_COVERAGE uses a different local slowdown rule.",
             ha="center", fontsize=9)
    fig.tight_layout(rect=(0, .08, 1, .95))
    fig.savefig(D / "speed_zones.png", dpi=160)
    plt.close(fig)
    print(json.dumps(data, indent=2))

if __name__ == "__main__":
    main()
