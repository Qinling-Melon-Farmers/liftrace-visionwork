#!/usr/bin/env python3
"""Count planner failure signatures in the frozen seed31-40 run logs."""

import json
import re
from pathlib import Path


REPORT_DIR = Path(__file__).resolve().parent
ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
PATTERNS = {
    "goal_occupied_or_outside": "kinodynamic goal occupied or outside map",
    "goal_neighborhood_blocked": "No collision-free goal within requested waypoint neighborhood",
    "start_occupied": "start point is occupied",
    "kino_search_fail": "kinodynamic search fail",
    "optimization_exception": "nlopt exception",
    "trajectory_collision": "current traj in collision",
    "pose_discontinuity": "pose discontinuity",
    "no_progress_replan": "no_physical_progress_replan",
    "liveness_exhausted": "liveness_budget_exhausted",
}


def main() -> None:
    items = json.loads((REPORT_DIR / "runs.json").read_text(encoding="utf-8"))
    rows = []
    for item in items:
        run = Path(item["run"])
        paths = [run / "run.log", *run.glob("roslog/**/*.log")]
        text = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in paths
            if path.is_file()
        )
        row = {"seed": item["seed"], "run": str(run)}
        row.update({key: text.count(value) for key, value in PATTERNS.items()})
        row["goal_blocked_examples"] = [
            ANSI_ESCAPE.sub("", line).strip()
            for line in text.splitlines()
            if "No collision-free goal" in line
        ][:3]
        rows.append(row)

    output = REPORT_DIR / "planner_warning_counts.json"
    output.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
