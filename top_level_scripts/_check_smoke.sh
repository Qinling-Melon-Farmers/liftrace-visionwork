#!/bin/bash
python3 - <<'PY'
import json
for name, path in [("NEW", "/tmp/smoke_new.json"),
                   ("LEGACY_RKNN", "/tmp/smoke_legacy.json"),
                   ("LEGACY_PT", "/tmp/smoke_pt.json")]:
    d = json.load(open(path))
    print(name, "class_hist:", d["derived"]["class_hist_total"],
          "| first_det:", d["derived"]["first_detections"][:2])
PY
