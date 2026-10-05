# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/seed38_resume_resume_off_seed38_20261005_161407

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 16.465 | 16.527 | 2.055 | (0.9996, -0.5847) | 0.1283 | random_red_cross | 0.1283 | False |
| bridge | 45.331 | 45.361 | 2.232 | (3.9197, 1.9931) | 0.1728 | random_bridge | 0.1728 | False |
| tent | 54.368 | 54.413 | 2.315 | (4.3158, -3.4871) | 2.8389 | random_pillbox | 0.3730 | True |
| pillbox | 54.513 | 54.605 | 2.311 | (4.3623, -3.4774) | 0.3282 | random_pillbox | 0.3282 | False |
| panzer | 54.779 | 54.935 | 2.301 | (4.4545, -3.4622) | 2.6817 | random_pillbox | 0.2455 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 48.118 | 56.680 | 2.311 | (3.9412, 1.7818) | 0.1216 | random_bridge | 0.1216 | False |
| panzer | 56.514 | 56.680 | 2.268 | (4.5577, -3.4555) | 2.5837 | random_pillbox | 0.1758 | True |
| pillbox | 56.239 | 56.680 | 2.267 | (4.5753, -3.4358) | 0.1503 | random_pillbox | 0.1503 | False |
| red_cross | 51.307 | 56.680 | 2.333 | (0.9786, -0.5925) | 0.1440 | random_red_cross | 0.1440 | False |
| tent | 54.456 | 56.680 | 2.313 | (4.3453, -3.4660) | 2.8122 | random_pillbox | 0.3368 | True |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 82.787 | 82.961 | 1.417 | (0.9846, -0.5829) | 0.1327 | random_red_cross | 0.1327 | False |
| bridge | 105.994 | 106.111 | 1.317 | (3.9672, 1.7758) | 0.1082 | random_bridge | 0.1082 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 9 | 0.1206 | 0.1664 | 0.1206 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 15 | 2.5828 | 2.6779 | 0.1654 | 15 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 6 | 0.2458 | 0.3202 | 0.2458 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1336 | 0.1346 | 0.1336 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 3 | 2.8243 | 2.8374 | 0.3543 | 3 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 19 | 0.1117 | 0.1202 | 0.1117 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 2 | 2.5849 | 2.5860 | 0.1766 | 2 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 51 | 0.1324 | 0.1536 | 0.1324 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1082 | 0.1082 | 0.1082 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 4 | 0.1545 | 0.1548 | 0.1545 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 3 | 0.1329 | 0.1330 | 0.1329 | 0 |
| mapped | DELIVERY | disabled | bridge | 33 | 0.0758 | 0.0788 | 0.0758 | 0 |
| mapped | DELIVERY | disabled | red_cross | 17 | 0.0971 | 0.1002 | 0.0971 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 77 | 0.0918 | 0.1164 | 0.0918 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 107 | 0.1040 | 0.1223 | 0.1040 | 0 |
| mapped | DESCEND | disabled | panzer | 4 | 2.5573 | 2.5616 | 0.1550 | 4 |
| mapped | DESCEND | disabled | pillbox | 21 | 0.1539 | 0.1951 | 0.1539 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 55 | 0.1238 | 0.1791 | 0.1238 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 20 | 2.5794 | 2.6006 | 0.1701 | 20 |
| mapped | HIGH_SURVEY | disabled | pillbox | 4 | 0.1695 | 0.1855 | 0.1695 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 147 | 0.1294 | 0.2519 | 0.1294 | 0 |
| mapped | LANDING | landing | landing_pad | 64 | 0.0419 | 0.0467 | 0.0419 | 0 |
| mapped | LOW_COVERAGE | disabled | bridge | 39 | 0.0879 | 0.1425 | 0.0879 | 0 |
| mapped | LOW_COVERAGE | disabled | panzer | 72 | 0.0819 | 0.0937 | 0.0819 | 0 |
| mapped | LOW_COVERAGE | disabled | red_cross | 108 | 0.0655 | 0.0737 | 0.0655 | 0 |
| mapped | LOW_COVERAGE | disabled | tent | 6 | 0.0633 | 0.0679 | 0.0633 | 0 |
| mapped | LOW_COVERAGE | drop_circle | panzer | 157 | 0.0880 | 0.2981 | 0.0880 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.0749 | 0.0749 | 0.0749 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 4 | 0.1258 | 0.1266 | 0.1258 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 5 | 0.0965 | 0.0982 | 0.0965 | 0 |
| mapped | REVISIT | disabled | bridge | 34 | 0.0756 | 0.0909 | 0.0756 | 0 |
| mapped | REVISIT | disabled | pillbox | 219 | 0.1187 | 0.1961 | 0.1187 | 0 |
| mapped | REVISIT | disabled | red_cross | 65 | 0.0958 | 0.1467 | 0.0958 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 36 | 0.1166 | 0.1223 | 0.1166 | 0 |
| mapped | TAIL | disabled | bridge | 13 | 0.0645 | 0.0688 | 0.0645 | 0 |
| mapped | TAIL | disabled | panzer | 26 | 0.0668 | 0.0791 | 0.0668 | 0 |
| selected | DELIVERY | disabled | bridge | 29 | 0.1026 | 0.1066 | 0.1026 | 0 |
| selected | DELIVERY | disabled | red_cross | 16 | 0.1308 | 0.1320 | 0.1308 | 0 |
| selected | DELIVERY | drop_circle | bridge | 68 | 0.0981 | 0.0990 | 0.0981 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 94 | 0.1269 | 0.1288 | 0.1269 | 0 |
| selected | DESCEND | disabled | pillbox | 18 | 0.1622 | 0.1657 | 0.1622 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 53 | 0.1137 | 0.1274 | 0.1137 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 6 | 2.5786 | 2.5902 | 0.1731 | 6 |
| selected | HIGH_SURVEY | disabled | red_cross | 125 | 0.1351 | 0.1539 | 0.1351 | 0 |
| selected | LOW_COVERAGE | disabled | bridge | 39 | 0.0939 | 0.0954 | 0.0939 | 0 |
| selected | LOW_COVERAGE | disabled | panzer | 67 | 0.0733 | 0.0797 | 0.0733 | 0 |
| selected | LOW_COVERAGE | disabled | red_cross | 106 | 0.1099 | 0.1156 | 0.1099 | 0 |
| selected | LOW_COVERAGE | disabled | tent | 4 | 0.0603 | 0.0618 | 0.0603 | 0 |
| selected | LOW_COVERAGE | drop_circle | panzer | 144 | 0.0832 | 0.0850 | 0.0832 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.1079 | 0.1082 | 0.1079 | 0 |
| selected | REACQUIRE | disabled | pillbox | 4 | 0.1545 | 0.1548 | 0.1545 | 0 |
| selected | REACQUIRE | disabled | red_cross | 5 | 0.1327 | 0.1330 | 0.1327 | 0 |
| selected | REVISIT | disabled | bridge | 32 | 0.1159 | 0.1258 | 0.1159 | 0 |
| selected | REVISIT | disabled | pillbox | 217 | 0.1511 | 0.1764 | 0.1511 | 0 |
| selected | REVISIT | disabled | red_cross | 44 | 0.1344 | 0.1398 | 0.1344 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 31 | 0.1144 | 0.1178 | 0.1144 | 0 |
| selected | TAIL | disabled | bridge | 11 | 0.0940 | 0.0946 | 0.0940 | 0 |
| selected | TAIL | disabled | panzer | 26 | 0.0741 | 0.0746 | 0.0741 | 0 |
| targets | DELIVERY | disabled | bridge | 31 | 0.1023 | 0.1066 | 0.1023 | 0 |
| targets | DELIVERY | disabled | red_cross | 16 | 0.1308 | 0.1320 | 0.1308 | 0 |
| targets | DELIVERY | drop_circle | bridge | 75 | 0.0981 | 0.0990 | 0.0981 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 100 | 0.1268 | 0.1290 | 0.1268 | 0 |
| targets | DESCEND | disabled | pillbox | 19 | 0.1625 | 0.1665 | 0.1625 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 55 | 0.1134 | 0.1274 | 0.1134 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 11 | 2.5832 | 2.6023 | 0.1756 | 11 |
| targets | HIGH_SURVEY | disabled | red_cross | 129 | 0.1351 | 0.1545 | 0.1351 | 0 |
| targets | LANDING | landing | landing_pad | 64 | 0.0447 | 0.0466 | 0.0447 | 0 |
| targets | LOW_COVERAGE | disabled | bridge | 39 | 0.0939 | 0.0954 | 0.0939 | 0 |
| targets | LOW_COVERAGE | disabled | panzer | 71 | 0.0733 | 0.0796 | 0.0733 | 0 |
| targets | LOW_COVERAGE | disabled | red_cross | 108 | 0.1100 | 0.1159 | 0.1100 | 0 |
| targets | LOW_COVERAGE | disabled | tent | 6 | 0.0591 | 0.0617 | 0.0591 | 0 |
| targets | LOW_COVERAGE | drop_circle | panzer | 156 | 0.0831 | 0.0850 | 0.0831 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.1079 | 0.1082 | 0.1079 | 0 |
| targets | REACQUIRE | disabled | pillbox | 4 | 0.1545 | 0.1548 | 0.1545 | 0 |
| targets | REACQUIRE | disabled | red_cross | 5 | 0.1327 | 0.1330 | 0.1327 | 0 |
| targets | REVISIT | disabled | bridge | 34 | 0.1165 | 0.1275 | 0.1165 | 0 |
| targets | REVISIT | disabled | pillbox | 219 | 0.1514 | 0.1764 | 0.1514 | 0 |
| targets | REVISIT | disabled | red_cross | 50 | 0.1342 | 0.1404 | 0.1342 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 33 | 0.1148 | 0.1178 | 0.1148 | 0 |
| targets | TAIL | disabled | bridge | 13 | 0.0942 | 0.0949 | 0.0942 | 0 |
| targets | TAIL | disabled | panzer | 26 | 0.0741 | 0.0746 | 0.0741 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.02776548414036502,
    "frame_id": "camera_init",
    "stamp_ns": 394073000000,
    "x": 8.729093366081067,
    "y": -4.218271145769452,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.02776548414036502,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.02776548414036502,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
