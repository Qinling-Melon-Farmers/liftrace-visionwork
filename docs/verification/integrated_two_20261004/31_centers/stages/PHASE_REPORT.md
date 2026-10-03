# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-board-frame-fix/logs/integrated_two_seed31_20261004_011753

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 18.094 | 18.139 | 2.676 | (0.5158, -1.5786) | 0.0931 | random_bridge | 0.0931 | False |
| panzer | 21.349 | 21.422 | 2.677 | (0.3126, 3.1679) | 4.4991 | random_pillbox | 0.3163 | True |
| pillbox | 21.415 | 21.437 | 2.675 | (0.3099, 3.2047) | 0.2825 | random_pillbox | 0.2825 | False |
| panzer | 27.751 | 27.843 | 2.719 | (2.9404, -0.2741) | 0.1985 | random_panzer | 0.1985 | False |
| red_cross | 32.690 | 32.743 | 2.708 | (2.6537, -4.2971) | 0.1333 | random_red_cross | 0.1333 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 32.351 | 32.945 | 2.719 | (0.4973, -1.6879) | 0.1705 | random_bridge | 0.1705 | False |
| panzer | 26.196 | 32.945 | 2.753 | (0.3389, 3.3074) | 4.5928 | random_pillbox | 0.1765 | True |
| panzer | 31.104 | 32.945 | 2.756 | (2.9439, -0.6362) | 0.3461 | random_panzer | 0.3461 | False |
| pillbox | 26.148 | 32.945 | 2.753 | (0.3267, 3.2863) | 0.2007 | random_pillbox | 0.2007 | False |
| red_cross | 32.850 | 32.945 | 2.703 | (2.6500, -4.3407) | 0.1645 | random_red_cross | 0.1645 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 47.902 | 48.022 | 1.473 | (2.6874, -4.3125) | 0.1184 | random_red_cross | 0.1184 | False |
| bridge | 84.956 | 85.037 | 1.431 | (0.5208, -1.6655) | 0.1383 | random_bridge | 0.1383 | False |
| panzer | 97.553 | 97.632 | 1.339 | (3.1093, -0.4091) | 0.0689 | random_panzer | 0.0689 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 16 | 0.1335 | 0.1652 | 0.1335 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 87 | 0.2368 | 4.6663 | 0.2326 | 21 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 7 | 0.2498 | 0.2770 | 0.2498 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1468 | 0.1629 | 0.1468 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 22 | 0.1782 | 0.1829 | 0.1782 | 0 |
| hint_vision | HIGH_SURVEY | disabled | pillbox | 29 | 0.2395 | 0.2616 | 0.2395 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 2 | 0.1384 | 0.1385 | 0.1384 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 2 | 0.0689 | 0.0689 | 0.0689 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1185 | 0.1185 | 0.1185 | 0 |
| mapped | DELIVERY | disabled | bridge | 29 | 0.1093 | 0.1116 | 0.1093 | 0 |
| mapped | DELIVERY | disabled | panzer | 44 | 0.0647 | 0.0792 | 0.0647 | 0 |
| mapped | DELIVERY | disabled | red_cross | 35 | 0.1066 | 0.1127 | 0.1066 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 70 | 0.0864 | 0.1116 | 0.0864 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 75 | 0.0716 | 0.0962 | 0.0716 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 168 | 0.0885 | 0.1089 | 0.0885 | 0 |
| mapped | DESCEND | disabled | red_cross | 17 | 0.1617 | 0.1689 | 0.1617 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 67 | 0.1618 | 0.2546 | 0.1618 | 1 |
| mapped | HIGH_SURVEY | disabled | panzer | 28 | 4.6494 | 4.6626 | 0.1319 | 28 |
| mapped | HIGH_SURVEY | disabled | pillbox | 66 | 0.1845 | 0.2630 | 0.1845 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 3 | 0.1523 | 0.1557 | 0.1523 | 0 |
| mapped | LANDING | landing | landing_pad | 133 | 0.0383 | 0.0439 | 0.0383 | 0 |
| mapped | REACQUIRE | disabled | bridge | 2 | 0.1084 | 0.1085 | 0.1084 | 0 |
| mapped | REACQUIRE | disabled | panzer | 2 | 0.0691 | 0.0696 | 0.0691 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 4 | 0.1108 | 0.1114 | 0.1108 | 0 |
| mapped | REVISIT | disabled | bridge | 94 | 0.1033 | 0.1213 | 0.1033 | 0 |
| mapped | REVISIT | disabled | panzer | 34 | 0.0687 | 0.0824 | 0.0687 | 0 |
| mapped | REVISIT | disabled | red_cross | 148 | 0.1022 | 0.1083 | 0.1022 | 0 |
| mapped | TAIL | disabled | panzer | 39 | 0.0754 | 0.0939 | 0.0754 | 0 |
| selected | DELIVERY | disabled | bridge | 25 | 0.1354 | 0.1378 | 0.1354 | 0 |
| selected | DELIVERY | disabled | panzer | 41 | 0.0676 | 0.0689 | 0.0676 | 0 |
| selected | DELIVERY | disabled | red_cross | 31 | 0.1166 | 0.1179 | 0.1166 | 0 |
| selected | DELIVERY | drop_circle | bridge | 63 | 0.1288 | 0.1339 | 0.1288 | 0 |
| selected | DELIVERY | drop_circle | panzer | 69 | 0.0724 | 0.0744 | 0.0724 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 146 | 0.1094 | 0.1144 | 0.1094 | 0 |
| selected | DESCEND | disabled | red_cross | 15 | 0.1565 | 0.1594 | 0.1565 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 62 | 0.1769 | 0.1851 | 0.1769 | 0 |
| selected | HIGH_SURVEY | disabled | pillbox | 56 | 0.2351 | 0.2616 | 0.2351 | 0 |
| selected | REACQUIRE | disabled | bridge | 2 | 0.1384 | 0.1385 | 0.1384 | 0 |
| selected | REACQUIRE | disabled | panzer | 2 | 0.0689 | 0.0689 | 0.0689 | 0 |
| selected | REACQUIRE | disabled | red_cross | 4 | 0.1183 | 0.1185 | 0.1183 | 0 |
| selected | REVISIT | disabled | bridge | 92 | 0.1390 | 0.1619 | 0.1390 | 0 |
| selected | REVISIT | disabled | panzer | 32 | 0.0719 | 0.0861 | 0.0719 | 0 |
| selected | REVISIT | disabled | red_cross | 146 | 0.0961 | 0.1373 | 0.0961 | 0 |
| selected | TAIL | disabled | panzer | 39 | 0.0639 | 0.0652 | 0.0639 | 0 |
| targets | DELIVERY | disabled | bridge | 27 | 0.1353 | 0.1377 | 0.1353 | 0 |
| targets | DELIVERY | disabled | panzer | 43 | 0.0676 | 0.0689 | 0.0676 | 0 |
| targets | DELIVERY | disabled | red_cross | 33 | 0.1165 | 0.1179 | 0.1165 | 0 |
| targets | DELIVERY | drop_circle | bridge | 69 | 0.1285 | 0.1342 | 0.1285 | 0 |
| targets | DELIVERY | drop_circle | panzer | 73 | 0.0721 | 0.0744 | 0.0721 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 154 | 0.1092 | 0.1146 | 0.1092 | 0 |
| targets | DESCEND | disabled | red_cross | 16 | 0.1566 | 0.1593 | 0.1566 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 66 | 0.1766 | 0.1851 | 0.1766 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 2 | 4.5381 | 4.5397 | 0.2684 | 2 |
| targets | HIGH_SURVEY | disabled | pillbox | 56 | 0.2351 | 0.2616 | 0.2351 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 1 | 0.1561 | 0.1561 | 0.1561 | 0 |
| targets | LANDING | landing | landing_pad | 133 | 0.0357 | 0.0372 | 0.0357 | 0 |
| targets | REACQUIRE | disabled | bridge | 2 | 0.1384 | 0.1385 | 0.1384 | 0 |
| targets | REACQUIRE | disabled | panzer | 2 | 0.0689 | 0.0689 | 0.0689 | 0 |
| targets | REACQUIRE | disabled | red_cross | 4 | 0.1183 | 0.1185 | 0.1183 | 0 |
| targets | REVISIT | disabled | bridge | 94 | 0.1392 | 0.1635 | 0.1392 | 0 |
| targets | REVISIT | disabled | panzer | 34 | 0.0724 | 0.0923 | 0.0724 | 0 |
| targets | REVISIT | disabled | red_cross | 148 | 0.0961 | 0.1400 | 0.0961 | 0 |
| targets | TAIL | disabled | panzer | 39 | 0.0639 | 0.0652 | 0.0639 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.031852757326493425,
    "frame_id": "camera_init",
    "stamp_ns": 191842000000,
    "x": 8.46818390118491,
    "y": -4.201527745230359,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.031852757326493425,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.031852757326493425,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
