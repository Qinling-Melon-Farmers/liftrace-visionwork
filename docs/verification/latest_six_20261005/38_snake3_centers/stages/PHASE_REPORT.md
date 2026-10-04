# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/latest_six_snake3_seed38_20261005_035440

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 17.607 | 17.668 | 2.628 | (0.9952, -0.6166) | 0.1598 | random_red_cross | 0.1598 | False |
| bridge | 45.101 | 45.160 | 2.761 | (3.9291, 1.9010) | 0.1105 | random_bridge | 0.1105 | False |
| tent | 48.521 | 48.580 | 2.845 | (4.7987, -0.6812) | 0.1087 | random_tent | 0.1087 | False |
| panzer | 51.977 | 52.021 | 2.834 | (4.5537, -3.2821) | 2.6383 | random_pillbox | 0.0859 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 49.267 | 52.312 | 2.853 | (3.9277, 1.7377) | 0.1627 | random_bridge | 0.1627 | False |
| panzer | 52.099 | 52.312 | 2.836 | (4.5461, -3.3646) | 2.6200 | random_pillbox | 0.1127 | True |
| red_cross | 33.462 | 52.312 | 2.860 | (0.9657, -0.6258) | 0.1795 | random_red_cross | 0.1795 | False |
| tent | 51.977 | 52.312 | 2.834 | (4.7990, -0.8649) | 0.1870 | random_tent | 0.1870 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 78.339 | 78.461 | 1.375 | (0.9697, -0.6203) | 0.1728 | random_red_cross | 0.1728 | False |
| bridge | 102.439 | 102.528 | 1.282 | (3.9483, 1.7511) | 0.1393 | random_bridge | 0.1393 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 9 | 0.1118 | 0.1378 | 0.1118 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 3 | 2.6339 | 2.6378 | 0.0900 | 3 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1640 | 0.1672 | 0.1640 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 9 | 0.1816 | 0.2048 | 0.1816 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 25 | 0.1401 | 0.1605 | 0.1401 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 45 | 0.1470 | 0.1761 | 0.1470 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 24 | 0.1848 | 0.1866 | 0.1848 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 2 | 0.1395 | 0.1397 | 0.1395 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1431 | 0.1431 | 0.1431 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 1 | 0.1728 | 0.1728 | 0.1728 | 0 |
| mapped | DELIVERY | disabled | bridge | 29 | 0.0940 | 0.0968 | 0.0940 | 0 |
| mapped | DELIVERY | disabled | red_cross | 21 | 0.0911 | 0.0955 | 0.0911 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 76 | 0.0887 | 0.1002 | 0.0887 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 136 | 0.0790 | 0.1015 | 0.0790 | 0 |
| mapped | DESCEND | disabled | panzer | 35 | 2.5665 | 2.6117 | 0.2032 | 35 |
| mapped | DESCEND | disabled | pillbox | 6 | 0.1824 | 0.2047 | 0.1824 | 0 |
| mapped | DESCEND | disabled | tent | 1 | 2.8043 | 2.8043 | 0.2079 | 1 |
| mapped | HIGH_SURVEY | disabled | bridge | 70 | 0.1684 | 0.2054 | 0.1684 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 4 | 2.6307 | 2.6322 | 0.1415 | 4 |
| mapped | HIGH_SURVEY | disabled | red_cross | 170 | 0.2010 | 0.3197 | 0.2010 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 70 | 0.1877 | 0.2497 | 0.1877 | 0 |
| mapped | LANDING | landing | landing_pad | 81 | 0.0465 | 0.0543 | 0.0465 | 0 |
| mapped | LOW_COVERAGE | disabled | bridge | 74 | 0.0922 | 0.1175 | 0.0922 | 0 |
| mapped | LOW_COVERAGE | disabled | panzer | 63 | 0.0998 | 0.1026 | 0.0998 | 0 |
| mapped | LOW_COVERAGE | disabled | red_cross | 131 | 0.0714 | 0.0796 | 0.0714 | 0 |
| mapped | LOW_COVERAGE | drop_circle | panzer | 171 | 0.0997 | 0.1067 | 0.0997 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.0971 | 0.0972 | 0.0971 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 2 | 0.1271 | 0.1273 | 0.1271 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 1 | 0.0892 | 0.0892 | 0.0892 | 0 |
| mapped | REVISIT | disabled | bridge | 44 | 0.0945 | 0.1162 | 0.0945 | 0 |
| mapped | REVISIT | disabled | pillbox | 195 | 0.1163 | 0.1268 | 0.1163 | 0 |
| mapped | REVISIT | disabled | red_cross | 66 | 0.0920 | 0.1248 | 0.0920 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 46 | 0.1139 | 0.1174 | 0.1139 | 0 |
| mapped | TAIL | disabled | panzer | 28 | 0.0676 | 0.0800 | 0.0676 | 0 |
| selected | DELIVERY | disabled | bridge | 26 | 0.1345 | 0.1382 | 0.1345 | 0 |
| selected | DELIVERY | disabled | red_cross | 21 | 0.1684 | 0.1719 | 0.1684 | 0 |
| selected | DELIVERY | drop_circle | bridge | 67 | 0.1229 | 0.1291 | 0.1229 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 124 | 0.1490 | 0.1621 | 0.1490 | 0 |
| selected | DESCEND | disabled | panzer | 22 | 2.5741 | 2.6112 | 0.2043 | 22 |
| selected | DESCEND | disabled | pillbox | 1 | 0.2041 | 0.2041 | 0.2041 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 68 | 0.1447 | 0.1662 | 0.1447 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 2 | 2.6229 | 2.6241 | 0.1414 | 2 |
| selected | HIGH_SURVEY | disabled | red_cross | 126 | 0.1469 | 0.1796 | 0.1469 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 62 | 0.1862 | 0.1887 | 0.1862 | 0 |
| selected | LOW_COVERAGE | disabled | bridge | 72 | 0.1096 | 0.1131 | 0.1096 | 0 |
| selected | LOW_COVERAGE | disabled | panzer | 58 | 0.1000 | 0.1059 | 0.1000 | 0 |
| selected | LOW_COVERAGE | disabled | red_cross | 129 | 0.1206 | 0.1282 | 0.1206 | 0 |
| selected | LOW_COVERAGE | drop_circle | panzer | 160 | 0.1004 | 0.1014 | 0.1004 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.1393 | 0.1397 | 0.1393 | 0 |
| selected | REACQUIRE | disabled | pillbox | 2 | 0.1432 | 0.1433 | 0.1432 | 0 |
| selected | REACQUIRE | disabled | red_cross | 1 | 0.1728 | 0.1728 | 0.1728 | 0 |
| selected | REVISIT | disabled | bridge | 42 | 0.1501 | 0.1648 | 0.1501 | 0 |
| selected | REVISIT | disabled | pillbox | 192 | 0.1332 | 0.1603 | 0.1332 | 0 |
| selected | REVISIT | disabled | red_cross | 44 | 0.1309 | 0.1812 | 0.1309 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 44 | 0.1140 | 0.1152 | 0.1140 | 0 |
| selected | TAIL | disabled | panzer | 28 | 0.0891 | 0.0902 | 0.0891 | 0 |
| targets | DELIVERY | disabled | bridge | 28 | 0.1342 | 0.1382 | 0.1342 | 0 |
| targets | DELIVERY | disabled | red_cross | 21 | 0.1684 | 0.1719 | 0.1684 | 0 |
| targets | DELIVERY | drop_circle | bridge | 73 | 0.1228 | 0.1296 | 0.1228 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 130 | 0.1488 | 0.1626 | 0.1488 | 0 |
| targets | DESCEND | disabled | panzer | 22 | 2.5741 | 2.6112 | 0.2043 | 22 |
| targets | DESCEND | disabled | pillbox | 1 | 0.2041 | 0.2041 | 0.2041 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 70 | 0.1443 | 0.1661 | 0.1443 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 4 | 2.6197 | 2.6239 | 0.1405 | 4 |
| targets | HIGH_SURVEY | disabled | red_cross | 128 | 0.1470 | 0.1795 | 0.1470 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 65 | 0.1862 | 0.1888 | 0.1862 | 0 |
| targets | LANDING | landing | landing_pad | 81 | 0.0500 | 0.0544 | 0.0500 | 0 |
| targets | LOW_COVERAGE | disabled | bridge | 74 | 0.1096 | 0.1130 | 0.1096 | 0 |
| targets | LOW_COVERAGE | disabled | panzer | 62 | 0.1000 | 0.1096 | 0.1000 | 0 |
| targets | LOW_COVERAGE | disabled | red_cross | 131 | 0.1207 | 0.1285 | 0.1207 | 0 |
| targets | LOW_COVERAGE | drop_circle | panzer | 169 | 0.1002 | 0.1014 | 0.1002 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.1393 | 0.1397 | 0.1393 | 0 |
| targets | REACQUIRE | disabled | pillbox | 2 | 0.1432 | 0.1433 | 0.1432 | 0 |
| targets | REACQUIRE | disabled | red_cross | 1 | 0.1728 | 0.1728 | 0.1728 | 0 |
| targets | REVISIT | disabled | bridge | 44 | 0.1506 | 0.1669 | 0.1506 | 0 |
| targets | REVISIT | disabled | pillbox | 193 | 0.1333 | 0.1616 | 0.1333 | 0 |
| targets | REVISIT | disabled | red_cross | 50 | 0.1310 | 0.1822 | 0.1310 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 46 | 0.1139 | 0.1152 | 0.1139 | 0 |
| targets | TAIL | disabled | panzer | 28 | 0.0891 | 0.0902 | 0.0891 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.023566037903638617,
    "frame_id": "camera_init",
    "stamp_ns": 413621000000,
    "x": 8.77267692803625,
    "y": -4.2064121039694085,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.023566037903638617,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.023566037903638617,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
