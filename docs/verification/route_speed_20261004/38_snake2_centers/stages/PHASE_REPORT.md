# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_snake2_seed38_20261005_004057

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 17.762 | 17.795 | 2.688 | (0.9902, -0.6114) | 0.1566 | random_red_cross | 0.1566 | False |
| bridge | 47.783 | 47.813 | 2.872 | (3.9307, 1.9067) | 0.1111 | random_bridge | 0.1111 | False |
| tent | 50.966 | 51.026 | 2.897 | (4.7557, -0.6826) | 0.1503 | random_tent | 0.1503 | False |
| bridge | 54.378 | 54.428 | 2.862 | (4.5485, -3.2942) | 5.1821 | random_pillbox | 0.0895 | True |
| panzer | 54.420 | 54.449 | 2.862 | (4.5569, -3.3134) | 2.6252 | random_pillbox | 0.0822 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 50.966 | 54.733 | 2.897 | (3.9258, 1.7460) | 0.1577 | random_bridge | 0.1577 | False |
| bridge | 54.518 | 54.733 | 2.861 | (4.5568, -3.3551) | 5.2436 | random_pillbox | 0.0986 | True |
| panzer | 54.624 | 54.733 | 2.861 | (4.5554, -3.3883) | 2.6043 | random_pillbox | 0.1215 | True |
| red_cross | 32.027 | 54.733 | 2.886 | (0.9577, -0.6175) | 0.1759 | random_red_cross | 0.1759 | False |
| tent | 54.420 | 54.733 | 2.862 | (4.8367, -0.9249) | 0.2253 | random_tent | 0.2253 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 78.540 | 78.661 | 1.374 | (0.9674, -0.6106) | 0.1652 | random_red_cross | 0.1652 | False |
| bridge | 102.287 | 102.425 | 1.318 | (3.9379, 1.7538) | 0.1439 | random_bridge | 0.1439 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 10 | 0.1244 | 5.2159 | 0.1164 | 2 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 4 | 2.6086 | 2.6234 | 0.0949 | 4 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 3 | 0.1578 | 0.1602 | 0.1578 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 9 | 0.1529 | 0.2014 | 0.1529 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 25 | 0.1542 | 0.1570 | 0.1542 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 43 | 0.1483 | 0.1735 | 0.1483 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 27 | 0.2069 | 0.2245 | 0.2069 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1439 | 0.1439 | 0.1439 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1258 | 0.1258 | 0.1258 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 1 | 0.1652 | 0.1652 | 0.1652 | 0 |
| mapped | DELIVERY | disabled | bridge | 40 | 0.0880 | 0.0994 | 0.0880 | 0 |
| mapped | DELIVERY | disabled | red_cross | 18 | 0.0921 | 0.0932 | 0.0921 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 87 | 0.0877 | 0.1043 | 0.0877 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 141 | 0.0837 | 0.1026 | 0.0837 | 0 |
| mapped | DESCEND | disabled | panzer | 41 | 2.5800 | 2.6198 | 0.1544 | 35 |
| mapped | DESCEND | disabled | pillbox | 9 | 0.1037 | 0.1408 | 0.1037 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 64 | 0.1607 | 0.1794 | 0.1604 | 1 |
| mapped | HIGH_SURVEY | disabled | panzer | 4 | 2.6215 | 2.6230 | 0.1319 | 4 |
| mapped | HIGH_SURVEY | disabled | red_cross | 147 | 0.1932 | 0.3204 | 0.1932 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 67 | 0.2396 | 0.2540 | 0.2396 | 0 |
| mapped | LANDING | landing | landing_pad | 65 | 0.0374 | 0.0439 | 0.0374 | 0 |
| mapped | LOW_COVERAGE | disabled | bridge | 73 | 0.0869 | 0.1112 | 0.0869 | 0 |
| mapped | LOW_COVERAGE | disabled | panzer | 60 | 0.0926 | 0.0984 | 0.0926 | 0 |
| mapped | LOW_COVERAGE | disabled | red_cross | 118 | 0.0808 | 0.0888 | 0.0808 | 0 |
| mapped | LOW_COVERAGE | drop_circle | panzer | 185 | 0.0866 | 0.1233 | 0.0866 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.1018 | 0.1022 | 0.1018 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 2 | 0.1240 | 0.1241 | 0.1240 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 2 | 0.0919 | 0.0920 | 0.0919 | 0 |
| mapped | REVISIT | disabled | bridge | 34 | 0.1056 | 0.1455 | 0.1056 | 0 |
| mapped | REVISIT | disabled | panzer | 1 | 2.6013 | 2.6013 | 0.0670 | 1 |
| mapped | REVISIT | disabled | pillbox | 180 | 0.1175 | 0.1251 | 0.1175 | 0 |
| mapped | REVISIT | disabled | red_cross | 81 | 0.0871 | 0.0934 | 0.0871 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 40 | 0.1274 | 0.1352 | 0.1274 | 0 |
| mapped | TAIL | disabled | panzer | 22 | 0.0845 | 0.0957 | 0.0845 | 0 |
| selected | DELIVERY | disabled | bridge | 37 | 0.1353 | 0.1419 | 0.1353 | 0 |
| selected | DELIVERY | disabled | red_cross | 17 | 0.1611 | 0.1640 | 0.1611 | 0 |
| selected | DELIVERY | drop_circle | bridge | 76 | 0.1207 | 0.1269 | 0.1207 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 133 | 0.1414 | 0.1551 | 0.1414 | 0 |
| selected | DESCEND | disabled | panzer | 31 | 2.5985 | 2.6206 | 0.1531 | 27 |
| selected | HIGH_SURVEY | disabled | bridge | 61 | 0.1551 | 0.1595 | 0.1551 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 1 | 2.6218 | 2.6218 | 0.1352 | 1 |
| selected | HIGH_SURVEY | disabled | red_cross | 105 | 0.1477 | 0.1767 | 0.1477 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 63 | 0.2127 | 0.2265 | 0.2127 | 0 |
| selected | LOW_COVERAGE | disabled | bridge | 71 | 0.1074 | 0.1103 | 0.1074 | 0 |
| selected | LOW_COVERAGE | disabled | panzer | 56 | 0.1106 | 0.1534 | 0.1106 | 0 |
| selected | LOW_COVERAGE | disabled | red_cross | 116 | 0.1130 | 0.1184 | 0.1130 | 0 |
| selected | LOW_COVERAGE | drop_circle | panzer | 176 | 0.1035 | 0.1078 | 0.1035 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.1434 | 0.1438 | 0.1434 | 0 |
| selected | REACQUIRE | disabled | pillbox | 2 | 0.1258 | 0.1258 | 0.1258 | 0 |
| selected | REACQUIRE | disabled | red_cross | 2 | 0.1650 | 0.1652 | 0.1650 | 0 |
| selected | REVISIT | disabled | bridge | 32 | 0.1520 | 0.1597 | 0.1520 | 0 |
| selected | REVISIT | disabled | panzer | 11 | 2.5900 | 2.5908 | 0.1277 | 11 |
| selected | REVISIT | disabled | pillbox | 167 | 0.1226 | 0.1263 | 0.1226 | 0 |
| selected | REVISIT | disabled | red_cross | 52 | 0.1216 | 0.1757 | 0.1216 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 37 | 0.1297 | 0.1321 | 0.1297 | 0 |
| selected | TAIL | disabled | panzer | 22 | 0.0900 | 0.0902 | 0.0900 | 0 |
| targets | DELIVERY | disabled | bridge | 39 | 0.1349 | 0.1418 | 0.1349 | 0 |
| targets | DELIVERY | disabled | red_cross | 17 | 0.1611 | 0.1640 | 0.1611 | 0 |
| targets | DELIVERY | drop_circle | bridge | 84 | 0.1205 | 0.1274 | 0.1205 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 137 | 0.1414 | 0.1557 | 0.1414 | 0 |
| targets | DESCEND | disabled | panzer | 35 | 2.5985 | 2.6206 | 0.1546 | 29 |
| targets | HIGH_SURVEY | disabled | bridge | 63 | 0.1551 | 0.1595 | 0.1551 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 3 | 2.6226 | 2.6232 | 0.1314 | 3 |
| targets | HIGH_SURVEY | disabled | red_cross | 107 | 0.1481 | 0.1767 | 0.1481 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 65 | 0.2116 | 0.2265 | 0.2116 | 0 |
| targets | LANDING | landing | landing_pad | 65 | 0.0418 | 0.0434 | 0.0418 | 0 |
| targets | LOW_COVERAGE | disabled | bridge | 73 | 0.1074 | 0.1102 | 0.1074 | 0 |
| targets | LOW_COVERAGE | disabled | panzer | 60 | 0.1106 | 0.1644 | 0.1106 | 0 |
| targets | LOW_COVERAGE | disabled | red_cross | 118 | 0.1131 | 0.1186 | 0.1131 | 0 |
| targets | LOW_COVERAGE | drop_circle | panzer | 185 | 0.1035 | 0.1078 | 0.1035 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.1434 | 0.1438 | 0.1434 | 0 |
| targets | REACQUIRE | disabled | pillbox | 2 | 0.1258 | 0.1258 | 0.1258 | 0 |
| targets | REACQUIRE | disabled | red_cross | 2 | 0.1650 | 0.1652 | 0.1650 | 0 |
| targets | REVISIT | disabled | bridge | 34 | 0.1526 | 0.1603 | 0.1526 | 0 |
| targets | REVISIT | disabled | panzer | 13 | 2.5901 | 2.5909 | 0.1283 | 13 |
| targets | REVISIT | disabled | pillbox | 167 | 0.1226 | 0.1263 | 0.1226 | 0 |
| targets | REVISIT | disabled | red_cross | 57 | 0.1218 | 0.1768 | 0.1218 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 39 | 0.1295 | 0.1321 | 0.1295 | 0 |
| targets | TAIL | disabled | panzer | 22 | 0.0900 | 0.0902 | 0.0900 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.028524339626802717,
    "frame_id": "camera_init",
    "stamp_ns": 392285000000,
    "x": 8.728612729924595,
    "y": -4.218873861021712,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.028524339626802717,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.028524339626802717,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
