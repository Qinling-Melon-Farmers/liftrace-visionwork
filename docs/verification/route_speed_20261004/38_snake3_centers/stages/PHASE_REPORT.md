# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_snake3_seed38_20261005_010738

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 17.109 | 17.209 | 2.631 | (0.9836, -0.6234) | 0.1701 | random_red_cross | 0.1701 | False |
| bridge | 45.250 | 45.288 | 2.815 | (3.9174, 1.9354) | 0.1364 | random_bridge | 0.1364 | False |
| tent | 48.289 | 48.374 | 2.860 | (4.7769, -0.6793) | 0.1303 | random_tent | 0.1303 | False |
| bridge | 51.312 | 51.372 | 2.871 | (4.5459, -3.2137) | 5.1017 | random_pillbox | 0.1256 | True |
| panzer | 51.347 | 51.372 | 2.871 | (4.5486, -3.2272) | 2.6612 | random_pillbox | 0.1147 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 48.441 | 51.661 | 2.857 | (3.9003, 1.7629) | 0.1653 | random_bridge | 0.1653 | False |
| bridge | 51.472 | 51.661 | 2.870 | (4.5473, -3.2803) | 5.1682 | random_pillbox | 0.0925 | True |
| panzer | 51.530 | 51.661 | 2.869 | (4.5541, -3.3084) | 2.6295 | random_pillbox | 0.0843 | True |
| red_cross | 31.957 | 51.661 | 2.890 | (0.9607, -0.6304) | 0.1859 | random_red_cross | 0.1859 | False |
| tent | 51.272 | 51.661 | 2.871 | (4.8030, -0.8463) | 0.1696 | random_tent | 0.1696 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 78.158 | 78.214 | 1.364 | (0.9669, -0.6191) | 0.1730 | random_red_cross | 0.1730 | False |
| bridge | 100.136 | 100.246 | 1.299 | (3.9334, 1.7644) | 0.1393 | random_bridge | 0.1393 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 11 | 0.1364 | 5.1592 | 0.1256 | 3 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 2 | 2.6453 | 2.6596 | 0.0995 | 2 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1708 | 0.1737 | 0.1708 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 8 | 0.1300 | 0.1515 | 0.1300 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 23 | 0.1528 | 0.1640 | 0.1528 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 41 | 0.1560 | 0.1831 | 0.1560 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 22 | 0.1633 | 0.1683 | 0.1633 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 2 | 0.1394 | 0.1396 | 0.1394 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 2 | 0.1070 | 0.1070 | 0.1070 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 1 | 0.1730 | 0.1730 | 0.1730 | 0 |
| mapped | DELIVERY | disabled | bridge | 40 | 0.1105 | 0.1167 | 0.1105 | 0 |
| mapped | DELIVERY | disabled | red_cross | 15 | 0.0844 | 0.0896 | 0.0844 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 53 | 0.1115 | 0.3702 | 0.1115 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 114 | 0.0829 | 0.1011 | 0.0829 | 0 |
| mapped | DESCEND | disabled | panzer | 33 | 2.5897 | 2.6352 | 0.1357 | 33 |
| mapped | DESCEND | disabled | pillbox | 8 | 0.1491 | 0.1544 | 0.1491 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 62 | 0.1713 | 0.2288 | 0.1685 | 2 |
| mapped | HIGH_SURVEY | disabled | panzer | 3 | 2.6468 | 2.6473 | 0.1135 | 3 |
| mapped | HIGH_SURVEY | disabled | red_cross | 155 | 0.1940 | 0.2977 | 0.1940 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 63 | 0.1682 | 0.2131 | 0.1682 | 0 |
| mapped | LOW_COVERAGE | disabled | bridge | 35 | 0.0671 | 0.0969 | 0.0671 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.1075 | 0.1075 | 0.1075 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 3 | 0.1099 | 0.1110 | 0.1099 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 1 | 0.0825 | 0.0825 | 0.0825 | 0 |
| mapped | REVISIT | disabled | bridge | 36 | 0.0945 | 0.1062 | 0.0945 | 0 |
| mapped | REVISIT | disabled | panzer | 1 | 2.6162 | 2.6162 | 0.0642 | 1 |
| mapped | REVISIT | disabled | pillbox | 184 | 0.1209 | 0.1332 | 0.1209 | 0 |
| mapped | REVISIT | disabled | red_cross | 50 | 0.0799 | 0.1094 | 0.0799 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 21 | 0.1092 | 0.1251 | 0.1092 | 0 |
| selected | DELIVERY | disabled | bridge | 36 | 0.1341 | 0.1381 | 0.1341 | 0 |
| selected | DELIVERY | disabled | red_cross | 13 | 0.1691 | 0.1721 | 0.1691 | 0 |
| selected | DELIVERY | drop_circle | bridge | 44 | 0.1291 | 0.1310 | 0.1291 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 103 | 0.1488 | 0.1626 | 0.1488 | 0 |
| selected | DESCEND | disabled | panzer | 25 | 2.6083 | 2.6391 | 0.1249 | 25 |
| selected | HIGH_SURVEY | disabled | bridge | 58 | 0.1550 | 0.1680 | 0.1550 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 1 | 2.6464 | 2.6464 | 0.1087 | 1 |
| selected | HIGH_SURVEY | disabled | red_cross | 107 | 0.1553 | 0.1865 | 0.1553 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 56 | 0.1637 | 0.1744 | 0.1637 | 0 |
| selected | LOW_COVERAGE | disabled | bridge | 35 | 0.1119 | 0.1173 | 0.1119 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.1393 | 0.1396 | 0.1393 | 0 |
| selected | REACQUIRE | disabled | pillbox | 3 | 0.1070 | 0.1071 | 0.1070 | 0 |
| selected | REACQUIRE | disabled | red_cross | 1 | 0.1730 | 0.1730 | 0.1730 | 0 |
| selected | REVISIT | disabled | bridge | 34 | 0.1481 | 0.1638 | 0.1481 | 0 |
| selected | REVISIT | disabled | panzer | 9 | 2.5935 | 2.5941 | 0.1189 | 9 |
| selected | REVISIT | disabled | pillbox | 172 | 0.1144 | 0.1171 | 0.1144 | 0 |
| selected | REVISIT | disabled | red_cross | 39 | 0.1748 | 0.1871 | 0.1748 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 18 | 0.1089 | 0.1113 | 0.1089 | 0 |
| targets | DELIVERY | disabled | bridge | 38 | 0.1339 | 0.1381 | 0.1339 | 0 |
| targets | DELIVERY | disabled | red_cross | 13 | 0.1691 | 0.1721 | 0.1691 | 0 |
| targets | DELIVERY | drop_circle | bridge | 52 | 0.1288 | 0.1312 | 0.1288 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 107 | 0.1488 | 0.1633 | 0.1488 | 0 |
| targets | DESCEND | disabled | panzer | 25 | 2.6083 | 2.6391 | 0.1249 | 25 |
| targets | HIGH_SURVEY | disabled | bridge | 60 | 0.1545 | 0.1679 | 0.1545 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 3 | 2.6464 | 2.6467 | 0.1059 | 3 |
| targets | HIGH_SURVEY | disabled | red_cross | 109 | 0.1555 | 0.1864 | 0.1555 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 59 | 0.1635 | 0.1743 | 0.1635 | 0 |
| targets | LOW_COVERAGE | disabled | bridge | 35 | 0.1119 | 0.1173 | 0.1119 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.1393 | 0.1396 | 0.1393 | 0 |
| targets | REACQUIRE | disabled | pillbox | 3 | 0.1070 | 0.1071 | 0.1070 | 0 |
| targets | REACQUIRE | disabled | red_cross | 1 | 0.1730 | 0.1730 | 0.1730 | 0 |
| targets | REVISIT | disabled | bridge | 36 | 0.1489 | 0.1666 | 0.1489 | 0 |
| targets | REVISIT | disabled | panzer | 11 | 2.5935 | 2.5941 | 0.1204 | 11 |
| targets | REVISIT | disabled | pillbox | 172 | 0.1144 | 0.1171 | 0.1144 | 0 |
| targets | REVISIT | disabled | red_cross | 43 | 0.1748 | 0.1885 | 0.1748 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 20 | 0.1092 | 0.1123 | 0.1092 | 0 |

## Final Gate H mark

No valid H mark recorded.

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
