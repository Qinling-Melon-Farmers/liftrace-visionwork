# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/seed38_resume_resume_off_seed38_20261005_155057

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 16.706 | 16.744 | 2.070 | (0.9981, -0.5816) | 0.1259 | random_red_cross | 0.1259 | False |
| bridge | 45.799 | 45.837 | 2.202 | (3.9103, 2.0233) | 0.2023 | random_bridge | 0.2023 | False |
| tent | 48.792 | 48.810 | 2.255 | (4.8121, -0.5906) | 0.1503 | random_tent | 0.1503 | False |
| panzer | 51.916 | 51.983 | 2.266 | (4.5712, -3.2866) | 2.6202 | random_pillbox | 0.0679 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 48.301 | 52.269 | 2.266 | (3.9205, 1.8140) | 0.1220 | random_bridge | 0.1220 | False |
| panzer | 52.059 | 52.269 | 2.263 | (4.5705, -3.3470) | 2.6020 | random_pillbox | 0.0827 | True |
| red_cross | 31.498 | 52.269 | 2.254 | (0.9735, -0.5934) | 0.1471 | random_red_cross | 0.1471 | False |
| tent | 51.359 | 52.269 | 2.287 | (4.7969, -0.7991) | 0.1391 | random_tent | 0.1391 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 75.439 | 75.560 | 1.368 | (0.9778, -0.5853) | 0.1380 | random_red_cross | 0.1380 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 10 | 0.1305 | 0.1928 | 0.1305 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 4 | 2.6111 | 2.6194 | 0.0713 | 4 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 3 | 0.1263 | 0.1280 | 0.1263 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 7 | 0.1201 | 0.1452 | 0.1201 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 17 | 0.1187 | 0.1216 | 0.1187 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 42 | 0.1238 | 0.1434 | 0.1238 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 18 | 0.1228 | 0.1375 | 0.1228 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1200 | 0.1200 | 0.1200 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1382 | 0.1384 | 0.1382 | 0 |
| mapped | DELIVERY | disabled | red_cross | 19 | 0.0873 | 0.0884 | 0.0873 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 111 | 0.0843 | 0.0997 | 0.0843 | 0 |
| mapped | DESCEND | disabled | panzer | 18 | 2.5403 | 2.5690 | 0.1925 | 18 |
| mapped | DESCEND | disabled | pillbox | 22 | 0.1332 | 0.1835 | 0.1332 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 45 | 0.1259 | 0.1520 | 0.1259 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 4 | 2.5819 | 2.5878 | 0.1401 | 4 |
| mapped | HIGH_SURVEY | disabled | red_cross | 109 | 0.1390 | 0.2550 | 0.1390 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 48 | 0.1489 | 0.2026 | 0.1489 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 3 | 0.1292 | 0.1293 | 0.1292 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 4 | 0.0860 | 0.0867 | 0.0860 | 0 |
| mapped | REVISIT | disabled | pillbox | 214 | 0.1150 | 0.1283 | 0.1150 | 0 |
| mapped | REVISIT | disabled | red_cross | 32 | 0.0786 | 0.0842 | 0.0786 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 36 | 0.1153 | 0.1209 | 0.1153 | 0 |
| selected | DELIVERY | disabled | red_cross | 17 | 0.1347 | 0.1368 | 0.1347 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 95 | 0.1240 | 0.1307 | 0.1240 | 0 |
| selected | DESCEND | disabled | panzer | 11 | 2.5693 | 2.5776 | 0.1595 | 11 |
| selected | DESCEND | disabled | pillbox | 19 | 0.1669 | 0.1734 | 0.1669 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 43 | 0.1204 | 0.1233 | 0.1204 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 2 | 2.5826 | 2.5840 | 0.1371 | 2 |
| selected | HIGH_SURVEY | disabled | red_cross | 92 | 0.1241 | 0.1504 | 0.1241 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 45 | 0.1282 | 0.1438 | 0.1282 | 0 |
| selected | REACQUIRE | disabled | pillbox | 3 | 0.1200 | 0.1201 | 0.1200 | 0 |
| selected | REACQUIRE | disabled | red_cross | 4 | 0.1379 | 0.1383 | 0.1379 | 0 |
| selected | REVISIT | disabled | pillbox | 211 | 0.1197 | 0.1246 | 0.1197 | 0 |
| selected | REVISIT | disabled | red_cross | 29 | 0.1443 | 0.1511 | 0.1443 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 30 | 0.1141 | 0.1176 | 0.1141 | 0 |
| targets | DELIVERY | disabled | red_cross | 17 | 0.1347 | 0.1368 | 0.1347 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 99 | 0.1240 | 0.1311 | 0.1240 | 0 |
| targets | DESCEND | disabled | panzer | 11 | 2.5693 | 2.5776 | 0.1595 | 11 |
| targets | DESCEND | disabled | pillbox | 19 | 0.1669 | 0.1734 | 0.1669 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 45 | 0.1202 | 0.1232 | 0.1202 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 4 | 2.5848 | 2.5883 | 0.1306 | 4 |
| targets | HIGH_SURVEY | disabled | red_cross | 94 | 0.1241 | 0.1503 | 0.1241 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 47 | 0.1272 | 0.1437 | 0.1272 | 0 |
| targets | REACQUIRE | disabled | pillbox | 3 | 0.1200 | 0.1201 | 0.1200 | 0 |
| targets | REACQUIRE | disabled | red_cross | 4 | 0.1379 | 0.1383 | 0.1379 | 0 |
| targets | REVISIT | disabled | pillbox | 213 | 0.1197 | 0.1259 | 0.1197 | 0 |
| targets | REVISIT | disabled | red_cross | 31 | 0.1448 | 0.1523 | 0.1448 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 32 | 0.1144 | 0.1182 | 0.1144 | 0 |

## Final Gate H mark

No valid H mark recorded.

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
