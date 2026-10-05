# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/seed38_resume_resume_on_seed38_20261005_165940

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 17.320 | 17.349 | 2.073 | (0.9966, -0.5810) | 0.1260 | random_red_cross | 0.1260 | False |
| bridge | 45.279 | 45.327 | 2.219 | (3.9253, 1.9759) | 0.1563 | random_bridge | 0.1563 | False |
| tent | 48.213 | 48.317 | 2.231 | (4.8071, -0.5583) | 0.1796 | random_tent | 0.1796 | False |
| panzer | 51.292 | 51.327 | 2.251 | (4.5425, -3.1679) | 2.6876 | random_pillbox | 0.1622 | True |
| panzer | 119.302 | 119.355 | 2.177 | (6.7608, -4.2664) | 0.3199 | random_panzer | 0.3199 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 47.658 | 51.625 | 2.229 | (3.9214, 1.8053) | 0.1248 | random_bridge | 0.1248 | False |
| panzer | 51.400 | 51.625 | 2.252 | (4.5455, -3.2207) | 2.6663 | random_pillbox | 0.1211 | True |
| red_cross | 30.935 | 51.625 | 2.275 | (0.9715, -0.5935) | 0.1482 | random_red_cross | 0.1482 | False |
| tent | 50.806 | 51.625 | 2.244 | (4.8057, -0.8126) | 0.1420 | random_tent | 0.1420 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 73.322 | 73.439 | 1.320 | (0.9766, -0.5870) | 0.1401 | random_red_cross | 0.1401 | False |
| bridge | 95.048 | 95.157 | 1.329 | (3.9500, 1.7815) | 0.1154 | random_bridge | 0.1154 | False |
| panzer | 131.569 | 131.653 | 1.429 | (7.0147, -4.2233) | 0.0981 | random_panzer | 0.0981 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 10 | 0.1210 | 0.1511 | 0.1210 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 12 | 0.2813 | 2.6800 | 0.2151 | 3 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1271 | 0.1289 | 0.1271 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 10 | 0.1049 | 0.1591 | 0.1049 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 17 | 0.1206 | 0.1243 | 0.1206 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 38 | 0.1223 | 0.1449 | 0.1223 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 18 | 0.1255 | 0.1400 | 0.1255 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1154 | 0.1154 | 0.1154 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 33 | 0.0884 | 0.0937 | 0.0884 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 2 | 0.1062 | 0.1062 | 0.1062 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1403 | 0.1405 | 0.1403 | 0 |
| mapped | DELIVERY | disabled | bridge | 34 | 0.1082 | 0.1147 | 0.1082 | 0 |
| mapped | DELIVERY | disabled | panzer | 16 | 0.0802 | 0.0817 | 0.0802 | 0 |
| mapped | DELIVERY | disabled | red_cross | 17 | 0.0752 | 0.0790 | 0.0752 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 72 | 0.1128 | 0.1217 | 0.1128 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 113 | 0.0936 | 0.1034 | 0.0936 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 105 | 0.0811 | 0.1009 | 0.0811 | 0 |
| mapped | DESCEND | disabled | panzer | 43 | 0.1493 | 2.5969 | 0.1416 | 12 |
| mapped | DESCEND | disabled | pillbox | 26 | 0.1474 | 0.1823 | 0.1474 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 46 | 0.1273 | 0.1686 | 0.1273 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 33 | 2.5861 | 2.6365 | 0.1350 | 33 |
| mapped | HIGH_SURVEY | disabled | pillbox | 75 | 0.1459 | 0.1612 | 0.1459 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 104 | 0.1379 | 0.2554 | 0.1379 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 50 | 0.1524 | 0.2268 | 0.1524 | 0 |
| mapped | LOW_COVERAGE | disabled | panzer | 269 | 0.0921 | 0.1204 | 0.0921 | 0 |
| mapped | LOW_COVERAGE | drop_circle | panzer | 122 | 0.0795 | 0.0859 | 0.0795 | 0 |
| mapped | REACQUIRE | disabled | bridge | 1 | 0.1013 | 0.1013 | 0.1013 | 0 |
| mapped | REACQUIRE | disabled | panzer | 37 | 0.1039 | 0.1126 | 0.1039 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 3 | 0.0996 | 0.1007 | 0.0996 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 4 | 0.0715 | 0.0724 | 0.0715 | 0 |
| mapped | RESUME_ASCEND | disabled | bridge | 59 | 0.1248 | 0.1354 | 0.1248 | 0 |
| mapped | RESUME_JOIN | disabled | bridge | 45 | 0.1373 | 0.1615 | 0.1373 | 0 |
| mapped | RESUME_JOIN | disabled | tent | 42 | 0.1397 | 0.1710 | 0.1397 | 0 |
| mapped | REVISIT | disabled | bridge | 34 | 0.1044 | 0.1145 | 0.1044 | 0 |
| mapped | REVISIT | disabled | panzer | 176 | 0.0823 | 0.1132 | 0.0823 | 2 |
| mapped | REVISIT | disabled | pillbox | 181 | 0.1081 | 0.1319 | 0.1081 | 0 |
| mapped | REVISIT | disabled | red_cross | 90 | 0.0903 | 0.1099 | 0.0903 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 41 | 0.1124 | 0.1194 | 0.1124 | 0 |
| mapped | TAIL | disabled | panzer | 23 | 0.0847 | 0.0895 | 0.0847 | 0 |
| selected | DELIVERY | disabled | bridge | 31 | 0.1126 | 0.1148 | 0.1126 | 0 |
| selected | DELIVERY | disabled | panzer | 14 | 0.0968 | 0.0978 | 0.0968 | 0 |
| selected | DELIVERY | disabled | red_cross | 17 | 0.1359 | 0.1386 | 0.1359 | 0 |
| selected | DELIVERY | drop_circle | bridge | 64 | 0.1123 | 0.1131 | 0.1123 | 0 |
| selected | DELIVERY | drop_circle | panzer | 108 | 0.0950 | 0.0960 | 0.0950 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 95 | 0.1245 | 0.1312 | 0.1245 | 0 |
| selected | DESCEND | disabled | panzer | 33 | 0.1493 | 2.6182 | 0.1462 | 4 |
| selected | DESCEND | disabled | pillbox | 16 | 0.1392 | 0.1403 | 0.1392 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 44 | 0.1224 | 0.1280 | 0.1224 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 11 | 2.5844 | 2.6382 | 0.1113 | 11 |
| selected | HIGH_SURVEY | disabled | pillbox | 47 | 0.1145 | 0.1168 | 0.1145 | 0 |
| selected | HIGH_SURVEY | disabled | red_cross | 96 | 0.1229 | 0.1509 | 0.1229 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 45 | 0.1294 | 0.1486 | 0.1294 | 0 |
| selected | LOW_COVERAGE | disabled | panzer | 265 | 0.0904 | 0.0908 | 0.0904 | 0 |
| selected | LOW_COVERAGE | drop_circle | panzer | 115 | 0.0897 | 0.0905 | 0.0897 | 0 |
| selected | REACQUIRE | disabled | bridge | 1 | 0.1154 | 0.1154 | 0.1154 | 0 |
| selected | REACQUIRE | disabled | panzer | 37 | 0.0886 | 0.0922 | 0.0886 | 0 |
| selected | REACQUIRE | disabled | pillbox | 3 | 0.1061 | 0.1062 | 0.1061 | 0 |
| selected | REACQUIRE | disabled | red_cross | 4 | 0.1399 | 0.1405 | 0.1399 | 0 |
| selected | RESUME_ASCEND | disabled | bridge | 59 | 0.1070 | 0.1090 | 0.1070 | 0 |
| selected | RESUME_JOIN | disabled | bridge | 45 | 0.1117 | 0.1132 | 0.1117 | 0 |
| selected | RESUME_JOIN | disabled | tent | 40 | 0.1430 | 0.1471 | 0.1430 | 0 |
| selected | REVISIT | disabled | bridge | 32 | 0.1196 | 0.1271 | 0.1196 | 0 |
| selected | REVISIT | disabled | panzer | 170 | 0.0908 | 0.1197 | 0.0908 | 0 |
| selected | REVISIT | disabled | pillbox | 177 | 0.1116 | 0.1190 | 0.1116 | 0 |
| selected | REVISIT | disabled | red_cross | 84 | 0.1090 | 0.1503 | 0.1090 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 35 | 0.1131 | 0.1189 | 0.1131 | 0 |
| selected | TAIL | disabled | panzer | 23 | 0.0880 | 0.0882 | 0.0880 | 0 |
| targets | DELIVERY | disabled | bridge | 33 | 0.1125 | 0.1148 | 0.1125 | 0 |
| targets | DELIVERY | disabled | panzer | 15 | 0.0967 | 0.0978 | 0.0967 | 0 |
| targets | DELIVERY | disabled | red_cross | 17 | 0.1359 | 0.1386 | 0.1359 | 0 |
| targets | DELIVERY | drop_circle | bridge | 70 | 0.1123 | 0.1131 | 0.1123 | 0 |
| targets | DELIVERY | drop_circle | panzer | 113 | 0.0949 | 0.0960 | 0.0949 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 101 | 0.1243 | 0.1317 | 0.1243 | 0 |
| targets | DESCEND | disabled | panzer | 37 | 0.1493 | 2.6218 | 0.1462 | 6 |
| targets | DESCEND | disabled | pillbox | 18 | 0.1391 | 0.1403 | 0.1391 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 46 | 0.1222 | 0.1279 | 0.1222 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 16 | 2.5844 | 2.6429 | 0.1110 | 16 |
| targets | HIGH_SURVEY | disabled | pillbox | 47 | 0.1145 | 0.1168 | 0.1145 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 98 | 0.1229 | 0.1508 | 0.1229 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 47 | 0.1285 | 0.1484 | 0.1285 | 0 |
| targets | LOW_COVERAGE | disabled | panzer | 269 | 0.0904 | 0.0908 | 0.0904 | 0 |
| targets | LOW_COVERAGE | drop_circle | panzer | 122 | 0.0897 | 0.0905 | 0.0897 | 0 |
| targets | REACQUIRE | disabled | bridge | 1 | 0.1154 | 0.1154 | 0.1154 | 0 |
| targets | REACQUIRE | disabled | panzer | 37 | 0.0886 | 0.0922 | 0.0886 | 0 |
| targets | REACQUIRE | disabled | pillbox | 3 | 0.1061 | 0.1062 | 0.1061 | 0 |
| targets | REACQUIRE | disabled | red_cross | 4 | 0.1399 | 0.1405 | 0.1399 | 0 |
| targets | RESUME_ASCEND | disabled | bridge | 59 | 0.1070 | 0.1090 | 0.1070 | 0 |
| targets | RESUME_JOIN | disabled | bridge | 45 | 0.1117 | 0.1132 | 0.1117 | 0 |
| targets | RESUME_JOIN | disabled | tent | 42 | 0.1431 | 0.1485 | 0.1431 | 0 |
| targets | REVISIT | disabled | bridge | 34 | 0.1199 | 0.1286 | 0.1199 | 0 |
| targets | REVISIT | disabled | panzer | 174 | 0.0908 | 0.1214 | 0.0908 | 0 |
| targets | REVISIT | disabled | pillbox | 180 | 0.1117 | 0.1206 | 0.1117 | 0 |
| targets | REVISIT | disabled | red_cross | 88 | 0.1090 | 0.1512 | 0.1090 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 37 | 0.1134 | 0.1196 | 0.1134 | 0 |
| targets | TAIL | disabled | panzer | 23 | 0.0880 | 0.0882 | 0.0880 | 0 |

## Final Gate H mark

No valid H mark recorded.

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
