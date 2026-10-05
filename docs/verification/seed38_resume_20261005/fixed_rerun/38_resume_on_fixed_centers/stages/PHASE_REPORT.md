# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/seed38_resume_resume_on_fixed_seed38_20261005_173322

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 17.030 | 17.096 | 2.113 | (0.9954, -0.5644) | 0.1113 | random_red_cross | 0.1113 | False |
| bridge | 46.367 | 46.388 | 2.250 | (3.9116, 2.0002) | 0.1835 | random_bridge | 0.1835 | False |
| tent | 49.367 | 49.388 | 2.260 | (4.8051, -0.5443) | 0.1926 | random_tent | 0.1926 | False |
| tent | 54.911 | 54.993 | 2.278 | (4.2861, -3.4407) | 2.7998 | random_pillbox | 0.3792 | True |
| pillbox | 55.047 | 55.093 | 2.285 | (4.3214, -3.4240) | 0.3402 | random_pillbox | 0.3402 | False |
| panzer | 55.146 | 55.208 | 2.290 | (4.3741, -3.4244) | 2.7691 | random_pillbox | 0.2921 | True |
| panzer | 60.030 | 60.088 | 2.311 | (6.6985, -4.2999) | 0.3906 | random_panzer | 0.3906 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 48.895 | 61.096 | 2.251 | (3.9262, 1.7862) | 0.1307 | random_bridge | 0.1307 | False |
| panzer | 60.936 | 61.096 | 2.249 | (4.5661, -3.4438) | 2.5787 | random_pillbox | 0.1615 | True |
| panzer | 60.979 | 61.096 | 2.247 | (6.9302, -4.2919) | 0.2004 | random_panzer | 0.2004 | False |
| pillbox | 60.064 | 61.096 | 2.309 | (4.5567, -3.4468) | 0.1685 | random_pillbox | 0.1685 | False |
| red_cross | 52.136 | 61.096 | 2.246 | (0.9785, -0.5917) | 0.1433 | random_red_cross | 0.1433 | False |
| tent | 50.279 | 61.096 | 2.268 | (4.5903, -0.8338) | 0.3369 | random_tent | 0.3369 | False |
| tent | 54.979 | 61.096 | 2.281 | (4.2991, -3.4179) | 2.7748 | random_pillbox | 0.3590 | True |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| panzer | 71.171 | 71.262 | 1.366 | (6.9771, -4.2471) | 0.1368 | random_panzer | 0.1368 | False |
| red_cross | 115.501 | 115.608 | 1.425 | (0.9867, -0.5862) | 0.1347 | random_red_cross | 0.1347 | False |
| bridge | 138.029 | 138.151 | 1.404 | (3.9566, 1.7702) | 0.1192 | random_bridge | 0.1192 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 11 | 0.1313 | 0.1710 | 0.1313 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 49 | 2.5853 | 2.6342 | 0.1830 | 30 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 19 | 0.2032 | 0.3105 | 0.2032 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 5 | 0.1133 | 0.1164 | 0.1133 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 21 | 0.2347 | 2.7748 | 0.2347 | 3 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 17 | 0.1246 | 0.1300 | 0.1246 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 1 | 0.2004 | 0.2004 | 0.2004 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 54 | 0.1253 | 0.1431 | 0.1253 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 2 | 0.1194 | 0.1196 | 0.1194 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1368 | 0.1368 | 0.1368 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1348 | 0.1349 | 0.1348 | 0 |
| mapped | DELIVERY | disabled | bridge | 30 | 0.0869 | 0.0951 | 0.0869 | 0 |
| mapped | DELIVERY | disabled | panzer | 25 | 0.0968 | 0.1001 | 0.0968 | 0 |
| mapped | DELIVERY | disabled | red_cross | 20 | 0.0822 | 0.0893 | 0.0822 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 101 | 0.0932 | 0.1083 | 0.0932 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 133 | 0.0983 | 0.3552 | 0.0983 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 98 | 0.0816 | 0.0902 | 0.0816 | 0 |
| mapped | DESCEND | disabled | panzer | 44 | 0.1568 | 0.1977 | 0.1568 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 49 | 0.1339 | 0.1729 | 0.1339 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 57 | 2.5867 | 2.6170 | 0.1916 | 49 |
| mapped | HIGH_SURVEY | disabled | pillbox | 34 | 0.1862 | 0.2088 | 0.1862 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 161 | 0.1509 | 0.2437 | 0.1509 | 0 |
| mapped | LANDING | landing | landing_pad | 61 | 0.0373 | 0.0465 | 0.0373 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.0894 | 0.0895 | 0.0894 | 0 |
| mapped | REACQUIRE | disabled | panzer | 1 | 0.1011 | 0.1011 | 0.1011 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 3 | 0.0806 | 0.0808 | 0.0806 | 0 |
| mapped | REVISIT | disabled | bridge | 41 | 0.1011 | 0.1437 | 0.1011 | 0 |
| mapped | REVISIT | disabled | panzer | 200 | 0.1054 | 0.1299 | 0.1054 | 0 |
| mapped | REVISIT | disabled | pillbox | 171 | 0.0937 | 0.0991 | 0.0937 | 0 |
| mapped | REVISIT | disabled | red_cross | 100 | 0.0512 | 0.0891 | 0.0512 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 43 | 0.1165 | 0.1269 | 0.1165 | 0 |
| mapped | TAIL | disabled | bridge | 40 | 0.0741 | 0.0866 | 0.0741 | 0 |
| mapped | TAIL | disabled | landing_pad | 1 | 0.0475 | 0.0475 | 0.0475 | 0 |
| selected | DELIVERY | disabled | bridge | 26 | 0.1146 | 0.1181 | 0.1146 | 0 |
| selected | DELIVERY | disabled | panzer | 22 | 0.1338 | 0.1362 | 0.1338 | 0 |
| selected | DELIVERY | disabled | red_cross | 19 | 0.1321 | 0.1340 | 0.1321 | 0 |
| selected | DELIVERY | drop_circle | bridge | 96 | 0.1095 | 0.1124 | 0.1095 | 0 |
| selected | DELIVERY | drop_circle | panzer | 117 | 0.1243 | 0.1300 | 0.1243 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 85 | 0.1233 | 0.1291 | 0.1233 | 0 |
| selected | DESCEND | disabled | panzer | 44 | 0.1855 | 0.1986 | 0.1855 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 47 | 0.1270 | 0.1338 | 0.1270 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 9 | 0.2007 | 2.6168 | 0.2000 | 3 |
| selected | HIGH_SURVEY | disabled | pillbox | 6 | 0.1900 | 0.1931 | 0.1900 | 0 |
| selected | HIGH_SURVEY | disabled | red_cross | 143 | 0.1260 | 0.1444 | 0.1260 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.1192 | 0.1196 | 0.1192 | 0 |
| selected | REACQUIRE | disabled | panzer | 1 | 0.1368 | 0.1368 | 0.1368 | 0 |
| selected | REACQUIRE | disabled | red_cross | 3 | 0.1347 | 0.1349 | 0.1347 | 0 |
| selected | REVISIT | disabled | bridge | 39 | 0.1284 | 0.1363 | 0.1284 | 0 |
| selected | REVISIT | disabled | panzer | 198 | 0.1053 | 0.1614 | 0.1053 | 0 |
| selected | REVISIT | disabled | pillbox | 169 | 0.1097 | 0.1562 | 0.1097 | 0 |
| selected | REVISIT | disabled | red_cross | 77 | 0.1132 | 0.1434 | 0.1132 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 41 | 0.1235 | 0.1263 | 0.1235 | 0 |
| selected | TAIL | disabled | bridge | 40 | 0.0970 | 0.0990 | 0.0970 | 0 |
| targets | DELIVERY | disabled | bridge | 28 | 0.1144 | 0.1180 | 0.1144 | 0 |
| targets | DELIVERY | disabled | panzer | 24 | 0.1336 | 0.1362 | 0.1336 | 0 |
| targets | DELIVERY | disabled | red_cross | 19 | 0.1321 | 0.1340 | 0.1321 | 0 |
| targets | DELIVERY | drop_circle | bridge | 100 | 0.1095 | 0.1127 | 0.1095 | 0 |
| targets | DELIVERY | drop_circle | panzer | 133 | 0.1234 | 0.1303 | 0.1234 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 93 | 0.1233 | 0.1294 | 0.1233 | 0 |
| targets | DESCEND | disabled | panzer | 44 | 0.1855 | 0.1986 | 0.1855 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 49 | 0.1268 | 0.1337 | 0.1268 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 18 | 2.5931 | 2.6313 | 0.1943 | 10 |
| targets | HIGH_SURVEY | disabled | pillbox | 7 | 0.1894 | 0.1930 | 0.1894 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 147 | 0.1262 | 0.1446 | 0.1262 | 0 |
| targets | LANDING | landing | landing_pad | 61 | 0.0396 | 0.0465 | 0.0396 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.1192 | 0.1196 | 0.1192 | 0 |
| targets | REACQUIRE | disabled | panzer | 1 | 0.1368 | 0.1368 | 0.1368 | 0 |
| targets | REACQUIRE | disabled | red_cross | 3 | 0.1347 | 0.1349 | 0.1347 | 0 |
| targets | REVISIT | disabled | bridge | 41 | 0.1290 | 0.1364 | 0.1290 | 0 |
| targets | REVISIT | disabled | panzer | 200 | 0.1053 | 0.1624 | 0.1053 | 0 |
| targets | REVISIT | disabled | pillbox | 170 | 0.1097 | 0.1582 | 0.1097 | 0 |
| targets | REVISIT | disabled | red_cross | 83 | 0.1134 | 0.1439 | 0.1134 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 43 | 0.1240 | 0.1263 | 0.1240 | 0 |
| targets | TAIL | disabled | bridge | 40 | 0.0970 | 0.0990 | 0.0970 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.038942232513047494,
    "frame_id": "camera_init",
    "stamp_ns": 213991000000,
    "x": 8.712397571513467,
    "y": -4.210126936605678,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.038942232513047494,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.038942232513047494,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
