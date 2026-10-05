# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/snake3_camera2m_snake3_seed38_20261005_101903

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 16.737 | 16.774 | 2.077 | (0.9980, -0.5826) | 0.1269 | random_red_cross | 0.1269 | False |
| tent | 31.235 | 31.268 | 2.329 | (2.0670, 1.6538) | 3.6922 | random_bridge | 1.9765 | False |
| bridge | 46.715 | 46.754 | 2.246 | (3.9359, 1.9958) | 0.1648 | random_bridge | 0.1648 | False |
| tent | 49.663 | 49.768 | 2.236 | (4.7747, -0.5339) | 0.2181 | random_tent | 0.2181 | False |
| panzer | 52.947 | 52.990 | 2.193 | (4.5795, -3.2942) | 2.6098 | random_pillbox | 0.0586 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 49.121 | 53.302 | 2.229 | (3.9382, 1.8056) | 0.1100 | random_bridge | 0.1100 | False |
| panzer | 53.141 | 53.302 | 2.190 | (4.5850, -3.3680) | 2.5819 | random_pillbox | 0.0868 | True |
| red_cross | 30.295 | 53.302 | 2.308 | (0.9668, -0.5892) | 0.1470 | random_red_cross | 0.1470 | False |
| tent | 31.235 | 53.302 | 2.329 | (2.0670, 1.6538) | 3.6922 | random_bridge | 1.9765 | False |
| tent | 52.206 | 53.302 | 2.219 | (4.7726, -0.8124) | 0.1664 | random_tent | 0.1664 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 77.137 | 77.246 | 1.375 | (0.9714, -0.5847) | 0.1407 | random_red_cross | 0.1407 | False |
| bridge | 99.726 | 99.826 | 1.301 | (3.9547, 1.7867) | 0.1085 | random_bridge | 0.1085 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 9 | 0.1127 | 0.1601 | 0.1127 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 5 | 2.5925 | 2.6079 | 0.0586 | 5 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 5 | 0.1269 | 0.1299 | 0.1269 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 15 | 0.1700 | 1.2603 | 0.1700 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 17 | 0.1010 | 0.1089 | 0.1010 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 35 | 0.1207 | 0.1435 | 0.1207 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 17 | 0.1628 | 0.1669 | 0.1628 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 3 | 0.1084 | 0.1085 | 0.1084 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1204 | 0.1204 | 0.1204 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1409 | 0.1411 | 0.1409 | 0 |
| mapped | DELIVERY | disabled | bridge | 39 | 0.1078 | 0.1142 | 0.1078 | 0 |
| mapped | DELIVERY | disabled | red_cross | 19 | 0.0824 | 0.0899 | 0.0824 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 51 | 0.1096 | 0.1193 | 0.1096 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 123 | 0.0850 | 0.1020 | 0.0850 | 0 |
| mapped | DESCEND | disabled | panzer | 16 | 2.5676 | 2.5847 | 0.1540 | 16 |
| mapped | DESCEND | disabled | pillbox | 15 | 0.1317 | 0.1705 | 0.1317 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 50 | 0.1146 | 0.1683 | 0.1146 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 5 | 2.5617 | 2.5658 | 0.1463 | 5 |
| mapped | HIGH_SURVEY | disabled | red_cross | 103 | 0.1344 | 0.2631 | 0.1344 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 46 | 0.1741 | 0.2640 | 0.1741 | 0 |
| mapped | LANDING | landing | landing_pad | 46 | 0.0266 | 0.0297 | 0.0266 | 0 |
| mapped | LOW_COVERAGE | disabled | bridge | 72 | 0.0743 | 0.0979 | 0.0743 | 0 |
| mapped | LOW_COVERAGE | disabled | panzer | 67 | 0.0834 | 0.0898 | 0.0834 | 0 |
| mapped | LOW_COVERAGE | disabled | red_cross | 25 | 0.1001 | 0.1090 | 0.1001 | 0 |
| mapped | LOW_COVERAGE | drop_circle | panzer | 134 | 0.0835 | 0.1085 | 0.0835 | 0 |
| mapped | REACQUIRE | disabled | bridge | 4 | 0.1094 | 0.1095 | 0.1094 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 3 | 0.1254 | 0.1260 | 0.1254 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 3 | 0.0808 | 0.0811 | 0.0808 | 0 |
| mapped | REVISIT | disabled | bridge | 37 | 0.0983 | 0.1090 | 0.0983 | 0 |
| mapped | REVISIT | disabled | panzer | 1 | 2.6057 | 2.6057 | 0.1005 | 1 |
| mapped | REVISIT | disabled | pillbox | 208 | 0.1135 | 0.1281 | 0.1135 | 0 |
| mapped | REVISIT | disabled | red_cross | 66 | 0.0840 | 0.1125 | 0.0840 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 41 | 0.1142 | 0.1184 | 0.1142 | 0 |
| mapped | TAIL | disabled | panzer | 25 | 0.0636 | 0.0744 | 0.0636 | 0 |
| selected | DELIVERY | disabled | bridge | 36 | 0.1081 | 0.1084 | 0.1081 | 0 |
| selected | DELIVERY | disabled | red_cross | 19 | 0.1368 | 0.1396 | 0.1368 | 0 |
| selected | DELIVERY | drop_circle | bridge | 45 | 0.1089 | 0.1094 | 0.1089 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 110 | 0.1250 | 0.1322 | 0.1250 | 0 |
| selected | DESCEND | disabled | panzer | 7 | 2.5658 | 2.5664 | 0.1481 | 7 |
| selected | DESCEND | disabled | pillbox | 9 | 0.1431 | 0.1462 | 0.1431 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 48 | 0.1052 | 0.1159 | 0.1052 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 3 | 2.5623 | 2.5625 | 0.1353 | 3 |
| selected | HIGH_SURVEY | disabled | red_cross | 87 | 0.1209 | 0.1505 | 0.1209 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 43 | 0.1628 | 0.1766 | 0.1628 | 0 |
| selected | LOW_COVERAGE | disabled | bridge | 70 | 0.0961 | 0.1018 | 0.0961 | 0 |
| selected | LOW_COVERAGE | disabled | panzer | 63 | 0.0838 | 0.0853 | 0.0838 | 0 |
| selected | LOW_COVERAGE | disabled | red_cross | 23 | 0.1097 | 0.1102 | 0.1097 | 0 |
| selected | LOW_COVERAGE | drop_circle | panzer | 124 | 0.0889 | 0.0925 | 0.0889 | 0 |
| selected | REACQUIRE | disabled | bridge | 4 | 0.1085 | 0.1085 | 0.1085 | 0 |
| selected | REACQUIRE | disabled | pillbox | 3 | 0.1205 | 0.1205 | 0.1205 | 0 |
| selected | REACQUIRE | disabled | red_cross | 3 | 0.1407 | 0.1411 | 0.1407 | 0 |
| selected | REVISIT | disabled | bridge | 35 | 0.1094 | 0.1165 | 0.1094 | 0 |
| selected | REVISIT | disabled | pillbox | 206 | 0.1183 | 0.1206 | 0.1183 | 0 |
| selected | REVISIT | disabled | red_cross | 54 | 0.1126 | 0.1513 | 0.1126 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 36 | 0.1136 | 0.1173 | 0.1136 | 0 |
| selected | TAIL | disabled | panzer | 25 | 0.0820 | 0.0831 | 0.0820 | 0 |
| targets | DELIVERY | disabled | bridge | 38 | 0.1081 | 0.1084 | 0.1081 | 0 |
| targets | DELIVERY | disabled | red_cross | 19 | 0.1368 | 0.1396 | 0.1368 | 0 |
| targets | DELIVERY | drop_circle | bridge | 50 | 0.1088 | 0.1094 | 0.1088 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 115 | 0.1249 | 0.1325 | 0.1249 | 0 |
| targets | DESCEND | disabled | panzer | 9 | 2.5658 | 2.5667 | 0.1476 | 9 |
| targets | DESCEND | disabled | pillbox | 9 | 0.1431 | 0.1462 | 0.1431 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 50 | 0.1048 | 0.1158 | 0.1048 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 5 | 2.5625 | 2.5663 | 0.1302 | 5 |
| targets | HIGH_SURVEY | disabled | red_cross | 89 | 0.1209 | 0.1505 | 0.1209 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 45 | 0.1629 | 0.1765 | 0.1629 | 0 |
| targets | LANDING | landing | landing_pad | 46 | 0.0279 | 0.0300 | 0.0279 | 0 |
| targets | LOW_COVERAGE | disabled | bridge | 72 | 0.0962 | 0.1018 | 0.0962 | 0 |
| targets | LOW_COVERAGE | disabled | panzer | 67 | 0.0839 | 0.0856 | 0.0839 | 0 |
| targets | LOW_COVERAGE | disabled | red_cross | 25 | 0.1098 | 0.1104 | 0.1098 | 0 |
| targets | LOW_COVERAGE | drop_circle | panzer | 133 | 0.0884 | 0.0925 | 0.0884 | 0 |
| targets | REACQUIRE | disabled | bridge | 4 | 0.1085 | 0.1085 | 0.1085 | 0 |
| targets | REACQUIRE | disabled | pillbox | 3 | 0.1205 | 0.1205 | 0.1205 | 0 |
| targets | REACQUIRE | disabled | red_cross | 3 | 0.1407 | 0.1411 | 0.1407 | 0 |
| targets | REVISIT | disabled | bridge | 37 | 0.1097 | 0.1179 | 0.1097 | 0 |
| targets | REVISIT | disabled | pillbox | 208 | 0.1183 | 0.1206 | 0.1183 | 0 |
| targets | REVISIT | disabled | red_cross | 58 | 0.1128 | 0.1521 | 0.1128 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 38 | 0.1139 | 0.1173 | 0.1139 | 0 |
| targets | TAIL | disabled | panzer | 25 | 0.0820 | 0.0831 | 0.0820 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.016275295583613427,
    "frame_id": "camera_init",
    "stamp_ns": 373228000000,
    "x": 8.734534111801759,
    "y": -4.205068683120249,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.016275295583613427,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.016275295583613427,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
