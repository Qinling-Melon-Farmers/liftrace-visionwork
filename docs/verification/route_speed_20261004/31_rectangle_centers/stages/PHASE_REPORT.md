# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_rectangle_seed31_20261004_224908

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 21.865 | 21.912 | 2.899 | (0.5412, -3.8400) | 0.1355 | random_tent | 0.1355 | False |
| bridge | 27.083 | 27.161 | 2.857 | (4.6835, -3.8255) | 0.4541 | random_bridge | 0.4541 | False |
| panzer | 35.959 | 35.992 | 2.895 | (3.9495, -0.9394) | 0.4239 | random_panzer | 0.4239 | False |
| pillbox | 49.602 | 49.656 | 2.807 | (0.8753, 2.2372) | 0.2134 | random_pillbox | 0.2134 | False |
| panzer | 49.713 | 49.824 | 2.813 | (0.8178, 2.2229) | 4.2227 | random_pillbox | 0.1974 | True |
| bridge | 51.036 | 51.086 | 2.858 | (0.7010, 2.1960) | 7.3007 | random_pillbox | 0.2153 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 86.472 | 86.552 | 1.425 | (4.9912, -3.8230) | 0.2156 | random_bridge | 0.2156 | False |
| panzer | 132.938 | 133.061 | 1.408 | (3.9521, -0.6924) | 0.1854 | random_panzer | 0.1854 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 21 | 0.3115 | 7.4213 | 0.2821 | 3 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 20 | 4.2892 | 4.3266 | 0.2006 | 14 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 11 | 0.1769 | 0.2085 | 0.1769 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 8 | 0.1465 | 0.1522 | 0.1465 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 63 | 0.2734 | 0.2890 | 0.2734 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 47 | 4.3052 | 4.3256 | 0.2106 | 24 |
| hint_vision | HIGH_SURVEY | disabled | tent | 58 | 0.2038 | 0.2204 | 0.2038 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 4 | 0.2162 | 0.2168 | 0.2162 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1854 | 0.1854 | 0.1854 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1476 | 0.1476 | 0.1476 | 0 |
| mapped | DELIVERY | disabled | bridge | 22 | 0.1233 | 0.1250 | 0.1233 | 0 |
| mapped | DELIVERY | disabled | panzer | 20 | 0.0583 | 0.0721 | 0.0583 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 110 | 0.0906 | 0.1243 | 0.0906 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 87 | 0.0759 | 0.1207 | 0.0759 | 0 |
| mapped | DESCEND | disabled | tent | 32 | 0.2082 | 0.2265 | 0.2082 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 154 | 0.2758 | 0.3157 | 0.2739 | 3 |
| mapped | HIGH_SURVEY | disabled | panzer | 131 | 4.2843 | 4.3483 | 0.2305 | 71 |
| mapped | HIGH_SURVEY | disabled | pillbox | 13 | 0.1891 | 0.2225 | 0.1891 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 150 | 0.2276 | 0.2659 | 0.2276 | 0 |
| mapped | LANDING | landing | landing_pad | 92 | 0.0308 | 0.0426 | 0.0308 | 0 |
| mapped | LOW_COVERAGE | disabled | panzer | 43 | 0.0606 | 0.0643 | 0.0606 | 0 |
| mapped | LOW_COVERAGE | disabled | red_cross | 53 | 0.0926 | 0.1074 | 0.0926 | 0 |
| mapped | LOW_COVERAGE | drop_cross | red_cross | 141 | 0.0824 | 0.1021 | 0.0824 | 0 |
| mapped | REACQUIRE | disabled | bridge | 4 | 0.1256 | 0.1261 | 0.1256 | 0 |
| mapped | REACQUIRE | disabled | panzer | 3 | 0.0526 | 0.0527 | 0.0526 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 2 | 0.0762 | 0.0762 | 0.0762 | 0 |
| mapped | REVISIT | disabled | bridge | 130 | 0.0996 | 0.1270 | 0.0996 | 0 |
| mapped | REVISIT | disabled | panzer | 42 | 0.0486 | 0.0530 | 0.0486 | 0 |
| mapped | REVISIT | disabled | pillbox | 110 | 0.0800 | 0.0895 | 0.0800 | 0 |
| mapped | TAIL | disabled | red_cross | 1 | 0.0895 | 0.0895 | 0.0895 | 0 |
| selected | DELIVERY | disabled | bridge | 19 | 0.2115 | 0.2148 | 0.2115 | 0 |
| selected | DELIVERY | disabled | panzer | 17 | 0.1733 | 0.1817 | 0.1733 | 0 |
| selected | DELIVERY | drop_circle | bridge | 102 | 0.1891 | 0.2055 | 0.1891 | 0 |
| selected | DELIVERY | drop_circle | panzer | 78 | 0.1453 | 0.1616 | 0.1453 | 0 |
| selected | DESCEND | disabled | tent | 32 | 0.2196 | 0.2198 | 0.2196 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 149 | 0.2727 | 0.2895 | 0.2727 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 110 | 0.3097 | 4.3258 | 0.2920 | 52 |
| selected | HIGH_SURVEY | disabled | pillbox | 1 | 0.1950 | 0.1950 | 0.1950 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 145 | 0.2061 | 0.2205 | 0.2061 | 0 |
| selected | LOW_COVERAGE | disabled | panzer | 43 | 0.1209 | 0.1268 | 0.1209 | 0 |
| selected | LOW_COVERAGE | disabled | red_cross | 43 | 0.0982 | 0.1094 | 0.0982 | 0 |
| selected | LOW_COVERAGE | drop_cross | red_cross | 81 | 0.0877 | 0.0918 | 0.0877 | 0 |
| selected | REACQUIRE | disabled | bridge | 4 | 0.2162 | 0.2168 | 0.2162 | 0 |
| selected | REACQUIRE | disabled | panzer | 3 | 0.1854 | 0.1866 | 0.1854 | 0 |
| selected | REACQUIRE | disabled | pillbox | 2 | 0.1472 | 0.1476 | 0.1472 | 0 |
| selected | REVISIT | disabled | bridge | 128 | 0.1691 | 0.2481 | 0.1691 | 0 |
| selected | REVISIT | disabled | panzer | 73 | 0.2684 | 4.3369 | 0.1925 | 33 |
| selected | REVISIT | disabled | pillbox | 75 | 0.1286 | 0.1456 | 0.1286 | 0 |
| targets | DELIVERY | disabled | bridge | 21 | 0.2111 | 0.2147 | 0.2111 | 0 |
| targets | DELIVERY | disabled | panzer | 19 | 0.1722 | 0.1816 | 0.1722 | 0 |
| targets | DELIVERY | drop_circle | bridge | 109 | 0.1885 | 0.2061 | 0.1885 | 0 |
| targets | DELIVERY | drop_circle | panzer | 84 | 0.1449 | 0.1628 | 0.1449 | 0 |
| targets | DESCEND | disabled | tent | 32 | 0.2196 | 0.2198 | 0.2196 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 151 | 0.2729 | 0.2895 | 0.2729 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 114 | 0.3158 | 4.3258 | 0.2920 | 54 |
| targets | HIGH_SURVEY | disabled | pillbox | 3 | 0.1933 | 0.1949 | 0.1933 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 149 | 0.2061 | 0.2205 | 0.2061 | 0 |
| targets | LANDING | landing | landing_pad | 92 | 0.0354 | 0.0434 | 0.0354 | 0 |
| targets | LOW_COVERAGE | disabled | panzer | 43 | 0.1209 | 0.1268 | 0.1209 | 0 |
| targets | LOW_COVERAGE | disabled | red_cross | 45 | 0.0985 | 0.1097 | 0.0985 | 0 |
| targets | LOW_COVERAGE | drop_cross | red_cross | 95 | 0.0884 | 0.0919 | 0.0884 | 0 |
| targets | REACQUIRE | disabled | bridge | 4 | 0.2162 | 0.2168 | 0.2162 | 0 |
| targets | REACQUIRE | disabled | panzer | 3 | 0.1854 | 0.1866 | 0.1854 | 0 |
| targets | REACQUIRE | disabled | pillbox | 2 | 0.1472 | 0.1476 | 0.1472 | 0 |
| targets | REVISIT | disabled | bridge | 130 | 0.1693 | 0.2497 | 0.1693 | 0 |
| targets | REVISIT | disabled | panzer | 77 | 0.2753 | 4.3369 | 0.1942 | 35 |
| targets | REVISIT | disabled | pillbox | 75 | 0.1286 | 0.1456 | 0.1286 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.021500575164399717,
    "frame_id": "camera_init",
    "stamp_ns": 230687000000,
    "x": 8.734702812264103,
    "y": -4.21510863262419,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.021500575164399717,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.021500575164399717,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
