# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_rectangle_seed38_20261005_002556

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 16.967 | 17.013 | 2.634 | (0.9830, -0.6199) | 0.1671 | random_red_cross | 0.1671 | False |
| panzer | 25.958 | 26.033 | 2.886 | (4.3516, -3.5045) | 2.7716 | random_pillbox | 0.3523 | True |
| panzer | 28.571 | 28.645 | 2.806 | (6.6211, -4.3101) | 0.4654 | random_panzer | 0.4654 | False |
| pillbox | 31.271 | 31.341 | 2.852 | (4.5629, -3.4470) | 0.1658 | random_pillbox | 0.1658 | False |
| tent | 33.528 | 33.581 | 2.841 | (4.7854, -1.1594) | 0.4649 | random_tent | 0.4649 | False |
| bridge | 33.621 | 33.670 | 2.840 | (4.4858, -3.5135) | 5.3948 | random_pillbox | 0.2629 | True |
| bridge | 37.000 | 37.070 | 2.849 | (4.1217, 1.5651) | 0.3102 | random_bridge | 0.3102 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| panzer | 79.320 | 79.401 | 1.365 | (4.5426, -3.4644) | 2.5960 | random_pillbox | 0.1908 | True |
| red_cross | 110.744 | 110.839 | 1.366 | (0.9657, -0.5945) | 0.1521 | random_red_cross | 0.1521 | False |
| bridge | 133.371 | 133.436 | 1.388 | (3.9492, 1.6873) | 0.1937 | random_bridge | 0.1937 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 26 | 0.2508 | 5.3700 | 0.2490 | 3 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 28 | 0.4518 | 2.7227 | 0.2716 | 13 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 2 | 0.2204 | 0.2695 | 0.2204 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1704 | 0.1713 | 0.1704 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 8 | 0.3910 | 0.4517 | 0.3910 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 16 | 0.2650 | 0.2848 | 0.2650 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 77 | 2.6380 | 2.6660 | 0.2575 | 48 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 42 | 0.1541 | 0.1646 | 0.1541 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 25 | 0.3324 | 0.3349 | 0.3324 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1937 | 0.1937 | 0.1937 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 2.5960 | 2.5960 | 0.1908 | 1 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1523 | 0.1525 | 0.1523 | 0 |
| mapped | DELIVERY | disabled | bridge | 19 | 0.0809 | 0.0947 | 0.0809 | 0 |
| mapped | DELIVERY | disabled | pillbox | 20 | 0.0895 | 0.0922 | 0.0895 | 0 |
| mapped | DELIVERY | disabled | red_cross | 17 | 0.0841 | 0.0854 | 0.0841 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 75 | 0.1008 | 0.1183 | 0.1008 | 0 |
| mapped | DELIVERY | drop_circle | pillbox | 144 | 0.0899 | 0.1404 | 0.0899 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 112 | 0.0726 | 0.0926 | 0.0726 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 46 | 0.2943 | 0.3442 | 0.2920 | 2 |
| mapped | HIGH_SURVEY | disabled | panzer | 214 | 2.5629 | 2.6723 | 0.2569 | 138 |
| mapped | HIGH_SURVEY | disabled | pillbox | 3 | 0.2190 | 0.2400 | 0.2190 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 141 | 0.1882 | 0.2065 | 0.1882 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 65 | 0.3259 | 0.3427 | 0.3259 | 0 |
| mapped | LANDING | landing | landing_pad | 66 | 0.0399 | 0.0461 | 0.0399 | 0 |
| mapped | REACQUIRE | disabled | bridge | 1 | 0.0786 | 0.0786 | 0.0786 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 3 | 0.0913 | 0.0916 | 0.0913 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 3 | 0.0829 | 0.0832 | 0.0829 | 0 |
| mapped | REVISIT | disabled | bridge | 36 | 0.0765 | 0.0980 | 0.0765 | 0 |
| mapped | REVISIT | disabled | pillbox | 174 | 0.0820 | 0.0937 | 0.0820 | 0 |
| mapped | REVISIT | disabled | red_cross | 72 | 0.0740 | 0.0825 | 0.0740 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 26 | 0.1117 | 0.1207 | 0.1117 | 0 |
| mapped | TAIL | disabled | bridge | 40 | 0.0715 | 0.0855 | 0.0715 | 0 |
| mapped | TAIL | disabled | landing_pad | 1 | 0.0385 | 0.0385 | 0.0385 | 0 |
| selected | DELIVERY | disabled | bridge | 17 | 0.1824 | 0.1912 | 0.1824 | 0 |
| selected | DELIVERY | disabled | panzer | 15 | 2.5937 | 2.5951 | 0.1848 | 15 |
| selected | DELIVERY | disabled | pillbox | 2 | 0.1356 | 0.1357 | 0.1356 | 0 |
| selected | DELIVERY | disabled | red_cross | 16 | 0.1482 | 0.1509 | 0.1482 | 0 |
| selected | DELIVERY | drop_circle | bridge | 68 | 0.1583 | 0.1720 | 0.1583 | 0 |
| selected | DELIVERY | drop_circle | panzer | 1 | 2.5918 | 2.5918 | 0.1793 | 1 |
| selected | DELIVERY | drop_circle | pillbox | 134 | 0.1558 | 0.1757 | 0.1558 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 99 | 0.1318 | 0.1427 | 0.1318 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 42 | 0.2714 | 0.2955 | 0.2714 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 126 | 0.2715 | 2.6651 | 0.2572 | 52 |
| selected | HIGH_SURVEY | disabled | red_cross | 112 | 0.1540 | 0.1653 | 0.1540 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 59 | 0.3291 | 0.3353 | 0.3291 | 0 |
| selected | REACQUIRE | disabled | bridge | 1 | 0.1937 | 0.1937 | 0.1937 | 0 |
| selected | REACQUIRE | disabled | panzer | 3 | 2.5957 | 2.5960 | 0.1902 | 3 |
| selected | REACQUIRE | disabled | red_cross | 3 | 0.1521 | 0.1525 | 0.1521 | 0 |
| selected | REVISIT | disabled | bridge | 34 | 0.2271 | 0.2770 | 0.2271 | 0 |
| selected | REVISIT | disabled | panzer | 53 | 2.6064 | 2.6208 | 0.2130 | 53 |
| selected | REVISIT | disabled | pillbox | 119 | 0.1256 | 0.1337 | 0.1256 | 0 |
| selected | REVISIT | disabled | red_cross | 54 | 0.1133 | 0.1623 | 0.1133 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 24 | 0.1116 | 0.1157 | 0.1116 | 0 |
| selected | TAIL | disabled | bridge | 40 | 0.1330 | 0.1389 | 0.1330 | 0 |
| targets | DELIVERY | disabled | bridge | 19 | 0.1814 | 0.1911 | 0.1814 | 0 |
| targets | DELIVERY | disabled | panzer | 15 | 2.5937 | 2.5951 | 0.1848 | 15 |
| targets | DELIVERY | disabled | pillbox | 4 | 0.1358 | 0.1362 | 0.1358 | 0 |
| targets | DELIVERY | disabled | red_cross | 16 | 0.1482 | 0.1509 | 0.1482 | 0 |
| targets | DELIVERY | drop_circle | bridge | 73 | 0.1581 | 0.1732 | 0.1581 | 0 |
| targets | DELIVERY | drop_circle | panzer | 3 | 2.5920 | 2.5921 | 0.1798 | 3 |
| targets | DELIVERY | drop_circle | pillbox | 141 | 0.1548 | 0.1755 | 0.1548 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 107 | 0.1318 | 0.1432 | 0.1318 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 44 | 0.2704 | 0.2954 | 0.2704 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 177 | 2.6264 | 2.6665 | 0.2566 | 101 |
| targets | HIGH_SURVEY | disabled | red_cross | 114 | 0.1542 | 0.1652 | 0.1542 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 61 | 0.3292 | 0.3356 | 0.3292 | 0 |
| targets | LANDING | landing | landing_pad | 66 | 0.0409 | 0.0430 | 0.0409 | 0 |
| targets | REACQUIRE | disabled | bridge | 1 | 0.1937 | 0.1937 | 0.1937 | 0 |
| targets | REACQUIRE | disabled | panzer | 3 | 2.5957 | 2.5960 | 0.1902 | 3 |
| targets | REACQUIRE | disabled | red_cross | 3 | 0.1521 | 0.1525 | 0.1521 | 0 |
| targets | REVISIT | disabled | bridge | 36 | 0.2296 | 0.2850 | 0.2296 | 0 |
| targets | REVISIT | disabled | panzer | 55 | 2.6068 | 2.6219 | 0.2140 | 55 |
| targets | REVISIT | disabled | pillbox | 119 | 0.1256 | 0.1337 | 0.1256 | 0 |
| targets | REVISIT | disabled | red_cross | 60 | 0.1135 | 0.1635 | 0.1135 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 26 | 0.1117 | 0.1165 | 0.1117 | 0 |
| targets | TAIL | disabled | bridge | 40 | 0.1330 | 0.1389 | 0.1330 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.02470811650147197,
    "frame_id": "camera_init",
    "stamp_ns": 208367000000,
    "x": 8.729224101387926,
    "y": -4.2133735955491085,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.02470811650147197,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.02470811650147197,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
