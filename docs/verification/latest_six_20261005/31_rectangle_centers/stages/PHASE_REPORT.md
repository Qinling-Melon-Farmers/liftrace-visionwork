# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/latest_six_rectangle_seed31_20261005_023409

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 21.898 | 21.966 | 2.856 | (0.5055, -3.8245) | 0.1740 | random_tent | 0.1740 | False |
| bridge | 26.633 | 26.656 | 2.887 | (4.6805, -3.8658) | 0.4751 | random_bridge | 0.4751 | False |
| red_cross | 34.464 | 34.508 | 2.876 | (7.3234, -1.3560) | 0.4031 | random_red_cross | 0.4031 | False |
| panzer | 34.931 | 35.000 | 2.877 | (3.9434, -0.9575) | 0.4428 | random_panzer | 0.4428 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 34.409 | 35.324 | 2.877 | (4.9645, -3.8516) | 0.2537 | random_bridge | 0.2537 | False |
| panzer | 35.152 | 35.324 | 2.880 | (3.9392, -0.8631) | 0.3515 | random_panzer | 0.3515 | False |
| red_cross | 35.152 | 35.324 | 2.880 | (7.2411, -1.2648) | 0.3559 | random_red_cross | 0.3559 | False |
| tent | 26.727 | 35.324 | 2.886 | (0.5432, -4.0434) | 0.2115 | random_tent | 0.2115 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| panzer | 45.072 | 45.110 | 1.430 | (3.9814, -0.6943) | 0.1776 | random_panzer | 0.1776 | False |
| bridge | 59.594 | 59.701 | 1.511 | (4.9889, -3.8243) | 0.2179 | random_bridge | 0.2179 | False |
| red_cross | 91.730 | 91.865 | 1.310 | (7.4033, -1.0643) | 0.1012 | random_red_cross | 0.1012 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 17 | 0.3260 | 0.4543 | 0.3260 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 5 | 0.3889 | 0.4400 | 0.3889 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 15 | 0.3551 | 0.3936 | 0.3551 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 5 | 0.1615 | 0.1741 | 0.1615 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 57 | 0.2770 | 0.2969 | 0.2770 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 38 | 0.1862 | 0.2078 | 0.1862 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.2179 | 0.2179 | 0.2179 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 4 | 0.1792 | 0.1807 | 0.1792 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1026 | 0.1039 | 0.1026 | 0 |
| mapped | DELIVERY | disabled | bridge | 20 | 0.1327 | 0.1389 | 0.1327 | 0 |
| mapped | DELIVERY | disabled | panzer | 20 | 0.0899 | 0.0981 | 0.0899 | 0 |
| mapped | DELIVERY | disabled | red_cross | 29 | 0.0792 | 0.0908 | 0.0792 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 159 | 0.1094 | 0.1450 | 0.1094 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 100 | 0.1100 | 0.1432 | 0.1100 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 101 | 0.0921 | 0.1021 | 0.0921 | 0 |
| mapped | DESCEND | disabled | panzer | 43 | 0.2270 | 0.3221 | 0.2270 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 142 | 0.2709 | 0.3151 | 0.2709 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 6 | 0.3171 | 0.3463 | 0.3171 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 2 | 0.3394 | 0.3400 | 0.3394 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 93 | 0.2178 | 0.3030 | 0.2178 | 0 |
| mapped | LANDING | landing | landing_pad | 86 | 0.0499 | 0.0578 | 0.0499 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.1231 | 0.1242 | 0.1231 | 0 |
| mapped | REACQUIRE | disabled | panzer | 4 | 0.0824 | 0.0844 | 0.0824 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 3 | 0.0676 | 0.0685 | 0.0676 | 0 |
| mapped | REVISIT | disabled | bridge | 150 | 0.0984 | 0.1208 | 0.0984 | 0 |
| mapped | REVISIT | disabled | panzer | 66 | 0.1041 | 0.1491 | 0.1041 | 0 |
| mapped | REVISIT | disabled | red_cross | 9 | 0.0568 | 0.0635 | 0.0568 | 0 |
| selected | DELIVERY | disabled | bridge | 17 | 0.2130 | 0.2160 | 0.2130 | 0 |
| selected | DELIVERY | disabled | panzer | 16 | 0.1686 | 0.1757 | 0.1686 | 0 |
| selected | DELIVERY | disabled | red_cross | 28 | 0.0853 | 0.0948 | 0.0853 | 0 |
| selected | DELIVERY | drop_circle | bridge | 150 | 0.1891 | 0.2074 | 0.1891 | 0 |
| selected | DELIVERY | drop_circle | panzer | 93 | 0.1482 | 0.1571 | 0.1482 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 82 | 0.0886 | 0.0914 | 0.0886 | 0 |
| selected | DESCEND | disabled | panzer | 43 | 0.2871 | 0.3166 | 0.2871 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 140 | 0.2728 | 0.2969 | 0.2728 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 4 | 0.3262 | 0.3339 | 0.3262 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 90 | 0.1894 | 0.2162 | 0.1894 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.2174 | 0.2178 | 0.2174 | 0 |
| selected | REACQUIRE | disabled | panzer | 4 | 0.1792 | 0.1807 | 0.1792 | 0 |
| selected | REACQUIRE | disabled | red_cross | 3 | 0.1012 | 0.1037 | 0.1012 | 0 |
| selected | REVISIT | disabled | bridge | 148 | 0.1602 | 0.2455 | 0.1602 | 0 |
| selected | REVISIT | disabled | panzer | 64 | 0.1837 | 0.2194 | 0.1837 | 0 |
| selected | REVISIT | disabled | red_cross | 7 | 0.1243 | 0.1594 | 0.1243 | 0 |
| targets | DELIVERY | disabled | bridge | 19 | 0.2126 | 0.2160 | 0.2126 | 0 |
| targets | DELIVERY | disabled | panzer | 18 | 0.1676 | 0.1755 | 0.1676 | 0 |
| targets | DELIVERY | disabled | red_cross | 28 | 0.0853 | 0.0948 | 0.0853 | 0 |
| targets | DELIVERY | drop_circle | bridge | 157 | 0.1887 | 0.2079 | 0.1887 | 0 |
| targets | DELIVERY | drop_circle | panzer | 99 | 0.1480 | 0.1582 | 0.1480 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 92 | 0.0886 | 0.0914 | 0.0886 | 0 |
| targets | DESCEND | disabled | panzer | 43 | 0.2871 | 0.3166 | 0.2871 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 142 | 0.2735 | 0.2973 | 0.2735 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 6 | 0.3287 | 0.3392 | 0.3287 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 2 | 0.3397 | 0.3400 | 0.3397 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 92 | 0.1888 | 0.2161 | 0.1888 | 0 |
| targets | LANDING | landing | landing_pad | 86 | 0.0525 | 0.0549 | 0.0525 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.2174 | 0.2178 | 0.2174 | 0 |
| targets | REACQUIRE | disabled | panzer | 4 | 0.1792 | 0.1807 | 0.1792 | 0 |
| targets | REACQUIRE | disabled | red_cross | 3 | 0.1012 | 0.1037 | 0.1012 | 0 |
| targets | REVISIT | disabled | bridge | 150 | 0.1604 | 0.2472 | 0.1604 | 0 |
| targets | REVISIT | disabled | panzer | 66 | 0.1848 | 0.2224 | 0.1848 | 0 |
| targets | REVISIT | disabled | red_cross | 9 | 0.1336 | 0.2220 | 0.1336 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.02048329345937323,
    "frame_id": "camera_init",
    "stamp_ns": 203102000000,
    "x": 8.733449979261334,
    "y": -4.212069056487254,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.02048329345937323,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.02048329345937323,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
