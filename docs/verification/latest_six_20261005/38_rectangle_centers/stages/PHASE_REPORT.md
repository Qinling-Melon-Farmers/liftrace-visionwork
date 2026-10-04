# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/latest_six_rectangle_seed38_20261005_032449

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 16.771 | 16.855 | 2.625 | (0.9874, -0.6195) | 0.1651 | random_red_cross | 0.1651 | False |
| panzer | 26.897 | 26.914 | 2.841 | (4.3669, -3.4716) | 2.7644 | random_pillbox | 0.3213 | True |
| panzer | 29.349 | 29.385 | 2.820 | (6.6496, -4.3365) | 0.4505 | random_panzer | 0.4505 | False |
| tent | 29.516 | 29.592 | 2.821 | (4.4564, -3.4800) | 2.8061 | random_pillbox | 0.2563 | True |
| pillbox | 30.428 | 30.501 | 2.823 | (4.4825, -3.4973) | 0.2519 | random_pillbox | 0.2519 | False |
| tent | 34.377 | 34.427 | 2.886 | (4.7736, -1.1773) | 0.4852 | random_tent | 0.4852 | False |
| bridge | 34.513 | 34.601 | 2.885 | (4.4923, -3.5233) | 5.4051 | random_pillbox | 0.2673 | True |
| bridge | 37.954 | 38.027 | 2.928 | (4.0454, 1.6178) | 0.2448 | random_bridge | 0.2448 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 98.152 | 98.346 | 1.356 | (0.9646, -0.5945) | 0.1526 | random_red_cross | 0.1526 | False |
| bridge | 120.474 | 120.636 | 1.342 | (3.9547, 1.6978) | 0.1819 | random_bridge | 0.1819 | False |
| panzer | 147.427 | 147.551 | 1.544 | (6.9818, -4.2722) | 0.1563 | random_panzer | 0.1563 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 30 | 0.2655 | 3.0585 | 0.2640 | 2 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 30 | 2.6784 | 2.7297 | 0.2623 | 17 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 1 | 0.2519 | 0.2519 | 0.2519 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1700 | 0.1746 | 0.1700 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 8 | 0.3843 | 1.9938 | 0.3553 | 1 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 11 | 0.2847 | 0.2951 | 0.2847 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 66 | 2.6471 | 2.6928 | 0.2630 | 40 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 48 | 0.1577 | 0.1650 | 0.1577 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 24 | 0.3356 | 0.3402 | 0.3356 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 3 | 0.1834 | 0.1849 | 0.1834 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1563 | 0.1563 | 0.1563 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1925 | 0.1925 | 0.1925 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 1 | 0.1526 | 0.1526 | 0.1526 | 0 |
| mapped | DELIVERY | disabled | bridge | 30 | 0.0787 | 0.0868 | 0.0787 | 0 |
| mapped | DELIVERY | disabled | panzer | 29 | 0.0952 | 0.1071 | 0.0952 | 0 |
| mapped | DELIVERY | disabled | red_cross | 15 | 0.0807 | 0.0864 | 0.0807 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 78 | 0.0877 | 0.0973 | 0.0877 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 178 | 0.0809 | 0.3116 | 0.0809 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 77 | 0.0928 | 0.1059 | 0.0928 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 36 | 0.3013 | 1.6116 | 0.3013 | 2 |
| mapped | HIGH_SURVEY | disabled | panzer | 196 | 2.5515 | 2.7013 | 0.2626 | 125 |
| mapped | HIGH_SURVEY | disabled | pillbox | 3 | 0.2378 | 0.2389 | 0.2378 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 150 | 0.1827 | 0.2011 | 0.1827 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 59 | 0.3324 | 0.3421 | 0.3324 | 0 |
| mapped | LANDING | landing | landing_pad | 129 | 0.0380 | 0.0468 | 0.0380 | 0 |
| mapped | REACQUIRE | disabled | bridge | 6 | 0.0693 | 0.0702 | 0.0693 | 0 |
| mapped | REACQUIRE | disabled | panzer | 3 | 0.0944 | 0.0944 | 0.0944 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 3 | 0.0859 | 0.0865 | 0.0859 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 4 | 0.0778 | 0.0781 | 0.0778 | 0 |
| mapped | REVISIT | disabled | bridge | 62 | 0.0736 | 0.0957 | 0.0736 | 0 |
| mapped | REVISIT | disabled | panzer | 102 | 0.0946 | 0.1083 | 0.0946 | 0 |
| mapped | REVISIT | disabled | pillbox | 181 | 0.0947 | 0.1147 | 0.0947 | 0 |
| mapped | REVISIT | disabled | red_cross | 135 | 0.0786 | 0.1088 | 0.0786 | 0 |
| mapped | REVISIT | disabled | tent | 22 | 0.0698 | 0.0855 | 0.0698 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 22 | 0.1111 | 0.1226 | 0.1111 | 0 |
| mapped | TAIL | disabled | landing_pad | 2 | 0.0338 | 0.0340 | 0.0338 | 0 |
| mapped | TAIL | disabled | panzer | 21 | 0.0895 | 0.0980 | 0.0895 | 0 |
| selected | DELIVERY | disabled | bridge | 26 | 0.1612 | 0.1742 | 0.1612 | 0 |
| selected | DELIVERY | disabled | panzer | 23 | 0.1516 | 0.1548 | 0.1516 | 0 |
| selected | DELIVERY | disabled | red_cross | 13 | 0.1483 | 0.1506 | 0.1483 | 0 |
| selected | DELIVERY | drop_circle | bridge | 72 | 0.1359 | 0.1489 | 0.1359 | 0 |
| selected | DELIVERY | drop_circle | panzer | 163 | 0.1333 | 0.1484 | 0.1333 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 68 | 0.1382 | 0.1442 | 0.1382 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 32 | 0.2902 | 0.3016 | 0.2902 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 118 | 0.2763 | 2.6948 | 0.2632 | 49 |
| selected | HIGH_SURVEY | disabled | red_cross | 114 | 0.1574 | 0.1666 | 0.1574 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 53 | 0.3348 | 0.3402 | 0.3348 | 0 |
| selected | REACQUIRE | disabled | bridge | 6 | 0.1811 | 0.1846 | 0.1811 | 0 |
| selected | REACQUIRE | disabled | panzer | 3 | 0.1559 | 0.1563 | 0.1559 | 0 |
| selected | REACQUIRE | disabled | pillbox | 3 | 0.1918 | 0.1924 | 0.1918 | 0 |
| selected | REACQUIRE | disabled | red_cross | 4 | 0.1519 | 0.1526 | 0.1519 | 0 |
| selected | REVISIT | disabled | bridge | 60 | 0.1911 | 0.2696 | 0.1911 | 0 |
| selected | REVISIT | disabled | panzer | 100 | 0.1789 | 0.2320 | 0.1789 | 0 |
| selected | REVISIT | disabled | pillbox | 178 | 0.1689 | 0.2424 | 0.1689 | 0 |
| selected | REVISIT | disabled | red_cross | 129 | 0.1185 | 0.1622 | 0.1185 | 0 |
| selected | REVISIT | disabled | tent | 20 | 0.2797 | 0.3121 | 0.2797 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 18 | 0.1100 | 0.1117 | 0.1100 | 0 |
| selected | TAIL | disabled | panzer | 21 | 0.1135 | 0.1143 | 0.1135 | 0 |
| targets | DELIVERY | disabled | bridge | 28 | 0.1603 | 0.1740 | 0.1603 | 0 |
| targets | DELIVERY | disabled | panzer | 25 | 0.1513 | 0.1548 | 0.1513 | 0 |
| targets | DELIVERY | disabled | red_cross | 13 | 0.1483 | 0.1506 | 0.1483 | 0 |
| targets | DELIVERY | drop_circle | bridge | 77 | 0.1357 | 0.1499 | 0.1357 | 0 |
| targets | DELIVERY | drop_circle | panzer | 176 | 0.1322 | 0.1487 | 0.1322 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 74 | 0.1380 | 0.1447 | 0.1380 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 34 | 0.2891 | 0.3016 | 0.2891 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 156 | 2.6330 | 2.6945 | 0.2625 | 85 |
| targets | HIGH_SURVEY | disabled | red_cross | 116 | 0.1576 | 0.1665 | 0.1576 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 55 | 0.3350 | 0.3430 | 0.3350 | 0 |
| targets | LANDING | landing | landing_pad | 129 | 0.0409 | 0.0439 | 0.0409 | 0 |
| targets | REACQUIRE | disabled | bridge | 6 | 0.1811 | 0.1846 | 0.1811 | 0 |
| targets | REACQUIRE | disabled | panzer | 3 | 0.1559 | 0.1563 | 0.1559 | 0 |
| targets | REACQUIRE | disabled | pillbox | 3 | 0.1918 | 0.1924 | 0.1918 | 0 |
| targets | REACQUIRE | disabled | red_cross | 4 | 0.1519 | 0.1526 | 0.1519 | 0 |
| targets | REVISIT | disabled | bridge | 62 | 0.1929 | 0.2790 | 0.1929 | 0 |
| targets | REVISIT | disabled | panzer | 102 | 0.1795 | 0.2361 | 0.1795 | 0 |
| targets | REVISIT | disabled | pillbox | 179 | 0.1691 | 0.2440 | 0.1691 | 0 |
| targets | REVISIT | disabled | red_cross | 133 | 0.1189 | 0.1632 | 0.1189 | 0 |
| targets | REVISIT | disabled | tent | 22 | 0.2830 | 0.3216 | 0.2830 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 20 | 0.1102 | 0.1122 | 0.1102 | 0 |
| targets | TAIL | disabled | panzer | 21 | 0.1135 | 0.1143 | 0.1135 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.017683901846496226,
    "frame_id": "camera_init",
    "stamp_ns": 239031000000,
    "x": 8.738108420429349,
    "y": -4.213088572108194,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.017683901846496226,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.017683901846496226,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
