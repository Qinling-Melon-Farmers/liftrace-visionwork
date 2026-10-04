# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_rectangle_baseline_seed38_20261005_001117

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 17.386 | 17.493 | 2.526 | (1.0049, -0.6312) | 0.1712 | random_red_cross | 0.1712 | False |
| panzer | 26.956 | 27.011 | 2.778 | (4.4135, -3.4961) | 2.7133 | random_pillbox | 0.2986 | True |
| pillbox | 26.990 | 27.070 | 2.777 | (4.4353, -3.4872) | 0.2765 | random_pillbox | 0.2765 | False |
| panzer | 29.402 | 29.469 | 2.742 | (6.6977, -4.3375) | 0.4088 | random_panzer | 0.4088 | False |
| tent | 34.610 | 34.672 | 2.713 | (4.7362, -1.1464) | 0.4676 | random_tent | 0.4676 | False |
| bridge | 34.711 | 34.769 | 2.712 | (4.4495, -3.4642) | 5.3427 | random_pillbox | 0.2505 | True |
| bridge | 37.614 | 37.675 | 2.748 | (3.8802, 1.4463) | 0.4429 | random_bridge | 0.4429 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| panzer | 94.259 | 94.378 | 1.287 | (7.0096, -4.3053) | 0.1780 | random_panzer | 0.1780 | False |
| red_cross | 117.772 | 117.841 | 1.325 | (0.9804, -0.5949) | 0.1453 | random_red_cross | 0.1453 | False |
| bridge | 132.937 | 133.025 | 1.330 | (3.9190, 1.6737) | 0.2200 | random_bridge | 0.2200 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 14 | 0.3467 | 2.1579 | 0.3352 | 1 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 27 | 0.4088 | 2.6743 | 0.2476 | 13 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 3 | 0.2071 | 0.2695 | 0.2071 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1717 | 0.1726 | 0.1717 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 9 | 0.3975 | 0.4599 | 0.3975 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 22 | 0.2799 | 0.3008 | 0.2799 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 71 | 2.6297 | 2.6610 | 0.2460 | 41 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 42 | 0.1629 | 0.1659 | 0.1629 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 22 | 0.3290 | 0.3307 | 0.3290 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.2200 | 0.2200 | 0.2200 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1780 | 0.1780 | 0.1780 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 3 | 0.1458 | 0.1463 | 0.1458 | 0 |
| mapped | DELIVERY | disabled | bridge | 32 | 0.0799 | 0.0940 | 0.0799 | 0 |
| mapped | DELIVERY | disabled | panzer | 28 | 0.0856 | 0.0932 | 0.0856 | 0 |
| mapped | DELIVERY | disabled | red_cross | 34 | 0.0683 | 0.0782 | 0.0683 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 82 | 0.1010 | 0.1184 | 0.1010 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 130 | 0.0937 | 0.1139 | 0.0937 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 79 | 0.0707 | 0.0815 | 0.0707 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 61 | 0.3120 | 0.3637 | 0.3120 | 1 |
| mapped | HIGH_SURVEY | disabled | panzer | 216 | 2.5912 | 2.6745 | 0.2515 | 139 |
| mapped | HIGH_SURVEY | disabled | pillbox | 2 | 0.2092 | 0.2186 | 0.2092 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 146 | 0.1788 | 0.2054 | 0.1788 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 64 | 0.3294 | 0.3401 | 0.3294 | 0 |
| mapped | LANDING | landing | landing_pad | 71 | 0.0340 | 0.0542 | 0.0340 | 0 |
| mapped | REACQUIRE | disabled | bridge | 1 | 0.0722 | 0.0722 | 0.0722 | 0 |
| mapped | REACQUIRE | disabled | panzer | 2 | 0.0849 | 0.0850 | 0.0849 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 3 | 0.0702 | 0.0705 | 0.0702 | 0 |
| mapped | REVISIT | disabled | bridge | 32 | 0.0609 | 0.0696 | 0.0609 | 0 |
| mapped | REVISIT | disabled | panzer | 87 | 0.0856 | 0.1029 | 0.0856 | 0 |
| mapped | REVISIT | disabled | pillbox | 40 | 0.1016 | 0.1248 | 0.1016 | 0 |
| mapped | REVISIT | disabled | red_cross | 72 | 0.0804 | 0.1316 | 0.0804 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 11 | 0.1190 | 0.1257 | 0.1190 | 0 |
| mapped | TAIL | disabled | bridge | 43 | 0.1000 | 0.1096 | 0.1000 | 0 |
| selected | DELIVERY | disabled | bridge | 30 | 0.2001 | 0.2163 | 0.2001 | 0 |
| selected | DELIVERY | disabled | panzer | 25 | 0.1690 | 0.1758 | 0.1690 | 0 |
| selected | DELIVERY | disabled | red_cross | 30 | 0.1271 | 0.1440 | 0.1271 | 0 |
| selected | DELIVERY | drop_circle | bridge | 75 | 0.1726 | 0.1897 | 0.1726 | 0 |
| selected | DELIVERY | drop_circle | panzer | 120 | 0.1480 | 0.1640 | 0.1480 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 63 | 0.1274 | 0.1357 | 0.1274 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 58 | 0.2830 | 0.3105 | 0.2830 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 120 | 0.2498 | 2.6602 | 0.2435 | 45 |
| selected | HIGH_SURVEY | disabled | red_cross | 114 | 0.1625 | 0.1666 | 0.1625 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 58 | 0.3292 | 0.3307 | 0.3292 | 0 |
| selected | REACQUIRE | disabled | bridge | 1 | 0.2200 | 0.2200 | 0.2200 | 0 |
| selected | REACQUIRE | disabled | panzer | 2 | 0.1776 | 0.1780 | 0.1776 | 0 |
| selected | REACQUIRE | disabled | red_cross | 3 | 0.1458 | 0.1463 | 0.1458 | 0 |
| selected | REVISIT | disabled | bridge | 30 | 0.2509 | 0.2939 | 0.2509 | 0 |
| selected | REVISIT | disabled | panzer | 123 | 0.2020 | 2.6256 | 0.2020 | 38 |
| selected | REVISIT | disabled | red_cross | 70 | 0.1128 | 0.1606 | 0.1128 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 9 | 0.1180 | 0.1200 | 0.1180 | 0 |
| selected | TAIL | disabled | bridge | 43 | 0.1433 | 0.1484 | 0.1433 | 0 |
| targets | DELIVERY | disabled | bridge | 32 | 0.1991 | 0.2162 | 0.1991 | 0 |
| targets | DELIVERY | disabled | panzer | 27 | 0.1685 | 0.1757 | 0.1685 | 0 |
| targets | DELIVERY | disabled | red_cross | 32 | 0.1164 | 0.1440 | 0.1164 | 0 |
| targets | DELIVERY | drop_circle | bridge | 81 | 0.1722 | 0.1909 | 0.1722 | 0 |
| targets | DELIVERY | drop_circle | panzer | 127 | 0.1476 | 0.1647 | 0.1476 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 71 | 0.1270 | 0.1363 | 0.1270 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 60 | 0.2838 | 0.3104 | 0.2838 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 160 | 2.6201 | 2.6610 | 0.2447 | 83 |
| targets | HIGH_SURVEY | disabled | red_cross | 116 | 0.1623 | 0.1666 | 0.1623 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 60 | 0.3293 | 0.3319 | 0.3293 | 0 |
| targets | LANDING | landing | landing_pad | 71 | 0.0475 | 0.0536 | 0.0475 | 0 |
| targets | REACQUIRE | disabled | bridge | 1 | 0.2200 | 0.2200 | 0.2200 | 0 |
| targets | REACQUIRE | disabled | panzer | 2 | 0.1776 | 0.1780 | 0.1776 | 0 |
| targets | REACQUIRE | disabled | red_cross | 3 | 0.1458 | 0.1463 | 0.1458 | 0 |
| targets | REVISIT | disabled | bridge | 32 | 0.2535 | 0.3017 | 0.2535 | 0 |
| targets | REVISIT | disabled | panzer | 127 | 0.2042 | 2.6260 | 0.2042 | 40 |
| targets | REVISIT | disabled | red_cross | 72 | 0.1129 | 0.1622 | 0.1129 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 11 | 0.1174 | 0.1199 | 0.1174 | 0 |
| targets | TAIL | disabled | bridge | 43 | 0.1433 | 0.1484 | 0.1433 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.010908940258288601,
    "frame_id": "camera_init",
    "stamp_ns": 227299000000,
    "x": 8.739157443648725,
    "y": -4.201201644426752,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.010908940258288601,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.010908940258288601,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
