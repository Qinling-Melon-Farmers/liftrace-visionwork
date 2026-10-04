# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/latest_six_snake2_seed31_20261005_025002

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 23.242 | 23.259 | 2.948 | (0.5234, -3.8118) | 0.1617 | random_tent | 0.1617 | False |
| panzer | 29.790 | 29.823 | 2.880 | (3.8295, -0.9895) | 0.5068 | random_panzer | 0.5068 | False |
| pillbox | 33.473 | 33.551 | 2.857 | (0.7127, 2.1294) | 0.2791 | random_pillbox | 0.2791 | False |
| panzer | 33.651 | 33.731 | 2.853 | (0.7072, 2.1500) | 4.2613 | random_pillbox | 0.2595 | True |
| bridge | 36.125 | 36.214 | 2.873 | (0.6073, 2.2861) | 7.4292 | random_pillbox | 0.1833 | True |
| red_cross | 52.048 | 52.082 | 2.900 | (7.3404, -1.1704) | 0.2240 | random_red_cross | 0.2240 | False |
| bridge | 55.443 | 55.481 | 2.940 | (4.9887, -3.6242) | 0.1070 | random_bridge | 0.1070 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 66.946 | 67.084 | 1.530 | (5.0219, -3.7702) | 0.1546 | random_bridge | 0.1546 | False |
| red_cross | 97.312 | 97.439 | 1.373 | (7.3655, -1.1238) | 0.1710 | random_red_cross | 0.1710 | False |
| panzer | 145.650 | 145.842 | 1.527 | (3.9593, -0.6738) | 0.1654 | random_panzer | 0.1654 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 22 | 7.4478 | 7.4956 | 0.1353 | 14 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 46 | 4.2698 | 4.3302 | 0.3002 | 26 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 8 | 0.2774 | 0.2946 | 0.2774 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 8 | 0.2352 | 0.2426 | 0.2352 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 7 | 0.1656 | 0.1704 | 0.1656 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 21 | 0.1739 | 0.1811 | 0.1739 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 23 | 0.2816 | 4.2794 | 0.2816 | 3 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 12 | 0.2405 | 0.2434 | 0.2405 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 44 | 0.1713 | 0.2123 | 0.1713 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1546 | 0.1546 | 0.1546 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1677 | 0.1677 | 0.1677 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 1 | 0.1710 | 0.1710 | 0.1710 | 0 |
| hint_vision | REVISIT | disabled | panzer | 2 | 0.1658 | 0.1662 | 0.1658 | 0 |
| mapped | DELIVERY | disabled | bridge | 28 | 0.1208 | 0.1233 | 0.1208 | 0 |
| mapped | DELIVERY | disabled | panzer | 27 | 0.0743 | 0.0781 | 0.0743 | 0 |
| mapped | DELIVERY | disabled | red_cross | 17 | 0.0850 | 0.0879 | 0.0850 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 111 | 0.1037 | 0.3947 | 0.1037 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 102 | 0.0686 | 0.0817 | 0.0686 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 85 | 0.0913 | 0.1077 | 0.0913 | 0 |
| mapped | DESCEND | disabled | bridge | 63 | 0.1776 | 0.2137 | 0.1776 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 74 | 0.1902 | 7.4554 | 0.1888 | 13 |
| mapped | HIGH_SURVEY | disabled | panzer | 118 | 0.3405 | 4.4282 | 0.2404 | 54 |
| mapped | HIGH_SURVEY | disabled | pillbox | 11 | 0.2837 | 0.3038 | 0.2837 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 37 | 0.2449 | 0.2485 | 0.2449 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 110 | 0.2413 | 0.3160 | 0.2413 | 0 |
| mapped | LANDING | landing | landing_pad | 79 | 0.0456 | 0.0606 | 0.0456 | 0 |
| mapped | REACQUIRE | disabled | bridge | 2 | 0.1222 | 0.1224 | 0.1222 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 2 | 0.0712 | 0.0716 | 0.0712 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 2 | 0.0867 | 0.0870 | 0.0867 | 0 |
| mapped | REVISIT | disabled | bridge | 192 | 0.1168 | 0.1307 | 0.1168 | 0 |
| mapped | REVISIT | disabled | panzer | 49 | 0.0835 | 0.0852 | 0.0835 | 0 |
| mapped | REVISIT | disabled | pillbox | 102 | 0.0779 | 0.0959 | 0.0779 | 0 |
| mapped | REVISIT | disabled | red_cross | 128 | 0.0793 | 0.0904 | 0.0793 | 0 |
| mapped | TAIL | disabled | landing_pad | 2 | 0.0609 | 0.0610 | 0.0609 | 0 |
| mapped | TAIL | disabled | panzer | 36 | 0.0453 | 0.0546 | 0.0453 | 0 |
| selected | DELIVERY | disabled | bridge | 25 | 0.1525 | 0.1541 | 0.1525 | 0 |
| selected | DELIVERY | disabled | panzer | 24 | 0.1547 | 0.1622 | 0.1547 | 0 |
| selected | DELIVERY | disabled | red_cross | 16 | 0.1592 | 0.1673 | 0.1592 | 0 |
| selected | DELIVERY | drop_circle | bridge | 99 | 0.1458 | 0.1504 | 0.1458 | 0 |
| selected | DELIVERY | drop_circle | panzer | 93 | 0.1310 | 0.1477 | 0.1310 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 56 | 0.1360 | 0.1474 | 0.1360 | 0 |
| selected | DESCEND | disabled | bridge | 63 | 0.1885 | 0.1913 | 0.1885 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 62 | 0.1766 | 0.1849 | 0.1766 | 3 |
| selected | HIGH_SURVEY | disabled | panzer | 49 | 0.3339 | 4.3063 | 0.2647 | 13 |
| selected | HIGH_SURVEY | disabled | pillbox | 1 | 0.2771 | 0.2771 | 0.2771 | 0 |
| selected | HIGH_SURVEY | disabled | red_cross | 30 | 0.2429 | 0.2440 | 0.2429 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 107 | 0.1757 | 0.2195 | 0.1757 | 0 |
| selected | REACQUIRE | disabled | bridge | 2 | 0.1545 | 0.1546 | 0.1545 | 0 |
| selected | REACQUIRE | disabled | pillbox | 2 | 0.1668 | 0.1676 | 0.1668 | 0 |
| selected | REACQUIRE | disabled | red_cross | 2 | 0.1703 | 0.1709 | 0.1703 | 0 |
| selected | REVISIT | disabled | bridge | 190 | 0.1552 | 0.1738 | 0.1552 | 0 |
| selected | REVISIT | disabled | panzer | 47 | 0.1853 | 0.2173 | 0.1853 | 0 |
| selected | REVISIT | disabled | pillbox | 100 | 0.1397 | 0.2216 | 0.1397 | 0 |
| selected | REVISIT | disabled | red_cross | 113 | 0.1069 | 0.2111 | 0.1069 | 0 |
| selected | TAIL | disabled | panzer | 36 | 0.1038 | 0.1081 | 0.1038 | 0 |
| targets | DELIVERY | disabled | bridge | 28 | 0.1523 | 0.1541 | 0.1523 | 0 |
| targets | DELIVERY | disabled | panzer | 26 | 0.1541 | 0.1622 | 0.1541 | 0 |
| targets | DELIVERY | disabled | red_cross | 16 | 0.1592 | 0.1673 | 0.1592 | 0 |
| targets | DELIVERY | drop_circle | bridge | 109 | 0.1452 | 0.1505 | 0.1452 | 0 |
| targets | DELIVERY | drop_circle | panzer | 101 | 0.1303 | 0.1485 | 0.1303 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 69 | 0.1352 | 0.1483 | 0.1352 | 0 |
| targets | DESCEND | disabled | bridge | 63 | 0.1885 | 0.1913 | 0.1885 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 66 | 0.1766 | 7.3319 | 0.1766 | 5 |
| targets | HIGH_SURVEY | disabled | panzer | 80 | 0.2887 | 4.3077 | 0.2786 | 17 |
| targets | HIGH_SURVEY | disabled | pillbox | 4 | 0.2782 | 0.2805 | 0.2782 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 32 | 0.2428 | 0.2440 | 0.2428 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 109 | 0.1748 | 0.2194 | 0.1748 | 0 |
| targets | LANDING | landing | landing_pad | 79 | 0.0528 | 0.0610 | 0.0528 | 0 |
| targets | REACQUIRE | disabled | bridge | 2 | 0.1545 | 0.1546 | 0.1545 | 0 |
| targets | REACQUIRE | disabled | pillbox | 2 | 0.1668 | 0.1676 | 0.1668 | 0 |
| targets | REACQUIRE | disabled | red_cross | 2 | 0.1703 | 0.1709 | 0.1703 | 0 |
| targets | REVISIT | disabled | bridge | 192 | 0.1551 | 0.1738 | 0.1551 | 0 |
| targets | REVISIT | disabled | panzer | 49 | 0.1864 | 0.2218 | 0.1864 | 0 |
| targets | REVISIT | disabled | pillbox | 101 | 0.1401 | 0.2250 | 0.1401 | 0 |
| targets | REVISIT | disabled | red_cross | 117 | 0.1073 | 0.2167 | 0.1073 | 0 |
| targets | TAIL | disabled | landing_pad | 1 | 0.0610 | 0.0610 | 0.0610 | 0 |
| targets | TAIL | disabled | panzer | 36 | 0.1038 | 0.1081 | 0.1038 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.02360161094714426,
    "frame_id": "camera_init",
    "stamp_ns": 225487000000,
    "x": 8.729520951245457,
    "y": -4.211732203604157,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.02360161094714426,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.02360161094714426,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
