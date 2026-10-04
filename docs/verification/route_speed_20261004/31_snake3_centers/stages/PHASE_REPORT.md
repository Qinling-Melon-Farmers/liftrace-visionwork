# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_snake3_seed31_20261004_235752

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 22.759 | 22.796 | 2.891 | (0.5173, -3.8206) | 0.1641 | random_tent | 0.1641 | False |
| panzer | 29.242 | 29.360 | 2.815 | (3.8188, -0.9821) | 0.5043 | random_panzer | 0.5043 | False |
| pillbox | 32.925 | 32.948 | 2.805 | (0.6710, 2.0913) | 0.3240 | random_pillbox | 0.3240 | False |
| panzer | 33.160 | 33.179 | 2.801 | (0.6535, 2.1397) | 4.2969 | random_pillbox | 0.2823 | True |
| bridge | 35.541 | 35.610 | 2.802 | (0.6155, 2.3004) | 7.4356 | random_pillbox | 0.1679 | True |
| bridge | 53.902 | 53.975 | 2.836 | (4.9921, -3.6157) | 0.1048 | random_bridge | 0.1048 | False |
| red_cross | 64.251 | 64.286 | 2.813 | (7.3260, -1.1570) | 0.2192 | random_red_cross | 0.2192 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| panzer | 92.325 | 92.432 | 1.383 | (3.9483, -0.6496) | 0.1492 | random_panzer | 0.1492 | False |
| bridge | 107.257 | 107.406 | 1.379 | (5.0026, -3.7984) | 0.1886 | random_bridge | 0.1886 | False |
| red_cross | 139.916 | 140.021 | 1.439 | (7.3684, -1.1032) | 0.1514 | random_red_cross | 0.1514 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 13 | 0.1591 | 7.5082 | 0.1198 | 6 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 55 | 0.3968 | 4.3237 | 0.3048 | 19 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 23 | 0.2788 | 0.3039 | 0.2788 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 10 | 0.2273 | 0.2325 | 0.2273 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 8 | 0.1570 | 0.1630 | 0.1570 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 79 | 0.2050 | 0.2156 | 0.2050 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 32 | 0.1413 | 4.2991 | 0.1413 | 3 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 17 | 0.2283 | 0.2292 | 0.2283 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 44 | 0.1678 | 0.2039 | 0.1678 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1886 | 0.1886 | 0.1886 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 2 | 0.1495 | 0.1498 | 0.1495 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1518 | 0.1522 | 0.1518 | 0 |
| mapped | DELIVERY | disabled | bridge | 21 | 0.0814 | 0.0850 | 0.0814 | 0 |
| mapped | DELIVERY | disabled | panzer | 35 | 0.0744 | 0.0826 | 0.0744 | 0 |
| mapped | DELIVERY | disabled | red_cross | 16 | 0.0907 | 0.0922 | 0.0907 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 141 | 0.0882 | 0.1157 | 0.0882 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 79 | 0.0765 | 0.0939 | 0.0765 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 161 | 0.0834 | 0.0930 | 0.0834 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 194 | 0.2194 | 0.2373 | 0.2193 | 2 |
| mapped | HIGH_SURVEY | disabled | panzer | 119 | 0.2385 | 4.3826 | 0.2361 | 38 |
| mapped | HIGH_SURVEY | disabled | pillbox | 16 | 0.3054 | 0.3142 | 0.3054 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 54 | 0.2205 | 0.2345 | 0.2205 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 111 | 0.2282 | 0.2997 | 0.2282 | 0 |
| mapped | LANDING | landing | landing_pad | 81 | 0.0557 | 0.0730 | 0.0557 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.0820 | 0.0829 | 0.0820 | 0 |
| mapped | REACQUIRE | disabled | panzer | 3 | 0.0746 | 0.0751 | 0.0746 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 3 | 0.0903 | 0.0915 | 0.0903 | 0 |
| mapped | REVISIT | disabled | bridge | 130 | 0.0805 | 0.0962 | 0.0805 | 0 |
| mapped | REVISIT | disabled | panzer | 65 | 0.0753 | 0.0861 | 0.0753 | 0 |
| mapped | REVISIT | disabled | red_cross | 35 | 0.0821 | 0.0889 | 0.0821 | 0 |
| mapped | TAIL | disabled | landing_pad | 1 | 0.0734 | 0.0734 | 0.0734 | 0 |
| mapped | TAIL | disabled | red_cross | 3 | 0.0476 | 0.0484 | 0.0476 | 0 |
| selected | DELIVERY | disabled | bridge | 18 | 0.1831 | 0.1867 | 0.1831 | 0 |
| selected | DELIVERY | disabled | panzer | 33 | 0.1389 | 0.1468 | 0.1389 | 0 |
| selected | DELIVERY | disabled | red_cross | 14 | 0.1452 | 0.1493 | 0.1452 | 0 |
| selected | DELIVERY | drop_circle | bridge | 132 | 0.1636 | 0.1766 | 0.1636 | 0 |
| selected | DELIVERY | drop_circle | panzer | 72 | 0.1235 | 0.1306 | 0.1235 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 125 | 0.1179 | 0.1357 | 0.1179 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 190 | 0.2059 | 0.2159 | 0.2059 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 94 | 0.1576 | 4.3140 | 0.1576 | 19 |
| selected | HIGH_SURVEY | disabled | pillbox | 4 | 0.3002 | 0.3029 | 0.3002 | 0 |
| selected | HIGH_SURVEY | disabled | red_cross | 38 | 0.2270 | 0.2304 | 0.2270 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 107 | 0.1724 | 0.2124 | 0.1724 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.1881 | 0.1885 | 0.1881 | 0 |
| selected | REACQUIRE | disabled | panzer | 3 | 0.1492 | 0.1498 | 0.1492 | 0 |
| selected | REACQUIRE | disabled | red_cross | 3 | 0.1514 | 0.1521 | 0.1514 | 0 |
| selected | REVISIT | disabled | bridge | 128 | 0.1434 | 0.2095 | 0.1434 | 0 |
| selected | REVISIT | disabled | panzer | 63 | 0.1527 | 0.1797 | 0.1527 | 0 |
| selected | REVISIT | disabled | red_cross | 32 | 0.1717 | 0.2024 | 0.1717 | 0 |
| targets | DELIVERY | disabled | bridge | 20 | 0.1826 | 0.1866 | 0.1826 | 0 |
| targets | DELIVERY | disabled | panzer | 35 | 0.1384 | 0.1467 | 0.1384 | 0 |
| targets | DELIVERY | disabled | red_cross | 14 | 0.1452 | 0.1493 | 0.1452 | 0 |
| targets | DELIVERY | drop_circle | bridge | 139 | 0.1633 | 0.1772 | 0.1633 | 0 |
| targets | DELIVERY | drop_circle | panzer | 77 | 0.1234 | 0.1312 | 0.1234 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 134 | 0.1184 | 0.1370 | 0.1184 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 192 | 0.2057 | 0.2159 | 0.2057 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 100 | 0.1590 | 4.3152 | 0.1590 | 21 |
| targets | HIGH_SURVEY | disabled | pillbox | 6 | 0.2988 | 0.3028 | 0.2988 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 40 | 0.2273 | 0.2303 | 0.2273 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 111 | 0.1724 | 0.2140 | 0.1724 | 0 |
| targets | LANDING | landing | landing_pad | 81 | 0.0648 | 0.0735 | 0.0648 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.1881 | 0.1885 | 0.1881 | 0 |
| targets | REACQUIRE | disabled | panzer | 3 | 0.1492 | 0.1498 | 0.1492 | 0 |
| targets | REACQUIRE | disabled | red_cross | 3 | 0.1514 | 0.1521 | 0.1514 | 0 |
| targets | REVISIT | disabled | bridge | 130 | 0.1436 | 0.2106 | 0.1436 | 0 |
| targets | REVISIT | disabled | panzer | 65 | 0.1535 | 0.1825 | 0.1535 | 0 |
| targets | REVISIT | disabled | red_cross | 34 | 0.1733 | 0.2085 | 0.1733 | 0 |
| targets | TAIL | disabled | red_cross | 1 | 0.0993 | 0.0993 | 0.0993 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.032208978272379424,
    "frame_id": "camera_init",
    "stamp_ns": 224169000000,
    "x": 8.720729932019148,
    "y": -4.2134417782211635,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.032208978272379424,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.032208978272379424,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
