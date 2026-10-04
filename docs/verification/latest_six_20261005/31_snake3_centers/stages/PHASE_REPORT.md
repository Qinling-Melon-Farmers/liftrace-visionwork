# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/latest_six_snake3_seed31_20261005_030704

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 22.369 | 22.607 | 2.918 | (0.5268, -3.7968) | 0.1651 | random_tent | 0.1651 | False |
| panzer | 28.212 | 28.299 | 2.918 | (3.8458, -0.9478) | 0.4620 | random_panzer | 0.4620 | False |
| bridge | 31.900 | 32.146 | 2.808 | (0.6073, 2.1282) | 7.3039 | random_pillbox | 0.3108 | True |
| panzer | 32.024 | 32.340 | 2.805 | (0.6005, 2.1538) | 4.3472 | random_pillbox | 0.2915 | True |
| pillbox | 32.148 | 32.437 | 2.802 | (0.5879, 2.1634) | 0.2898 | random_pillbox | 0.2898 | False |
| bridge | 50.380 | 50.594 | 2.890 | (4.9655, -3.5562) | 0.1514 | random_bridge | 0.1514 | False |
| red_cross | 60.459 | 60.650 | 2.891 | (7.3306, -1.2722) | 0.3214 | random_red_cross | 0.3214 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| panzer | 88.646 | 88.840 | 1.383 | (3.9376, -0.6599) | 0.1635 | random_panzer | 0.1635 | False |
| bridge | 103.271 | 103.533 | 1.322 | (5.0032, -3.8051) | 0.1941 | random_bridge | 0.1941 | False |
| red_cross | 141.924 | 142.121 | 1.534 | (7.3835, -1.0972) | 0.1393 | random_red_cross | 0.1393 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 14 | 0.1686 | 7.5772 | 0.1562 | 6 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 73 | 0.3234 | 4.2904 | 0.2915 | 19 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 20 | 0.2713 | 0.2883 | 0.2713 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 9 | 0.2668 | 0.3162 | 0.2668 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 8 | 0.1441 | 0.1639 | 0.1441 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 77 | 0.2202 | 0.2335 | 0.2202 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 52 | 0.1747 | 4.3515 | 0.1747 | 6 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 1 | 0.2697 | 0.2697 | 0.2697 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 38 | 0.1707 | 0.2039 | 0.1707 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1941 | 0.1941 | 0.1941 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1635 | 0.1635 | 0.1635 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 5 | 0.1419 | 0.1445 | 0.1419 | 0 |
| mapped | DELIVERY | disabled | bridge | 122 | 0.0912 | 0.0973 | 0.0912 | 0 |
| mapped | DELIVERY | disabled | panzer | 27 | 0.0663 | 0.0755 | 0.0663 | 0 |
| mapped | DELIVERY | disabled | red_cross | 87 | 0.0987 | 0.1119 | 0.0987 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 123 | 0.0884 | 0.1019 | 0.0884 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 92 | 0.0706 | 0.0880 | 0.0706 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 140 | 0.0771 | 0.1079 | 0.0771 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 171 | 0.2181 | 0.2706 | 0.2181 | 3 |
| mapped | HIGH_SURVEY | disabled | panzer | 153 | 0.2075 | 4.4571 | 0.2067 | 29 |
| mapped | HIGH_SURVEY | disabled | pillbox | 4 | 0.3020 | 0.3165 | 0.3020 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 8 | 0.2688 | 0.2718 | 0.2688 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 84 | 0.2089 | 0.3397 | 0.2089 | 0 |
| mapped | LANDING | landing | landing_pad | 93 | 0.0411 | 0.0509 | 0.0411 | 0 |
| mapped | REACQUIRE | disabled | bridge | 5 | 0.0854 | 0.0949 | 0.0854 | 0 |
| mapped | REACQUIRE | disabled | panzer | 3 | 0.0766 | 0.0771 | 0.0766 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 9 | 0.0997 | 0.1000 | 0.0997 | 0 |
| mapped | REVISIT | disabled | bridge | 167 | 0.0961 | 0.1167 | 0.0961 | 0 |
| mapped | REVISIT | disabled | panzer | 61 | 0.0806 | 0.0978 | 0.0806 | 0 |
| mapped | REVISIT | disabled | red_cross | 26 | 0.0983 | 0.1010 | 0.0983 | 0 |
| mapped | TAIL | disabled | landing_pad | 3 | 0.0358 | 0.0373 | 0.0358 | 0 |
| mapped | TAIL | disabled | red_cross | 30 | 0.1160 | 0.1322 | 0.1160 | 0 |
| selected | DELIVERY | disabled | bridge | 118 | 0.1435 | 0.1886 | 0.1435 | 0 |
| selected | DELIVERY | disabled | panzer | 22 | 0.1555 | 0.1610 | 0.1555 | 0 |
| selected | DELIVERY | disabled | red_cross | 82 | 0.0883 | 0.1328 | 0.0883 | 0 |
| selected | DELIVERY | drop_circle | bridge | 112 | 0.1659 | 0.1806 | 0.1659 | 0 |
| selected | DELIVERY | drop_circle | panzer | 85 | 0.1371 | 0.1495 | 0.1371 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 104 | 0.1056 | 0.1228 | 0.1056 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 166 | 0.2190 | 0.2338 | 0.2190 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 129 | 0.1748 | 4.3553 | 0.1748 | 14 |
| selected | HIGH_SURVEY | disabled | red_cross | 6 | 0.2694 | 0.2702 | 0.2694 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 82 | 0.1741 | 0.2157 | 0.1741 | 0 |
| selected | REACQUIRE | disabled | bridge | 5 | 0.1931 | 0.1940 | 0.1931 | 0 |
| selected | REACQUIRE | disabled | panzer | 3 | 0.1629 | 0.1634 | 0.1629 | 0 |
| selected | REACQUIRE | disabled | red_cross | 6 | 0.1412 | 0.1445 | 0.1412 | 0 |
| selected | REVISIT | disabled | bridge | 164 | 0.1335 | 0.2118 | 0.1335 | 0 |
| selected | REVISIT | disabled | panzer | 59 | 0.1648 | 0.1819 | 0.1648 | 0 |
| selected | REVISIT | disabled | red_cross | 18 | 0.1681 | 0.2131 | 0.1681 | 0 |
| selected | TAIL | disabled | red_cross | 29 | 0.0922 | 0.0934 | 0.0922 | 0 |
| targets | DELIVERY | disabled | bridge | 120 | 0.1437 | 0.1886 | 0.1437 | 0 |
| targets | DELIVERY | disabled | panzer | 24 | 0.1549 | 0.1609 | 0.1549 | 0 |
| targets | DELIVERY | disabled | red_cross | 84 | 0.0884 | 0.1327 | 0.0884 | 0 |
| targets | DELIVERY | drop_circle | bridge | 119 | 0.1655 | 0.1813 | 0.1655 | 0 |
| targets | DELIVERY | drop_circle | panzer | 91 | 0.1369 | 0.1503 | 0.1369 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 118 | 0.1066 | 0.1246 | 0.1066 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 169 | 0.2189 | 0.2338 | 0.2189 | 1 |
| targets | HIGH_SURVEY | disabled | panzer | 138 | 0.1756 | 4.3498 | 0.1756 | 17 |
| targets | HIGH_SURVEY | disabled | red_cross | 8 | 0.2695 | 0.2719 | 0.2695 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 84 | 0.1733 | 0.2156 | 0.1733 | 0 |
| targets | LANDING | landing | landing_pad | 93 | 0.0435 | 0.0461 | 0.0435 | 0 |
| targets | REACQUIRE | disabled | bridge | 5 | 0.1931 | 0.1940 | 0.1931 | 0 |
| targets | REACQUIRE | disabled | panzer | 3 | 0.1629 | 0.1634 | 0.1629 | 0 |
| targets | REACQUIRE | disabled | red_cross | 6 | 0.1412 | 0.1445 | 0.1412 | 0 |
| targets | REVISIT | disabled | bridge | 167 | 0.1335 | 0.2129 | 0.1335 | 0 |
| targets | REVISIT | disabled | panzer | 61 | 0.1654 | 0.1832 | 0.1654 | 0 |
| targets | REVISIT | disabled | red_cross | 20 | 0.1720 | 0.2352 | 0.1720 | 0 |
| targets | TAIL | disabled | red_cross | 29 | 0.0922 | 0.0934 | 0.0922 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.022281814156458232,
    "frame_id": "camera_init",
    "stamp_ns": 238419000000,
    "x": 8.736451641693572,
    "y": -4.217689579681371,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.022281814156458232,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.022281814156458232,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
