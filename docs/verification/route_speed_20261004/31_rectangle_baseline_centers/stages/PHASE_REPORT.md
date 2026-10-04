# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_rectangle_baseline_seed31_20261004_223346

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 22.209 | 22.229 | 2.893 | (0.5103, -3.8420) | 0.1651 | random_tent | 0.1651 | False |
| bridge | 27.015 | 27.054 | 2.850 | (4.6576, -3.8474) | 0.4868 | random_bridge | 0.4868 | False |
| red_cross | 35.240 | 35.311 | 2.835 | (7.2995, -1.3552) | 0.4100 | random_red_cross | 0.4100 | False |
| panzer | 35.655 | 35.794 | 2.847 | (3.9331, -0.9670) | 0.4542 | random_panzer | 0.4542 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 35.098 | 35.998 | 2.832 | (4.9660, -3.8516) | 0.2529 | random_bridge | 0.2529 | False |
| panzer | 35.784 | 35.998 | 2.851 | (3.9269, -0.9268) | 0.4164 | random_panzer | 0.4164 | False |
| red_cross | 35.784 | 35.998 | 2.851 | (7.2511, -1.2907) | 0.3725 | random_red_cross | 0.3725 | False |
| tent | 27.165 | 35.998 | 2.849 | (0.5417, -4.0409) | 0.2105 | random_tent | 0.2105 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| panzer | 45.607 | 45.726 | 1.395 | (3.9868, -0.7002) | 0.1820 | random_panzer | 0.1820 | False |
| bridge | 60.712 | 60.835 | 1.504 | (4.9840, -3.8290) | 0.2243 | random_bridge | 0.2243 | False |
| red_cross | 88.225 | 88.328 | 1.366 | (7.3943, -1.0975) | 0.1354 | random_red_cross | 0.1354 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 17 | 0.3368 | 0.4765 | 0.3368 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 3 | 0.4392 | 0.4527 | 0.4392 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 11 | 0.3624 | 0.3981 | 0.3624 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 5 | 0.1532 | 0.1664 | 0.1532 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 60 | 0.2727 | 0.2840 | 0.2727 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 39 | 0.1728 | 0.2055 | 0.1728 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.2243 | 0.2243 | 0.2243 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1820 | 0.1820 | 0.1820 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 3 | 0.1380 | 0.1406 | 0.1380 | 0 |
| mapped | DELIVERY | disabled | bridge | 29 | 0.1339 | 0.1372 | 0.1339 | 0 |
| mapped | DELIVERY | disabled | panzer | 30 | 0.0824 | 0.0979 | 0.0824 | 0 |
| mapped | DELIVERY | disabled | red_cross | 36 | 0.0747 | 0.0945 | 0.0747 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 102 | 0.1130 | 0.1379 | 0.1130 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 113 | 0.1108 | 0.1441 | 0.1108 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 145 | 0.0750 | 0.0923 | 0.0750 | 0 |
| mapped | DESCEND | disabled | panzer | 41 | 0.2465 | 0.3449 | 0.2465 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 147 | 0.2737 | 0.3100 | 0.2737 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 5 | 0.3194 | 0.3529 | 0.3194 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 4 | 0.3475 | 0.3490 | 0.3475 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 99 | 0.2181 | 0.3139 | 0.2181 | 0 |
| mapped | LANDING | landing | landing_pad | 92 | 0.0309 | 0.0595 | 0.0309 | 0 |
| mapped | REACQUIRE | disabled | bridge | 2 | 0.1371 | 0.1371 | 0.1371 | 0 |
| mapped | REACQUIRE | disabled | panzer | 2 | 0.0833 | 0.0838 | 0.0833 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 6 | 0.0814 | 0.0836 | 0.0814 | 0 |
| mapped | REVISIT | disabled | bridge | 160 | 0.1227 | 0.1420 | 0.1227 | 0 |
| mapped | REVISIT | disabled | panzer | 67 | 0.1061 | 0.1566 | 0.1061 | 0 |
| mapped | REVISIT | disabled | red_cross | 15 | 0.0921 | 0.0977 | 0.0921 | 0 |
| mapped | TAIL | disabled | red_cross | 29 | 0.1001 | 0.1236 | 0.1001 | 0 |
| selected | DELIVERY | disabled | bridge | 25 | 0.2182 | 0.2229 | 0.2182 | 0 |
| selected | DELIVERY | disabled | panzer | 27 | 0.1633 | 0.1776 | 0.1633 | 0 |
| selected | DELIVERY | disabled | red_cross | 30 | 0.1073 | 0.1260 | 0.1073 | 0 |
| selected | DELIVERY | drop_circle | bridge | 96 | 0.2003 | 0.2140 | 0.2003 | 0 |
| selected | DELIVERY | drop_circle | panzer | 102 | 0.1462 | 0.1541 | 0.1462 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 121 | 0.0911 | 0.0979 | 0.0911 | 0 |
| selected | DESCEND | disabled | panzer | 41 | 0.2963 | 0.3204 | 0.2963 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 145 | 0.2715 | 0.2844 | 0.2715 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 2 | 0.3213 | 0.3221 | 0.3213 | 0 |
| selected | HIGH_SURVEY | disabled | red_cross | 2 | 0.3462 | 0.3466 | 0.3462 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 96 | 0.1787 | 0.2148 | 0.1787 | 0 |
| selected | REACQUIRE | disabled | bridge | 2 | 0.2241 | 0.2243 | 0.2241 | 0 |
| selected | REACQUIRE | disabled | panzer | 2 | 0.1813 | 0.1819 | 0.1813 | 0 |
| selected | REACQUIRE | disabled | red_cross | 5 | 0.1354 | 0.1403 | 0.1354 | 0 |
| selected | REVISIT | disabled | bridge | 156 | 0.1725 | 0.2445 | 0.1725 | 0 |
| selected | REVISIT | disabled | panzer | 65 | 0.1215 | 0.2263 | 0.1215 | 0 |
| selected | REVISIT | disabled | red_cross | 13 | 0.1706 | 0.2276 | 0.1706 | 0 |
| selected | TAIL | disabled | red_cross | 29 | 0.0779 | 0.0799 | 0.0779 | 0 |
| targets | DELIVERY | disabled | bridge | 27 | 0.2178 | 0.2228 | 0.2178 | 0 |
| targets | DELIVERY | disabled | panzer | 29 | 0.1623 | 0.1774 | 0.1623 | 0 |
| targets | DELIVERY | disabled | red_cross | 32 | 0.1063 | 0.1258 | 0.1063 | 0 |
| targets | DELIVERY | drop_circle | bridge | 100 | 0.2003 | 0.2147 | 0.2003 | 0 |
| targets | DELIVERY | drop_circle | panzer | 111 | 0.1461 | 0.1550 | 0.1461 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 131 | 0.0912 | 0.0987 | 0.0912 | 0 |
| targets | DESCEND | disabled | panzer | 41 | 0.2963 | 0.3204 | 0.2963 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 147 | 0.2717 | 0.2844 | 0.2717 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 4 | 0.3235 | 0.3302 | 0.3235 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 4 | 0.3453 | 0.3465 | 0.3453 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 98 | 0.1778 | 0.2147 | 0.1778 | 0 |
| targets | LANDING | landing | landing_pad | 91 | 0.0449 | 0.0596 | 0.0449 | 0 |
| targets | REACQUIRE | disabled | bridge | 2 | 0.2241 | 0.2243 | 0.2241 | 0 |
| targets | REACQUIRE | disabled | panzer | 2 | 0.1813 | 0.1819 | 0.1813 | 0 |
| targets | REACQUIRE | disabled | red_cross | 5 | 0.1354 | 0.1403 | 0.1354 | 0 |
| targets | REVISIT | disabled | bridge | 160 | 0.1725 | 0.2460 | 0.1725 | 0 |
| targets | REVISIT | disabled | panzer | 67 | 0.1834 | 0.2296 | 0.1834 | 0 |
| targets | REVISIT | disabled | red_cross | 15 | 0.1773 | 0.2727 | 0.1773 | 0 |
| targets | TAIL | disabled | red_cross | 29 | 0.0779 | 0.0799 | 0.0779 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.03416492381894644,
    "frame_id": "camera_init",
    "stamp_ns": 188109000000,
    "x": 8.721813491746593,
    "y": -4.219307065339793,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.03416492381894644,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.03416492381894644,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
