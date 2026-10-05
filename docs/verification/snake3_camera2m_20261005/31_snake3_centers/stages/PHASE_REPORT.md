# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/snake3_camera2m_snake3_seed31_20261005_095902

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 22.497 | 22.594 | 2.286 | (0.5529, -3.7277) | 0.1898 | random_tent | 0.1898 | False |
| pillbox | 33.431 | 33.464 | 2.316 | (0.7433, 2.1076) | 0.2990 | random_pillbox | 0.2990 | False |
| panzer | 35.642 | 35.672 | 2.247 | (0.6663, 2.3046) | 4.3911 | random_pillbox | 0.1292 | True |
| bridge | 35.835 | 35.920 | 2.242 | (0.6308, 2.3724) | 7.4841 | random_pillbox | 0.1197 | True |
| panzer | 51.343 | 51.426 | 2.265 | (3.9522, -0.4019) | 0.1416 | random_panzer | 0.1416 | False |
| bridge | 55.131 | 55.196 | 2.254 | (5.0110, -3.5688) | 0.1066 | random_bridge | 0.1066 | False |
| red_cross | 67.361 | 67.442 | 2.293 | (7.3621, -1.1941) | 0.2372 | random_red_cross | 0.2372 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 35.942 | 67.790 | 2.239 | (0.6525, 2.4047) | 7.4971 | random_pillbox | 0.0931 | True |
| bridge | 66.571 | 67.790 | 2.299 | (4.9803, -3.7737) | 0.1808 | random_bridge | 0.1808 | False |
| panzer | 35.869 | 67.790 | 2.241 | (0.6314, 2.3716) | 4.4610 | random_pillbox | 0.1193 | True |
| panzer | 53.770 | 67.790 | 2.222 | (3.9644, -0.6122) | 0.1090 | random_panzer | 0.1090 | False |
| pillbox | 35.477 | 67.790 | 2.252 | (0.7195, 2.1875) | 0.2207 | random_pillbox | 0.2207 | False |
| red_cross | 67.538 | 67.790 | 2.282 | (7.3660, -1.1288) | 0.1753 | random_red_cross | 0.1753 | False |
| tent | 28.478 | 67.790 | 2.309 | (0.5870, -4.0063) | 0.1559 | random_tent | 0.1559 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 76.897 | 76.971 | 1.396 | (7.3757, -1.0652) | 0.1152 | random_red_cross | 0.1152 | False |
| bridge | 112.746 | 112.895 | 1.426 | (4.9974, -3.7535) | 0.1543 | random_bridge | 0.1543 | False |
| panzer | 137.148 | 137.368 | 1.445 | (3.9774, -0.6203) | 0.1094 | random_panzer | 0.1094 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 12 | 0.1157 | 7.4899 | 0.1054 | 2 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 11 | 0.1065 | 4.4261 | 0.1065 | 3 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 7 | 0.2192 | 0.2857 | 0.2192 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 5 | 0.1967 | 0.2349 | 0.1967 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 7 | 0.1153 | 0.1827 | 0.1153 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 76 | 0.1804 | 0.1825 | 0.1804 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 18 | 0.0816 | 0.1056 | 0.0816 | 0 |
| hint_vision | HIGH_SURVEY | disabled | pillbox | 11 | 0.2313 | 0.2404 | 0.2313 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 45 | 0.1141 | 0.1525 | 0.1141 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 2 | 0.1544 | 0.1545 | 0.1544 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1094 | 0.1094 | 0.1094 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 1 | 0.1152 | 0.1152 | 0.1152 | 0 |
| mapped | DELIVERY | disabled | bridge | 32 | 0.0936 | 0.1040 | 0.0936 | 0 |
| mapped | DELIVERY | disabled | panzer | 30 | 0.0738 | 0.0786 | 0.0738 | 0 |
| mapped | DELIVERY | disabled | red_cross | 16 | 0.0833 | 0.0866 | 0.0833 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 120 | 0.0946 | 0.1090 | 0.0946 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 91 | 0.0705 | 0.0825 | 0.0705 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 121 | 0.0860 | 0.1009 | 0.0860 | 0 |
| mapped | DESCEND | disabled | red_cross | 47 | 0.1318 | 0.1880 | 0.1318 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 184 | 0.1844 | 0.2089 | 0.1844 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 51 | 0.1363 | 4.3220 | 0.1363 | 5 |
| mapped | HIGH_SURVEY | disabled | pillbox | 39 | 0.2206 | 0.2562 | 0.2206 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 4 | 0.1785 | 0.1807 | 0.1785 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 109 | 0.1816 | 0.2454 | 0.1816 | 0 |
| mapped | LANDING | landing | landing_pad | 65 | 0.0367 | 0.0501 | 0.0367 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.0976 | 0.0979 | 0.0976 | 0 |
| mapped | REACQUIRE | disabled | panzer | 4 | 0.0796 | 0.0800 | 0.0796 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 1 | 0.0884 | 0.0884 | 0.0884 | 0 |
| mapped | REVISIT | disabled | bridge | 179 | 0.0890 | 0.1102 | 0.0890 | 0 |
| mapped | REVISIT | disabled | panzer | 34 | 0.0870 | 0.1092 | 0.0870 | 0 |
| mapped | REVISIT | disabled | red_cross | 43 | 0.0920 | 0.0983 | 0.0920 | 0 |
| mapped | TAIL | disabled | landing_pad | 2 | 0.0500 | 0.0502 | 0.0500 | 0 |
| mapped | TAIL | disabled | panzer | 36 | 0.0705 | 0.0786 | 0.0705 | 0 |
| selected | DELIVERY | disabled | bridge | 28 | 0.1511 | 0.1536 | 0.1511 | 0 |
| selected | DELIVERY | disabled | panzer | 26 | 0.1039 | 0.1075 | 0.1039 | 0 |
| selected | DELIVERY | disabled | red_cross | 15 | 0.1124 | 0.1146 | 0.1124 | 0 |
| selected | DELIVERY | drop_circle | bridge | 113 | 0.1423 | 0.1496 | 0.1423 | 0 |
| selected | DELIVERY | drop_circle | panzer | 85 | 0.0955 | 0.1015 | 0.0955 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 103 | 0.1039 | 0.1081 | 0.1039 | 0 |
| selected | DESCEND | disabled | red_cross | 40 | 0.1551 | 0.1817 | 0.1551 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 180 | 0.1805 | 0.1825 | 0.1805 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 44 | 0.0864 | 0.1201 | 0.0864 | 0 |
| selected | HIGH_SURVEY | disabled | pillbox | 27 | 0.2212 | 0.2442 | 0.2212 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 107 | 0.1162 | 0.1582 | 0.1162 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.1543 | 0.1545 | 0.1543 | 0 |
| selected | REACQUIRE | disabled | panzer | 4 | 0.1088 | 0.1093 | 0.1088 | 0 |
| selected | REACQUIRE | disabled | red_cross | 1 | 0.1152 | 0.1152 | 0.1152 | 0 |
| selected | REVISIT | disabled | bridge | 175 | 0.1272 | 0.1732 | 0.1272 | 0 |
| selected | REVISIT | disabled | panzer | 32 | 0.1160 | 0.1227 | 0.1160 | 0 |
| selected | REVISIT | disabled | red_cross | 36 | 0.1219 | 0.1319 | 0.1219 | 0 |
| selected | TAIL | disabled | panzer | 36 | 0.0816 | 0.0824 | 0.0816 | 0 |
| targets | DELIVERY | disabled | bridge | 30 | 0.1510 | 0.1536 | 0.1510 | 0 |
| targets | DELIVERY | disabled | panzer | 28 | 0.1037 | 0.1075 | 0.1037 | 0 |
| targets | DELIVERY | disabled | red_cross | 15 | 0.1124 | 0.1146 | 0.1124 | 0 |
| targets | DELIVERY | drop_circle | bridge | 118 | 0.1423 | 0.1498 | 0.1423 | 0 |
| targets | DELIVERY | drop_circle | panzer | 89 | 0.0955 | 0.1019 | 0.0955 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 111 | 0.1041 | 0.1085 | 0.1041 | 0 |
| targets | DESCEND | disabled | red_cross | 40 | 0.1551 | 0.1817 | 0.1551 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 184 | 0.1805 | 0.1825 | 0.1805 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 46 | 0.0852 | 0.1199 | 0.0852 | 0 |
| targets | HIGH_SURVEY | disabled | pillbox | 29 | 0.2218 | 0.2492 | 0.2218 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 2 | 0.1765 | 0.1769 | 0.1765 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 109 | 0.1152 | 0.1581 | 0.1152 | 0 |
| targets | LANDING | landing | landing_pad | 65 | 0.0439 | 0.0501 | 0.0439 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.1543 | 0.1545 | 0.1543 | 0 |
| targets | REACQUIRE | disabled | panzer | 4 | 0.1088 | 0.1093 | 0.1088 | 0 |
| targets | REACQUIRE | disabled | red_cross | 1 | 0.1152 | 0.1152 | 0.1152 | 0 |
| targets | REVISIT | disabled | bridge | 179 | 0.1275 | 0.1749 | 0.1275 | 0 |
| targets | REVISIT | disabled | panzer | 34 | 0.1165 | 0.1236 | 0.1165 | 0 |
| targets | REVISIT | disabled | red_cross | 39 | 0.1221 | 0.1338 | 0.1221 | 0 |
| targets | TAIL | disabled | panzer | 36 | 0.0816 | 0.0824 | 0.0816 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.03733048158106841,
    "frame_id": "camera_init",
    "stamp_ns": 214320000000,
    "x": 8.713137156452403,
    "y": -4.205890298859978,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.03733048158106841,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.03733048158106841,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
