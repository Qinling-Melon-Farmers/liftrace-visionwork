# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/route_speed_snake2_seed31_20261004_231619

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 22.614 | 22.640 | 2.887 | (0.5138, -3.8411) | 0.1618 | random_tent | 0.1618 | False |
| panzer | 28.473 | 28.498 | 2.879 | (3.7964, -0.9826) | 0.5143 | random_panzer | 0.5143 | False |
| pillbox | 32.017 | 32.111 | 2.882 | (0.6899, 2.0803) | 0.3310 | random_pillbox | 0.3310 | False |
| panzer | 32.229 | 32.275 | 2.882 | (0.6929, 2.1349) | 4.2630 | random_pillbox | 0.2767 | True |
| bridge | 38.012 | 38.088 | 2.900 | (0.7001, 2.4489) | 7.5047 | random_pillbox | 0.0620 | True |
| red_cross | 50.742 | 50.867 | 2.867 | (7.1883, -1.0678) | 0.2711 | random_red_cross | 0.2711 | False |
| bridge | 52.637 | 52.698 | 2.875 | (5.0017, -3.5532) | 0.1238 | random_bridge | 0.1238 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 64.124 | 64.227 | 1.503 | (5.0276, -3.7704) | 0.1521 | random_bridge | 0.1521 | False |
| panzer | 87.745 | 87.875 | 1.439 | (3.9679, -0.6300) | 0.1223 | random_panzer | 0.1223 | False |
| red_cross | 109.568 | 109.604 | 1.359 | (7.4295, -1.0367) | 0.0673 | random_red_cross | 0.0673 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 7 | 0.1027 | 5.2905 | 0.1008 | 1 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 74 | 0.3069 | 4.2580 | 0.2908 | 7 |
| hint_bbox | HIGH_SURVEY | disabled | pillbox | 10 | 0.2683 | 0.3161 | 0.2683 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 18 | 0.2588 | 0.2683 | 0.2588 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 6 | 0.1596 | 0.1678 | 0.1596 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 20 | 0.1549 | 0.1879 | 0.1549 | 0 |
| hint_vision | HIGH_SURVEY | disabled | panzer | 41 | 0.1342 | 4.2937 | 0.1342 | 17 |
| hint_vision | HIGH_SURVEY | disabled | tent | 40 | 0.1612 | 0.1997 | 0.1612 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1521 | 0.1521 | 0.1521 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 1 | 0.1223 | 0.1223 | 0.1223 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.0673 | 0.0673 | 0.0673 | 0 |
| mapped | DELIVERY | disabled | bridge | 23 | 0.1239 | 0.1303 | 0.1239 | 0 |
| mapped | DELIVERY | disabled | panzer | 25 | 0.0920 | 0.0980 | 0.0920 | 0 |
| mapped | DELIVERY | disabled | red_cross | 27 | 0.0816 | 0.0898 | 0.0816 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 130 | 0.1116 | 0.1448 | 0.1116 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 76 | 0.0735 | 0.0965 | 0.0735 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 114 | 0.0775 | 0.0950 | 0.0775 | 0 |
| mapped | DESCEND | disabled | bridge | 63 | 0.1314 | 0.2375 | 0.1314 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 54 | 0.2158 | 0.2508 | 0.2158 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 129 | 0.1720 | 4.4144 | 0.1450 | 63 |
| mapped | HIGH_SURVEY | disabled | pillbox | 11 | 0.2888 | 0.3172 | 0.2888 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 94 | 0.2068 | 0.3227 | 0.2068 | 0 |
| mapped | LANDING | landing | landing_pad | 111 | 0.0364 | 0.0527 | 0.0364 | 0 |
| mapped | REACQUIRE | disabled | bridge | 2 | 0.1238 | 0.1240 | 0.1238 | 0 |
| mapped | REACQUIRE | disabled | panzer | 2 | 0.0993 | 0.0994 | 0.0993 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 3 | 0.0708 | 0.0713 | 0.0708 | 0 |
| mapped | REVISIT | disabled | bridge | 181 | 0.1049 | 0.1469 | 0.1049 | 0 |
| mapped | REVISIT | disabled | panzer | 68 | 0.0866 | 0.1201 | 0.0866 | 0 |
| mapped | REVISIT | disabled | red_cross | 62 | 0.0676 | 0.0724 | 0.0676 | 0 |
| mapped | TAIL | disabled | landing_pad | 1 | 0.0563 | 0.0563 | 0.0563 | 0 |
| selected | DELIVERY | disabled | bridge | 20 | 0.1507 | 0.1517 | 0.1507 | 0 |
| selected | DELIVERY | disabled | panzer | 21 | 0.1193 | 0.1216 | 0.1193 | 0 |
| selected | DELIVERY | disabled | red_cross | 26 | 0.0681 | 0.0699 | 0.0681 | 0 |
| selected | DELIVERY | drop_circle | bridge | 120 | 0.1464 | 0.1495 | 0.1464 | 0 |
| selected | DELIVERY | drop_circle | panzer | 67 | 0.1079 | 0.1152 | 0.1079 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 98 | 0.0747 | 0.0761 | 0.0747 | 0 |
| selected | DESCEND | disabled | bridge | 63 | 0.1987 | 0.2067 | 0.1987 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 52 | 0.1668 | 0.1994 | 0.1668 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 94 | 0.1345 | 4.2964 | 0.1345 | 31 |
| selected | HIGH_SURVEY | disabled | tent | 92 | 0.1662 | 0.2103 | 0.1662 | 0 |
| selected | REACQUIRE | disabled | bridge | 2 | 0.1520 | 0.1521 | 0.1520 | 0 |
| selected | REACQUIRE | disabled | panzer | 2 | 0.1222 | 0.1223 | 0.1222 | 0 |
| selected | REACQUIRE | disabled | red_cross | 3 | 0.0673 | 0.0673 | 0.0673 | 0 |
| selected | REVISIT | disabled | bridge | 181 | 0.1536 | 0.1657 | 0.1536 | 0 |
| selected | REVISIT | disabled | panzer | 66 | 0.0975 | 0.1300 | 0.0975 | 0 |
| selected | REVISIT | disabled | red_cross | 60 | 0.0681 | 0.0692 | 0.0681 | 0 |
| targets | DELIVERY | disabled | bridge | 22 | 0.1506 | 0.1517 | 0.1506 | 0 |
| targets | DELIVERY | disabled | panzer | 23 | 0.1191 | 0.1216 | 0.1191 | 0 |
| targets | DELIVERY | disabled | red_cross | 26 | 0.0681 | 0.0699 | 0.0681 | 0 |
| targets | DELIVERY | drop_circle | bridge | 128 | 0.1462 | 0.1496 | 0.1462 | 0 |
| targets | DELIVERY | drop_circle | panzer | 72 | 0.1078 | 0.1157 | 0.1078 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 106 | 0.0744 | 0.0761 | 0.0744 | 0 |
| targets | DESCEND | disabled | bridge | 63 | 0.1987 | 0.2067 | 0.1987 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 54 | 0.1651 | 0.1993 | 0.1651 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 98 | 0.1345 | 4.2958 | 0.1345 | 33 |
| targets | HIGH_SURVEY | disabled | pillbox | 2 | 0.3011 | 0.3059 | 0.3011 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 94 | 0.1653 | 0.2102 | 0.1653 | 0 |
| targets | LANDING | landing | landing_pad | 111 | 0.0362 | 0.0542 | 0.0362 | 0 |
| targets | REACQUIRE | disabled | bridge | 2 | 0.1520 | 0.1521 | 0.1520 | 0 |
| targets | REACQUIRE | disabled | panzer | 2 | 0.1222 | 0.1223 | 0.1222 | 0 |
| targets | REACQUIRE | disabled | red_cross | 3 | 0.0673 | 0.0673 | 0.0673 | 0 |
| targets | REVISIT | disabled | bridge | 181 | 0.1536 | 0.1657 | 0.1536 | 0 |
| targets | REVISIT | disabled | panzer | 68 | 0.0977 | 0.1306 | 0.0977 | 0 |
| targets | REVISIT | disabled | red_cross | 62 | 0.0680 | 0.0692 | 0.0680 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.03656589209984244,
    "frame_id": "camera_init",
    "stamp_ns": 190211000000,
    "x": 8.716813497929298,
    "y": -4.215353193328054,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.03656589209984244,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.03656589209984244,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
