# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-board-frame-fix/logs/integrated_two_seed38_20261004_013059

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| tent | 17.136 | 17.197 | 2.457 | (0.8445, -1.0583) | 0.0394 | random_tent | 0.0394 | False |
| tent | 21.994 | 22.014 | 2.665 | (2.5224, 1.8267) | 3.3025 | random_pillbox | 2.8692 | False |
| red_cross | 37.521 | 37.567 | 2.727 | (4.1748, -4.5684) | 0.3426 | random_red_cross | 0.3426 | False |
| bridge | 45.249 | 45.313 | 2.586 | (3.5803, -1.5340) | 0.4922 | random_bridge | 0.4922 | False |
| panzer | 46.102 | 46.179 | 2.569 | (4.9427, -0.8129) | 0.4778 | random_panzer | 0.4778 | False |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 46.242 | 46.392 | 2.572 | (3.6436, -1.3359) | 0.2861 | random_bridge | 0.2861 | False |
| panzer | 46.242 | 46.392 | 2.572 | (4.9406, -0.7413) | 0.4093 | random_panzer | 0.4093 | False |
| red_cross | 44.205 | 46.392 | 2.650 | (4.3254, -4.5773) | 0.2267 | random_red_cross | 0.2267 | False |
| tent | 31.884 | 46.392 | 2.757 | (0.8076, -1.1605) | 0.1419 | random_tent | 0.1419 | False |
| tent | 25.747 | 46.392 | 2.735 | (2.6073, 2.1020) | 3.5840 | random_pillbox | 2.8168 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 71.602 | 71.810 | 1.364 | (4.3599, -4.5625) | 0.1931 | random_red_cross | 0.1931 | False |
| bridge | 90.059 | 90.244 | 1.343 | (3.7409, -1.1357) | 0.0641 | random_bridge | 0.0641 | False |
| panzer | 102.137 | 102.416 | 1.292 | (4.9839, -0.5152) | 0.1818 | random_panzer | 0.1818 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 16 | 0.3085 | 0.4787 | 0.3085 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 3 | 0.4528 | 0.4753 | 0.4528 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 9 | 0.2786 | 0.3352 | 0.2786 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 11 | 3.3662 | 3.6062 | 2.8219 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 28 | 0.2492 | 0.2763 | 0.2492 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 37 | 0.1373 | 0.1417 | 0.1373 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.0641 | 0.0641 | 0.0641 | 0 |
| hint_vision | REACQUIRE | disabled | panzer | 2 | 0.1826 | 0.1833 | 0.1826 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 1 | 0.1931 | 0.1931 | 0.1931 | 0 |
| mapped | DELIVERY | disabled | bridge | 41 | 0.0614 | 0.0704 | 0.0614 | 0 |
| mapped | DELIVERY | disabled | panzer | 45 | 0.0820 | 0.0911 | 0.0820 | 0 |
| mapped | DELIVERY | disabled | red_cross | 25 | 0.0882 | 0.0926 | 0.0882 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 95 | 0.0794 | 0.7592 | 0.0794 | 0 |
| mapped | DELIVERY | drop_circle | panzer | 99 | 0.0837 | 0.0921 | 0.0837 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 187 | 0.0650 | 0.0984 | 0.0650 | 0 |
| mapped | DESCEND | disabled | panzer | 42 | 0.2597 | 0.3424 | 0.2597 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 1 | 0.3401 | 0.3401 | 0.3401 | 0 |
| mapped | HIGH_SURVEY | disabled | panzer | 3 | 0.3359 | 0.3566 | 0.3359 | 0 |
| mapped | HIGH_SURVEY | disabled | red_cross | 73 | 0.2115 | 0.2814 | 0.2115 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 102 | 0.1472 | 0.2017 | 0.1472 | 0 |
| mapped | LANDING | landing | landing_pad | 95 | 0.0383 | 0.0553 | 0.0383 | 0 |
| mapped | REACQUIRE | disabled | bridge | 3 | 0.0513 | 0.0514 | 0.0513 | 0 |
| mapped | REACQUIRE | disabled | panzer | 6 | 0.0793 | 0.0797 | 0.0793 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 5 | 0.0933 | 0.0939 | 0.0933 | 0 |
| mapped | REVISIT | disabled | bridge | 59 | 0.0733 | 0.1506 | 0.0733 | 0 |
| mapped | REVISIT | disabled | panzer | 25 | 0.0915 | 0.1028 | 0.0915 | 0 |
| mapped | REVISIT | disabled | red_cross | 58 | 0.1069 | 0.1292 | 0.1069 | 0 |
| mapped | TAIL | disabled | landing_pad | 2 | 0.0547 | 0.0552 | 0.0547 | 0 |
| mapped | TAIL | disabled | panzer | 35 | 0.0939 | 0.1062 | 0.0939 | 0 |
| mapped | TAIL | disabled | pillbox | 22 | 0.0969 | 0.1136 | 0.0969 | 0 |
| selected | DELIVERY | disabled | bridge | 35 | 0.0599 | 0.0651 | 0.0599 | 0 |
| selected | DELIVERY | disabled | panzer | 40 | 0.1550 | 0.1722 | 0.1550 | 0 |
| selected | DELIVERY | disabled | red_cross | 20 | 0.1810 | 0.1884 | 0.1810 | 0 |
| selected | DELIVERY | drop_circle | bridge | 69 | 0.0669 | 0.0685 | 0.0669 | 0 |
| selected | DELIVERY | drop_circle | panzer | 89 | 0.1336 | 0.1436 | 0.1336 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 143 | 0.1444 | 0.1716 | 0.1444 | 0 |
| selected | DESCEND | disabled | panzer | 42 | 0.3068 | 0.3355 | 0.3068 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 1 | 0.3385 | 0.3385 | 0.3385 | 0 |
| selected | HIGH_SURVEY | disabled | red_cross | 69 | 0.2436 | 0.2771 | 0.2436 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 93 | 0.1374 | 0.1420 | 0.1374 | 0 |
| selected | REACQUIRE | disabled | bridge | 3 | 0.0636 | 0.0640 | 0.0636 | 0 |
| selected | REACQUIRE | disabled | panzer | 6 | 0.1796 | 0.1830 | 0.1796 | 0 |
| selected | REACQUIRE | disabled | red_cross | 5 | 0.1921 | 0.1939 | 0.1921 | 0 |
| selected | REVISIT | disabled | bridge | 57 | 0.0679 | 0.1027 | 0.0679 | 0 |
| selected | REVISIT | disabled | panzer | 23 | 0.2055 | 0.2296 | 0.2055 | 0 |
| selected | REVISIT | disabled | red_cross | 56 | 0.1112 | 0.2211 | 0.1112 | 0 |
| selected | TAIL | disabled | panzer | 35 | 0.1149 | 0.1173 | 0.1149 | 0 |
| selected | TAIL | disabled | pillbox | 20 | 0.0856 | 0.0961 | 0.0856 | 0 |
| targets | DELIVERY | disabled | bridge | 37 | 0.0601 | 0.0652 | 0.0601 | 0 |
| targets | DELIVERY | disabled | panzer | 42 | 0.1543 | 0.1720 | 0.1543 | 0 |
| targets | DELIVERY | disabled | red_cross | 22 | 0.1802 | 0.1883 | 0.1802 | 0 |
| targets | DELIVERY | drop_circle | bridge | 86 | 0.0673 | 0.6666 | 0.0673 | 0 |
| targets | DELIVERY | drop_circle | panzer | 96 | 0.1332 | 0.1442 | 0.1332 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 158 | 0.1421 | 0.1725 | 0.1421 | 0 |
| targets | DESCEND | disabled | panzer | 42 | 0.3068 | 0.3355 | 0.3068 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 1 | 0.3401 | 0.3401 | 0.3401 | 0 |
| targets | HIGH_SURVEY | disabled | panzer | 3 | 0.3473 | 0.3577 | 0.3473 | 0 |
| targets | HIGH_SURVEY | disabled | red_cross | 73 | 0.2458 | 0.2770 | 0.2458 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 97 | 0.1374 | 0.1420 | 0.1374 | 0 |
| targets | LANDING | landing | landing_pad | 95 | 0.0455 | 0.0552 | 0.0455 | 0 |
| targets | REACQUIRE | disabled | bridge | 3 | 0.0636 | 0.0640 | 0.0636 | 0 |
| targets | REACQUIRE | disabled | panzer | 6 | 0.1796 | 0.1830 | 0.1796 | 0 |
| targets | REACQUIRE | disabled | red_cross | 5 | 0.1921 | 0.1939 | 0.1921 | 0 |
| targets | REVISIT | disabled | bridge | 59 | 0.0680 | 0.1194 | 0.0680 | 0 |
| targets | REVISIT | disabled | panzer | 25 | 0.2077 | 0.2350 | 0.2077 | 0 |
| targets | REVISIT | disabled | red_cross | 58 | 0.1112 | 0.2239 | 0.1112 | 0 |
| targets | TAIL | disabled | panzer | 35 | 0.1149 | 0.1173 | 0.1149 | 0 |
| targets | TAIL | disabled | pillbox | 22 | 0.0855 | 0.0960 | 0.0855 | 0 |

## Final Gate H mark

{
  "recorded_mark": {
    "anchor_error_m": 0.03592776821633617,
    "frame_id": "camera_init",
    "stamp_ns": 200269000000,
    "x": 8.477016067660633,
    "y": -4.227614188078345,
    "z": -0.22
  },
  "nearest_any_instance": "landing_h_clone",
  "nearest_any_class": "landing_pad",
  "nearest_any_distance_m": 0.03592776821633617,
  "same_class_instance": "landing_h_clone",
  "same_class_distance_m": 0.03592776821633617,
  "category_confusion_suspected": false
}

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
