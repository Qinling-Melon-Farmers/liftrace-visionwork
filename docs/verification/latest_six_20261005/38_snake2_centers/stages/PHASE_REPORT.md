# Centers by observation phase and frozen high-view snapshots

Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/latest_six_snake2_seed38_20261005_034318

These snapshots supersede any interpretation of final mutable top_hints as initial high-view measurements. Full-mission aggregates in the parent report are not high/low stage estimates.

Source observation time; latest recorded align mode; event transition time where available, else first status publication. FC AGL interpolated from truth_pose, no extrapolation.
Status-only transitions can lag actual transitions; timing_source and first_publication_ros_s are retained.

Distances are diagnostic nearest truth associations, not independently verified object identity or parcel impact accuracy. A wrong class near another instance is shown explicitly.

## First high SURVEY observations (per predicted class / nearest instance)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 17.820 | 17.897 | 2.660 | (0.9837, -0.6144) | 0.1618 | random_red_cross | 0.1618 | False |
| bridge | 45.516 | 45.541 | 2.795 | (3.8787, 1.9070) | 0.1602 | random_bridge | 0.1602 | False |
| tent | 48.634 | 48.675 | 2.849 | (4.7613, -0.7344) | 0.1444 | random_tent | 0.1444 | False |
| bridge | 51.681 | 51.737 | 2.864 | (4.5763, -3.2629) | 5.1539 | random_pillbox | 0.0714 | True |
| panzer | 51.716 | 51.737 | 2.864 | (4.5847, -3.2836) | 2.6084 | random_pillbox | 0.0554 | True |

## Frozen SURVEY interrupt support (all hypotheses retained)

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| bridge | 48.579 | 52.054 | 2.851 | (3.9183, 1.7572) | 0.1551 | random_bridge | 0.1551 | False |
| bridge | 51.824 | 52.054 | 2.862 | (4.5768, -3.3401) | 5.2307 | random_pillbox | 0.0736 | True |
| panzer | 51.926 | 52.054 | 2.860 | (4.5817, -3.3891) | 2.5789 | random_pillbox | 0.1061 | True |
| red_cross | 31.480 | 52.054 | 2.814 | (0.9534, -0.6182) | 0.1787 | random_red_cross | 0.1787 | False |
| tent | 51.781 | 52.054 | 2.863 | (4.8378, -0.8583) | 0.1624 | random_tent | 0.1624 | False |

## Low reacquisition events

| Predicted | Source ROS/s | Publication ROS/s | FC AGL/m | XY/m | Same-class distance/m | Nearest instance | Nearest distance/m | Confusion |
|---|---:|---:|---:|---|---:|---|---:|---|
| red_cross | 75.763 | 75.850 | 1.367 | (0.9579, -0.6100) | 0.1694 | random_red_cross | 0.1694 | False |
| bridge | 97.211 | 97.362 | 1.361 | (3.9458, 1.7587) | 0.1350 | random_bridge | 0.1350 | False |

## Fresh unique valid centers by source phase

All associations below include class confusions. CSV also provides a nearest-class-agrees subset. No stale memory republish is counted as a new phase observation.

| Stream | Phase | Align mode | Class | N | Median same-class/m | P95 same-class/m | Median nearest/m | Confusion N |
|---|---|---|---|---:|---:|---:|---:|---:|
| hint_bbox | HIGH_SURVEY | disabled | bridge | 11 | 0.1602 | 5.1923 | 0.1502 | 2 |
| hint_bbox | HIGH_SURVEY | disabled | panzer | 4 | 2.5914 | 2.6063 | 0.0754 | 4 |
| hint_bbox | HIGH_SURVEY | disabled | red_cross | 4 | 0.1653 | 0.1671 | 0.1653 | 0 |
| hint_bbox | HIGH_SURVEY | disabled | tent | 7 | 0.1498 | 0.1756 | 0.1498 | 0 |
| hint_vision | HIGH_SURVEY | disabled | bridge | 22 | 0.1574 | 0.1619 | 0.1574 | 0 |
| hint_vision | HIGH_SURVEY | disabled | red_cross | 41 | 0.1466 | 0.1749 | 0.1466 | 0 |
| hint_vision | HIGH_SURVEY | disabled | tent | 25 | 0.1633 | 0.1761 | 0.1633 | 0 |
| hint_vision | REACQUIRE | disabled | bridge | 1 | 0.1350 | 0.1350 | 0.1350 | 0 |
| hint_vision | REACQUIRE | disabled | pillbox | 1 | 0.1323 | 0.1323 | 0.1323 | 0 |
| hint_vision | REACQUIRE | disabled | red_cross | 2 | 0.1696 | 0.1698 | 0.1696 | 0 |
| mapped | DELIVERY | disabled | bridge | 31 | 0.0978 | 0.1010 | 0.0978 | 0 |
| mapped | DELIVERY | disabled | red_cross | 20 | 0.0888 | 0.0902 | 0.0888 | 0 |
| mapped | DELIVERY | drop_circle | bridge | 54 | 0.1054 | 0.1160 | 0.1054 | 0 |
| mapped | DELIVERY | drop_cross | red_cross | 109 | 0.0822 | 0.1009 | 0.0822 | 0 |
| mapped | DESCEND | disabled | panzer | 35 | 2.5783 | 2.5991 | 0.2220 | 28 |
| mapped | DESCEND | disabled | pillbox | 7 | 0.1897 | 0.2160 | 0.1897 | 0 |
| mapped | HIGH_SURVEY | disabled | bridge | 58 | 0.1572 | 0.1784 | 0.1567 | 1 |
| mapped | HIGH_SURVEY | disabled | panzer | 5 | 2.5993 | 2.6037 | 0.1273 | 5 |
| mapped | HIGH_SURVEY | disabled | red_cross | 147 | 0.1864 | 0.3100 | 0.1864 | 0 |
| mapped | HIGH_SURVEY | disabled | tent | 63 | 0.1700 | 0.2092 | 0.1700 | 0 |
| mapped | LOW_COVERAGE | disabled | bridge | 101 | 0.0943 | 0.1063 | 0.0943 | 0 |
| mapped | REACQUIRE | disabled | bridge | 1 | 0.1013 | 0.1013 | 0.1013 | 0 |
| mapped | REACQUIRE | disabled | pillbox | 2 | 0.1173 | 0.1179 | 0.1173 | 0 |
| mapped | REACQUIRE | disabled | red_cross | 2 | 0.0860 | 0.0863 | 0.0860 | 0 |
| mapped | REVISIT | disabled | bridge | 35 | 0.1044 | 0.1113 | 0.1044 | 0 |
| mapped | REVISIT | disabled | pillbox | 213 | 0.1193 | 0.1353 | 0.1193 | 0 |
| mapped | REVISIT | disabled | red_cross | 132 | 0.0957 | 0.1074 | 0.0957 | 0 |
| mapped | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 42 | 0.1143 | 0.1187 | 0.1143 | 0 |
| selected | DELIVERY | disabled | bridge | 28 | 0.1298 | 0.1341 | 0.1298 | 0 |
| selected | DELIVERY | disabled | red_cross | 20 | 0.1649 | 0.1685 | 0.1649 | 0 |
| selected | DELIVERY | drop_circle | bridge | 45 | 0.1231 | 0.1255 | 0.1231 | 0 |
| selected | DELIVERY | drop_cross | red_cross | 98 | 0.1472 | 0.1584 | 0.1472 | 0 |
| selected | DESCEND | disabled | panzer | 20 | 2.5846 | 2.5981 | 0.1898 | 15 |
| selected | DESCEND | disabled | pillbox | 4 | 0.1978 | 0.1986 | 0.1978 | 0 |
| selected | HIGH_SURVEY | disabled | bridge | 55 | 0.1565 | 0.1626 | 0.1565 | 0 |
| selected | HIGH_SURVEY | disabled | panzer | 1 | 2.5986 | 2.5986 | 0.1272 | 1 |
| selected | HIGH_SURVEY | disabled | red_cross | 109 | 0.1454 | 0.1811 | 0.1454 | 0 |
| selected | HIGH_SURVEY | disabled | tent | 59 | 0.1630 | 0.1773 | 0.1630 | 0 |
| selected | LOW_COVERAGE | disabled | bridge | 97 | 0.1092 | 0.1156 | 0.1092 | 0 |
| selected | REACQUIRE | disabled | bridge | 1 | 0.1350 | 0.1350 | 0.1350 | 0 |
| selected | REACQUIRE | disabled | pillbox | 2 | 0.1322 | 0.1323 | 0.1322 | 0 |
| selected | REACQUIRE | disabled | red_cross | 2 | 0.1696 | 0.1698 | 0.1696 | 0 |
| selected | REVISIT | disabled | bridge | 33 | 0.1424 | 0.1532 | 0.1424 | 0 |
| selected | REVISIT | disabled | pillbox | 211 | 0.1305 | 0.1524 | 0.1305 | 0 |
| selected | REVISIT | disabled | red_cross | 118 | 0.1266 | 0.1789 | 0.1266 | 0 |
| selected | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 39 | 0.1152 | 0.1165 | 0.1152 | 0 |
| targets | DELIVERY | disabled | bridge | 30 | 0.1296 | 0.1340 | 0.1296 | 0 |
| targets | DELIVERY | disabled | red_cross | 20 | 0.1649 | 0.1685 | 0.1649 | 0 |
| targets | DELIVERY | drop_circle | bridge | 52 | 0.1230 | 0.1258 | 0.1230 | 0 |
| targets | DELIVERY | drop_cross | red_cross | 102 | 0.1472 | 0.1590 | 0.1472 | 0 |
| targets | DESCEND | disabled | panzer | 28 | 2.5846 | 2.5979 | 0.1880 | 21 |
| targets | DESCEND | disabled | pillbox | 4 | 0.1978 | 0.1986 | 0.1978 | 0 |
| targets | HIGH_SURVEY | disabled | bridge | 58 | 0.1566 | 0.1626 | 0.1564 | 1 |
| targets | HIGH_SURVEY | disabled | panzer | 2 | 2.5989 | 2.5991 | 0.1250 | 2 |
| targets | HIGH_SURVEY | disabled | red_cross | 111 | 0.1457 | 0.1810 | 0.1457 | 0 |
| targets | HIGH_SURVEY | disabled | tent | 61 | 0.1634 | 0.1773 | 0.1634 | 0 |
| targets | LOW_COVERAGE | disabled | bridge | 101 | 0.1092 | 0.1156 | 0.1092 | 0 |
| targets | REACQUIRE | disabled | bridge | 1 | 0.1350 | 0.1350 | 0.1350 | 0 |
| targets | REACQUIRE | disabled | pillbox | 2 | 0.1322 | 0.1323 | 0.1322 | 0 |
| targets | REACQUIRE | disabled | red_cross | 2 | 0.1696 | 0.1698 | 0.1696 | 0 |
| targets | REVISIT | disabled | bridge | 35 | 0.1430 | 0.1553 | 0.1430 | 0 |
| targets | REVISIT | disabled | pillbox | 213 | 0.1306 | 0.1557 | 0.1306 | 0 |
| targets | REVISIT | disabled | red_cross | 124 | 0.1267 | 0.1800 | 0.1267 | 0 |
| targets | SURVEY_BELOW_HIGH_OR_NO_POSE | disabled | red_cross | 41 | 0.1151 | 0.1165 | 0.1151 | 0 |

## Final Gate H mark

No valid H mark recorded.

See phase_summary.json for original frozen hint/event fields, phase_statistics.csv for statistics and center_samples_by_phase.csv for every row including exclusions.
