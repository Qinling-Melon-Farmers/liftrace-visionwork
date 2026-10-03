# Corrected formal center and downward-video results

Corrected formal runs only; excludes 002834 preparation and 003709 diagnostic.
Different code/model/inflation/ceiling/H geometry from history; not strict A/B.

All video entries below passed actual ffprobe and complete FFmpeg decode with one thread. Source video and bag files were not modified. The overlay uses recorded image-stamped detections, never offline inference.

Initial high-view values come from first published SURVEY support and frozen SURVEY interrupt events. Final mutable top_hints is excluded. Nearest-any-instance identity is a diagnostic association, not independently verified classification ground truth.

## Seed 31

Run: `/home/xhj/liftrace-worktrees/r2026-board-frame-fix/logs/integrated_two_seed31_20261004_011753`; Gate **PASS**.

[Center report](31_centers/CENTER_REPORT.md) | [Source-phase and frozen snapshots](31_centers/stages/PHASE_REPORT.md) | [Overlay MP4](31_mapped_overlay.mp4) | [Validation](31_mapped_overlay.validation.json)

| Low reacquisition event | XY error / m | Source ROS / s |
|---|---:|---:|
| red_cross | 0.11841 | 47.902 |
| bridge | 0.13830 | 84.956 |
| panzer | 0.06893 | 97.553 |

H landing observations, with align mode evaluated at source image time:
- landing_h_clone: mapped N=133, median 0.03829 m, P95 0.04392 m.
- Parent center_summary also retains receipt-time mode statistics; boundary samples can differ (seed38: 97 receipt-mode versus 95 source-mode H samples).
- Final Gate H mark: 0.03185 m; distinct from the distribution of mapped observations.

Recorded mapped class/instance ambiguities:
- bridge near random_pillbox: N=1, median nearest distance 0.27515 m versus same-class 4.77917 m. Do not describe the latter as localization-only error.
- panzer near random_pillbox: N=28, median nearest distance 0.13192 m versus same-class 4.64938 m. Do not describe the latter as localization-only error.
- Phase HIGH_SURVEY, align mode disabled: bridge has 1 suspected confusions among 67 fresh unique observations; source ROS interval 18.094 to 32.417 s. Full phase interval includes non-confused observations where present.
- Phase HIGH_SURVEY, align mode disabled: panzer has 28 suspected confusions among 28 fresh unique observations; source ROS interval 21.349 to 26.196 s. Full phase interval includes non-confused observations where present.

Separate distance outliers (more than 0.5 m from every truth center; cause not inferred as class confusion):
- None among fresh unique valid samples.

Video: 1964 frames at 10 fps; 196.4 s. Visited source frames: 1622; nonempty matched 530, empty matched 1092, unmatched 0. Maximum absolute image-stamp match difference 0.000 ms. Held frames are labelled; complete per-frame mapping is retained.

## Seed 38

Run: `/home/xhj/liftrace-worktrees/r2026-board-frame-fix/logs/integrated_two_seed38_20261004_013059`; Gate **PASS**.

[Center report](38_centers/CENTER_REPORT.md) | [Source-phase and frozen snapshots](38_centers/stages/PHASE_REPORT.md) | [Overlay MP4](38_mapped_overlay.mp4) | [Validation](38_mapped_overlay.validation.json)

| Low reacquisition event | XY error / m | Source ROS / s |
|---|---:|---:|
| red_cross | 0.19311 | 71.602 |
| bridge | 0.06409 | 90.059 |
| panzer | 0.18181 | 102.137 |

H landing observations, with align mode evaluated at source image time:
- landing_h_clone: mapped N=95, median 0.03825 m, P95 0.05525 m.
- Parent center_summary also retains receipt-time mode statistics; boundary samples can differ (seed38: 97 receipt-mode versus 95 source-mode H samples).
- Final Gate H mark: 0.03593 m; distinct from the distribution of mapped observations.

Recorded mapped class/instance ambiguities:
- None under the stated nearest-instance suspicion thresholds.

Separate distance outliers (more than 0.5 m from every truth center; cause not inferred as class confusion):
- hint_bbox / HIGH_SURVEY / tent: N=7, median nearest-any 2.8600 m, max 2.8723 m.
- mapped / DELIVERY / bridge: N=18, median nearest-any 0.6958 m, max 0.7493 m.
- targets / DELIVERY / bridge: N=12, median nearest-any 0.6589 m, max 0.6978 m.

Video: 2054 frames at 10 fps; 205.4 s. Visited source frames: 1678; nonempty matched 501, empty matched 1177, unmatched 0. Maximum absolute image-stamp match difference 0.000 ms. Held frames are labelled; complete per-frame mapping is retained.

Machine-readable data and relative links for the main index: [center_results.json](center_results.json).
Final coverage, LAND publications, release association and long-tail diagnosis: [FINAL_CENTER_DIAGNOSTICS.md](FINAL_CENTER_DIAGNOSTICS.md).
Raw CSV includes invalid/stale/duplicate rows; use the report filters. No sample means missing evidence, not zero error.
