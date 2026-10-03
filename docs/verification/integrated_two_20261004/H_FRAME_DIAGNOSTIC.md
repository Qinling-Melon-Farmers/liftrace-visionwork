# Saved H frame: recorded false versus restored true

The saved standard-H frame is detected by the production geometry code when the existing research stroke fallback is restored. Raising the capture height is not required to detect this frame.

This is an offline single-image diagnosis, not a flight PASS. Run `integrated_two_seed31_20261004_003709` is diagnostic and excluded from the corrected two-run results. The earlier `002834` preparation run is also excluded (H asset shadowing).

Input: [saved frame](H_diagnostic_frame.png), recorded ROS time 281.465, 1280 x 720. Effective parameters come from `logs/integrated_two_seed31_20261004_003709/rosparams.yaml`. The standalone probe extracts unmodified `detectLandingPad` and `validateHStructure` from the production C++ source and includes the production `h_stroke_detector.h`. It uses system OpenCV 4.2.0 without a ROS node or simulator.

Source compatibility checked: `git diff da637e59 4cb75a3b` is empty for `landing_detector_node.cpp`, `landing_detector_node.h` and `h_stroke_detector.h`. The saved-image result therefore also applies to the corrected source revision's geometry implementation.

| Offline case | Found | Center / px | Returned radius / px |
|---|---:|---|---:|
| Recorded parameters: stroke fallback false, radius max 300 | 0 | unavailable | unavailable |
| Change only stroke fallback to true | 1 | 635, 377.5 | 189.5 |
| Diagnostic only: change radius max to 600, keep fallback false | 1 | 637.0561, 376.0745 | 356.5453 |

Under the recorded configuration, the two nearly circular contours have radii 320.0009 and 356.5453 px and fail the radius test before H-structure validation. Their aspect ratios are 0.9890 and 0.9966. Other eligible contours fail aspect ratio. This identifies the concrete rejection on this image. The fallback result's radius describes its stroke-based result; it is not the outer-ring radius.

The evidence supports the added false override as the cause of losing detection on this frame. It does not establish a new H-code regression, model misdetection, or success of an entire corrected flight. No H shape fit should be presented as recorded telemetry: these pixel results are explicitly offline computations.

The corrected runs retain H size 0.8 m, local capture Z 0.68 m, Gate 1.2 m, and restore `landing_enable_h_stroke_fallback=true`. With ground Z -0.22 m, requested FC capture AGL is 0.90 m. Requested height is distinct from instantaneous FC and camera height. No 1.8 m capture change was made by this reporting task.

Raw output: [probe output](H_diagnostic_probe.txt); parameter/method metadata: `/home/xhj/sim_h_seed31_probe_20261004/provenance.json`. The same directory preserves the generated C++ and executable.

Reproduction from WSL (use a new output directory; refuses overwrite):

```bash
C=/home/xhj/liftrace-worktrees/r2026-board-frame-fix
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 19 python -B \
  "$C/docs/verification/integrated_two_20261004/h_geometry_probe.py" \
  --root "$C" --image /home/xhj/h_live_seed31_20261004.png \
  --params "$C/logs/integrated_two_seed31_20261004_003709/rosparams.yaml" \
  --out /home/xhj/sim_h_seed31_probe_recheck_20261004
```

On Windows, execute the ASCII shell command through `wsl -e bash -c '...'`. This compiles only a standalone offline probe; it does not build or modify runtime packages.
