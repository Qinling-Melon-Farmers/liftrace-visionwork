# Recorded centers and downward-video processing

Corrected source: `4cb75a3b1308ef0e17fcf6c7a296c946e0d9e751`. Seed31 started at `logs/integrated_two_seed31_20261004_011753`; obtain the final seed38 path and completion status from `logs/integrated_two_20261004_batch/matrix.json`. Process only closed bags after the runner closes the run. Do not mistake an existing finalized bag for completion of every other recorder.

Excluded from corrected results: seed31 `002834` (preparation, H asset shadowing) and `003709` (diagnostic, added false stroke-fallback override). See [H frame diagnosis](H_FRAME_DIAGNOSTIC.md). Corrected settings keep capture local Z 0.68 m, Gate 1.2 m, H asset 0.8 x 0.8 m and restore stroke fallback true. Historical flights have different revisions, model, inflation and H geometry; comparisons are not strict A/B.

The main agent owns `process_results.py`, the overall report and overview/follow composition. Its history import was checked and still resolves to `history_31_40_20260920/analyze_current.py`. The scripts below complement that pipeline and never launch ROS, simulation, inference or SSH.

## Commands after a run closes

Run this ASCII shell body through Windows `wsl -e bash -c '...'`, or save it as a UTF-8 shell file and invoke that file through the same wrapper. Use new output names if repeating analysis; these tools refuse overwrites. Large exported telemetry stays outside Git. Original media is read only.

```bash
C=/home/xhj/liftrace-worktrees/r2026-board-frame-fix
D="$C/docs/verification/integrated_two_20261004"
SEED=31
RUN="$C/logs/integrated_two_seed31_20261004_011753"
OUT=/home/xhj/sim_reports_20261004
mkdir -p "$OUT"
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1

# Offline ROS bag library only: no roscore/node is created.
(
  source /opt/ros/noetic/setup.bash
  nice -n 19 /usr/bin/python3 -B "$D/center_compare.py" export \
    --bag "$RUN/vision_metrics.bag" \
    --out "$OUT/seed${SEED}_center_inputs.jsonl"
)

nice -n 19 python -B "$D/center_compare.py" analyze \
  --data "$OUT/seed${SEED}_center_inputs.jsonl" --run "$RUN" \
  --world "$D/seed_${SEED}/field.world" --out "$D/${SEED}_centers" \
  --pipeline-root "$C" --eval-frame camera_init \
  --world-to-eval-xyz 0 0 -.22 --world-to-eval-yaw 0 \
  --transform-note 'World XY aligned to camera_init; world Z minus 0.22; recorded TF used for other frames.' \
  --h-size .8 --max-age .5 --hint-frame camera_init --hint-plane-z -.22 \
  --scope-label CORRECTED_CURRENT_RUN

nice -n 19 python -B "$D/center_stage_report.py" \
  --run "$RUN" --centers "$D/${SEED}_centers" \
  --data "$OUT/seed${SEED}_center_inputs.jsonl" --truth-local-ground-z -.22

nice -n 19 python -B "$D/mapped_video_overlay.py" \
  --video "$RUN/downward.mp4" --timestamps "$RUN/downward.csv" \
  --data "$OUT/seed${SEED}_center_inputs.jsonl" \
  --centers "$D/${SEED}_centers/center_samples.csv" \
  --output "$D/${SEED}_mapped_overlay.mp4" \
  --fps 10 --match-ms 30 --threads 1 --label "Corrected seed${SEED}"

nice -n 19 python -B "$D/verify_overlay.py" "$D/${SEED}_mapped_overlay.mp4"
```

For seed38 change `SEED` and `RUN` to the closed matrix result. The coordinate transform above is specific to these scene conventions and must be checked against the saved parameters if a future scene changes it.

## Interpretation and limits

- `CENTER_REPORT.md`, `center_summary.json`, `center_samples.csv` and `class_instance_associations.csv` compare mapped detections, memory targets, selected targets and navigation support. A nearest same-class distance is reported alongside nearest any-class/instance distance; suspected class confusion must not be described as a 5 m localization error. Repeated publications, invalid map points, stale data and missing evidence are counted separately.
- `stages/PHASE_REPORT.md` and `phase_summary.json` retain first high SURVEY observations, every hypothesis in the frozen SURVEY interrupt event, and low reacquisition events. Source observation time and align mode determine phase; truth pose verifies high AGL. Final mutable `top_hints` is never used as an initial-high measurement. The parent report's whole-mission aggregates are not high-only or low-only estimates.
- `verify_overlay.py` runs actual ffprobe and a complete single-thread FFmpeg decode, and writes `.validation.json` including exit codes, stderr, frame-count and timestamp-sidecar checks.
- H includes are recognized when the model URI basename starts with `landing_h`, including `landing_h_80cm`. The world instance names `landing_h` and `landing_h_clone` remain separate. H size is metadata; map-center errors use the world centers and recorded coordinates.
- The overlay draws only recorded mapped `roi` and `center_px`, matching detection image stamps within 30 ms of recorded downward frame stamps. It shows class, map XY, error, ambiguity and timestamps. No detector is rerun and no recorded H-shape fit is claimed.
- Empty mapped arrays differ from missing matched data. Video gaps hold the previous recorded frame with a visible label; CSV timestamps define the 1x ROS timeline. Each output frame has a timestamp/match sidecar row. Source video frame count must agree with its CSV.
- `vision_metrics.bag` has no images. Separately recorded `downward.mp4` and `downward.csv` are therefore required for this overlay. The generic bag replay camera renderer cannot supply this missing image stream.
- Synthetic center/TF fixtures, closed preparation-bag analysis, class-confusion separation and a tiny timestamped video were validated. These checks are tool validation, not formal-flight results. Formal metrics are produced only from the corrected closed runs.

The exact ring confirmation parameter is `/target_memory/drop_circle_geometry_confidence=0.75`, distinct from circle detector quality and YOLO confidence. The corrected formal wrapper records `/tf`, `/tf_static`, `/uav_vision/detections_mapped`, `/uav_vision/targets`, `/uav_vision/selected_target`, `/uav_vision/drop_offset`, `/uav_vision/align_mode`, `/mavros/local_position/pose`, `/mission/release_result`. Existing release ACKs are also recorded in `key_events.jsonl`. Earlier preparation notes naming `/mission/release_event` do not describe the corrected formal wrapper.
