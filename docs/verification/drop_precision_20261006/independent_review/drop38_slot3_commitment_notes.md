# Seed38 022323 slot3 read-only diagnosis
Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/drop_precision_seed38_20261007_022323
Closed vision_metrics.bag read successfully after .active became .bag. Outer manifest final cleanup completion was not yet recorded when inspected; cleanup authority remains C/main.
Excerpt export: /tmp/drop38_slot3_bag.jsonl (112-165s).
No repository writes/build/simulation.

Observed:
- First and only slot3 arbiter lock log 114.998: panzer/7, anchor XY=(7.007,-4.242), action decision15 attempt1 slot3.
  Same-time bag FC pose=(7.0068593025,-4.2417464256); use as anchor approximation because log rounds to millimetres and subscriber delivery can differ.
- First descent-entry log 115.022. Controller logs goal 7.018559,-4.227048; exactly matches offset source114.943 receipt114.998, full goal=(7.0185593845,-4.2270476706), tolerance=.05920244m.
  Initial goal-anchor separation only about1.88cm, not a .27m lock-point discrepancy.
- Frozen goal logged unchanged through118.670.
- At118.686 generic rejected stale/invalid/unbound exact drop projection.
- At118.721 controller actual adjust target becomes7.04,-4.19; matching new offsets instead of captured goal.
- At119.002 bag offset source118.977 jumps to(7.4706224,-3.9839523), radius28.46px vs398.72px at prior source118.698. Same geometry ID8; quality .9753 and map_valid remain true. This proves an abrupt candidate change, not its visual cause (no image inspection).
- At119.020 logged adjust target=(7.42,-4.01), distance exactly .500000 (existing approach clamp).
- At122.020 emitted setpoint=(7.4205184,-4.0108266,.1000000).
- Only one descent-entry log in entire run after113s; no evidence of a second stable capture.
- Arbiter permission true from commitment121.102; first commitment_position_drift121.306, not142.702. Closest FC at121.301 is about.208m from initial anchor;142.699 about.436m.
- Context records112-165s for slot3 remain active, same decision15 attempt1. No newaction/cancel visible.

Source mechanism:
dropOffsetCallback -> failed projectExactDropOffsetToTarget -> clearExactDropCommitment.
With no pending/completed physical call, this clears capture_tolerance_m and count_aligning.
It does not raise/reset align_height. Subsequent valid offsets then enter the pre-capture target-update path.
WayPointDetectDone continues publishing that updated XY at retained low release Z. Thus freeze can be exited and low-level tracking restarted without a successful recapture.
This differs from the earlier intentional update-while-captured implementation: here frozen mode first exits through rejection.
Existing arbiter anchor does not reset merely because controller capture clears. It correctly denies drift past .20m.

Trigger boundary:
All offsets around118.686 in bag are map_valid, finite, quality valid, source age ~.1s, action unchanged.
Context received in bag118.686 carries header118.689 (3ms future); previous context118.639.
dropObservationFresh requires now>=stamp, and projectExactDropOffsetToTarget checks context freshness before the captured-state early return.
A controller callback at logged118.686 seeing that future context would therefore reject and clear the capture.
This is a specific, strongly consistent timing trigger, NOT proven exact predicate: the rejection log omits which check failed and controller subscriber ordering differs from bag recorder ordering. Do not label it stale >.5s or malformed geometry as a proven fact.
Raw FC/context/evidence logs support the reset mechanism; internal capture scalar/count were not directly bagged.

Conclusion:
Initial anchor and capture goal agree within1.9cm. Evidence supports rejection -> capture reset -> unlatched XY updates while Z remains low -> departure beyond original authorization anchor. No successful recapture observed.
Candidate remains non-promoted; this note proposes no production change or rerun.
