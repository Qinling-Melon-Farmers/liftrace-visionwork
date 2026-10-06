# Seed31 020102 read-only decomposition
Run: /home/xhj/liftrace-worktrees/r2026-high-view-search/logs/drop_precision_seed31_20261007_020102
Frozen scene: docs/verification/snake3_camera2m_20261005/generated/snake3_31/snake3_seed31
Inputs: bag DropOffset + full-rate MAVROS PoseStamped; CSV MAVROS setpoint + Gazebo truth; precision_eval.py analyze ACK identity/time/catalog.
Offsets: /tmp/drop31_exact_offsets.jsonl
Full FC poses: /tmp/drop31_fc_pose.jsonl
Results: /tmp/drop31_ack_decomposition.json
Red unique match diagnostic: /tmp/drop31_red_capture_match.json

Captured source stamps: red_cross 76.110 (receipt 76.133), bridge 110.724 (110.747), panzer 132.908 (132.942).
Bridge/panzer terminal setpoints directly match unique offsets after float32 command conversion.
Red has no unclipped descent CSV sample before ACK; reconstructing the existing 3D limiter using bag FC poses identifies the 76.110 source offset. All recorded descent setpoints match that fixed goal; runner-up source 76.021 has max transverse residual 0.165mm rather than floating-point residual. No nearest-offset guess.
The limiter produces changing emitted XY while its underlying goal remains fixed; this is not post-capture geometry retargeting.
Physical slot compensation is zero for this simulation.

Definitions (all XY in camera_init/world-aligned coordinates):
T = FC_ack - goal
P = goal - truth_target - (FC_src - GT_src)
D = (FC_src - GT_src) - (FC_ack - GT_ack)
T+P+D = GT_ack - truth_target.
Closure is an algebraic consistency check, not proof of perfect timestamp alignment.
Use source timestamp interpolation for FC, Gazebo recorder-time interpolation for GT.
FC brackets 28-33ms; GT brackets 100-102ms.
At capture GT nearest ages red/bridge/panzer=19/32/28ms; ACK=45/20/33ms.
Capture GT bracket XY displacement red/bridge/panzer=1.06/0.70/0.47cm; ACK=0.077/0.602/0.245cm. These are sensitivity scales, not rigorous error bounds.
Unknown Gazebo transport delay and truth sampling curvature remain; capture event is not a separately logged full precision controller snapshot.
Recovered offset match and setpoint replay make source identification much stronger than selecting the last image near ACK.

Largest term for first two is relative capture projection (9.82/11.38cm).
Tracking 3.04/4.62cm; bias change 1.66/4.09cm. Vector directions matter.
Red tracking partially cancels projection; bridge tracking amplifies negative-Y error and bias change amplifies negative-X error.
Panzer total 3.35cm arises partly from cancellation despite a 7.14cm projection term.
The projection term combines image-center selection, attitude/TF/extrinsics, ground-plane model, and residual time/frame errors; this decomposition alone cannot separate those.
This identifies dominant terms in the new run, not a controlled causal proof of the old-to-new degradation. No old capture-frame decomposition was inferred.
All metrics describe body origin at mock ACK, not parcel impact.
No repository writes, builds, ROS nodes, or simulations.
