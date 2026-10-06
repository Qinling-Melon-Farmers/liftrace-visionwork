# Seed31 020102: camera pose and geometry timestamp checks (read-only)
Source tree: /home/xhj/liftrace-worktrees/r2026-high-view-search, frozen run 7d312fa2.
No repository writes, builds, nodes or simulations.

## Camera TF / model
Actual replay include chain: drop_precision replay -> seed38_resume replay -> fov_inner_repair -> vehicle_sdf iris_mid360_start_fov/model.sdf.
Do not confuse this with the older iris_mid360_ks2a543_installed model: the actual start_fov link has yaw=-pi/2.
start_fov camera link pose: xyz=(0,0,-0.16), rpy=(0,pi/2,-pi/2).
After Gazebo-camera to optical axes conversion this equals bag static vision_body->downward_camera_optical_frame xyz=(0,0,-0.16), quaternion=(0,1,0,0).
No fixed mounting translation/tilt mismatch found in the configured model vs recorded TF.
Bag dynamic camera_init->vision_body is sourced from /mavros/local_position/pose.
No camera LinkStates or raw image topic in this bag, so a separate measured runtime camera mounting pose is unavailable.

At captured source red/bridge/panzer, interpolated FC vs Gazebo body rotation differences are 1.492/1.268/0.831 degrees.
Rotation difference vectors (world axes, degrees): (-.611,+1.344,-.215),(-.693,+1.056,-.104),(-.153,+.751,-.320).
These are sampled estimator/body differences; three snapshots do not prove a fixed calibration offset.
Camera lever-arm translation difference after removing body-position bias is only (-.375,-.170),(-.295,-.193),(-.210,-.042) cm XY.

Offline same-pixel projection using recorded CameraInfo K/D and static extrinsics reproduces original goals to <=1.4e-8m.
Replacing FC pose with interpolated Gazebo pose, without changing center or intrinsics:
- red: pose effect after removing FC-GT XY bias=(-3.50,-1.60)cm; remaining residual=(-1.32,-6.95)cm.
- bridge: pose effect=(-2.84,-1.84)cm; remaining residual=(-1.95,-8.48)cm.
- panzer: pose effect=(-2.05,-.45)cm; remaining residual=(-1.39,-5.81)cm.
Pose effect includes attitude, lever-arm and height differences. It is not purely attitude.
Thus camera/body estimate mismatch contributes but does not explain the large common negative-Y residual.
Remaining residual is not proof of a bad geometry center: image center, rendering/intrinsics consistency, ground plane height, and sampling timing remain combined.
CameraInfo K/D numerically match the declared start_fov plugin configuration; actual rendered-ray versus CameraInfo calibration was not established here.
FC TF slerp spans 32-33ms at captures, truth spans100-101ms; Gazebo truth stamped at recorder receipt. Preserve prior uncertainty caveats.

## Center/frame stamps
circle_detector_node.cpp and cross_detector_node.cpp compute on callback image and publish that image header. target_detector.py also carries input image header.
target_refiner has no asynchronous geometry cache: it associates detections in one incoming array.
However detection_fusion.py source_sync_slop_sec=.05 permits different source stamps in one bucket and uses target_detector header as bucket authority.
target_refiner copies ring center into a deepcopy of semantic detection and does not copy/check ring header.
Red dual confirmation deepcopies geometry (preserves geometry header), but target_memory chooses array header as now/last_seen.
target_map_projector uses array header stamp. exact drop_aligner selects array==last_seen and requests TF at last_seen.
Therefore downstream exact lookup alone does not guarantee same-exposure geometry for arbitrary allowed fusion inputs.

Closed bag captures:
76.110 red_cross: array stamp == geometry detection stamp, center_source red_cross_geometry.
110.724 bridge: array == bridge == retained circle stamp; centers identical.
132.908 panzer: array == panzer == retained circle stamp; centers identical.
All original goal map_points exactly match these mapped detections.
Whole bag mapped geometry-bearing records: circle763, red_cross323, bridge409, panzer191, tent115, pillbox41. Zero individual-header vs array-header timestamp mismatches.
Semantic headers can conceal copied-ring source stamps in principle; retained circles at the captured frames supply the needed independent check here.
Raw detector outputs/images were not bagged. Do not claim independent raw-exposure proof beyond recorded headers and inspected callback paths.
Conclusion: source code permits near-frame fusion but no evidence it affected these three captures. No proven asynchronous cache bug at target_refiner.

Artifacts:
/tmp/drop31_camera_tf_check.json
/tmp/drop31_camera_pose_projection_check.json
/tmp/drop31_detection_stamp_check.json
/tmp/drop31_tf_mapped.jsonl
