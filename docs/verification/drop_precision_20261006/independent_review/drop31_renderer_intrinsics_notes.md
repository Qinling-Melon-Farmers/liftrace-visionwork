# Seed31 camera rendering/CameraInfo consistency: final read-only check
Installed packages: gazebo11/libgazebo11 11.15.1-1~focal; ros-noetic-gazebo-plugins 2.9.3-1focal.20250521.004734.
Official version-tagged source was read via web. Distortion.cc and gazebo_ros_camera_utils.cpp were cached in /tmp/drop31_renderer_source; Camera.cc download failed, so its authoritative URL remains in sources.json. This is upstream source matched to package versions, not a recompiled/dynamically traced installed binary.

Actual model: vision_ws/src/uav_vision_eval/models/iris_mid360_start_fov/model.sdf, selected through frozen replay include chain.
Camera has horizontal_fov=1.44593453190313, image1280x720, renderer distortion coefficients and center(.493493643,.552175530).
ROS plugin also sets Cx631.671863/Cy397.566381, focalLength725.351006, autoDistortion=true, borderCrop=false.
NO camera/lens/intrinsics is present.

Source evidence:
- Camera.cc: constructor cameraUsingIntrinsics=false (line68); LoadCameraIntrinsics lines192-208 only uses lens/intrinsics to update fx/fy/cx/cy; UpdateFOV lines1936-1956 otherwise sets symmetric Ogre FOV/aspect.
- Camera.cc lines183-187 loads renderer distortion. It is NOT metadata-only.
- Distortion.cc lines117-124 loads coefficients and lensCenter; SetCamera lines156-238 builds distortion mapping; RefreshCompositor lines431-436 enables image compositor.
- Distortion::Distort lines488-512 subtracts center, applies Brown radial/tangential terms, then adds center back. At all-zero coefficients output equals input for any center. Thus distortion center is not a base pinhole principal-point shift.
- gazebo_ros_camera_utils.cpp lines516-532: autoDistortion copies coefficients FROM renderer INTO CameraInfo. It does not install plugin Cx/Cy in renderer.
- Same file lines543-570 puts Cx/Cy into published K/P. No UpdateCameraIntrinsics/custom projection setter in this plugin source.
Inference from model plus matched-version source: base pinhole rendering remains centered, while published K treats631.67/397.57 as pinhole principal point. Off-axis distortion-center configuration does not make these equivalent. This is a concrete simulator calibration-contract mismatch, not evidence of an incorrect real camera calibration.
Do not claim renderer has D=0: renderer distortion is configured and applied.

Requested diagnostic (no parameter fitting):
Same captured raw center pixels, same interpolated Gazebo body pose and known extrinsics, same ground_z=-.22.
Three fixed camera choices:
1 actual recorded K+D;
2 actual K with D=0 (separates small local distortion effect);
3 ideal symmetric K: cx=640,cy=360,fx=fy=1280/(2*tan(hfov/2))=725.3510059644452, D=0.
Truth is used solely as evaluation pose/target. No parameters chosen by minimizing errors.
Residual norm cm, actual K+D -> symmetric K/D0:
red_cross 7.0768 -> .2106, XY(-1.3188,-6.9529)->(+.2032,+.0551).
bridge 8.7024 ->1.3504, XY(-1.9539,-8.4803)->(-.3770,-1.2968).
panzer 5.9707 ->1.4517, XY(-1.3878,-5.8072)->(+.1735,+1.4413).
Actual K/D0 residuals7.0778/8.7037/5.9712: eliminating D alone does not explain the large difference at these near-center samples.
Numbers: /tmp/drop31_renderer_K_diagnostic.json

Boundary:
Symmetric K/D0 is a diagnostic approximation, NOT an exact inverse of the configured renderer's distortion compositor. Full renderer-specific warp, sampling, target-surface height, detection-center and timestamp errors remain unseparated.
Results strongly support the pinhole-principal-point mismatch as the main common negative-Y component in these three samples; they do not prove all flight error or seed38 control failure has this cause.
No runtime camera projection matrix or pre-distortion image was recorded, so this is source/configuration evidence plus offline numerical support, not runtime calibration acceptance.
Do not modify real-camera K/D, production or sim models from this diagnostic. Candidate remains off/no promotion. No builds/simulation/repository writes.
