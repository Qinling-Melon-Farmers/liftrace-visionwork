tmux kill-session -t lubao
tmux new-session -d -s lubao

tmux send-keys "rosbag record /camera/color/image_raw /livox/lidar /livox/imu /tf /tf_static /fastplanner/goal /mavros/setpoint_position/local /freedom/static_pointcloud /planning_vis/trajectory /cloud_registered /sdf_map/occupancy_inflate /freedom/static_voxels /mavros/local_position/pose /iris_mid360/camera/rgb/compressed" C-m 

tmux -2 attach-session -t lubao
 