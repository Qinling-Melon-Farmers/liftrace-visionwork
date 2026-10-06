# Experimental shadow interface, not a MAVROS measurement.
# Pose = T_LI (IMU in LIO world); velocity/gravity in L; biases in I.
std_msgs/Header header
string imu_frame
uint64 epoch
bool valid
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 gyro_bias
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gravity
# Row-major error covariance: [p_L, right_rotation_I, v_L, bg_I, ba_I, g_L].
float64[324] covariance
# Same sample-noise variance convention as FAST-LIO, [gyro, acc, bg, ba].
float64[12] process_noise
# Apply raw_acc * accel_scale BEFORE subtracting accel_bias.
float64 accel_scale
# Corrected IMU stamp = raw stamp + imu_time_offset (same as slow path).
float64 imu_time_offset
