# Independent simulation truth for one target at one image timestamp.
Header header
string target_id
string class_name
string gazebo_link_name
bool pose_valid
bool projection_valid
bool center_in_frame
bool fully_in_frame
geometry_msgs/Point world_center
geometry_msgs/Point camera_center
geometry_msgs/Point pixel_center
sensor_msgs/RegionOfInterest roi
float32 distance_m

