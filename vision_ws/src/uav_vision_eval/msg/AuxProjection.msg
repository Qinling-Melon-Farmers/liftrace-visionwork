# 斜下辅助相机单个粗定位结果。仅用于可行性评测，不是控制接口。
std_msgs/Header header
uint32 detection_index
string class_name
bool valid
string reason
# mono_ground / depth_roi / mono_fallback / invalid
string range_source
# 相机光学坐标系角度：bearing 向图像右为正，elevation_down 向图像下为正 [rad]
float32 bearing_rad
float32 elevation_down_rad
float32 range_m
float32 position_uncertainty_m
float32 depth_median_m
float32 depth_mad_m
float32 depth_valid_fraction
geometry_msgs/Point map_point
string map_frame
